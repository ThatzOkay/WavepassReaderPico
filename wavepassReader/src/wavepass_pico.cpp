#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "bsp/board.h"

#include "hardware/uart.h"

#include "tusb.h"
#include "usb_descriptors.h"

#include "ACIO.h"
#include "ICCx.h"

#define WITH_USBHID

#define DEBUG
#define ICCX_DEBUG true

/* HID Keycode: https://github.com/hathach/tinyusb/blob/master/src/class/hid/hid.h */
// Numpad: 1234567890-.
#define KEYPAD_NKRO_MAP "\x59\x5a\x5b\x5c\x5d\x5e\x5f\x60\x61\x62\x56\x63"

#ifdef WITH_USBHID
#define USB_HID_COOLDOWN 3000
#define cardio
#endif

/* ICCA-only (slotted) options */
#define PIN_EJECT_BUTTON 8
#define KEYPAD_BLANK_EJECT 1 // make blank key from keypad eject currently inserted card (ICCA only)
#define AUTO_EJECT_TIMER 0   // auto eject valid cards after a set delay (in ms), 0 to disable (note: must be smaller than USB_HID_COOLDOWN)

// #define PRESS_KEY_ON_BOOT //press a key on boot (useful for some motherboards)
#define PRESS_KEY_TIMER 5000
#define PRESS_KEY_DURATION 500
#define PRESS_KEY KEY_F1

bool g_passthrough = false; // native mode (use pico as simple TTL to USB)
bool g_encrypted = true;    // FeliCa support and new readers (set to false for ICCA support, set to true otherwise)

static int g_keypad_mask[12] = 
{ICCx_KEYPAD_MASK_0, ICCx_KEYPAD_MASK_00, ICCx_KEYPAD_MASK_EMPTY, 
 ICCx_KEYPAD_MASK_1, ICCx_KEYPAD_MASK_2, ICCx_KEYPAD_MASK_3, 
 ICCx_KEYPAD_MASK_4, ICCx_KEYPAD_MASK_5, ICCx_KEYPAD_MASK_6, 
 ICCx_KEYPAD_MASK_7, ICCx_KEYPAD_MASK_8, ICCx_KEYPAD_MASK_9};

static bool keypad_vals[12];

static struct
{
    uint8_t current[9];
    uint8_t reported[9];
    uint64_t report_time;
} hid_cardio;

void passthrough_loop()
{
    while (1)
    {
        tud_task();
        if (tud_cdc_connected())
        {
            if (tud_cdc_available())
            {
                uint8_t buf[64];
                uint32_t count = tud_cdc_read(buf, sizeof(buf));
                for (uint32_t i = 0; i < count; i++)
                {
                    uart_putc(uart1, buf[i]);
                }
            }
            if (tud_cdc_write_flush())
            {
                uint8_t buf[64];
                uint32_t count = tud_cdc_read(buf, sizeof(buf));
                for (uint32_t i = 0; i < count; i++)
                {
                    uart_putc(uart1, buf[i]);
                }
            }
        }
    }
}

void report_hid_cardio()
{
    if (!tud_hid_ready())
    {
        return;
    }

    uint64_t now = time_us_64();

    if ((memcmp(hid_cardio.current, hid_cardio.reported, 9) != 0) &&
        (now - hid_cardio.report_time > 1000000))
    {

        tud_hid_n_report(0x00, hid_cardio.current[0], hid_cardio.current + 1, 8);
        memcpy(hid_cardio.reported, hid_cardio.current, 9);
        hid_cardio.report_time = now;
    }
}

struct __attribute__((packed)) {
    uint8_t modifier;
    uint8_t keymap[15];
} hid_nkro;

static const char keymap[13] = KEYPAD_NKRO_MAP;

void report_hid_key()
{
    if (!tud_hid_ready()) {
        return;
    }

    uint16_t keys = 0;

    for (int i = 0; i < 12; i++) {
        if (keypad_vals[i]) {
            keys |= (1 << i);
        }
    }

    for (int i = 0; i < 1; i++) {
        uint8_t code = keymap[i];
        uint8_t byte = code / 8;
        uint8_t bit = code % 8;
        if (keys & (1 << i)) {
            hid_nkro.keymap[byte] |= (1 << bit);
        } else {
            hid_nkro.keymap[byte] &= ~(1 << bit);
        }
    }
    tud_hid_n_report(1, 0, &hid_nkro, sizeof(hid_nkro));
}

int main(void)
{
    sleep_ms(50);
    set_sys_clock_khz(150000, true);
    board_init();

    tusb_init();
    stdio_init_all();

    gpio_init(PIN_EJECT_BUTTON);
    gpio_pull_up(PIN_EJECT_BUTTON);

    uart_init(uart1, 57600);
    gpio_set_function(6, GPIO_FUNC_UART);
    gpio_set_function(7, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, false);

    // Enable the UART
    uart_set_irq_enables(uart1, true, false);

    bool opened = acio_open();

    sleep_ms(500);

    if (opened)
    {
        iccx_init(0, g_encrypted);
    }

    while (1)
    {
        if (g_passthrough)
        {
            passthrough_loop();
            return 0;
        }
        tud_task();

        static unsigned long lastReport = 0;

        uint8_t uid[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        uint8_t type = 0;
        uint16_t keystate = 0;
        static uint16_t prev_keystate = 0;

        if (!g_encrypted && gpio_get(PIN_EJECT_BUTTON) == 0)
        {
            iccx_eject_card(AC_IO_ICCA_SLOT_STATE_OPEN);
        }

#if AUTO_EJECT_TIMER > 0
        static bool already_eject = false;
        if (!g_encrypted && !already_eject && ((to_ms_since_boot(get_absolute_time()) - lastReport) >= AUTO_EJECT_TIMER))
        {
            iccx_eject_card(AC_IO_ICCA_SLOT_STATE_OPEN);
            already_eject = true;
        }
#endif

        if (!iccx_scan_card(&type, uid, &keystate, g_encrypted))
        {
#ifdef DEBUG
            printf("Error communicating with wavepass reader.");
#endif
        }
#ifdef KEYPAD_BLANK_EJECT
        if (!g_encrypted && (keystate & ICCx_KEYPAD_MASK_EMPTY))
        {
            iccx_eject_card(AC_IO_ICCA_SLOT_STATE_OPEN);
        }
#endif

        for (int i = 0; i < 12; i++)
        {
            //check_key(i, keystate, prev_keystate);
        }
        prev_keystate = keystate;

        if (type)
        {
#ifdef DEBUG
            printf("Found a card of type ");
            if (type == 1)
                printf("ISO15693");
            else
                printf("FeliCa");
            printf(" with uid =");
            for (int i = 0; i < 8; i++)
            {
                printf(" ");
                if (uid[i] < 0x10)
                    printf("0");
                printf("%X", uid[i]);
            }
            printf("\n");
#endif

            if (to_ms_since_boot(get_absolute_time()) - lastReport < USB_HID_COOLDOWN)
                continue;

            if (type == 1)
            {
                hid_cardio.current[0] = 0x01;
                memcpy(hid_cardio.current + 1, uid, 8);
            }
        }

        report_hid_cardio();
        report_hid_key();
    }
    return 0;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen)
{
    printf("Get from USB %d-%d\n", report_id, report_type);
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize)
{
    printf("Set from USB %d-%d\n", report_id, report_type);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    printf("\nCDC Line State: %d %d", dtr, rts);
}

#define IS_PRESSED(x) ((keystate&x)&&(!(prev_keystate&x)))
#define IS_RELEASED(x) ((!(keystate&x))&&(prev_keystate&x))
static void check_key(uint8_t i, int keystate, int prev_keystate)
{

 if (IS_PRESSED(g_keypad_mask[i]))
 {
#ifdef WITH_USBHID
    keypad_vals[i] = true;
 }
 else if (IS_RELEASED(g_keypad_mask[i]))
 { 
    keypad_vals[i] = false;
#endif  
 }

}