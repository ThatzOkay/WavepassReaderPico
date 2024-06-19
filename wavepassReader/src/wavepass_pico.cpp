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

#ifdef WITH_USBHID
#define USB_HID_COOLDOWN 3000
#define cardio
#endif

/* ICCA-only (slotted) options */
#define PIN_EJECT_BUTTON 7
#define KEYPAD_BLANK_EJECT 1 // make blank key from keypad eject currently inserted card (ICCA only)
#define AUTO_EJECT_TIMER 0   // auto eject valid cards after a set delay (in ms), 0 to disable (note: must be smaller than USB_HID_COOLDOWN)

// #define PRESS_KEY_ON_BOOT //press a key on boot (useful for some motherboards)
#define PRESS_KEY_TIMER 5000
#define PRESS_KEY_DURATION 500
#define PRESS_KEY KEY_F1

bool g_passthrough = false; // native mode (use pico as simple TTL to USB)
bool g_encrypted = true;    // FeliCa support and new readers (set to false for ICCA support, set to true otherwise)

static struct
{
    uint8_t current[9];
    uint8_t reported[9];
    uint64_t report_time;
} hid_cardio;

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

int main()
{
    sleep_ms(50);
    set_sys_clock_khz(150000, true);
    board_init();

    tusb_init();
    stdio_init_all();

    gpio_init(PIN_EJECT_BUTTON);
    gpio_pull_up(PIN_EJECT_BUTTON);

    uart_init(uart1, 57600);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, false);

    // Enable the UART
    uart_set_irq_enables(uart1, true, false);

    acio_open();

    sleep_ms(1000);
    iccx_init(0, g_encrypted);

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

        // TODO keychecking

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
                return;

            if (type == 1)
            {
                hid_cardio.current[0] = 0x01;
                memcpy(hid_cardio.current + 1, uid, 8);
            }
        }

        report_hid_cardio();
    }
    return 0;
}

void passthrough_loop() {
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
                    putchar(buf[i]);
                }
            }
            if (tud_cdc_write_flush())
            {
                uint8_t buf[64];
                uint32_t count = tud_cdc_read(buf, sizeof(buf));
                for (uint32_t i = 0; i < count; i++)
                {
                    putchar(buf[i]);
                }
            }
        }
    }
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
