#include "ACIO.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <cstring>

#define ACIO_DEBUG

static uint8_t acio_msg_counter = 1;
static uint8_t acio_node_count;
static char acio_node_products[16][4];

bool acio_send(const uint8_t *buffer, int length)
{
    uint8_t send_buf[512];
    int send_buf_pos = 0;
    uint8_t checksum = 0;

    if (length > sizeof(send_buf))
    {
#ifdef ACIO_DEBUG
        printf("Send buffer overflow \n");
#endif
        return false;
    }

    send_buf[send_buf_pos++] = AC_IO_SOF;

    for (int i = 0; i < length; i++)
    {
        if (buffer[i] == AC_IO_SOF || buffer[i] == AC_IO_ESCAPE)
        {
            send_buf[send_buf_pos++] = AC_IO_ESCAPE;
            send_buf[send_buf_pos++] = ~buffer[i];
        }
        else
        {
            send_buf[send_buf_pos++] = buffer[i];
        }

        checksum += buffer[i];
    }

    if (checksum == AC_IO_SOF || checksum == AC_IO_ESCAPE)
    {
        send_buf[send_buf_pos++] = AC_IO_ESCAPE;
        send_buf[send_buf_pos++] = ~checksum;
    }
    else
    {
        send_buf[send_buf_pos++] = checksum;
    }
#ifdef ACIO_DEBUG
    printf("SEND : ");
    for (int i = 0; i < send_buf_pos; i++)
    {
        if (send_buf[i] < 0x10)
            printf("0");
        printf("%X", send_buf[i]);
        printf(" ");
    }
    printf("\n");
#endif
    uart_write_blocking(uart1, send_buf, send_buf_pos);
//     size_t written = 
//     if (Serial1.write(send_buf, send_buf_pos) != send_buf_pos)
//     {
// #ifdef ACIO_DEBUG
//         printfln("Sending data failed");
// #endif
//         return false;
//     }

    return true;
}

int acio_receive(uint8_t *buffer, int size)
{
    uint8_t recv_buf[512];
    int recv_size = 0;
    int read = 0;
    uint8_t checksum = 0;
    int result_size = 0;
    /* reading a byte stream, we are getting a varying amount
       of 0xAAs before we get a valid message. */
    recv_buf[0] = AC_IO_SOF;
    do {
        while (!uart_is_readable(uart1)) {
            tight_loop_contents();
        }

        // Read a single byte from the UART
        uart_read_blocking(uart1, recv_buf, 1);
    } while (recv_buf[0] == AC_IO_SOF);
    read = uart_is_readable(uart1);

    if (read > 0)
    {

        /* recv_buf[0] is already the first byte of the message.
           now read until nothing's left */
        recv_size++;

        /* important: we have to know how much data we expect
           and have to read until we reach the requested amount.
           Because this can be interrupted by 0 reads and we
           need to handle escaping (which relies on an up to
           date recv_buf[recv_size]) we loop until we get a
           non-zero read. */
        while (size > 0)
        {

            /* we reached the NUMBYTE field, update size accordingly */
            if (recv_size == 5)
            {
#ifdef ACIO_DEBUG
                printf(" remaining data size is ");
                printf("%d", recv_buf[recv_size - 1]);
#endif
                size = recv_buf[recv_size - 1] + 1;
            }

            do
            {
                while (!uart_is_readable(uart1)) {
                    tight_loop_contents();
                }
                uart_read_blocking(uart1, recv_buf + recv_size, 1);
                read = 1;
            } while (read == 0);

            if (read < 0)
            {
                break;
            }

            /* check for escape byte. these don't count towards the
               size we expect! */
            if (recv_buf[recv_size] == AC_IO_ESCAPE)
            {
                /* next byte is our real data
                   overwrite escape byte */
                do
                {
                    while (!uart_is_readable(uart1)) {
                        tight_loop_contents();
                    }
                    uart_read_blocking(uart1, recv_buf + recv_size, 1);
                    read = 1;
                } while (read == 0);

                if (read < 0)
                {
                    break;
                }

                recv_buf[recv_size] = ~recv_buf[recv_size];
            }

            recv_size += read;
            size -= read;
        }

        /* recv_size - 1: omit checksum for checksum calc */
        for (int i = 0; i < recv_size - 1; i++)
        {
            checksum += recv_buf[i];
            buffer[i] = recv_buf[i]; // copy to buffer
        }

        result_size = recv_size - 1;
#ifdef ACIO_DEBUG
        printf("RECV : ");
        for (int i = 0; i < recv_size; i++)
        {
            if (recv_buf[i] < 0x10)
                printf("0");
            printf("%X", recv_buf[i]);
            printf(" ");
        }
        printf("\n");
#endif
        if (checksum != recv_buf[recv_size - 1])
        {
#ifdef ACIO_DEBUG
            printf("Invalid message checksum: \n");
            printf("%X", checksum);
            printf(" != ");
            printf("%X", recv_buf[recv_size - 1]);
            printf("\n");
#endif
            return -1;
        }

        return result_size; // checksum doesn't count
    }

    return -1;
}

int acio_get_counter_and_increase()
{
    return acio_msg_counter++;
}

bool acio_send_and_recv(struct ac_io_message *msg, int resp_size)
{
#ifdef ACIO_DEBUG
    printf("ACIO SEND AND RECV\n");
#endif
    msg->cmd.seq_no = acio_msg_counter++;
    int send_size = offsetof(struct ac_io_message, cmd.raw) + msg->cmd.nbytes;

    if (acio_send((uint8_t *)msg, send_size) <= 0)
    {
        return false;
    }

    // delay(100); //wait a little between send and receive

    /* remember the sent cmd for sanity check */
    uint16_t req_code = msg->cmd.code;

    if (acio_receive((uint8_t *)msg, resp_size) <= 0)
    {
        return false;
    }

    /* sanity check */
    if (req_code != msg->cmd.code)
    {
#ifdef ACIO_DEBUG
        printf("Received invalid response ");
        printf("%X", msg->cmd.code);
        printf(" for request ");
        printf("%X", req_code);
        printf("\n");
#endif
        return false;
    }

    return true;
}

static void acio_init(void)
{

#ifdef ACIO_DEBUG
    printf("INIT DEVICE \n");
#endif
    uint8_t read_buff = 0x00;

    /* init/reset the device by sending 0xAA until 0xAA is returned */
    do
    {
        uart_putc(uart1, AC_IO_SOF);

#ifdef ACIO_DEBUG
        printf("Sent : 0xAA \n");
#endif
        read_buff = uart_getc(uart1);

#ifdef ACIO_DEBUG
        printf("Recv : 0x");
        printf("%X", read_buff);
        printf("\n");
#endif
    } while ((read_buff != AC_IO_SOF));

#ifdef ACIO_DEBUG
    printf("Obtained SOF, clearing out buffer now \n");
#endif
    while (uart_is_readable(uart1))
    {
        uart_getc(uart1);
    }

#ifdef ACIO_DEBUG
    printf("Buffer cleared \n");
#endif
}

static uint8_t acio_enum_nodes(void)
{
    struct ac_io_message msg;

    msg.addr = 0x00;                                  // not to a particular node
    msg.cmd.code = ac_io_u16(AC_IO_CMD_ASSIGN_ADDRS); // enumerate command (00 01)
    msg.cmd.nbytes = 1;
    msg.cmd.count = 0; // 0 on request, will be set to nodecount on reply by acio_send_and_recv

#ifdef ACIO_DEBUG
    printf("Enumerating nodes... \n");
#endif
    if (!acio_send_and_recv(&msg, offsetof(struct ac_io_message, cmd.raw) + 1))
    {
#ifdef ACIO_DEBUG
        printf("Enumerating nodes failed \n");
#endif
        return 0;
    }
#ifdef ACIO_DEBUG
    printf("Enumerating nodes success, got ");
    printf("%d", msg.cmd.count);
    printf(" nodes. \n");
#endif
    return msg.cmd.count;
}

static bool acio_get_version(uint8_t node_id, char product[4])
{
    struct ac_io_message msg;

    msg.addr = node_id;
    msg.cmd.code = ac_io_u16(AC_IO_CMD_GET_VERSION);
    msg.cmd.nbytes = 0;

    if (!acio_send_and_recv(
            &msg,
            offsetof(struct ac_io_message, cmd.raw) +
                sizeof(struct ac_io_version)))
    {
        //        printf("Get version of node %d failed\n", node_id);
        return false;
    }
#ifdef ACIO_DEBUG
    printf("Node ");
    printf("%d", node_id);
    printf(": type ");
    printf("%d", msg.cmd.version.type);
    printf(", flag ");
    printf("%d", msg.cmd.version.flag);
    printf(", version ");
    printf("%d", msg.cmd.version.major);
    printf(".");
    printf("%d", msg.cmd.version.minor);
    printf(".");
    printf("%d", msg.cmd.version.revision);
    printf(", product ");
    printf("%d", msg.cmd.version.product_code[0]);
    printf("%d", msg.cmd.version.product_code[1]);
    printf("%d", msg.cmd.version.product_code[2]);
    printf("%d", msg.cmd.version.product_code[3]);
    printf(", build date: ");
    printf(msg.cmd.version.date);
    printf(" ");
    printf(msg.cmd.version.time);
#endif
    memcpy(product, msg.cmd.version.product_code, 4);

    return true;
}

static bool acio_start_node(uint8_t node_id)
{
    struct ac_io_message msg;

    msg.addr = node_id;
    msg.cmd.code = ac_io_u16(AC_IO_CMD_START_UP);
    msg.cmd.nbytes = 0;

    if (!acio_send_and_recv(
            &msg, offsetof(struct ac_io_message, cmd.raw) + 1))
    {
        //        printf("Starting node %d failed\n", node_id);
        return false;
    }

    //    printf("Started node %d, status: %d\n", node_id, msg.cmd.status);
    return true;
}

bool acio_open()
{
    acio_init();
    acio_node_count = acio_enum_nodes();
    if (acio_node_count == 0)
    {
        return false;
    }
    for (uint8_t i = 0; i < acio_node_count; i++)
    {
        sleep_ms(500);
        if (!acio_get_version(
                i + 1, acio_node_products[i]))
        {
            return false;
        }
    }

    for (uint8_t i = 0; i < acio_node_count; i++)
    {
        sleep_ms(500);
        if (!acio_start_node(i + 1))
        {
            return false;
        }
    }

    return true;
}