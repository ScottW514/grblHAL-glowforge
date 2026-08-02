/*
  serial.c - Grbl protocol stream over TCP or stdio

  Part of grblHAL-glowforge. The io_stream_t + ring-buffer shape is from
  the grblHAL Simulator's serial.c (Copyright (c) 2017-2025 Terje Io);
  the emulated-UART pump is replaced by serial_poll(), which moves bytes
  between real fds and the rings on the protocol thread.

  A client disconnect or write failure must never exit the process: the
  pulse-device fd is flock'd as the kernel dead man's switch, so dying on
  a UI disconnect would e-stop a running job. Drop the client, keep
  running, let it reconnect.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

// grbl headers first: glibc's <sys/stat.h> (via fcntl.h) defines st_mtime
// as a macro, which must not be in scope when the core's vfs.h declares
// its struct field of the same name.
#include "serial.h"
#include "driver.h"
#include "platform.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static int listen_fd = -1;
static int client_fd = -1;

void serial_set_listen_fd (int fd)
{
    listen_fd = fd;
}

static void drop_client (void)
{
    if(client_fd >= 0) {
        close(client_fd);
        client_fd = -1;
    }
}

//
// serialGetC - returns -1 if no data available
//
static int32_t serialGetC (void)
{
    int32_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    data = (int32_t)rxbuffer.data[bptr++];          // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return data;
}

static inline uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

static void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}

static void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

static bool serialPutC (const uint8_t c)
{
    uint_fast16_t next_head;

    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

    while(txbuffer.tail == next_head) {                         // Buffer full, block until space is available...
        // hal.stream_blocking_callback -> protocol_execute_realtime -> our
        // realtime hook -> serial_poll() drains TX on this same thread.
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
    txbuffer.head = next_head;                                  // and update head pointer

    return true;
}

static void serialWriteS (const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static uint16_t serialTxCount (void)
{
    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    return &stream;
}

/* --- fd transport pump --------------------------------------------------- */

static void rx_byte (uint8_t data)
{
    if(data == 0x06) {          // ^F: request clean shutdown
        driver_request_exit();
        return;
    }

    if(!enqueue_realtime_command(data)) {
        uint_fast16_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);
        if(bptr == rxbuffer.tail)
            rxbuffer.overflow = 1;
        else {
            rxbuffer.data[rxbuffer.head] = data;
            rxbuffer.head = bptr;
        }
    }
}

static void rx_poll (void)
{
    if(listen_fd >= 0) {

        if(client_fd < 0) {
            int fd = accept(listen_fd, NULL, NULL);
            if(fd >= 0) {
                int flags = fcntl(fd, F_GETFL, 0);
                if(flags != -1)
                    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
                // Disable Nagle: senders poll with single-byte '?' and
                // responses are small; Nagle + delayed ACK adds latency.
                int nodelay = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
                client_fd = fd;
            }
        }

        if(client_fd >= 0) {
            uint8_t buf[64];
            ssize_t n = read(client_fd, buf, sizeof(buf));
            if(n > 0) {
                for(ssize_t i = 0; i < n; i++)
                    rx_byte(buf[i]);
            } else if(n == 0)
                drop_client();              // client hung up; keep running
            // n < 0: EAGAIN (no data) or transient error - ignore
        }

    } else {
        uint8_t c = platform_poll_stdin();
        if(c && c != 0xFF)
            rx_byte(c);
    }
}

static void tx_drain (void)
{
    while(txbuffer.tail != txbuffer.head) {

        int fd = client_fd >= 0 ? client_fd : (listen_fd < 0 ? STDOUT_FILENO : -1);

        if(fd < 0) {
            // Port mode with no client connected: discard output so the
            // ring can never wedge the protocol thread.
            txbuffer.tail = txbuffer.head;
            break;
        }

        uint_fast16_t tail = txbuffer.tail;
        uint_fast16_t n = txbuffer.head >= tail ? txbuffer.head - tail : TX_BUFFER_SIZE - tail;

        ssize_t w = write(fd, &txbuffer.data[tail], n);
        if(w > 0)
            txbuffer.tail = (tail + (uint_fast16_t)w) & (TX_BUFFER_SIZE - 1);
        else if(w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            break;                          // socket full; retry next poll
        else {
            drop_client();                  // write error: drop, keep running
            if(listen_fd < 0)
                break;                      // stdout failed; nothing to do
        }
    }
}

void serial_poll (void)
{
    rx_poll();
    tx_drain();
}
