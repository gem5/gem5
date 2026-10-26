/*
 * Copyright (c) 2026 Akanksha Chaudhari
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Host program for the SALAM kernel harness. inputs.h is written by the
// test and defines N_IN, N_OUT, N_RUNS and inputs[].

#include <stdint.h>
#include <stdio.h>

#include <gem5/m5ops.h>

#include "../harness_clstr_hw_defines.h"
#include "bench.h"
#include "inputs.h"

volatile uint8_t *top = (uint8_t *)KERNEL;
volatile int stage = 0;

#ifdef DEVICE_TEST_KIND
#ifndef DEVICE_TRANSFER_BYTES
#define DEVICE_TRANSFER_BYTES 128
#endif
#ifndef DEVICE_DMA_ROUTE
#define DEVICE_DMA_ROUTE 0
#endif
// MMIO payloads are fixed independently of the counters under test. The
// test config places a slow memory at 0x2ff10000, a stream buffer at
// 0x2ff20000, and a stream DMA at 0x2ff30000.
static void
setRegister(volatile uint8_t *base, unsigned offset, uint64_t value,
            unsigned bytes)
{
    for (unsigned i = 0; i < bytes; i++) {
        base[offset + i] = (uint8_t)(value >> (8 * i));
    }
}

// Fix packet widths independently of compiler selection of STM/STRD/LDM.
// Each 64-bit payload uses two 4-byte writes and one paired LDM read
// (an 8-byte MicroLdr2Uop in gem5).
static void
writeStream(volatile uint64_t *port, uint64_t value)
{
    volatile uint32_t *words = (volatile uint32_t *)port;
    *words = (uint32_t)value;
    *words = (uint32_t)(value >> 32);
}

static uint64_t
readStream(volatile uint64_t *port)
{
    uint64_t value;
    __asm__ volatile("ldm %1, {%Q0, %R0}"
                     : "=&r"(value)
                     : "r"(port)
                     : "memory");
    return value;
}

#ifdef DEVICE_READ_WIDTHS
// Consume 16 payload bytes with every supported read width. The final
// byte checks that an 8-byte read consumed exactly eight FIFO bytes.
static void
readStreamBlock(volatile uint64_t *port, uint64_t *first, uint64_t *second)
{
    const unsigned widths[] = {1, 2, 4, 8, 1};
    unsigned offset = 0;
    *first = *second = 0;
    for (unsigned i = 0; i < 5; i++) {
        unsigned width = widths[i];
        uint64_t value;
        switch (width) {
            case 1:
                value = *(volatile uint8_t *)port;
                break;
            case 2:
                value = *(volatile uint16_t *)port;
                break;
            case 4:
                value = *(volatile uint32_t *)port;
                break;
            default:
                value = readStream(port);
                break;
        }
        for (unsigned j = 0; j < width; j++, offset++) {
            uint64_t byte = (value >> (8 * j)) & 0xff;
            if (offset < 8) {
                *first |= byte << (8 * offset);
            } else {
                *second |= byte << (8 * (offset - 8));
            }
        }
    }
}
#endif

static int
testDevices(void)
{
    volatile uint32_t *src = (volatile uint32_t *)0x2ff10000;
    volatile uint32_t *dst = (volatile uint32_t *)0x2ff10100;
    if (DEVICE_DMA_ROUTE == 1) {
        dst = (volatile uint32_t *)0x2ff11000;
    } else if (DEVICE_DMA_ROUTE == 2) {
        src = (volatile uint32_t *)0x2ff11000;
    }
    volatile uint64_t *stream = (volatile uint64_t *)0x2ff20000;
    volatile uint8_t *sdma = (volatile uint8_t *)0x2ff30000;
    volatile uint64_t *dma_stream = (volatile uint64_t *)0x2ff30020;
    volatile uint32_t *dma_status = (volatile uint32_t *)0x2ff30028;

    volatile uint64_t *dma_read = dma_stream;
    if (DEVICE_TEST_KIND == 3) {
        // The 8-byte data aperture at PIO offset 32 selects read().
        // Keep the separate stream/status ports outside that aperture.
        dma_stream = (volatile uint64_t *)0x2ff30040;
        dma_status = (volatile uint32_t *)0x2ff30048;
    }

    if (DEVICE_TEST_KIND == 1) {
#ifdef DEVICE_RESET
        // Reset while the first stream write's response is outstanding.
        m5_dump_reset_stats(10000, 0);
#endif
        writeStream(stream, UINT64_C(0x0123456789abcdef));
        writeStream(stream, UINT64_C(0xfedcba9876543210));
        uint64_t first, second;
#ifdef DEVICE_READ_WIDTHS
        readStreamBlock(stream, &first, &second);
#else
        first = readStream(stream);
        second = readStream(stream);
#endif
        if (first != UINT64_C(0x0123456789abcdef) ||
            second != UINT64_C(0xfedcba9876543210)) {
            printf("DEVICE_CHECK_FAILED stream buffer\n");
            m5_exit(0);
            return 1;
        }
    } else {
        for (unsigned i = 0; i < DEVICE_TRANSFER_BYTES / 4; i++) {
            src[i] = 17 * i + 3;
        }
        if (DEVICE_TEST_KIND == 0) {
            volatile uint8_t *dma = (volatile uint8_t *)DMA_Flags;
            setRegister(dma, 1, (uintptr_t)src, 8);
            setRegister(dma, 9, (uintptr_t)dst, 8);
            setRegister(dma, 17, DEVICE_TRANSFER_BYTES, 4);
            *dma = 1;
#ifdef DEVICE_RESET
            m5_dump_reset_stats(0, 0);
#endif
            while (!(*dma & 4))
                ;
        } else {
            setRegister(sdma, 4, (uintptr_t)src, 8);
            setRegister(sdma, 12, (uintptr_t)dst, 8);
            setRegister(sdma, 20, DEVICE_TRANSFER_BYTES, 4);
            setRegister(sdma, 24, 1, 1);
            setRegister(sdma, 25, 1, 1);
            setRegister(sdma, 26, DEVICE_TRANSFER_BYTES, 4);
            setRegister(sdma, 30, 1, 1);
            setRegister(sdma, 31, 1, 1);
            *sdma = 3;
#ifdef DEVICE_RESET
            m5_dump_reset_stats(0, 0);
#endif
#ifndef DEVICE_INCREMENTAL_STREAM
            while (*dma_status < DEVICE_TRANSFER_BYTES)
                ;
#endif
#ifdef DEVICE_READ_WIDTHS
            for (unsigned i = 0; i < 8; i++) {
                uint64_t first, second;
                readStreamBlock(dma_read, &first, &second);
                writeStream(dma_stream, first);
                writeStream(dma_stream, second);
            }
#else
            for (unsigned i = 0; i < DEVICE_TRANSFER_BYTES / 8; i++) {
#ifdef DEVICE_INCREMENTAL_STREAM
                while (*dma_status < 8)
                    ;
#endif
                uint64_t value = readStream(dma_stream);
                writeStream(dma_stream, value);
            }
#endif
            while (*sdma & 12)
                ;
        }
        for (unsigned i = 0; i < DEVICE_TRANSFER_BYTES / 4; i++) {
            if (dst[i] != 17 * i + 3) {
                printf("DEVICE_CHECK_FAILED word %u\n", i);
                m5_exit(0);
                return 1;
            }
        }
    }
    printf("DEVICE_CHECK_PASSED\n");
    m5_exit(0);
    return 0;
}
#endif

int
main(void)
{
#ifdef DEVICE_TEST_KIND
    return testDevices();
#endif
    volatile int32_t *in = (volatile int32_t *)IN;
    volatile int32_t *out = (volatile int32_t *)OUT;

    for (int i = 0; i < N_IN; i++) {
        in[i] = inputs[i];
    }
    // RegisterBank writes become visible after its delta time; wait
    // for the inputs before starting the kernel.
    for (int i = 0; i < N_IN; i++) {
        while (in[i] != inputs[i])
            ;
    }
    for (int run = 0; run < N_RUNS; run++) {
        stage = 0;
        *top = 0x01;
#ifdef STATS_RESET_AFTER_START
        m5_reset_stats(0, 0);
#endif
        while (stage < 1)
            ;
        for (int i = 0; i < N_OUT; i++) {
            printf("RUN[%d] OUT[%d]=%d 0x%08x\n", run, i, (int)out[i],
                   (unsigned)out[i]);
        }
    }
#ifdef STATS_DUMPS_AT_END
    for (int i = 0; i < STATS_DUMPS_AT_END; i++) {
        m5_dump_stats(0, 0);
    }
#endif
    m5_exit(0);
    return 0;
}
