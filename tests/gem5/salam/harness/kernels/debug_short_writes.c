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

// Exercise the three CommInterface write ports with short payloads.
#include <stdint.h>

#include "../harness_clstr_hw_defines.h"

static void
writeBytes(volatile uint8_t *ptr)
{
    ptr[0] = 0x12;
    ptr[1] = 0x34;
    *(volatile uint16_t *)(ptr + 2) = 0x5678;
    *(volatile uint32_t *)(ptr + 4) = 0x12345678;
}

void
top(void)
{
    volatile uint32_t *out = (volatile uint32_t *)OUT;
    volatile uint32_t *spm = (volatile uint32_t *)MEM;
    volatile uint32_t *ram = (volatile uint32_t *)0x90000000;
    writeBytes((volatile uint8_t *)out);
    writeBytes((volatile uint8_t *)spm);
    writeBytes((volatile uint8_t *)ram);
    out[2] = spm[0];
    out[3] = spm[1];
    out[4] = ram[0];
    out[5] = ram[1];
}
