/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "queue_dma.h"
#include "stdio.h"

int
launchCpy()
{
    if (valid[rdQueuePtr] == 1) {
        *SRC = jobs[rdQueuePtr].src;
        *DST = jobs[rdQueuePtr].dst;
        *LEN = jobs[rdQueuePtr].len;
        *FLAGS |= 0x01;
        valid[rdQueuePtr] = 0;
        inQueue--;
        if (rdQueuePtr < (QUEUE_SIZE - 1)) {
            rdQueuePtr++;
        } else {
            rdQueuePtr = 0;
        }
        return 0;
    } else {
        *FLAGS = 0;
        return -1;
    }
}

extern int dma_signal;

int
dmacpy(void *dst, void *src, int len)
{
    if (valid[wrQueuePtr] == 0) {
        dma_signal = 0;
        jobs[wrQueuePtr].src = (size_t)src;
        jobs[wrQueuePtr].dst = (size_t)dst;
        jobs[wrQueuePtr].len = len;
        valid[wrQueuePtr] = 1;
        inQueue++;
        if (wrQueuePtr < (QUEUE_SIZE - 1)) {
            wrQueuePtr++;
        } else {
            wrQueuePtr = 0;
        }
        if ((*FLAGS & 0x02) != 0x02) {
            launchCpy();
        }
        return 0;
    } else {
        return -1;
    }
}

int
pollDma()
{
    return ((*FLAGS & 0x04) == 0x04);
}
void
resetDma()
{
    *FLAGS = 0;
}
