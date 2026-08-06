/*
 * Copyright (c) 2026 Mao Weiming
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

/*
 * Stress many outstanding L1D misses so LSQ completions can flood
 * IEW::instToCommit in one tick. Used to reproduce gem5 issue #3096
 * (iewQueue TimeBuffer overflow when wbCycle exceeds forwardComSize).
 *
 * Stride-64 loads target distinct cache lines to maximize MLP. The
 * working set is intentionally larger than a typical 32KiB L1D.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LINE = 64, LINES = 1 << 16 }; /* 4 MiB working set */

static uint64_t
pass(volatile uint8_t *buf, size_t nlines)
{
    uint64_t sum = 0;

    for (size_t i = 0; i < nlines; i++) {
        sum += buf[i * LINE];
    }

    return sum;
}

int
main(void)
{
    size_t bytes = (size_t)LINES * LINE;
    volatile uint8_t *buf = aligned_alloc(LINE, bytes);

    if (!buf) {
        perror("aligned_alloc");
        return 1;
    }

    /* First-touch pages so later load passes are not dominated by faults. */
    memset((void *)buf, 0x5a, bytes);

    uint64_t s = 0;
    for (int r = 0; r < 8; r++) {
        /* Rotate the base so successive passes miss L1/L2 more often. */
        size_t off = ((size_t)r * 17) % LINES;
        s += pass(buf + off * LINE, LINES - off);
        s += pass(buf, LINES / 2);
    }

    /* Dense independent loads to raise simultaneous LSQ wakeups. */
    for (size_t i = 0; i < LINES; i++) {
        s += buf[((i * 131) & (LINES - 1)) * LINE];
        s += buf[((i * 17 + 3) & (LINES - 1)) * LINE];
        s += buf[((i * 41 + 7) & (LINES - 1)) * LINE];
        s += buf[((i * 73 + 11) & (LINES - 1)) * LINE];
    }

    printf("sum=%llu\n", (unsigned long long)s);
    free((void *)buf);
    return 0;
}
