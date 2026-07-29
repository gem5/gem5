/*
 * Copyright (c) 2026 Magnushst
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

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static volatile uint64_t sink;

static uint64_t
parse_u64(const char *text, const char *name)
{
    char *end = NULL;
    errno = 0;
    const unsigned long long value = strtoull(text, &end, 0);
    if (errno || !end || *end != '\0') {
        fprintf(stderr, "invalid %s: %s\n", name, text);
        exit(2);
    }
    return (uint64_t)value;
}

static inline uint64_t
xorshift64(uint64_t value)
{
    value ^= value << 13;
    value ^= value >> 7;
    value ^= value << 17;
    return value;
}

static uint64_t
arithmetic(uint64_t iterations)
{
    uint64_t a = 0x9e3779b97f4a7c15ULL;
    uint64_t b = 0xbf58476d1ce4e5b9ULL;
    uint64_t c = 0x94d049bb133111ebULL;
    uint64_t d = 0xd6e8feb86659fd93ULL;
    for (uint64_t i = 0; i < iterations; ++i) {
        a = (a + b) ^ (c >> 11);
        b = (b * 33) + (d ^ i);
        c = (c + d) ^ (a << 7);
        d = (d * 17) + (b >> 5);
    }
    return a ^ b ^ c ^ d;
}

static uint64_t
branch_heavy(uint64_t iterations)
{
    uint64_t state = 0x243f6a8885a308d3ULL;
    uint64_t result = 0;
    for (uint64_t i = 0; i < iterations; ++i) {
        state = xorshift64(state);
        if (state & 1) {
            result += state ^ i;
        } else {
            result -= (state >> 3) + i;
        }
        if ((state & 0x18) == 0x18) {
            result ^= state << 9;
        }
    }
    return result ^ state;
}

static uint64_t
streaming(uint64_t iterations, size_t elements)
{
    uint64_t *a = malloc(elements * sizeof(*a));
    uint64_t *b = malloc(elements * sizeof(*b));
    if (!a || !b) {
        fputs("allocation failed\n", stderr);
        exit(2);
    }
    for (size_t i = 0; i < elements; ++i) {
        a[i] = (uint64_t)i * 17 + 3;
        b[i] = (uint64_t)i ^ 0x5a5a5a5a5a5a5a5aULL;
    }
    for (uint64_t pass = 0; pass < iterations; ++pass) {
        for (size_t i = 0; i < elements; ++i) {
            a[i] = a[i] + (b[i] ^ pass);
            b[i] = (b[i] << 1) ^ (a[i] >> 3);
        }
    }
    uint64_t result = 0;
    for (size_t i = 0; i < elements; i += 257) {
        result ^= a[i] + b[i];
    }
    free(b);
    free(a);
    return result;
}

static uint64_t
pointer_chase(uint64_t iterations, size_t elements)
{
    uint32_t *order = malloc(elements * sizeof(*order));
    uint32_t *next = malloc(elements * sizeof(*next));
    if (!order || !next || elements > UINT32_MAX) {
        fputs("allocation failed or working set too large\n", stderr);
        exit(2);
    }
    for (size_t i = 0; i < elements; ++i) {
        order[i] = (uint32_t)i;
    }
    uint64_t state = 0x13198a2e03707344ULL;
    for (size_t i = elements - 1; i > 0; --i) {
        state = xorshift64(state);
        const size_t other = state % (i + 1);
        const uint32_t tmp = order[i];
        order[i] = order[other];
        order[other] = tmp;
    }
    for (size_t i = 0; i < elements; ++i) {
        next[order[i]] = order[(i + 1) % elements];
    }
    uint32_t current = order[0];
    for (uint64_t i = 0; i < iterations; ++i) {
        current = next[current];
    }
    const uint64_t result = current ^ state;
    free(next);
    free(order);
    return result;
}

static uint64_t
mixed(uint64_t iterations, size_t elements)
{
    uint64_t *data = malloc(elements * sizeof(*data));
    if (!data) {
        fputs("allocation failed\n", stderr);
        exit(2);
    }
    for (size_t i = 0; i < elements; ++i) {
        data[i] = (uint64_t)i * 0x9e3779b1U;
    }
    uint64_t state = 0xa4093822299f31d0ULL;
    uint64_t result = 0;
    for (uint64_t i = 0; i < iterations; ++i) {
        state = xorshift64(state);
        const size_t index = (state >> 16) % elements;
        const uint64_t value = data[index];
        if ((value ^ state) & 4) {
            result += value * 13 + i;
        } else {
            result ^= (value >> 7) + state;
        }
        data[index] = value + result + (state & 0xff);
    }
    result ^= data[state % elements];
    free(data);
    return result;
}

static uint64_t
queue_pressure(uint64_t iterations, unsigned dependency_steps)
{
    uint64_t a0 = 1, a1 = 3, a2 = 5, a3 = 7;
    uint64_t a4 = 11, a5 = 13, a6 = 17, a7 = 19;
    uint64_t dependency = 0x082efa98ec4e6c89ULL;
    for (uint64_t i = 0; i < iterations; ++i) {
        a0 = a0 * 3 + i;
        a1 = a1 * 5 + i;
        a2 = a2 * 7 + i;
        a3 = a3 * 11 + i;
        a4 = a4 * 13 + i;
        a5 = a5 * 17 + i;
        a6 = a6 * 19 + i;
        a7 = a7 * 23 + i;
        for (unsigned step = 0; step < dependency_steps; ++step) {
            dependency = dependency * 33 + (dependency >> 11) + i;
        }
    }
    return a0 ^ a1 ^ a2 ^ a3 ^ a4 ^ a5 ^ a6 ^ a7 ^ dependency;
}

static void
usage(const char *program)
{
    fprintf(
        stderr,
        "usage: %s MODE ITERATIONS [SIZE_OR_DEPENDENCY]\n"
        "modes: arithmetic branch stream pointer mixed queue\n",
        program);
    exit(2);
}

int
main(int argc, char **argv)
{
    if (argc < 3) {
        usage(argv[0]);
    }
    const char *mode = argv[1];
    const uint64_t iterations = parse_u64(argv[2], "iterations");
    const uint64_t parameter =
        argc > 3 ? parse_u64(argv[3], "size/dependency") : 65536;
    uint64_t result;

    if (!strcmp(mode, "arithmetic")) {
        result = arithmetic(iterations);
    } else if (!strcmp(mode, "branch")) {
        result = branch_heavy(iterations);
    } else if (!strcmp(mode, "stream")) {
        result = streaming(iterations, (size_t)parameter);
    } else if (!strcmp(mode, "pointer")) {
        result = pointer_chase(iterations, (size_t)parameter);
    } else if (!strcmp(mode, "mixed")) {
        result = mixed(iterations, (size_t)parameter);
    } else if (!strcmp(mode, "queue")) {
        result = queue_pressure(iterations, (unsigned)parameter);
    } else {
        usage(argv[0]);
    }

    sink = result;
    printf("%s:%" PRIu64 "\n", mode, result);
    return sink == UINT64_MAX;
}
