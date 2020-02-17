/*
 * Copyright (c) 2019 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/mman.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

int main(void)
{
    uint64_t page_size = 0x1000;
    uint64_t num_pages = 0x10000;
    uint64_t length = page_size * num_pages;

    void *raw = mmap(NULL, length, PROT_WRITE, MAP_ANON|MAP_PRIVATE, -1, 0);
    uint8_t *mem = reinterpret_cast<uint8_t*>(raw);

    srand(0xABCD);

    uint64_t last_byte = page_size - 1;
    uint64_t page_boundaries = num_pages - 1;

    for (int i = 0; i < 2000; i++) {
        uint64_t random_boundary = rand() % page_boundaries;
        uint64_t boundary_offset = random_boundary * page_size;
        uint64_t boundary_last_byte = boundary_offset + last_byte;
        uint32_t *poke = reinterpret_cast<uint32_t*>(mem + boundary_last_byte);
        printf("%p\n", poke);
        uint32_t value = *poke;
    }

    return 0;
}
