/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

/** @file
 * LZSSCompression definitions.
 */

#include <assert.h>

#include "base/compression/lzss_compression.hh"

#include "base/misc.hh" //for fatal

void
LZSSCompression::findSubString(uint8_t *src, int back, int size, uint16_t &L,
                               uint16_t &P)
{
    int front = 0;
    int max_length = size - back;
    L = 0;
    P = back - 1;
    while (front < back) {
        while (src[front] != src[back] && front < back) ++front;
        if (front >= back) {
            return;
        }
        int i = 1;
        while (src[front+i] == src[back+i] && i < max_length) ++i;
        if (i >= L) {
            L = i;
            P = front;
        }
        if (src[front+i] != src[back+i-1]) {
            // can't find a longer substring until past this point.
            front += i;
        } else {
            ++front;
        }
    }
}

int
LZSSCompression::emitByte(uint8_t *dest, uint8_t byte)
{
    if ((byte >> 5 & 0x7) == 0 || (byte >> 5 & 0x7) == 7) {
        // If the top 3 bits are the same, emit 00<6bits>
        dest[0] = byte & 0x3f;
        return 1;
    } else {
        // emit 01XXXXXX <8 bits>
        dest[0] = 0x40;
        dest[1] = byte;
        return 2;
    }
}

void
LZSSCompression::emitString(uint8_t *dest, uint16_t P, uint16_t L)
{
    // Emit 1<7P> <5P><3L> <8L>
    dest[0] = 1<<7 | (P >> 5 & 0x7f);
    dest[1] = ((P & 0x1f) << 3) | (L>>8 & 0x3);
    dest[2] = L & 0xFF;
}

int
LZSSCompression::compress(uint8_t *dest, uint8_t *src, int size)
{
    if (size > 4096) {
        fatal("Compression can only handle block sizes of 4096 bytes or less");
    }

    // Encode the first byte.
    int dest_index = emitByte(dest, src[0]);
    int i = 1;
    // A 11 bit field
    uint16_t L;
    // A 12 bit field
    uint16_t P = 0;

    while (i < size && dest_index < size) {
        L = 0;

        if (dest_index+3 >= size) {
            dest_index = size;
            continue;
        }

        if (i == size - 1) {
            // Output the character
            dest_index += emitByte(&dest[dest_index], src[i]);
            ++i;
            continue;
        }
        findSubString(src, i, size, L, P);
        if (L > 1) {
            // Output the string reference
            emitString(&dest[dest_index], P, L);
            dest_index += 3;
            i = i+L;
        } else {
            // Output the character
            dest_index += emitByte(&dest[dest_index], src[i]);
            ++i;
        }
    }

    if (dest_index >= size) {
        // Have expansion instead of compression, just copy.
        memcpy(dest,src,size);
        return size;
    }
    return dest_index;
}

int
LZSSCompression::uncompress(uint8_t *dest, uint8_t *src, int size)
{
    int index = 0;
    int i = 0;
    while (i < size) {
        if (src[i] & 1<<7 ) {
            // We have a string
            // Extract P
            int start = (src[i] & 0x3f)<<5 | ((src[i+1] >> 3) & 0x1f);
            // Extract L
            int len = (src[i+1] & 0x07)<<8 | src[i+2];
            i += 3;
            for (int j = start; j < start+len; ++j) {
                dest[index++] = dest[j];
            }
        } else {
            // We have a character
            if (src[i] & 1<<6) {
                // Value is in the next byte
                dest[index++] = src[i+1];
                i += 2;
            } else {
                // just extend the lower 6 bits
                dest[index++] = (src[i] & 0x3f) | ((src[i] & 1<<5) ? 0xC0 : 0);
                ++i;
            }
        }
    }
    return index;
}
