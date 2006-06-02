/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Erik Hallnor
 */

#ifndef __LZSS_COMPRESSION_HH__
#define __LZSS_COMPRESSION_HH__

/** @file
 * LZSSCompression declarations.
 */

#include "sim/host.hh" // for uint8_t

/**
 * Simple LZSS compression scheme.
 */
class LZSSCompression
{
    /**
     * Finds the longest substring for the given offset.
     * @param src The source block that we search for substrings.
     * @param back The larger offset.
     * @param size The size of the source block.
     * @param L The length of the largest substring.
     * @param P The starting offset of the largest substring.
     */
    void findSubString(uint8_t *src, int back, int size, uint16_t &L,
                       uint16_t &P);

    /**
     * Emit an encoded byte to the compressed data array. If the 2 high
     * order bits can be signed extended, use 1 byte encoding, if not use 2
     * bytes.
     * @param dest The compressed data.
     * @param byte The byte to emit.
     * @return The number of bytes used to encode.
     */
    int emitByte(uint8_t *dest, uint8_t byte);

    /**
     * Emit a string reference to the compressed data array. A string reference
     * always uses 3 bytes. 1 flag bit, 12 bits for the starting position, and
     * 11 bits for the length of the string. This allows compression of 4096
     * byte blocks with string lengths of up to 2048 bytes.
     * @param dest The compressed data.
     * @param P The starting position in the uncompressed data.
     * @param L The length in bytes of the string.
     */
    void emitString(uint8_t *dest, uint16_t P, uint16_t L);

  public:
    /**
     * Compresses the source block and stores it in the destination block. If
     * the compressed block grows to larger than the source block, it aborts
     * and just performs a copy.
     * @param dest The destination block.
     * @param src The block to be compressed.
     * @param size The size of the source block.
     * @return The size of the compressed block.
     *
     * @pre Destination has enough storage to hold the compressed block.
     */
    int compress(uint8_t *dest, uint8_t *src, int size);

    /**
     * Unompresses the source block and stores it in the destination block.
     * @param dest The destination block.
     * @param src The block to be uncompressed.
     * @param size The size of the source block.
     * @return The size of the uncompressed block.
     *
     * @pre Destination has enough storage to hold the uncompressed block.
     */
    int uncompress(uint8_t *dest, uint8_t *src, int size);
};

#endif //__LZSS_COMPRESSION_HH__
