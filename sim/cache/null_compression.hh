/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __NULL_COMPRESSION_HH__
#define __NULL_COMPRESSION_HH__

/**
 * @file
 * This file defines a doNothing compression algorithm.
 */

/**
 * A dummy compression class to use when no data compression is desired.
 */
class NullCompression
{
  public:
    /**
     * Uncompress the data, causes a fatal since no data should be compressed.
     * @param dest The output buffer.
     * @param src  The compressed data.
     * @param size The number of bytes in src.
     *
     * @retval The size of the uncompressed data.
     */
    static int uncompress(uint8_t * dest, uint8_t *src, int size)
    {
        fatal("Can't uncompress data");
    }

    /**
     * Compress the data, just returns the source data.
     * @param dest The output buffer.
     * @param src  The data to be compressed.
     * @param size The number of bytes in src.
     *
     * @retval The size of the compressed data.
     */

    static int compress(uint8_t *dest, uint8_t *src, int size)
    {
        memcpy(dest,src,size);
        return size;
    }
};

#endif //__NULL_COMPRESSION_HH__
