/*
 * Copyright (c) 2019 The Regents of the University of California
 * All rights reserved
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
 * Authors: Bobby R. Bruce
 */


#include <gtest/gtest.h>

#include "base/loader/exec_aout.h"

#define OMAGIC_STRUCT (*((const aout_exechdr *)omagic))
#define NMAGIC_STRUCT (*((const aout_exechdr *)nmagic))
#define ZMAGIC_STRUCT (*((const aout_exechdr *)zmagic))
#define NO_MAGIC_STRUCT (*((const aout_exechdr *)no_magic))

#if (defined(_BIG_ENDIAN) || !defined(_LITTLE_ENDIAN))\
    && BYTE_ORDER==BIG_ENDIAN
const uint8_t omagic[] {
    0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t nmagic[] {
    0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t zmagic[] {
    0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t no_magic[] {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00
};
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
const uint8_t omagic[] {
    0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t nmagic[] {
    0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t zmagic[] {
    0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t no_magic[] {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01
};
#else
    #error Invalid Endianess
#endif

TEST(ExecAoutTest, IsNotBadMagicNumber)
{
    EXPECT_FALSE(N_BADMAG(OMAGIC_STRUCT));
    EXPECT_FALSE(N_BADMAG(NMAGIC_STRUCT));
    EXPECT_FALSE(N_BADMAG(ZMAGIC_STRUCT));
}

TEST(ExecAoutTest, IsBadMagicNumber)
{
    EXPECT_TRUE(N_BADMAG(NO_MAGIC_STRUCT));
}

TEST(ExecAoutTest, AlignNotZmagic)
{
    /*
     * N_ALIGN will return x if the aout_exechdr does not have ZMAGIC as a
     * magic number.
     */
    int64_t x = 0xABCD;
    EXPECT_EQ(x, N_ALIGN(OMAGIC_STRUCT, x));
    EXPECT_EQ(x, N_ALIGN(NMAGIC_STRUCT, x));
    EXPECT_EQ(x, N_ALIGN(NO_MAGIC_STRUCT, x));
}

TEST(ExecAoutTest, AlignIsZmagic)
{
    /*
     * N_ALIGN will round up X by AOUT_LDPGSZ (1 << 13) if the aout_exechdr
     * has ZMAGIC as a magic number.
     */
    int64_t x = (1 << 20) + (1 << 18) + (1 << 12) + (1 << 5) + 1;
    EXPECT_EQ((1 << 20) + (1 << 18) + (1 << 13), N_ALIGN(ZMAGIC_STRUCT, x));
}

TEST(ExecAoutTest, AlignIsZmagicNoRounding)
{
    /*
     * In this case, there's no rounding needed.
     */
    int64_t x = (1 << 20) + (1 << 18) + (1 << 13);
    EXPECT_EQ(x, N_ALIGN(ZMAGIC_STRUCT, x));
}