/*
 * Copyright (c) 2018 ARM Limited
 * All rights reserved
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

#ifndef __ARCH_ARM_INSTS_CRYPTO_HH__
#define __ARCH_ARM_INSTS_CRYPTO_HH__

#include <cstdint>

namespace gem5
{

namespace ArmISA {

class Crypto
{
    enum SHAOp : uint8_t
    {
        CHOOSE = 0,
        PARITY,
        MAJORITY
    };

    /** Look up table for subByttes transformation */
    static const uint8_t aesSBOX[256];

    /** Look up table for inverse subBytes transformation */
    static const uint8_t aesInvSBOX[256];

    static const uint8_t aesSHIFT[16];
    static const uint8_t aesINVSHIFT[16];

    /**
     * Look up table for Finite Field logarithm where the base
     * is the element {03} in the field G(256)
     */
    static const uint8_t aesFFLOG[256];

    /**
     * Look up table for {03}^X where {03} and X are elements
     * in the filed G(256)
     */
    static const uint8_t aesFFEXP[256];

    /** Finite field multiplication of two elements in the field G(256) */
    uint8_t aesFFMul(uint8_t a, uint8_t b);

    uint8_t aesFFMul2(uint8_t a)
    {
        return ((a & 0x80) ? ((a << 1) ^ 0x1b) : (a << 1));
    }

    void aesSubBytes(uint8_t *output, uint8_t *input);
    void aesInvSubBytes(uint8_t *output, uint8_t *input);
    void aesShiftRows(uint8_t *output, uint8_t *input);
    void aesInvShiftRows(uint8_t *output, uint8_t *input);
    void aesAddRoundKey(uint8_t *output, uint8_t *input, uint8_t *key);

    uint32_t ror(uint32_t x, uint8_t shift)
    {
        return (x >> shift) | (x << (32 - shift));
    }

    uint32_t choose(uint32_t X, uint32_t Y, uint32_t Z)
    {
        return (((Y ^ Z) & X) ^ Z);
    }

    uint32_t parity(uint32_t X, uint32_t Y, uint32_t Z)
    {
        return (X ^ Y ^ Z);
    }

    uint32_t majority(uint32_t X, uint32_t Y, uint32_t Z)
    {
        return ((X & Y) | ((X | Y) & Z));
    }

    uint32_t sigma0(uint32_t X)
    {
        return ror(X,2) ^ ror(X,13) ^ ror(X,22);
    }

    uint32_t sigma1(uint32_t X)
    {
        return ror(X,6) ^ ror(X,11) ^ ror(X,25);
    }

    void sha256Op(uint32_t *X, uint32_t *Y, uint32_t *Z);
    void sha1Op(uint8_t *output, uint8_t *input, uint8_t *input2, SHAOp op);
    void _sha1Op(uint32_t *X, uint32_t *Y, uint32_t *Z, SHAOp op);

    void load2Reg(uint32_t *X, uint32_t *Y, uint8_t *output, uint8_t *input);
    void load3Reg(uint32_t *X, uint32_t *Y, uint32_t *Z,
                  uint8_t *output, uint8_t *input, uint8_t *input2);
    void store1Reg(uint8_t *output, uint32_t *X);

  public:
    void aesMixColumns(uint8_t *output, uint8_t *input);
    void aesInvMixColumns(uint8_t *output, uint8_t *input);
    void aesEncrypt(uint8_t *output, uint8_t *input, uint8_t *key);
    void aesDecrypt(uint8_t *output, uint8_t *input, uint8_t *key);
    void sha256H(uint8_t *output, uint8_t *input, uint8_t *input2);
    void sha256H2(uint8_t *output, uint8_t *input, uint8_t *input2);
    void sha256Su0(uint8_t *output, uint8_t *input);
    void sha256Su1(uint8_t *output, uint8_t *input, uint8_t *input2);

    void sha1C(uint8_t *output, uint8_t *input, uint8_t *input2);
    void sha1P(uint8_t *output, uint8_t *input, uint8_t *input2);
    void sha1M(uint8_t *output, uint8_t *input, uint8_t *input2);
    void sha1H(uint8_t *output, uint8_t *input);
    void sha1Su0(uint8_t *output, uint8_t *input, uint8_t *input2);
    void sha1Su1(uint8_t *output, uint8_t *input);
};

} // namespace ArmISA
} // namespace gem5

#endif //__ARCH_ARM_INSTS_CRYPTO_HH__
