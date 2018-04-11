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
 *
 * Authors: Matt Horsnell
 *          Prakash Ramrakhyani
 */

#include <cstdio>
#include <iostream>
#include <string>

#include "crypto.hh"

namespace ArmISA {

void
Crypto::sha256Op(
    uint32_t *X,
    uint32_t *Y,
    uint32_t *Z)
{
    uint32_t T0, T1, T2, T3;
    for (int i = 0; i < 4; ++i) {
        T0 = choose(Y[0], Y[1], Y[2]);
        T1 = majority(X[0], X[1], X[2]);
        T2 = Y[3] + sigma1(Y[0]) + T0 + Z[i];
        X[3] = T2 + X[3];
        Y[3] = T2 + sigma0(X[0]) + T1;
        // Rotate
        T3 = Y[3];
        Y[3] = Y[2]; Y[2] = Y[1]; Y[1] = Y[0]; Y[0] = X[3];
        X[3] = X[2]; X[2] = X[1]; X[1] = X[0]; X[0] = T3;
    }
}

void
Crypto::_sha1Op(
    uint32_t *X,
    uint32_t *Y,
    uint32_t *Z,
    SHAOp op)
{
    uint32_t T1, T2;

    for (int i = 0; i < 4; ++i) {
        switch (op) {
          case CHOOSE:   T1 = choose(X[1], X[2], X[3]); break;
          case PARITY:   T1 = parity(X[1], X[2], X[3]); break;
          case MAJORITY: T1 = majority(X[1], X[2], X[3]); break;
          default: return;
        }
        Y[0] += ror(X[0], 27) + T1 + Z[i];
        X[1] = ror(X[1], 2);
        T2 = Y[0];
        Y[0] = X[3];
        X[3] = X[2]; X[2] = X[1]; X[1] = X[0]; X[0] = T2;
    }
}

void
Crypto::sha256H(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    uint32_t X[4], Y[4], Z[4];
    load3Reg(&X[0], &Y[0], &Z[0], output, input, input2);
    sha256Op(&X[0], &Y[0], &Z[0]);
    store1Reg(output, &X[0]);
}

void
Crypto::sha256H2(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    uint32_t X[4], Y[4], Z[4];
    load3Reg(&X[0], &Y[0], &Z[0], output, input, input2);
    sha256Op(&Y[0], &X[0], &Z[0]);
    store1Reg(output, &X[0]);
}

void
Crypto::sha256Su0(uint8_t *output, uint8_t *input)
{
    uint32_t X[4], Y[4];
    uint32_t T[4];

    load2Reg(&X[0], &Y[0], output, input);

    T[3] = Y[0]; T[2] = X[3]; T[1] = X[2]; T[0] = X[1];

    T[3] = ror(T[3], 7) ^ ror(T[3], 18) ^ (T[3] >> 3);
    T[2] = ror(T[2], 7) ^ ror(T[2], 18) ^ (T[2] >> 3);
    T[1] = ror(T[1], 7) ^ ror(T[1], 18) ^ (T[1] >> 3);
    T[0] = ror(T[0], 7) ^ ror(T[0], 18) ^ (T[0] >> 3);

    X[3] += T[3];
    X[2] += T[2];
    X[1] += T[1];
    X[0] += T[0];

    store1Reg(output, &X[0]);
}

void
Crypto::sha256Su1(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    uint32_t X[4], Y[4], Z[4];
    uint32_t T0[4], T1[4], T2[4], T3[4];

    load3Reg(&X[0], &Y[0], &Z[0], output, input, input2);

    T0[3] = Z[0]; T0[2] = Y[3]; T0[1] = Y[2]; T0[0] = Y[1];
    T1[1] = Z[3]; T1[0] = Z[2];
    T1[1] = ror(T1[1], 17) ^ ror(T1[1], 19) ^ (T1[1] >> 10);
    T1[0] = ror(T1[0], 17) ^ ror(T1[0], 19) ^ (T1[0] >> 10);
    T3[1] = X[1] + T0[1]; T3[0] = X[0] + T0[0];
    T1[1] = T3[1] + T1[1]; T1[0] = T3[0] + T1[0];
    T2[1] = ror(T1[1], 17) ^ ror(T1[1], 19) ^ (T1[1] >> 10);
    T2[0] = ror(T1[0], 17) ^ ror(T1[0], 19) ^ (T1[0] >> 10);
    T3[1] = X[3] + T0[3]; T3[0] = X[2] + T0[2];
    X[3] = T3[1] + T2[1];
    X[2] = T3[0] + T2[0];
    X[1] = T1[1]; X[0] = T1[0];

    store1Reg(output, &X[0]);
}

void
Crypto::sha1Op(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2,
    SHAOp op)
{
    uint32_t X[4], Y[4], Z[4];
    load3Reg(&X[0], &Y[0], &Z[0], output, input, input2);
    _sha1Op(&X[0], &Y[0], &Z[0], op);
    store1Reg(output, &X[0]);
}

void
Crypto::sha1C(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    sha1Op(output, input, input2, CHOOSE);
}

void
Crypto::sha1P(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    sha1Op(output, input, input2, PARITY);
}

void
Crypto::sha1M(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    sha1Op(output, input, input2, MAJORITY);
}

void
Crypto::sha1H(uint8_t *output, uint8_t *input)
{
    uint32_t X[4], Y[4];
    load2Reg(&X[0], &Y[0], output, input);
    X[0] = ror(Y[0], 2);
    store1Reg(output, &X[0]);
}

void
Crypto::sha1Su0(
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    uint32_t X[4], Y[4], Z[4], T[4];
    load3Reg(&X[0], &Y[0], &Z[0], output, input, input2);

    T[3] = Y[1]; T[2] = Y[0]; T[1] = X[3]; T[0] = X[2];
    X[3] = T[3] ^ X[3] ^ Z[3];
    X[2] = T[2] ^ X[2] ^ Z[2];
    X[1] = T[1] ^ X[1] ^ Z[1];
    X[0] = T[0] ^ X[0] ^ Z[0];

    store1Reg(output, &X[0]);
}

void
Crypto::sha1Su1(uint8_t *output, uint8_t *input)
{
    uint32_t X[4], Y[4], T[4];
    load2Reg(&X[0], &Y[0], output, input);

    T[3] = X[3] ^ 0x0;
    T[2] = X[2] ^ Y[3];
    T[1] = X[1] ^ Y[2];
    T[0] = X[0] ^ Y[1];
    X[2] = ror(T[2], 31); X[1] = ror(T[1], 31); X[0] = ror(T[0], 31);
    X[3] = ror(T[3], 31) ^ ror(T[0], 30);

    store1Reg(output, &X[0]);
}

void
Crypto::load2Reg(
    uint32_t *X,
    uint32_t *Y,
    uint8_t *output,
    uint8_t *input)
{
    for (int i = 0; i < 4; ++i) {
        X[i] = *((uint32_t *)&output[i*4]);
        Y[i] = *((uint32_t *)&input[i*4]);
    }
}

void
Crypto::load3Reg(
    uint32_t *X,
    uint32_t *Y,
    uint32_t *Z,
    uint8_t *output,
    uint8_t *input,
    uint8_t *input2)
{
    for (int i = 0; i < 4; ++i) {
        X[i] = *((uint32_t *)&output[i*4]);
        Y[i] = *((uint32_t *)&input[i*4]);
        Z[i] = *((uint32_t *)&input2[i*4]);
    }
}

void
Crypto::store1Reg(uint8_t *output, uint32_t *X)
{
    for (int i = 0; i < 4; ++i) {
        output[i*4] = (uint8_t)(X[i]);
        output[i*4+1] = (uint8_t)(X[i] >> 8);
        output[i*4+2] = (uint8_t)(X[i] >> 16);
        output[i*4+3] = (uint8_t)(X[i] >> 24);
    }
}

} // namespace ArmISA
