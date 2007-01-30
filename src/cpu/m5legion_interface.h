/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#include <unistd.h>

#define VERSION         0xA1000009
#define OWN_M5          0x000000AA
#define OWN_LEGION      0x00000055

/** !!!  VVV Increment VERSION on change VVV !!! **/

typedef struct {
    uint32_t flags;
    uint32_t version;

    uint64_t pc;
    uint64_t new_pc;
    uint64_t cycle_count;
    uint64_t new_cycle_count;
    uint32_t instruction;
    uint32_t new_instruction;
    uint64_t intregs[32];
    uint64_t fpregs[32];

    uint64_t tpc[8];
    uint64_t tnpc[8];
    uint64_t tstate[8];
    uint16_t tt[8];
    uint64_t tba;

    uint64_t hpstate;
    uint64_t htstate[8];
    uint64_t htba;
    uint16_t pstate;

    uint64_t y;
    uint64_t fsr;
    uint8_t ccr;
    uint8_t tl;
    uint8_t gl;
    uint8_t asi;
    uint8_t pil;

    uint8_t cwp;
    uint8_t cansave;
    uint8_t canrestore;
    uint8_t otherwin;
    uint8_t cleanwin;

    uint64_t itb[64];
    uint64_t dtb[64];

} SharedData;

/** !!! ^^^  Increment VERSION on change ^^^ !!! **/

