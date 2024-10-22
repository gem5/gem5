/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-Only */
/*
 * Copyright (c) 2024 The Regents of the University of California
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

#ifndef __GEM5_POKE_H__
#define __GEM5_POKE_H__

/* Global variable for setting poke target */
extern u64 gem5_poke_opcode;

/* Location to store op result, accessible via /dev/gem5/retval */
extern u64 gem5_poke_retval;

/* C declarations for calling / linking to driver source */
u64 noinline gem5_poke_func(u64 a, u64 b, u64 c, u64 d, u64 e, u64 f);
#define POKE0() \
    gem5_poke_func(0, 0, 0, 0, 0, 0)
#define POKE1(a) \
    gem5_poke_func((u64)(a), 0, 0, 0, 0, 0)
#define POKE2(a, b) \
    gem5_poke_func((u64)(a), (u64)(b), 0, 0, 0, 0)
#define POKE3(a, b, c) \
    gem5_poke_func((u64)(a), (u64)(b), (u64)(c), 0, 0, 0)
#define POKE4(a, b, c, d) \
    gem5_poke_func((u64)(a), (u64)(b), (u64)(c), (u64)(d), 0, 0)
#define POKE5(a, b, c, d, e) \
    gem5_poke_func((u64)(a), (u64)(b), (u64)(c), (u64)(d), (u64)(e), 0)
#define POKE6(a, b, c, d, e, f) \
    gem5_poke_func((u64)(a), (u64)(b), (u64)(c), (u64)(d), (u64)(e), (u64)(f))

#endif /* __GEM5_POKE_H__ */
