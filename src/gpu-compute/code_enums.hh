/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 *
 * Author: Anthony Gutierrez
 */

#ifndef __CODE_ENUMS_HH__
#define __CODE_ENUMS_HH__

#define IS_OT_GLOBAL(a) ((a)>=Enums::OT_GLOBAL_READ \
                    && (a)<=Enums::OT_GLOBAL_LDAS)
#define IS_OT_SHARED(a) ((a)>=Enums::OT_SHARED_READ \
                    && (a)<=Enums::OT_SHARED_LDAS)
#define IS_OT_PRIVATE(a) ((a)>=Enums::OT_PRIVATE_READ \
                    && (a)<=Enums::OT_PRIVATE_LDAS)
#define IS_OT_SPILL(a) ((a)>=Enums::OT_SPILL_READ \
                    && (a)<=Enums::OT_SPILL_LDAS)
#define IS_OT_READONLY(a) ((a)>=Enums::OT_READONLY_READ \
                    && (a)<=Enums::OT_READONLY_LDAS)
#define IS_OT_FLAT(a) ((a)>=Enums::OT_FLAT_READ && (a)<=Enums::OT_FLAT_LDAS)

#define IS_OT_LDAS(a) ((a)==Enums::OT_GLOBAL_LDAS||(a)==Enums::OT_SHARED_LDAS \
                    ||(a)==Enums::OT_PRIVATE_LDAS||(a)==Enums::OT_SPILL_LDAS \
                    ||(a)==Enums::OT_READONLY_LDAS||(a)==Enums::OT_FLAT_LDAS)

#define IS_OT_READ(a) ((a)==Enums::OT_GLOBAL_READ||(a)==Enums::OT_SHARED_READ \
                    ||(a)==Enums::OT_PRIVATE_READ||(a)==Enums::OT_SPILL_READ \
                    ||(a)==Enums::OT_READONLY_READ||(a)==Enums::OT_FLAT_READ)

#define IS_OT_READ_GM(a) \
    ((a)==Enums::OT_GLOBAL_READ||(a)==Enums::OT_SPILL_READ \
    ||(a)==Enums::OT_READONLY_READ)

#define IS_OT_READ_LM(a) ((a)==Enums::OT_SHARED_READ)

#define IS_OT_READ_RM(a) ((a)==Enums::OT_READONLY_READ)

#define IS_OT_READ_PM(a) ((a)==Enums::OT_PRIVATE_READ)

#define IS_OT_WRITE(a) \
    ((a)==Enums::OT_GLOBAL_WRITE||(a)==Enums::OT_SHARED_WRITE \
    ||(a)==Enums::OT_PRIVATE_WRITE||(a)==Enums::OT_SPILL_WRITE \
    ||(a)==Enums::OT_READONLY_WRITE||(a)==Enums::OT_FLAT_WRITE)

#define IS_OT_WRITE_GM(a) \
    ((a)==Enums::OT_GLOBAL_WRITE||(a)==Enums::OT_SPILL_WRITE \
    ||(a)==Enums::OT_READONLY_WRITE)

#define IS_OT_WRITE_LM(a) ((a)==Enums::OT_SHARED_WRITE)

#define IS_OT_WRITE_PM(a) ((a)==Enums::OT_PRIVATE_WRITE)

#define IS_OT_ATOMIC(a) ((a)==Enums::OT_GLOBAL_ATOMIC \
                    ||(a)==Enums::OT_SHARED_ATOMIC \
                    ||(a)==Enums::OT_PRIVATE_ATOMIC \
                    ||(a)==Enums::OT_SPILL_ATOMIC \
                    ||(a)==Enums::OT_READONLY_ATOMIC \
                    ||(a)==Enums::OT_BOTH_MEMFENCE \
                    ||(a)==Enums::OT_FLAT_ATOMIC)

#define IS_OT_ATOMIC_GM(a) ((a)==Enums::OT_GLOBAL_ATOMIC \
                    ||(a)==Enums::OT_SPILL_ATOMIC \
                    ||(a)==Enums::OT_READONLY_ATOMIC \
                    ||(a)==Enums::OT_GLOBAL_MEMFENCE \
                    ||(a)==Enums::OT_BOTH_MEMFENCE)

#define IS_OT_ATOMIC_LM(a) ((a)==Enums::OT_SHARED_ATOMIC \
                    ||(a)==Enums::OT_SHARED_MEMFENCE)

#define IS_OT_ATOMIC_PM(a) ((a)==Enums::OT_PRIVATE_ATOMIC)

#define IS_OT_HIST(a) ((a)==Enums::OT_GLOBAL_HIST \
                    ||(a)==Enums::OT_SHARED_HIST \
                    ||(a)==Enums::OT_PRIVATE_HIST \
                    ||(a)==Enums::OT_SPILL_HIST \
                    ||(a)==Enums::OT_READONLY_HIST \
                    ||(a)==Enums::OT_FLAT_HIST)

#define IS_OT_HIST_GM(a) ((a)==Enums::OT_GLOBAL_HIST \
                    ||(a)==Enums::OT_SPILL_HIST \
                    ||(a)==Enums::OT_READONLY_HIST)

#define IS_OT_HIST_LM(a) ((a)==Enums::OT_SHARED_HIST)

#define IS_OT_HIST_PM(a) ((a)==Enums::OT_PRIVATE_HIST)

#endif // __CODE_ENUMS_HH__
