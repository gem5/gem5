/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
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
 *
 * Author: Brandon Potter
 */

#include "sim/aux_vector.hh"

#include <inttypes.h>

#include "config/the_isa.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/linux/process.hh"

#elif THE_ISA == SPARC_ISA
#include "arch/sparc/linux/process.hh"
#include "arch/sparc/solaris/process.hh"

#elif THE_ISA == MIPS_ISA
#include "arch/mips/linux/process.hh"

#elif THE_ISA == ARM_ISA
#include "arch/arm/freebsd/process.hh"
#include "arch/arm/linux/process.hh"

#elif THE_ISA == X86_ISA
#include "arch/x86/linux/process.hh"

#elif THE_ISA == RISCV_ISA
#include "arch/riscv/linux/process.hh"

#elif THE_ISA == POWER_ISA
#include "arch/power/linux/process.hh"

#else
#error "THE_ISA not set"
#endif

template<class IntType>
AuxVector<IntType>::AuxVector(IntType type, IntType val)
{
    a_type = TheISA::htog(type);
    a_val = TheISA::htog(val);
}

template struct AuxVector<uint32_t>;
template struct AuxVector<uint64_t>;
