/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __CPU_O3_CPU_ALPHA_IMPL_HH__
#define __CPU_O3_CPU_ALPHA_IMPL_HH__

#include "arch/alpha/isa_traits.hh"

#include "cpu/o3/alpha_params.hh"
#include "cpu/o3/cpu_policy.hh"

// Forward declarations.
template <class Impl>
class AlphaDynInst;

template <class Impl>
class AlphaFullCPU;

/** Implementation specific struct that defines several key things to the
 *  CPU, the stages within the CPU, the time buffers, and the DynInst.
 *  The struct defines the ISA, the CPU policy, the specific DynInst, the
 *  specific FullCPU, and all of the structs from the time buffers to do
 *  communication.
 *  This is one of the key things that must be defined for each hardware
 *  specific CPU implementation.
 */
struct AlphaSimpleImpl
{
    /** The ISA to be used. */
    typedef AlphaISA ISA;

    /** The type of MachInst. */
    typedef ISA::MachInst MachInst;

    /** The CPU policy to be used (ie fetch, decode, etc.). */
    typedef SimpleCPUPolicy<AlphaSimpleImpl> CPUPol;

    /** The DynInst to be used. */
    typedef AlphaDynInst<AlphaSimpleImpl> DynInst;

    /** The refcounted DynInst pointer to be used.  In most cases this is
     *  what should be used, and not DynInst *.
     */
    typedef RefCountingPtr<DynInst> DynInstPtr;

    /** The FullCPU to be used. */
    typedef AlphaFullCPU<AlphaSimpleImpl> FullCPU;

    /** The Params to be passed to each stage. */
    typedef AlphaSimpleParams Params;

    enum {
        MaxWidth = 8
    };
};

#endif // __CPU_O3_CPU_ALPHA_IMPL_HH__
