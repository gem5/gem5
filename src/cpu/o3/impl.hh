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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_IMPL_HH__
#define __CPU_O3_IMPL_HH__

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/o3/cpu_policy.hh"

// Forward declarations.
template <class Impl>
class BaseO3DynInst;

template <class Impl>
class FullO3CPU;

/** Implementation specific struct that defines several key types to the
 *  CPU, the stages within the CPU, the time buffers, and the DynInst.
 *  The struct defines the ISA, the CPU policy, the specific DynInst, the
 *  specific O3CPU, and all of the structs from the time buffers to do
 *  communication.
 *  This is one of the key things that must be defined for each hardware
 *  specific CPU implementation.
 */
struct O3CPUImpl
{
    /** The type of MachInst. */
    typedef TheISA::MachInst MachInst;

    /** The CPU policy to be used, which defines all of the CPU stages. */
    typedef SimpleCPUPolicy<O3CPUImpl> CPUPol;

    /** The DynInst type to be used. */
    typedef BaseO3DynInst<O3CPUImpl> DynInst;

    /** The refcounted DynInst pointer to be used.  In most cases this is
     *  what should be used, and not DynInst *.
     */
    typedef RefCountingPtr<DynInst> DynInstPtr;
    typedef RefCountingPtr<const DynInst> DynInstConstPtr;

    /** The O3CPU type to be used. */
    typedef FullO3CPU<O3CPUImpl> O3CPU;

    /** Same typedef, but for CPUType.  BaseDynInst may not always use
     * an O3 CPU, so it's clearer to call it CPUType instead in that
     * case.
     */
    typedef O3CPU CPUType;

    enum {
      MaxWidth = 8,
      MaxThreads = 4
    };
};

#endif // __CPU_O3_SPARC_IMPL_HH__
