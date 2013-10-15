/*
 * Copyright (c) 2013 Andreas Sandberg
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
 * Authors: Andreas Sandberg
 */

#ifndef __ARCH_GENERIC_MMAPPED_IPR_HH__
#define __ARCH_GENERIC_MMAPPED_IPR_HH__

#include "base/types.hh"
#include "mem/packet.hh"

class ThreadContext;

/**
 * @file
 *
 * ISA-generic helper functions for memory mapped IPR accesses.
 */

namespace GenericISA
{
    /** @{ */
    /**
     * Memory requests with the MMAPPED_IPR flag are generally mapped
     * to registers. There is a class of these registers that are
     * internal to gem5, for example gem5 pseudo-ops in virtualized
     * mode. Such IPRs always have the flag GENERIC_IPR set and are
     * handled by this code.
     */

    /** Shift amount when extracting the class of a generic IPR */
    const int IPR_CLASS_SHIFT = 48;

    /** Mask to extract the offset in within a generic IPR class */
    const Addr IPR_IN_CLASS_MASK = ULL(0x0000FFFFFFFFFFFF);

    /** gem5 pseudo-inst emulation.
     *
     * Read and writes to this class execute gem5
     * pseudo-instructions. A write discards the return value of the
     * instruction, while a read returns it.
     *
     * @see pseudoInst()
     */
    const Addr IPR_CLASS_PSEUDO_INST = 0x0;

    /** @} */

    /**
     * Generate a generic IPR address that emulates a pseudo inst
     *
     * @see PseudoInst::pseudoInst()
     *
     * @param func Function ID to call.
     * @param subfunc Sub-function, usually 0.
     * @return Address in the IPR space corresponding to the call.
     */
    inline Addr
    iprAddressPseudoInst(uint8_t func, uint8_t subfunc)
    {
        return (IPR_CLASS_PSEUDO_INST << IPR_CLASS_SHIFT)  |
            (func << 8) | subfunc;
    }

    /**
     * Check if this is an platform independent IPR access
     *
     * Accesses to internal platform independent gem5 registers are
     * handled by handleGenericIprRead() and
     * handleGenericIprWrite(). This method determines if a packet
     * should be routed to those functions instead of the platform
     * specific code.
     *
     * @see handleGenericIprRead
     * @see handleGenericIprWrite
     */
    inline bool
    isGenericIprAccess(const Packet *pkt)
    {
        Request::Flags flags(pkt->req->getFlags());
        return (flags & Request::MMAPPED_IPR) &&
            (flags & Request::GENERIC_IPR);
    }

    /**
     * Handle generic IPR reads
     *
     * @param xc Thread context of the current thread.
     * @param pkt Packet from the CPU
     * @return Latency in CPU cycles
     */
    Cycles handleGenericIprRead(ThreadContext *xc, Packet *pkt);
    /**
     * Handle generic IPR writes
     *
     * @param xc Thread context of the current thread.
     * @param pkt Packet from the CPU
     * @return Latency in CPU cycles
     */
    Cycles handleGenericIprWrite(ThreadContext *xc, Packet *pkt);

    /**
     * Helper function to handle IPRs when the target architecture doesn't
     * need its own IPR handling.
     *
     * This function calls handleGenericIprRead if the accessing a
     * generic IPR and panics otherwise.
     *
     * @param xc Thread context of the current thread.
     * @param pkt Packet from the CPU
     * @return Latency in CPU cycles
     */
    inline Cycles
    handleIprRead(ThreadContext *xc, Packet *pkt)
    {
        if (!isGenericIprAccess(pkt))
            panic("Unhandled IPR access\n");

        return handleGenericIprRead(xc, pkt);
    }


    /**
     * Helper function to handle IPRs when the target architecture
     * doesn't need its own IPR handling.
     *
     * This function calls handleGenericIprWrite if the accessing a
     * generic IPR and panics otherwise.
     *
     * @param xc Thread context of the current thread.
     * @param pkt Packet from the CPU
     * @return Latency in CPU cycles
     */
    inline Cycles
    handleIprWrite(ThreadContext *xc, Packet *pkt)
    {
        if (!isGenericIprAccess(pkt))
            panic("Unhandled IPR access\n");

        return handleGenericIprWrite(xc, pkt);
    }

} // namespace GenericISA



#endif
