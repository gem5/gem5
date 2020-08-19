/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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

#ifndef __ARCH_X86_UTILITY_HH__
#define __ARCH_X86_UTILITY_HH__

#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "sim/full_system.hh"

namespace X86ISA
{

    inline PCState
    buildRetPC(const PCState &curPC, const PCState &callPC)
    {
        PCState retPC = callPC;
        retPC.uEnd();
        return retPC;
    }

    uint64_t
    getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp);

    static inline bool
    inUserMode(ThreadContext *tc)
    {
        if (!FullSystem) {
            return true;
        } else {
            HandyM5Reg m5reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
            return m5reg.cpl == 3;
        }
    }

    void copyRegs(ThreadContext *src, ThreadContext *dest);

    void copyMiscRegs(ThreadContext *src, ThreadContext *dest);

    inline void
    advancePC(PCState &pc, const StaticInstPtr &inst)
    {
        inst->advancePC(pc);
    }

    inline uint64_t
    getExecutingAsid(ThreadContext *tc)
    {
        return 0;
    }


    /**
     * Reconstruct the rflags register from the internal gem5 register
     * state.
     *
     * gem5 stores rflags in several different registers to avoid
     * pipeline dependencies. In order to get the true rflags value,
     * we can't simply read the value of MISCREG_RFLAGS. Instead, we
     * need to read out various state from microcode registers and
     * merge that with MISCREG_RFLAGS.
     *
     * @param tc Thread context to read rflags from.
     * @return rflags as seen by the guest.
     */
    uint64_t getRFlags(ThreadContext *tc);

    /**
     * Set update the rflags register and internal gem5 state.
     *
     * @note This function does not update MISCREG_M5_REG. You might
     * need to update this register by writing anything to
     * MISCREG_M5_REG with side-effects.
     *
     * @see X86ISA::getRFlags()
     *
     * @param tc Thread context to update
     * @param val New rflags value to store in TC
     */
    void setRFlags(ThreadContext *tc, uint64_t val);

    /**
     * Convert an x87 tag word to abridged tag format.
     *
     * Convert from the x87 tag representation to the tag abridged
     * representation used in the FXSAVE area. The classic format uses
     * 2 bits per stack position to indicate if a position is valid,
     * zero, special, or empty. The abridged format only stores
     * whether a position is empty or not.
     *
     * @param ftw Tag word in classic x87 format.
     * @return Tag word in the abridged format.
     */
    uint8_t convX87TagsToXTags(uint16_t ftw);

    /**
     * Convert an x87 xtag word to normal tags format.
     *
     * Convert from the abridged x87 tag representation used in the
     * FXSAVE area to a full x87 tag. The classic format uses 2 bits
     * per stack position to indicate if a position is valid, zero,
     * special, or empty. The abridged format only stores whether a
     * position is empty or not.
     *
     * @todo Reconstruct the correct state of stack positions instead
     * of just valid/invalid.
     *
     * @param ftwx Tag word in the abridged format.
     * @return Tag word in classic x87 format.
     */
    uint16_t convX87XTagsToTags(uint8_t ftwx);

    /**
     * Generate and updated x87 tag register after a push/pop
     * operation.
     *
     * @note There is currently no support for setting other tags than
     * valid and invalid. A real x87 will set the tag value to zero or
     * special for some special floating point values.
     *
     * @param ftw Current value of the FTW register.
     * @param top Current x87 TOP value.
     * @param spm Stack displacement.
     * @return New value of the FTW register.
     */
    uint16_t genX87Tags(uint16_t ftw, uint8_t top, int8_t spm);

    /**
     * Load an 80-bit float from memory and convert it to double.
     *
     * @param mem Pointer to an 80-bit float.
     * @return double representation of the 80-bit float.
     */
    double loadFloat80(const void *mem);

    /**
     * Convert and store a double as an 80-bit float.
     *
     * @param mem Pointer to destination for the 80-bit float.
     * @param value Double precision float to store.
     */
    void storeFloat80(void *mem, double value);
}

#endif // __ARCH_X86_UTILITY_HH__
