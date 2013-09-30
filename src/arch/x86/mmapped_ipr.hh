/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_MMAPPEDIPR_HH__
#define __ARCH_X86_MMAPPEDIPR_HH__

/**
 * @file
 *
 * ISA-specific helper functions for memory mapped IPR accesses.
 */

#include "arch/generic/mmapped_ipr.hh"
#include "arch/x86/regs/misc.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/packet.hh"

namespace X86ISA
{
    inline Cycles
    handleIprRead(ThreadContext *xc, Packet *pkt)
    {
        if (GenericISA::isGenericIprAccess(pkt)) {
            return GenericISA::handleGenericIprRead(xc, pkt);
        } else {
            Addr offset = pkt->getAddr() & mask(3);
            MiscRegIndex index = (MiscRegIndex)(
                pkt->getAddr() / sizeof(MiscReg));
            MiscReg data = htog(xc->readMiscReg(index));
            // Make sure we don't trot off the end of data.
            assert(offset + pkt->getSize() <= sizeof(MiscReg));
            pkt->setData(((uint8_t *)&data) + offset);
            return Cycles(1);
        }
    }

    inline Cycles
    handleIprWrite(ThreadContext *xc, Packet *pkt)
    {
        if (GenericISA::isGenericIprAccess(pkt)) {
            return GenericISA::handleGenericIprWrite(xc, pkt);
        } else {
            Addr offset = pkt->getAddr() & mask(3);
            MiscRegIndex index = (MiscRegIndex)(
                pkt->getAddr() / sizeof(MiscReg));
            MiscReg data;
            data = htog(xc->readMiscRegNoEffect(index));
            // Make sure we don't trot off the end of data.
            assert(offset + pkt->getSize() <= sizeof(MiscReg));
            pkt->writeData(((uint8_t *)&data) + offset);
            xc->setMiscReg(index, gtoh(data));
            return Cycles(1);
        }
    }
}

#endif // __ARCH_X86_MMAPPEDIPR_HH__
