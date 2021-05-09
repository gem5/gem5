/*
 * Copyright (c) 2020 ARM Limited
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

#ifndef __ARCH_GENERIC_MMU_HH__
#define __ARCH_GENERIC_MMU_HH__

#include "arch/generic/tlb.hh"

#include "params/BaseMMU.hh"

namespace gem5
{

class BaseMMU : public SimObject
{
  protected:
    typedef BaseMMUParams Params;

    BaseMMU(const Params &p)
      : SimObject(p), dtb(p.dtb), itb(p.itb)
    {}

    BaseTLB*
    getTlb(BaseTLB::Mode mode) const
    {
        if (mode == BaseTLB::Execute)
            return itb;
        else
            return dtb;
    }

  public:
    virtual void
    flushAll()
    {
        dtb->flushAll();
        itb->flushAll();
    }

    void
    demapPage(Addr vaddr, uint64_t asn)
    {
        itb->demapPage(vaddr, asn);
        dtb->demapPage(vaddr, asn);
    }

    Fault
    translateAtomic(const RequestPtr &req, ThreadContext *tc,
                    BaseTLB::Mode mode)
    {
        return getTlb(mode)->translateAtomic(req, tc, mode);
    }

    void
    translateTiming(const RequestPtr &req, ThreadContext *tc,
                    BaseTLB::Translation *translation, BaseTLB::Mode mode)
    {
        return getTlb(mode)->translateTiming(req, tc, translation, mode);
    }

    Fault
    translateFunctional(const RequestPtr &req, ThreadContext *tc,
                        BaseTLB::Mode mode)
    {
        return getTlb(mode)->translateFunctional(req, tc, mode);
    }

    Fault
    finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                     BaseTLB::Mode mode) const
    {
        return getTlb(mode)->finalizePhysical(req, tc, mode);
    }

    virtual void takeOverFrom(BaseMMU *old_mmu);

  public:
    BaseTLB* dtb;
    BaseTLB* itb;
};

} // namespace gem5

#endif
