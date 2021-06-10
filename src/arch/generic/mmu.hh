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

#include "params/BaseMMU.hh"
#include "mem/request.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class BaseTLB;

class BaseMMU : public SimObject
{
  public:
    enum Mode { Read, Write, Execute };

    class Translation
    {
      public:
        virtual ~Translation()
        {}

        /**
         * Signal that the translation has been delayed due to a hw page table
         * walk.
         */
        virtual void markDelayed() = 0;

        /*
         * The memory for this object may be dynamically allocated, and it may
         * be responsible for cleaning itself up which will happen in this
         * function. Once it's called, the object is no longer valid.
         */
        virtual void finish(const Fault &fault, const RequestPtr &req,
                            ThreadContext *tc, BaseMMU::Mode mode) = 0;

        /** This function is used by the page table walker to determine
         * if it should translate the a pending request or if the underlying
         * request has been squashed.
         * @ return Is the instruction that requested this translation
         * squashed?
         */
        virtual bool squashed() const { return false; }
    };

  protected:
    typedef BaseMMUParams Params;

    BaseMMU(const Params &p)
      : SimObject(p), dtb(p.dtb), itb(p.itb)
    {}

    BaseTLB*
    getTlb(Mode mode) const
    {
        if (mode == Execute)
            return itb;
        else
            return dtb;
    }

  public:
    virtual void flushAll();

    void demapPage(Addr vaddr, uint64_t asn);

    Fault translateAtomic(const RequestPtr &req, ThreadContext *tc,
                          Mode mode);

    void translateTiming(const RequestPtr &req, ThreadContext *tc,
                         Translation *translation, Mode mode);

    Fault translateFunctional(const RequestPtr &req, ThreadContext *tc,
                              Mode mode);

    Fault finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                           Mode mode) const;

    virtual void takeOverFrom(BaseMMU *old_mmu);

  public:
    BaseTLB* dtb;
    BaseTLB* itb;
};

} // namespace gem5

#endif
