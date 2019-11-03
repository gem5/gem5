/*
 * Copyright (c) 2010-2013 ARM Limited
 * All rights reserved
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
 * Authors: Ali Saidi
 *          Giacomo Gabrielli
 */

#ifndef __ARCH_ARM_STAGE2_LOOKUP_HH__
#define __ARCH_ARM_STAGE2_LOOKUP_HH__

#include <list>

#include "arch/arm/system.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/tlb.hh"
#include "mem/request.hh"

class ThreadContext;

namespace ArmISA {
class Translation;
class TLB;


class Stage2LookUp : public BaseTLB::Translation
{
  private:
    TLB                     *stage1Tlb;
    TLB               *stage2Tlb;
    TlbEntry                stage1Te;
    RequestPtr              s1Req;
    TLB::Translation        *transState;
    BaseTLB::Mode           mode;
    bool                    timing;
    bool                    functional;
    TLB::ArmTranslationType tranType;
    TlbEntry                *stage2Te;
    RequestPtr              req;
    Fault                   fault;
    bool                    complete;
    bool                    selfDelete;

  public:
    Stage2LookUp(TLB *s1Tlb, TLB *s2Tlb, TlbEntry s1Te, const RequestPtr &_req,
        TLB::Translation *_transState, BaseTLB::Mode _mode, bool _timing,
        bool _functional, TLB::ArmTranslationType _tranType) :
        stage1Tlb(s1Tlb), stage2Tlb(s2Tlb), stage1Te(s1Te), s1Req(_req),
        transState(_transState), mode(_mode), timing(_timing),
        functional(_functional), tranType(_tranType), stage2Te(nullptr),
        fault(NoFault), complete(false), selfDelete(false)
    {
        req = std::make_shared<Request>();
        req->setVirt(0, s1Te.pAddr(s1Req->getVaddr()), s1Req->getSize(),
                     s1Req->getFlags(), s1Req->masterId(), 0);
    }

    Fault getTe(ThreadContext *tc, TlbEntry *destTe);

    void mergeTe(const RequestPtr &req, BaseTLB::Mode mode);

    void setSelfDelete() { selfDelete = true; }

    bool isComplete() const { return complete; }

    void markDelayed() {}

    void finish(const Fault &fault, const RequestPtr &req, ThreadContext *tc,
                BaseTLB::Mode mode);
};


} // namespace ArmISA

#endif //__ARCH_ARM_STAGE2_LOOKUP_HH__

