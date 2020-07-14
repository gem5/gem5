/*
 * Copyright (c) 2010-2013, 2016, 2018 ARM Limited
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
 */

#include "arch/arm/stage2_lookup.hh"

#include "arch/arm/faults.hh"
#include "arch/arm/system.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/tlb.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Checkpoint.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "sim/system.hh"

using namespace ArmISA;

Fault
Stage2LookUp::getTe(ThreadContext *tc, TlbEntry *destTe)

{
    fault = stage2Tlb->getTE(&stage2Te, req, tc, mode, this, timing,
                                   functional, secure, tranType);
    // Call finish if we're done already
    if ((fault != NoFault) || (stage2Te != NULL)) {
        // Since we directly requested the table entry (which we need later on
        // to merge the attributes) then we've skipped some stage2 permissions
        // checking. So call translate on stage 2 to do the checking. As the
        // entry is now in the TLB this should always hit the cache.
        if (fault == NoFault) {
            if (ELIs64(tc, EL2))
                fault = stage2Tlb->checkPermissions64(stage2Te, req, mode, tc);
            else
                fault = stage2Tlb->checkPermissions(stage2Te, req, mode);
        }

        mergeTe(req, mode);
        *destTe = stage1Te;
    }
    return fault;
}

void
Stage2LookUp::mergeTe(const RequestPtr &req, BaseTLB::Mode mode)
{
    // Check again that we haven't got a fault
    if (fault == NoFault) {
        assert(stage2Te != NULL);

        // Now we have the table entries for both stages of translation
        // merge them and insert the result into the stage 1 TLB. See
        // CombineS1S2Desc() in pseudocode
        stage1Te.nonCacheable |= stage2Te->nonCacheable;
        stage1Te.xn           |= stage2Te->xn;

        if (stage1Te.size > stage2Te->size) {
            // Size mismatch also implies vpn mismatch (this is shifted by
            // sizebits!).
            stage1Te.vpn  = s1Req->getVaddr() >> stage2Te->N;
            stage1Te.pfn  = stage2Te->pfn;
            stage1Te.size = stage2Te->size;
            stage1Te.N    = stage2Te->N;
        } else if (stage1Te.size < stage2Te->size) {
            // Guest 4K could well be section-backed by host hugepage!  In this
            // case a 4K entry is added but pfn needs to be adjusted.  New PFN =
            // offset into section PFN given by stage2 IPA treated as a stage1
            // page size.
            const Addr pa = (stage2Te->pfn << stage2Te->N);
            const Addr ipa = (stage1Te.pfn << stage1Te.N);
            stage1Te.pfn = (pa | (ipa & mask(stage2Te->N))) >> stage1Te.N;
            // Size remains smaller of the two.
        } else {
            // Matching sizes
            stage1Te.pfn = stage2Te->pfn;
        }

        if (stage2Te->mtype == TlbEntry::MemoryType::StronglyOrdered ||
            stage1Te.mtype  == TlbEntry::MemoryType::StronglyOrdered) {
            stage1Te.mtype  =  TlbEntry::MemoryType::StronglyOrdered;
        } else if (stage2Te->mtype == TlbEntry::MemoryType::Device ||
                   stage1Te.mtype  == TlbEntry::MemoryType::Device) {
            stage1Te.mtype = TlbEntry::MemoryType::Device;
        } else {
            stage1Te.mtype = TlbEntry::MemoryType::Normal;
        }

        if (stage1Te.mtype == TlbEntry::MemoryType::Normal) {

            if (stage2Te->innerAttrs == 0 ||
                stage1Te.innerAttrs  == 0) {
                // either encoding Non-cacheable
                stage1Te.innerAttrs = 0;
            } else if (stage2Te->innerAttrs == 2 ||
                       stage1Te.innerAttrs  == 2) {
                // either encoding Write-Through cacheable
                stage1Te.innerAttrs = 2;
            } else {
                // both encodings Write-Back
                stage1Te.innerAttrs = 3;
            }

            if (stage2Te->outerAttrs == 0 ||
                stage1Te.outerAttrs  == 0) {
                // either encoding Non-cacheable
                stage1Te.outerAttrs = 0;
            } else if (stage2Te->outerAttrs == 2 ||
                       stage1Te.outerAttrs  == 2) {
                // either encoding Write-Through cacheable
                stage1Te.outerAttrs = 2;
            } else {
                // both encodings Write-Back
                stage1Te.outerAttrs = 3;
            }

            stage1Te.shareable       |= stage2Te->shareable;
            stage1Te.outerShareable |= stage2Te->outerShareable;
            if (stage1Te.innerAttrs == 0 &&
                stage1Te.outerAttrs == 0) {
                // something Non-cacheable at each level is outer shareable
                stage1Te.shareable       = true;
                stage1Te.outerShareable = true;
            }
        } else {
            stage1Te.shareable       = true;
            stage1Te.outerShareable = true;
        }
        stage1Te.updateAttributes();
    }

    // if there's a fault annotate it,
    if (fault != NoFault) {
        // If the second stage of translation generated a fault add the
        // details of the original stage 1 virtual address
        reinterpret_cast<ArmFault *>(fault.get())->annotate(ArmFault::OVA,
            s1Req->getVaddr());
    }
    complete = true;
}

void
Stage2LookUp::finish(const Fault &_fault, const RequestPtr &req,
    ThreadContext *tc, BaseTLB::Mode mode)
{
    fault = _fault;
    // if we haven't got the table entry get it now
    if ((fault == NoFault) && (stage2Te == NULL)) {
        fault = stage2Tlb->getTE(&stage2Te, req, tc, mode, this,
            timing, functional, secure, tranType);
    }

    // Now we have the stage 2 table entry we need to merge it with the stage
    // 1 entry we were given at the start
    mergeTe(req, mode);

    if (fault != NoFault) {
        // Returning with a fault requires the original request
        transState->finish(fault, s1Req, tc, mode);
    } else if (timing) {
        // Now notify the original stage 1 translation that we finally have
        // a result
        stage1Tlb->translateComplete(s1Req, tc, transState, mode, tranType, true);
    }
    // if we have been asked to delete ourselfs do it now
    if (selfDelete) {
        delete this;
    }
}

