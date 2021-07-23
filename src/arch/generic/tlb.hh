/*
 * Copyright (c) 2011, 2021 Arm Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __ARCH_GENERIC_TLB_HH__
#define __ARCH_GENERIC_TLB_HH__

#include <type_traits>

#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "enums/TypeTLB.hh"
#include "mem/request.hh"
#include "params/BaseTLB.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ThreadContext;

class BaseTLB : public SimObject
{
  protected:
    BaseTLB(const BaseTLBParams &p)
      : SimObject(p), _type(p.entry_type), _nextLevel(p.next_level)
    {}

    TypeTLB _type;

    BaseTLB *_nextLevel;

  public:
    virtual void demapPage(Addr vaddr, uint64_t asn) = 0;

    virtual Fault translateAtomic(
            const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode) = 0;
    virtual void translateTiming(
            const RequestPtr &req, ThreadContext *tc,
            BaseMMU::Translation *translation, BaseMMU::Mode mode) = 0;
    virtual Fault
    translateFunctional(const RequestPtr &req, ThreadContext *tc,
                        BaseMMU::Mode mode)
    {
        panic("Not implemented.\n");
    }

    /**
     * Do post-translation physical address finalization.
     *
     * This method is used by some architectures that need
     * post-translation massaging of physical addresses. For example,
     * X86 uses this to remap physical addresses in the APIC range to
     * a range of physical memory not normally available to real x86
     * implementations.
     *
     * @param req Request to updated in-place.
     * @param tc Thread context that created the request.
     * @param mode Request type (read/write/execute).
     * @return A fault on failure, NoFault otherwise.
     */
    virtual Fault finalizePhysical(
            const RequestPtr &req, ThreadContext *tc,
            BaseMMU::Mode mode) const = 0;

    /**
     * Remove all entries from the TLB
     */
    virtual void flushAll() = 0;

    /**
     * Take over from an old tlb context
     */
    virtual void takeOverFrom(BaseTLB *otlb) = 0;

    /**
     * Get the table walker port if present. This is used for
     * migrating port connections during a CPU takeOverFrom()
     * call. For architectures that do not have a table walker, NULL
     * is returned, hence the use of a pointer rather than a
     * reference.
     *
     * @return A pointer to the walker port or NULL if not present
     */
    virtual Port* getTableWalkerPort() { return NULL; }

    void memInvalidate() { flushAll(); }

    TypeTLB type() const { return _type; }

    BaseTLB* nextLevel() const { return _nextLevel; }
};

/** Implementing the "&" bitwise operator for TypeTLB allows us to handle
 * TypeTLB::unified efficiently. For example if I want to check if a TLB
 * is storing instruction entries I can do this with:
 *
 * tlb->type() & TypeTLB::instruction
 *
 * which will cover both TypeTLB::instruction and TypeTLB::unified TLBs
 */
inline auto
operator&(TypeTLB lhs, TypeTLB rhs)
{
    using T = std::underlying_type_t<TypeTLB>;
    return static_cast<T>(lhs) & static_cast<T>(rhs);
}

} // namespace gem5

#endif // __ARCH_GENERIC_TLB_HH__
