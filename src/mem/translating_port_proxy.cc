/*
 * Copyright (c) 2011,2013 ARM Limited
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

/**
 * @file
 * Port object definitions.
 */

#include "mem/translating_port_proxy.hh"

#include "arch/generic/mmu.hh"
#include "base/chunk_generator.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"

namespace gem5
{

TranslatingPortProxy::TranslatingPortProxy(ThreadContext *tc,
                                           Request::Flags _flags)
    : PortProxy(tc, tc->getSystemPtr()->cacheLineSize()),
      _tc(tc),
      flags(_flags)
{}

bool
TranslatingPortProxy::tryOnBlob(
    BaseMMU::Mode mode, TranslationGenPtr gen,
    std::function<void(const TranslationGen::Range &)> func) const
{
    // Wether we're trying to get past a fault.
    bool faulting = false;
    for (const auto &range : *gen) {
        // Was there a fault this time?
        if (range.fault) {
            // If there was a fault last time too, or the fixup this time
            // fails, then the operation has failed.
            if (faulting || !fixupRange(range, mode))
                return false;
            // This must be the first time we've tried this translation, so
            // record that we're making a second attempt and continue.
            faulting = true;
            continue;
        }

        // Run func() on this successful translation.
        faulting = false;
        func(range);
    }
    return true;
}

bool
TranslatingPortProxy::tryReadBlob(Addr addr, void *p, uint64_t size) const
{
    constexpr auto mode = BaseMMU::Read;
    return tryOnBlob(
        mode,
        _tc->getMMUPtr()->translateFunctional(addr, size, _tc, mode, flags),
        [this, &p](const auto &range) {
            PortProxy::readBlobPhys(range.paddr, flags, p, range.size);
            p = static_cast<uint8_t *>(p) + range.size;
        });
}

bool
TranslatingPortProxy::tryWriteBlob(Addr addr, const void *p,
                                   uint64_t size) const
{
    constexpr auto mode = BaseMMU::Write;
    return tryOnBlob(
        mode,
        _tc->getMMUPtr()->translateFunctional(addr, size, _tc, mode, flags),
        [this, &p](const auto &range) {
            PortProxy::writeBlobPhys(range.paddr, flags, p, range.size);
            p = static_cast<const uint8_t *>(p) + range.size;
        });
}

bool
TranslatingPortProxy::tryMemsetBlob(Addr addr, uint8_t v, uint64_t size) const
{
    constexpr auto mode = BaseMMU::Write;
    return tryOnBlob(
        mode,
        _tc->getMMUPtr()->translateFunctional(addr, size, _tc, mode, flags),
        [this, v](const auto &range) {
            PortProxy::memsetBlobPhys(range.paddr, flags, v, range.size);
        });
}

} // namespace gem5
