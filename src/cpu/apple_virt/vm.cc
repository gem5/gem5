/*
 * Copyright (c) 2026 The Regents of The University of California
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

#include "cpu/apple_virt/vm.hh"

#include <Hypervisor/hv_types.h>
#include <Hypervisor/hv_vm.h>

#include "base/logging.hh"
#include "mem/physical.hh"
#include "params/AppleVirtVM.hh"
#include "sim/system.hh"

namespace gem5
{

AppleVirtVM::AppleVirtVM(const AppleVirtVMParams &params)
    : SimObject(params), initialized(false)
{}

AppleVirtVM::~AppleVirtVM()
{
    if (initialized) {
        hv_return_t hv_err = hv_vm_destroy();
        if (hv_err != HV_SUCCESS) {
            warn("hv_vm_destroy failed (%d)", hv_err);
        }
    }
}

void
AppleVirtVM::ensureInitialized(System &system)
{
    if (initialized) {
        return;
    }

    hv_return_t hv_err = hv_vm_create(nullptr);
    fatal_if(hv_err != HV_SUCCESS,
             "hv_vm_create failed while bringing up AppleVirt VM (err=%d)",
             hv_err);

    mapSystemMemory(system);
    initialized = true;
}

void
AppleVirtVM::mapSystemMemory(System &system)
{
    auto &phys_mem = system.getPhysMem();
    const auto backing = phys_mem.getBackingStore();
    for (const auto &entry : backing) {
        if (!entry.kvmMap || !entry.pmem) {
            continue;
        }

        const AddrRange &range = entry.range;
        const uint64_t size = range.size();
        hv_return_t hv_err =
            hv_vm_map(static_cast<void *>(entry.pmem), range.start(), size,
                      HV_MEMORY_READ | HV_MEMORY_WRITE | HV_MEMORY_EXEC);
        fatal_if(hv_err != HV_SUCCESS,
                 "hv_vm_map failed for range [%#llx, %#llx) (err=%d)",
                 static_cast<unsigned long long>(range.start()),
                 static_cast<unsigned long long>(range.end()), hv_err);
    }
}

} // namespace gem5
