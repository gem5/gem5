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

#ifndef __CPU_APPLE_VIRT_VM_HH__
#define __CPU_APPLE_VIRT_VM_HH__

#include <Hypervisor/hv.h>
#include <Hypervisor/hv_vm.h>

#include "base/addr_range.hh"
#include "params/AppleVirtVM.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class System;

/**
 * Thin wrapper around the Apple Hypervisor (HVF) virtual machine APIs. It is
 * responsible for creating the VM instance that all AppleVirt CPUs share and
 * for mapping the gem5 backing store into the guest physical address space.
 */
class AppleVirtVM : public SimObject
{
  public:
    AppleVirtVM(const AppleVirtVMParams &params);
    ~AppleVirtVM() override;

    /** Map the system backing store into the guest address space. */
    void mapSystemMemory(System &system);

    /** Ensure the VM is initialised before any CPU touches it. */
    void ensureInitialized(System &system);

    /** Register the single vCPU supported by the initial backend. */
    void registerCPU();

    /** Record that the vCPU has been destroyed. */
    void unregisterCPU();

  private:
    bool initialized;
    unsigned activeCPUs;
    long hostPageSize;
};

} // namespace gem5

#endif
