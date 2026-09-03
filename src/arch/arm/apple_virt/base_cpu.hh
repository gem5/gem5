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

/*
 * Apple virtualization backend for Arm CPUs.
 */

#ifndef __ARCH_ARM_APPLE_VIRT_BASE_CPU_HH__
#define __ARCH_ARM_APPLE_VIRT_BASE_CPU_HH__

#include "cpu/apple_virt/base.hh"

namespace gem5
{

struct BaseArmAppleVirtCPUParams;

class BaseArmAppleVirtCPU : public BaseAppleVirtCPU
{
  public:
    BaseArmAppleVirtCPU(const BaseArmAppleVirtCPUParams &params);
    ~BaseArmAppleVirtCPU() override = default;

    void startup() override;

  protected:
    void advancePC() override;
    void syncThreadToHV() override;
    void syncHVToThread() override;
    void updateInterrupts() override;
    Tick handleException(const hv_vcpu_exit_exception_t &exception) override;
    void handleVTimerActivated() override;
};

} // namespace gem5

#endif
