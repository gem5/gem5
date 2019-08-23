/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_IRIS_ARM_CPU_HH__
#define __ARCH_ARM_FASTMODEL_IRIS_ARM_CPU_HH__

#include "arch/arm/fastmodel/iris/arm/thread_context.hh"
#include "arch/arm/fastmodel/iris/cpu.hh"

namespace Iris
{

// This class specializes the generic Iris CPU template to use the Arm
// Iris ThreadContext.
class ArmCPU : public CPU<ArmThreadContext>
{
  public:
    using CPU<ArmThreadContext>::CPU;

    void
    clockPeriodUpdated() override
    {
        CPU<ArmThreadContext>::clockPeriodUpdated();

        // FIXME(b/139447397): this is a workaround since CNTFRQ_EL0 should not
        // be modified after clock is changed in real hardwares. Remove or
        // modify this after a more reasonable solution is found.
        for (auto *tc : threadContexts) {
            tc->setMiscRegNoEffect(ArmISA::MISCREG_CNTFRQ_EL0, frequency());
        }
    }
};

} // namespace Iris

#endif // __ARCH_ARM_FASTMODEL_IRIS_ARM_CPU_HH__
