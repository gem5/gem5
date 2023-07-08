/*
 * Copyright 2021 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_RESET_CONTROLLER_EXAMPLE_HH__
#define __ARCH_ARM_FASTMODEL_RESET_CONTROLLER_EXAMPLE_HH__

#include <string>

#include "arch/arm/fastmodel/iris/cpu.hh"
#include "dev/io_device.hh"
#include "dev/reg_bank.hh"
#include "mem/packet_access.hh"
#include "params/FastModelResetControllerExample.hh"
#include "sim/signal.hh"

namespace gem5
{

namespace fastmodel
{

class ResetControllerExample : public BasicPioDevice
{
  private:
    struct CorePins
    {
        SignalSourcePort<bool> reset;
        SignalSourcePort<bool> halt;

        explicit CorePins(const std::string &);
    };

    class Registers : public RegisterBankLE
    {
      private:
        Iris::BaseCPU *cpu;
        CorePins *pins;

        Register64 nsrvbar;
        Register64 rvbar;
        Register32 reset;
        Register32 halt;

      public:
        Registers(const std::string &, Iris::BaseCPU *, CorePins *);
    };

    CorePins pins;
    Registers registers;

  public:
    using Params = FastModelResetControllerExampleParams;
    explicit ResetControllerExample(const Params &);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    Port &getPort(const std::string &, PortID = InvalidPortID) override;
};

} // namespace fastmodel
} // namespace gem5

#endif  // __ARCH_ARM_FASTMODEL_RESET_CONTROLLER_EXAMPLE_HH__
