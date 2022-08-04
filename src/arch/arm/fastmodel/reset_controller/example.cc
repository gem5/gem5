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

#include "arch/arm/fastmodel/reset_controller/example.hh"

#include <algorithm>

#include "base/logging.hh"

namespace gem5
{
namespace fastmodel
{

ResetControllerExample::CorePins::CorePins(const std::string &module_name)
    : reset(module_name + ".reset", 0, this),
      halt(module_name + ".halt", 0, this)
{}

ResetControllerExample::Registers::Registers(
    const std::string &module_name, Iris::BaseCPU *c, CorePins *p)
    : RegisterBankLE(module_name, 0), cpu(c), pins(p),
      nsrvbar(module_name + ".nsrvbar"),
      rvbar(module_name + ".rvbar"),
      reset(module_name + ".reset"),
      halt(module_name + ".halt")
{
      panic_if(cpu == nullptr, "ResetControllerExample needs a target cpu.");
      nsrvbar.writer(
          [this] (auto &reg, auto val)
          {
              cpu->setResetAddr(val, false);
          });
      rvbar.writer(
          [this] (auto &reg, auto val)
          {
              cpu->setResetAddr(val, true);
          });
      reset.writer(
          [this] (auto &reg, auto val)
          {
              panic_if(!pins->reset.isConnected(),
                       "%s is not connected.", pins->reset.name());

              if (val)
                  pins->reset.raise();
              else
                  pins->reset.lower();
          });
      halt.writer(
          [this] (auto &reg, auto val)
          {
              panic_if(!pins->halt.isConnected(),
                       "%s is not connected.", pins->halt.name());

              if (val)
                  pins->halt.raise();
              else
                  pins->halt.lower();
          });

      addRegisters({
          nsrvbar,
          rvbar,
          reset,
          halt,
      });
}

ResetControllerExample::ResetControllerExample(const Params &p)
    : BasicPioDevice(p, 0x20),
      pins(p.name + ".pins"),
      registers(p.name  + ".registers", p.cpu, &pins)
{}

Tick
ResetControllerExample::read(PacketPtr pkt)
{
    pkt->makeResponse();
    auto data = pkt->getPtr<uint8_t>();
    auto size = pkt->getSize();
    std::fill(data, data + size, 0);
    return pioDelay;
}

Tick
ResetControllerExample::write(PacketPtr pkt)
{
    pkt->makeResponse();
    size_t size = pkt->getSize();
    if (size != 4 && size != 8) {
        pkt->setBadAddress();
    } else {
        auto addr = pkt->getAddr() - pioAddr;
        registers.write(addr, pkt->getPtr<void>(), size);
    }
    return pioDelay;
}

Port &
ResetControllerExample::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "reset")
        return pins.reset;
    else if (if_name == "halt")
        return pins.halt;

    return BasicPioDevice::getPort(if_name, idx);
}

} // namespace fastmodel
} // namespace gem5
