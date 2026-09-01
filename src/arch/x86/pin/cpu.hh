/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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

#ifndef __ARCH_X86_PIN_CPU_HH__
#define __ARCH_X86_PIN_CPU_HH__

#include <memory>
#include <optional>
#include <set>

#include "cpu/base.hh"
#include "mem/port.hh"

namespace gem5
{

// Forward declarations.
class SimpleThread;
class X86PinCPUParams;
class System;

namespace X86ISA
{

class PinCPU final : public BaseCPU
{
  public:
    PinCPU(const X86PinCPUParams &params);

    void init() override;
    void startup() override;

    void serializeThread(CheckpointOut &cp, ThreadID tid) const override;

    void activateContext(ThreadID tid = 0) override;

    class PinRequestPort final : public RequestPort
    {
      public:
        PinRequestPort(const std::string &name, PinCPU *cpu)
            : RequestPort(name), cpu(cpu)
        {}

      private:
        PinCPU *cpu;

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    };

    Port &getDataPort() override;
    Port &getInstPort() override;
    void wakeup(ThreadID tid) override;
    Counter totalInsts() const override;
    Counter totalOps() const override;

    void tick();

    enum Status
    {
        Idle,
        Running,
    };

  private:
    std::unique_ptr<SimpleThread> thread;
    ThreadContext *tc;
    EventFunctionWrapper tickEvent;
    Status _status;
    PinRequestPort dataPort;
    PinRequestPort instPort;

    // Pin paths.
    std::string pinExe;
    std::string pinGuest;
    std::string pinTool;
    std::vector<std::string> pinArgs;
    std::vector<std::string> pinToolArgs;

    pid_t pinPid = -1;
    int reqFd = -1;
    int respFd = -1;
    System *system;
    std::optional<Counter> ctrInsts;

    const std::string &getPinTool() const;
    const std::string &getPinExe() const;
    const std::string &getGuest() const;

    void pinRun();

    void syncStateToPin(bool full);
    void syncStateFromPin(bool full);

    void handlePageFault(Addr vaddr);
    void handleSyscall();

    void unmapFromPin(Addr vaddr, Addr size);

    void handleCPUID();

    void haltContext();

    bool isPinRunning() const;

    void mapCode();

  public:
    std::string executePinCommand(const std::string &command);
};

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_PIN_CPU_HH__
