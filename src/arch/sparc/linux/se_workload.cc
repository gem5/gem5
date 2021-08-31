/*
 * Copyright 2003-2005 The Regents of The University of Michigan
 * Copyright 2020 Google Inc.
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

#include "arch/sparc/linux/se_workload.hh"

#include <sys/syscall.h>

#include "arch/sparc/page_size.hh"
#include "arch/sparc/process.hh"
#include "arch/sparc/regs/int.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "sim/syscall_desc.hh"

namespace gem5
{

namespace
{

class LinuxLoader : public Process::Loader
{
  public:
    Process *
    load(const ProcessParams &params, loader::ObjectFile *obj) override
    {
        auto arch = obj->getArch();
        auto opsys = obj->getOpSys();

        if (arch != loader::SPARC64 && arch != loader::SPARC32)
            return nullptr;

        if (opsys == loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = loader::Linux;
        }

        if (opsys != loader::Linux)
            return nullptr;

        if (arch == loader::SPARC64)
            return new Sparc64Process(params, obj);
        else
            return new Sparc32Process(params, obj);
    }
};

LinuxLoader linuxLoader;

} // anonymous namespace

namespace SparcISA
{

EmuLinux::EmuLinux(const Params &p) : SEWorkload(p, PageShift)
{}

void
EmuLinux::handleTrap(ThreadContext *tc, int trapNum)
{
    if (is64(tc)) {
        switch (trapNum) {
          // case 0x10: // Linux 32 bit syscall trap
          case 0x6d: // Linux 64 bit syscall trap
            syscall64(tc);
            return;
          case 0x6e: // Linux 64 bit getcontext trap
            warn("The getcontext trap is not implemented on SPARC");
            return;
          case 0x6f: // Linux 64 bit setcontext trap
            panic("The setcontext trap is not implemented on SPARC");
          default:
            break;
        }
    } else {
        switch (trapNum) {
          case 0x10: //Linux 32 bit syscall trap
            syscall32(tc);
            return;
          default:
            break;
        }
    }
    SEWorkload::handleTrap(tc, trapNum);
}

void
EmuLinux::syscall32(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    syscall32Descs.get(tc->getReg(int_reg::G1))->doSyscall(tc);
}

void
EmuLinux::syscall64(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    syscallDescs.get(tc->getReg(int_reg::G1))->doSyscall(tc);
}

void
EmuLinux::syscall(ThreadContext *tc)
{
    if (is64(tc))
        syscall64(tc);
    else
        syscall32(tc);
}

} // namespace SparcISA
} // namespace gem5
