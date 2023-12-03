/*
 * Copyright 2007 The Hewlett-Packard Development Company
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

#include "arch/x86/linux/se_workload.hh"

#include <sys/syscall.h>

#include "arch/x86/linux/linux.hh"
#include "arch/x86/page_size.hh"
#include "arch/x86/process.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/se_workload.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

namespace gem5
{

namespace
{

class LinuxLoader : public Process::Loader
{
  public:
    Process *
    load(const ProcessParams &params, loader::ObjectFile *obj_file)
    {
        auto arch = obj_file->getArch();
        auto opsys = obj_file->getOpSys();

        if (arch != loader::X86_64 && arch != loader::I386)
            return nullptr;

        if (opsys == loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = loader::Linux;
        }

        if (opsys != loader::Linux)
            return nullptr;

        if (arch == loader::X86_64)
            return new X86ISA::X86_64Process(params, obj_file);
        else
            return new X86ISA::I386Process(params, obj_file);
    }
};

LinuxLoader linuxLoader;

} // anonymous namespace

namespace X86ISA
{

EmuLinux::EmuLinux(const Params &p) : SEWorkload(p, PageShift) {}

const std::vector<RegId> EmuLinux::SyscallABI64::ArgumentRegs = {
    int_reg::Rdi, int_reg::Rsi, int_reg::Rdx,
    int_reg::R10, int_reg::R8,  int_reg::R9
};

const std::vector<RegId> EmuLinux::SyscallABI32::ArgumentRegs = {
    int_reg::Ebx, int_reg::Ecx, int_reg::Edx,
    int_reg::Esi, int_reg::Edi, int_reg::Ebp
};

void
EmuLinux::syscall(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    RegVal rax = tc->getReg(int_reg::Rax);
    if (dynamic_cast<X86_64Process *>(process)) {
        syscallDescs64.get(rax)->doSyscall(tc);
    } else if (auto *proc32 = dynamic_cast<I386Process *>(process)) {
        PCState pc = tc->pcState().as<PCState>();
        Addr eip = pc.pc();
        const auto &vsyscall = proc32->getVSyscallPage();
        if (eip >= vsyscall.base && eip < vsyscall.base + vsyscall.size) {
            pc.set(vsyscall.base + vsyscall.vsysexitOffset);
            tc->pcState(pc);
        }
        syscallDescs32.get(rax)->doSyscall(tc);
    } else {
        panic("Unrecognized process type.");
    }
}

void
EmuLinux::event(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    Addr pc = tc->pcState().instAddr();

    if (process->kvmInSE) {
        Addr pc_page = mbits(pc, 63, 12);
        if (pc_page == syscallCodeVirtAddr) {
            syscall(tc);
            return;
        } else if (pc_page == PFHandlerVirtAddr) {
            pageFault(tc);
            return;
        }
    }
    warn("Unexpected workload event at pc %#x.", pc);
}

void
EmuLinux::pageFault(ThreadContext *tc)
{
    Process *p = tc->getProcessPtr();
    if (!p->fixupFault(tc->readMiscReg(misc_reg::Cr2))) {
        SETranslatingPortProxy proxy(tc);
        // at this point we should have 6 values on the interrupt stack
        int size = 6;
        uint64_t is[size];
        // reading the interrupt handler stack
        proxy.readBlob(ISTVirtAddr + PageBytes - size * sizeof(uint64_t), &is,
                       sizeof(is));
        panic("Page fault at addr %#x\n\tInterrupt handler stack:\n"
              "\tss: %#x\n"
              "\trsp: %#x\n"
              "\trflags: %#x\n"
              "\tcs: %#x\n"
              "\trip: %#x\n"
              "\terr_code: %#x\n",
              tc->readMiscReg(misc_reg::Cr2), is[5], is[4], is[3], is[2],
              is[1], is[0]);
    }
}

} // namespace X86ISA
} // namespace gem5
