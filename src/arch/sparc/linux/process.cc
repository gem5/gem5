/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "arch/sparc/linux/process.hh"

#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/registers.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace SparcISA;

namespace
{

class SparcLinuxObjectFileLoader : public Process::Loader
{
  public:
    Process *
    load(ProcessParams *params, ::Loader::ObjectFile *obj_file) override
    {
        auto arch = obj_file->getArch();
        auto opsys = obj_file->getOpSys();

        if (arch != ::Loader::SPARC64 && arch != ::Loader::SPARC32)
            return nullptr;

        if (opsys == ::Loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = ::Loader::Linux;
        }

        if (opsys != ::Loader::Linux)
            return nullptr;

        if (arch == ::Loader::SPARC64)
            return new Sparc64LinuxProcess(params, obj_file);
        else
            return new Sparc32LinuxProcess(params, obj_file);
    }
};

SparcLinuxObjectFileLoader loader;

} // anonymous namespace

Sparc32LinuxProcess::Sparc32LinuxProcess(ProcessParams * params,
                                         ::Loader::ObjectFile *objFile)
    : Sparc32Process(params, objFile)
{}

void
Sparc32LinuxProcess::syscall(ThreadContext *tc, Fault *fault)
{
    Sparc32Process::syscall(tc, fault);
    syscall32Descs.get(tc->readIntReg(1))->doSyscall(tc, fault);
}

void
Sparc32LinuxProcess::handleTrap(int trapNum, ThreadContext *tc, Fault *fault)
{
    switch (trapNum) {
      case 0x10: //Linux 32 bit syscall trap
        tc->syscall(fault);
        break;
      default:
        SparcProcess::handleTrap(trapNum, tc, fault);
    }
}

Sparc64LinuxProcess::Sparc64LinuxProcess(ProcessParams * params,
                                         ::Loader::ObjectFile *objFile)
    : Sparc64Process(params, objFile)
{}

void
Sparc64LinuxProcess::syscall(ThreadContext *tc, Fault *fault)
{
    Sparc64Process::syscall(tc, fault);
    syscallDescs.get(tc->readIntReg(1))->doSyscall(tc, fault);
}

void
Sparc64LinuxProcess::getContext(ThreadContext *tc)
{
    warn("The getcontext trap is not implemented on SPARC");
}

void
Sparc64LinuxProcess::setContext(ThreadContext *tc)
{
    panic("The setcontext trap is not implemented on SPARC");
}

void
Sparc64LinuxProcess::handleTrap(int trapNum, ThreadContext *tc, Fault *fault)
{
    switch (trapNum) {
      // case 0x10: // Linux 32 bit syscall trap
      case 0x6d: // Linux 64 bit syscall trap
        tc->syscall(fault);
        break;
      case 0x6e: // Linux 64 bit getcontext trap
        getContext(tc);
        break;
      case 0x6f: // Linux 64 bit setcontext trap
        setContext(tc);
        break;
      default:
        SparcProcess::handleTrap(trapNum, tc, fault);
    }
}
