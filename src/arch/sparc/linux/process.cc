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
 *
 * Authors: Steve Reinhardt
 *          Gabe Black
 *          Ali Saidi
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

class SparcLinuxObjectFileLoader : public ObjectFile::Loader
{
  public:
    Process *
    load(ProcessParams *params, ObjectFile *obj_file) override
    {
        auto arch = obj_file->getArch();
        auto opsys = obj_file->getOpSys();

        if (arch != ObjectFile::SPARC64 && arch != ObjectFile::SPARC32)
            return nullptr;

        if (opsys == ObjectFile::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = ObjectFile::Linux;
        }

        if (opsys != ObjectFile::Linux)
            return nullptr;

        if (arch == ObjectFile::SPARC64)
            return new Sparc64LinuxProcess(params, obj_file);
        else
            return new Sparc32LinuxProcess(params, obj_file);
    }
};

SparcLinuxObjectFileLoader loader;

} // anonymous namespace

SyscallDesc*
SparcLinuxProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum >= Num_Syscall_Descs)
        return NULL;
    return &syscallDescs[callnum];
}

SyscallDesc*
SparcLinuxProcess::getDesc32(int callnum)
{
    if (callnum < 0 || callnum >= Num_Syscall32_Descs)
        return NULL;
    return &syscall32Descs[callnum];
}

Sparc32LinuxProcess::Sparc32LinuxProcess(ProcessParams * params,
                                         ObjectFile *objFile)
    : Sparc32Process(params, objFile)
{}

void Sparc32LinuxProcess::handleTrap(int trapNum, ThreadContext *tc,
                                     Fault *fault)
{
    switch (trapNum) {
      case 0x10: //Linux 32 bit syscall trap
        tc->syscall(tc->readIntReg(1), fault);
        break;
      default:
        SparcProcess::handleTrap(trapNum, tc, fault);
    }
}

Sparc64LinuxProcess::Sparc64LinuxProcess(ProcessParams * params,
                                         ObjectFile *objFile)
    : Sparc64Process(params, objFile)
{}

void Sparc64LinuxProcess::handleTrap(int trapNum, ThreadContext *tc,
                                     Fault *fault)
{
    switch (trapNum) {
      // case 0x10: // Linux 32 bit syscall trap
      case 0x6d: // Linux 64 bit syscall trap
        tc->syscall(tc->readIntReg(1), fault);
        break;
      default:
        SparcProcess::handleTrap(trapNum, tc, fault);
    }
}
