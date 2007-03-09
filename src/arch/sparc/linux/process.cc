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

#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/linux/process.hh"
#include "arch/sparc/regfile.hh"

#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"

#include "sim/process.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace SparcISA;

SyscallDesc*
SparcLinuxProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum > Num_Syscall_Descs)
        return NULL;
    return &syscallDescs[callnum];
}

SyscallDesc*
SparcLinuxProcess::getDesc32(int callnum)
{
    if (callnum < 0 || callnum > Num_Syscall32_Descs)
        return NULL;
    return &syscall32Descs[callnum];
}

SparcLinuxProcess::SparcLinuxProcess() :
    Num_Syscall_Descs(284), //sizeof(syscallDescs) / sizeof(SyscallDesc)),
    Num_Syscall32_Descs(299) //sizeof(syscall32Descs) / sizeof(SyscallDesc))
{
    // The sparc syscall table must be <= 284 entries because that is all there
    // is space for.
    assert(Num_Syscall_Descs <= 284);
    // The sparc 32 bit syscall table bust be <= 299 entries because that is
    // all there is space for.
    assert(Num_Syscall_Descs <= 299);
}

Sparc32LinuxProcess::Sparc32LinuxProcess(const std::string &name,
                                         ObjectFile *objFile,
                                         System * system,
                                         int stdin_fd,
                                         int stdout_fd,
                                         int stderr_fd,
                                         std::vector<std::string> &argv,
                                         std::vector<std::string> &envp,
                                         const std::string &cwd,
                                         uint64_t _uid, uint64_t _euid,
                                         uint64_t _gid, uint64_t _egid,
                                         uint64_t _pid, uint64_t _ppid)
    : Sparc32LiveProcess(name, objFile, system,
            stdin_fd, stdout_fd, stderr_fd, argv, envp, cwd,
            _uid, _euid, _gid, _egid, _pid, _ppid)
{}

void Sparc32LinuxProcess::handleTrap(int trapNum, ThreadContext *tc)
{
    switch(trapNum)
    {
      case 0x10: //Linux 32 bit syscall trap
        tc->syscall(tc->readIntReg(1));
        break;
      default:
        SparcLiveProcess::handleTrap(trapNum, tc);
    }
}

Sparc64LinuxProcess::Sparc64LinuxProcess(const std::string &name,
                                         ObjectFile *objFile,
                                         System * system,
                                         int stdin_fd,
                                         int stdout_fd,
                                         int stderr_fd,
                                         std::vector<std::string> &argv,
                                         std::vector<std::string> &envp,
                                         const std::string &cwd,
                                         uint64_t _uid, uint64_t _euid,
                                         uint64_t _gid, uint64_t _egid,
                                         uint64_t _pid, uint64_t _ppid)
    : Sparc64LiveProcess(name, objFile, system,
            stdin_fd, stdout_fd, stderr_fd, argv, envp, cwd,
            _uid, _euid, _gid, _egid, _pid, _ppid)
{}

void Sparc64LinuxProcess::handleTrap(int trapNum, ThreadContext *tc)
{
    switch(trapNum)
    {
      //case 0x10: //Linux 32 bit syscall trap
      case 0x6d: //Linux 64 bit syscall trap
        tc->syscall(tc->readIntReg(1));
        break;
      default:
        SparcLiveProcess::handleTrap(trapNum, tc);
    }
}
