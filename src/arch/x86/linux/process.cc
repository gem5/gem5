/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#include "arch/x86/isa_traits.hh"
#include "arch/x86/linux/process.hh"
#include "arch/x86/regfile.hh"

#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"

#include "sim/process.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace X86ISA;

SyscallDesc*
X86LinuxProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum > Num_Syscall_Descs)
        return NULL;
    return &syscallDescs[callnum];
}

X86LinuxProcess::X86LinuxProcess(const std::string &name,
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
    : X86LiveProcess(name, objFile, system,
            stdin_fd, stdout_fd, stderr_fd, argv, envp, cwd,
            _uid, _euid, _gid, _egid, _pid, _ppid),
    Num_Syscall_Descs(273)
{}

void X86LinuxProcess::handleTrap(int trapNum, ThreadContext *tc)
{
    switch(trapNum)
    {
      //This implementation is from SPARC
      case 0x10: //Linux 32 bit syscall trap
        tc->syscall(tc->readIntReg(1));
        break;
      default:
        X86LiveProcess::handleTrap(trapNum, tc);
    }
}
