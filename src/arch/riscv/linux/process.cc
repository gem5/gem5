/*
 * Copyright (c) 2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Gabe Black
 *          Korey Sewell
 *          Alec Roelke
 */

#include "arch/riscv/linux/process.hh"

#include <map>

#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/linux/linux.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/SyscallVerbose.hh"
#include "kern/linux/linux.hh"
#include "sim/eventq.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace RiscvISA;

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, Process *process,
          ThreadContext *tc)
{
    int index = 0;
    TypedBufferArg<Linux::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename,"sim.gem5.org");
    strcpy(name->release, "3.0.0");
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "riscv");

    name.copyOut(tc->getMemProxy());
    return 0;
}

std::map<int, SyscallDesc> RiscvLinuxProcess::syscallDescs = {
    {17, SyscallDesc("getcwd", getcwdFunc)},
    {23, SyscallDesc("dup", dupFunc)},
    {25, SyscallDesc("fcntl", fcntl64Func)},
    {29, SyscallDesc("ioctl", ioctlFunc<RiscvLinux>)},
    {34, SyscallDesc("mkdirat", unimplementedFunc)},
    {35, SyscallDesc("unlinkat", unlinkatFunc<RiscvLinux>)},
    {37, SyscallDesc("linkat", unimplementedFunc)},
    {38, SyscallDesc("renameat", renameatFunc<RiscvLinux>)},
    {46, SyscallDesc("ftruncate", ftruncate64Func)},
    {48, SyscallDesc("faccessat", faccessatFunc<RiscvLinux>)},
    {49, SyscallDesc("chdir", unimplementedFunc)},
    {56, SyscallDesc("openat", openatFunc<RiscvLinux>)},
    {57, SyscallDesc("close", closeFunc)},
    {61, SyscallDesc("getdents", unimplementedFunc)},
    {62, SyscallDesc("lseek", lseekFunc)},
    {63, SyscallDesc("read", readFunc)},
    {64, SyscallDesc("write", writeFunc)},
    {66, SyscallDesc("writev", writevFunc<RiscvLinux>)},
    {67, SyscallDesc("pread", unimplementedFunc)},
    {68, SyscallDesc("pwrite", pwrite64Func<RiscvLinux>)},
    {78, SyscallDesc("readlinkat", readlinkatFunc<RiscvLinux>)},
    {79, SyscallDesc("fstatat", fstatat64Func<RiscvLinux>)},
    {80, SyscallDesc("fstat", fstat64Func<RiscvLinux>)},
    {93, SyscallDesc("exit", exitFunc)},
    {94, SyscallDesc("exit_group", exitGroupFunc)},
    {113, SyscallDesc("clock_gettime", clock_gettimeFunc<RiscvLinux>)},
    {129, SyscallDesc("kill", unimplementedFunc)},
    {134, SyscallDesc("rt_sigaction", ignoreFunc, SyscallDesc::WarnOnce)},
    {135, SyscallDesc("rt_sigprocmask", ignoreFunc, SyscallDesc::WarnOnce)},
    {153, SyscallDesc("times", timesFunc<RiscvLinux>)},
    {160, SyscallDesc("uname", unameFunc)},
    {163, SyscallDesc("getrlimit", getrlimitFunc<RiscvLinux>)},
    {164, SyscallDesc("setrlimit", ignoreFunc)},
    {165, SyscallDesc("getrusage", getrusageFunc<RiscvLinux>)},
    {169, SyscallDesc("gettimeofday", gettimeofdayFunc<RiscvLinux>)},
    {172, SyscallDesc("getpid", getpidFunc)},
    {174, SyscallDesc("getuid", getuidFunc)},
    {175, SyscallDesc("geteuid", geteuidFunc)},
    {176, SyscallDesc("getgid", getgidFunc)},
    {177, SyscallDesc("getegid", getegidFunc)},
    {214, SyscallDesc("brk", brkFunc)},
    {215, SyscallDesc("munmap", munmapFunc)},
    {216, SyscallDesc("mremap", mremapFunc<RiscvLinux>)},
    {222, SyscallDesc("mmap", mmapFunc<RiscvLinux>)},
    {226, SyscallDesc("mprotect", ignoreFunc)},
    {1024, SyscallDesc("open", openFunc<RiscvLinux>)},
    {1025, SyscallDesc("link", unimplementedFunc)},
    {1026, SyscallDesc("unlink", unlinkFunc)},
    {1030, SyscallDesc("mkdir", mkdirFunc)},
    {1033, SyscallDesc("access", accessFunc)},
    {1038, SyscallDesc("stat", stat64Func<RiscvLinux>)},
    {1039, SyscallDesc("lstat", lstat64Func<RiscvLinux>)},
    {1062, SyscallDesc("time", timeFunc<RiscvLinux>)},
    {2011, SyscallDesc("getmainvars", unimplementedFunc)},
};

RiscvLinuxProcess::RiscvLinuxProcess(ProcessParams * params,
    ObjectFile *objFile) : RiscvProcess(params, objFile)
{}

SyscallDesc*
RiscvLinuxProcess::getDesc(int callnum)
{
    return syscallDescs.find(callnum) != syscallDescs.end() ?
        &syscallDescs.at(callnum) : nullptr;
}
