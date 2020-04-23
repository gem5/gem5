/*
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#include "arch/arm/freebsd/process.hh"

#include <sys/mman.h>
#include <sys/param.h>
#include <sys/syscall.h>
#if !defined ( __GNU_LIBRARY__ )
#include <sys/sysctl.h>
#endif
#include <sys/types.h>
#include <utime.h>

#include "arch/arm/freebsd/freebsd.hh"
#include "arch/arm/isa_traits.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/freebsd/freebsd.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace ArmISA;

namespace
{

class ArmFreebsdObjectFileLoader : public Process::Loader
{
  public:
    Process *
    load(ProcessParams *params, ::Loader::ObjectFile *obj_file) override
    {
        auto arch = obj_file->getArch();
        auto opsys = obj_file->getOpSys();

        if (arch != ::Loader::Arm && arch != ::Loader::Thumb &&
                arch != ::Loader::Arm64) {
            return nullptr;
        }

        if (opsys != ::Loader::FreeBSD)
            return nullptr;

        if (arch == ::Loader::Arm64)
            return new ArmFreebsdProcess64(params, obj_file, arch);
        else
            return new ArmFreebsdProcess32(params, obj_file, arch);
    }
};

ArmFreebsdObjectFileLoader loader;

} // anonymous namespace

static SyscallReturn
issetugidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    return 0;
}

#if !defined ( __GNU_LIBRARY__ )
static SyscallReturn
sysctlFunc(SyscallDesc *desc, ThreadContext *tc, Addr namep, size_t nameLen,
           Addr oldp, Addr oldlenp, Addr newp, size_t newlen)
{
    uint64_t ret;

    BufferArg buf(namep, sizeof(size_t));
    BufferArg buf2(oldp, sizeof(size_t));
    BufferArg buf3(oldlenp, sizeof(size_t));
    BufferArg buf4(newp, sizeof(size_t));

    buf.copyIn(tc->getVirtProxy());
    buf2.copyIn(tc->getVirtProxy());
    buf3.copyIn(tc->getVirtProxy());

    void *hnewp = NULL;
    if (newp) {
        buf4.copyIn(tc->getVirtProxy());
        hnewp = (void *)buf4.bufferPtr();
    }

    uint32_t *hnamep = (uint32_t *)buf.bufferPtr();
    void *holdp = (void *)buf2.bufferPtr();
    size_t *holdlenp = (size_t *)buf3.bufferPtr();

    ret = sysctl((int *)hnamep, nameLen, holdp, holdlenp, hnewp, newlen);

    buf.copyOut(tc->getVirtProxy());
    buf2.copyOut(tc->getVirtProxy());
    buf3.copyOut(tc->getVirtProxy());
    if (newp)
        buf4.copyOut(tc->getVirtProxy());

    return (ret);
}
#endif

static SyscallDescTable<ArmFreebsdProcess32::SyscallABI> syscallDescs32({});

static SyscallDescTable<ArmFreebsdProcess64::SyscallABI> syscallDescs64 = {
    {    1, "exit", exitFunc },
    {    3, "read", readFunc<ArmFreebsd64> },
    {    4, "write", writeFunc<ArmFreebsd64> },
    {   17, "obreak", brkFunc },
    {   54, "ioctl", ioctlFunc<ArmFreebsd64> },
    {   58, "readlink", readlinkFunc },
    {  117, "getrusage", getrusageFunc<ArmFreebsd64> },
    {  189, "fstat", fstatFunc<ArmFreebsd64> },
#if !defined ( __GNU_LIBRARY__ )
    {  202, "sysctl", sysctlFunc },
#else
    {  202, "sysctl" },
#endif
    {  253, "issetugid", issetugidFunc },
    {  477, "mmap", mmapFunc<ArmFreebsd64> }
};

ArmFreebsdProcess32::ArmFreebsdProcess32(ProcessParams * params,
        ::Loader::ObjectFile *objFile, ::Loader::Arch _arch) :
    ArmProcess32(params, objFile, _arch)
{}

ArmFreebsdProcess64::ArmFreebsdProcess64(ProcessParams * params,
        ::Loader::ObjectFile *objFile, ::Loader::Arch _arch) :
    ArmProcess64(params, objFile, _arch)
{}

void
ArmFreebsdProcess32::initState()
{
    ArmProcess32::initState();
    // The 32 bit equivalent of the comm page would be set up here.
}

void
ArmFreebsdProcess64::initState()
{
    ArmProcess64::initState();
    // The 64 bit equivalent of the comm page would be set up here.
}

void
ArmFreebsdProcess32::syscall(ThreadContext *tc, Fault *fault)
{
    ArmProcess32::syscall(tc, fault);
    syscallDescs32.get(tc->readIntReg(INTREG_R7))->doSyscall(tc, fault);
}

void
ArmFreebsdProcess64::syscall(ThreadContext *tc, Fault *fault)
{
    ArmProcess64::syscall(tc, fault);
    syscallDescs64.get(tc->readIntReg(INTREG_X8))->doSyscall(tc, fault);
}
