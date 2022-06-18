/*
 * Copyright 2015 Ruslan Bukin <br@bsdpad.com>
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#include "arch/arm/freebsd/se_workload.hh"

#include <sys/syscall.h>
#if !defined( __GNU_LIBRARY__ ) && (defined(__FreeBSD__) || defined(__APPLE__))
#include <sys/sysctl.h>
#endif

#include "arch/arm/process.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/syscall_emul.hh"

namespace gem5
{

namespace
{

class FreebsdLoader : public Process::Loader
{
  public:
    Process *
    load(const ProcessParams &params, loader::ObjectFile *obj) override
    {
        auto arch = obj->getArch();
        auto opsys = obj->getOpSys();

        if (arch != loader::Arm && arch != loader::Thumb &&
                arch != loader::Arm64) {
            return nullptr;
        }

        if (opsys != loader::FreeBSD)
            return nullptr;

        if (arch == loader::Arm64)
            return new ArmProcess64(params, obj, arch);
        else
            return new ArmProcess32(params, obj, arch);
    }
};

FreebsdLoader freebsdLoader;

} // anonymous namespace

namespace ArmISA
{

static SyscallReturn
issetugidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    return 0;
}

#if !defined ( __GNU_LIBRARY__ )
static SyscallReturn
sysctlFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> namep, size_t nameLen,
           VPtr<> oldp, VPtr<> oldlenp, VPtr<> newp, size_t newlen)
{
    uint64_t ret;

    SETranslatingPortProxy proxy(tc);

    BufferArg buf(namep, sizeof(size_t));
    BufferArg buf2(oldp, sizeof(size_t));
    BufferArg buf3(oldlenp, sizeof(size_t));
    BufferArg buf4(newp, sizeof(size_t));

    buf.copyIn(proxy);
    buf2.copyIn(proxy);
    buf3.copyIn(proxy);

    void *hnewp = NULL;
    if (newp) {
        buf4.copyIn(proxy);
        hnewp = (void *)buf4.bufferPtr();
    }

    uint32_t *hnamep = (uint32_t *)buf.bufferPtr();
    void *holdp = (void *)buf2.bufferPtr();
    size_t *holdlenp = (size_t *)buf3.bufferPtr();

    ret = sysctl((int *)hnamep, nameLen, holdp, holdlenp, hnewp, newlen);

    buf.copyOut(proxy);
    buf2.copyOut(proxy);
    buf3.copyOut(proxy);
    if (newp)
        buf4.copyOut(proxy);

    return (ret);
}
#endif

static SyscallDescTable<EmuFreebsd::SyscallABI32> syscallDescs32({});

static SyscallDescTable<EmuFreebsd::SyscallABI64> syscallDescs64 = {
    {    1, "exit", exitFunc },
    {    3, "read", readFunc<ArmFreebsd64> },
    {    4, "write", writeFunc<ArmFreebsd64> },
    {   17, "obreak", brkFunc },
    {   54, "ioctl", ioctlFunc<ArmFreebsd64> },
    {   58, "readlink", readlinkFunc<ArmFreebsd64> },
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

void
EmuFreebsd::syscall(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    if (dynamic_cast<ArmProcess64 *>(process))
        syscallDescs64.get(tc->getReg(int_reg::X8))->doSyscall(tc);
    else
        syscallDescs32.get(tc->getReg(int_reg::R7))->doSyscall(tc);
}

} // namespace ArmISA
} // namespace gem5
