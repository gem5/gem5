/*
 * Copyright (c) 2014-2016 Advanced Micro Devices, Inc.
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Ali Saidi
 *          Brandon Potter
 */

#include "sim/process.hh"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <map>
#include <string>
#include <vector>

#include "base/intmath.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "mem/se_translating_port_proxy.hh"
#include "params/Process.hh"
#include "sim/emul_driver.hh"
#include "sim/fd_array.hh"
#include "sim/fd_entry.hh"
#include "sim/syscall_desc.hh"
#include "sim/system.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/linux/process.hh"
#elif THE_ISA == SPARC_ISA
#include "arch/sparc/linux/process.hh"
#include "arch/sparc/solaris/process.hh"
#elif THE_ISA == MIPS_ISA
#include "arch/mips/linux/process.hh"
#elif THE_ISA == ARM_ISA
#include "arch/arm/linux/process.hh"
#include "arch/arm/freebsd/process.hh"
#elif THE_ISA == X86_ISA
#include "arch/x86/linux/process.hh"
#elif THE_ISA == POWER_ISA
#include "arch/power/linux/process.hh"
#elif THE_ISA == RISCV_ISA
#include "arch/riscv/linux/process.hh"
#else
#error "THE_ISA not set"
#endif


using namespace std;
using namespace TheISA;

Process::Process(ProcessParams * params, ObjectFile * obj_file)
    : SimObject(params), system(params->system),
      brk_point(0), stack_base(0), stack_size(0), stack_min(0),
      max_stack_size(params->max_stack_size),
      next_thread_stack_base(0),
      useArchPT(params->useArchPT),
      kvmInSE(params->kvmInSE),
      pTable(useArchPT ?
        static_cast<PageTableBase *>(new ArchPageTable(name(), params->pid,
            system)) :
        static_cast<PageTableBase *>(new FuncPageTable(name(), params->pid))),
      initVirtMem(system->getSystemPort(), this,
                  SETranslatingPortProxy::Always),
      objFile(obj_file),
      argv(params->cmd), envp(params->env), cwd(params->cwd),
      executable(params->executable),
      _uid(params->uid), _euid(params->euid),
      _gid(params->gid), _egid(params->egid),
      _pid(params->pid), _ppid(params->ppid),
      drivers(params->drivers),
      fds(make_shared<FDArray>(params->input, params->output, params->errout))
{
    mmap_end = 0;

    // load up symbols, if any... these may be used for debugging or
    // profiling.
    if (!debugSymbolTable) {
        debugSymbolTable = new SymbolTable();
        if (!objFile->loadGlobalSymbols(debugSymbolTable) ||
            !objFile->loadLocalSymbols(debugSymbolTable) ||
            !objFile->loadWeakSymbols(debugSymbolTable)) {
            // didn't load any symbols
            delete debugSymbolTable;
            debugSymbolTable = NULL;
        }
    }
}

void
Process::regStats()
{
    SimObject::regStats();

    using namespace Stats;

    num_syscalls
        .name(name() + ".num_syscalls")
        .desc("Number of system calls")
        ;
}

ThreadContext *
Process::findFreeContext()
{
    for (int id : contextIds) {
        ThreadContext *tc = system->getThreadContext(id);
        if (tc->status() == ThreadContext::Halted)
            return tc;
    }
    return NULL;
}

void
Process::initState()
{
    if (contextIds.empty())
        fatal("Process %s is not associated with any HW contexts!\n", name());

    // first thread context for this process... initialize & enable
    ThreadContext *tc = system->getThreadContext(contextIds[0]);

    // mark this context as active so it will start ticking.
    tc->activate();

    pTable->initState(tc);
}

DrainState
Process::drain()
{
    fds->updateFileOffsets();
    return DrainState::Drained;
}

void
Process::allocateMem(Addr vaddr, int64_t size, bool clobber)
{
    int npages = divCeil(size, (int64_t)PageBytes);
    Addr paddr = system->allocPhysPages(npages);
    pTable->map(vaddr, paddr, size,
                clobber ? PageTableBase::Clobber : PageTableBase::Zero);
}

bool
Process::fixupStackFault(Addr vaddr)
{
    // Check if this is already on the stack and there's just no page there
    // yet.
    if (vaddr >= stack_min && vaddr < stack_base) {
        allocateMem(roundDown(vaddr, PageBytes), PageBytes);
        return true;
    }

    // We've accessed the next page of the stack, so extend it to include
    // this address.
    if (vaddr < stack_min && vaddr >= stack_base - max_stack_size) {
        while (vaddr < stack_min) {
            stack_min -= TheISA::PageBytes;
            if (stack_base - stack_min > max_stack_size)
                fatal("Maximum stack size exceeded\n");
            allocateMem(stack_min, TheISA::PageBytes);
            inform("Increasing stack size by one page.");
        };
        return true;
    }
    return false;
}

void
Process::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(brk_point);
    SERIALIZE_SCALAR(stack_base);
    SERIALIZE_SCALAR(stack_size);
    SERIALIZE_SCALAR(stack_min);
    SERIALIZE_SCALAR(next_thread_stack_base);
    SERIALIZE_SCALAR(mmap_end);
    pTable->serialize(cp);
    /**
     * Checkpoints for file descriptors currently do not work. Need to
     * come back and fix them at a later date.
     */

    warn("Checkpoints for file descriptors currently do not work.");
#if 0
    for (int x = 0; x < fds->getSize(); x++)
        (*fds)[x].serializeSection(cp, csprintf("FDEntry%d", x));
#endif

}

void
Process::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(brk_point);
    UNSERIALIZE_SCALAR(stack_base);
    UNSERIALIZE_SCALAR(stack_size);
    UNSERIALIZE_SCALAR(stack_min);
    UNSERIALIZE_SCALAR(next_thread_stack_base);
    UNSERIALIZE_SCALAR(mmap_end);
    pTable->unserialize(cp);
    /**
     * Checkpoints for file descriptors currently do not work. Need to
     * come back and fix them at a later date.
     */
    warn("Checkpoints for file descriptors currently do not work.");
#if 0
    for (int x = 0; x < fds->getSize(); x++)
        (*fds)[x]->unserializeSection(cp, csprintf("FDEntry%d", x));
    fds->restoreFileOffsets();
#endif
    // The above returns a bool so that you could do something if you don't
    // find the param in the checkpoint if you wanted to, like set a default
    // but in this case we'll just stick with the instantiated value if not
    // found.
}

bool
Process::map(Addr vaddr, Addr paddr, int size, bool cacheable)
{
    pTable->map(vaddr, paddr, size,
                cacheable ? PageTableBase::Zero : PageTableBase::Uncacheable);
    return true;
}

void
Process::syscall(int64_t callnum, ThreadContext *tc)
{
    num_syscalls++;

    SyscallDesc *desc = getDesc(callnum);
    if (desc == NULL)
        fatal("Syscall %d out of range", callnum);

    desc->doSyscall(callnum, this, tc);
}

IntReg
Process::getSyscallArg(ThreadContext *tc, int &i, int width)
{
    return getSyscallArg(tc, i);
}

EmulatedDriver *
Process::findDriver(std::string filename)
{
    for (EmulatedDriver *d : drivers) {
        if (d->match(filename))
            return d;
    }

    return NULL;
}

void
Process::updateBias()
{
    ObjectFile *interp = objFile->getInterpreter();

    if (!interp || !interp->relocatable())
        return;

    // Determine how large the interpreters footprint will be in the process
    // address space.
    Addr interp_mapsize = roundUp(interp->mapSize(), TheISA::PageBytes);

    // We are allocating the memory area; set the bias to the lowest address
    // in the allocated memory region.
    Addr ld_bias = mmapGrowsDown() ? mmap_end - interp_mapsize : mmap_end;

    // Adjust the process mmap area to give the interpreter room; the real
    // execve system call would just invoke the kernel's internal mmap
    // functions to make these adjustments.
    mmap_end = mmapGrowsDown() ? ld_bias : mmap_end + interp_mapsize;

    interp->updateBias(ld_bias);
}

ObjectFile *
Process::getInterpreter()
{
    return objFile->getInterpreter();
}

Addr
Process::getBias()
{
    ObjectFile *interp = getInterpreter();

    return interp ? interp->bias() : objFile->bias();
}

Addr
Process::getStartPC()
{
    ObjectFile *interp = getInterpreter();

    return interp ? interp->entryPoint() : objFile->entryPoint();
}

Process *
ProcessParams::create()
{
    Process *process = NULL;

    // If not specified, set the executable parameter equal to the
    // simulated system's zeroth command line parameter
    if (executable == "") {
        executable = cmd[0];
    }

    ObjectFile *obj_file = createObjectFile(executable);
    if (obj_file == NULL) {
        fatal("Can't load object file %s", executable);
    }

#if THE_ISA == ALPHA_ISA
    if (obj_file->getArch() != ObjectFile::Alpha)
        fatal("Object file architecture does not match compiled ISA (Alpha).");

    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new AlphaLinuxProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == SPARC_ISA
    if (obj_file->getArch() != ObjectFile::SPARC64 &&
        obj_file->getArch() != ObjectFile::SPARC32)
        fatal("Object file architecture does not match compiled ISA (SPARC).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (obj_file->getArch() == ObjectFile::SPARC64) {
            process = new Sparc64LinuxProcess(this, obj_file);
        } else {
            process = new Sparc32LinuxProcess(this, obj_file);
        }
        break;

      case ObjectFile::Solaris:
        process = new SparcSolarisProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == X86_ISA
    if (obj_file->getArch() != ObjectFile::X86_64 &&
        obj_file->getArch() != ObjectFile::I386)
        fatal("Object file architecture does not match compiled ISA (x86).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (obj_file->getArch() == ObjectFile::X86_64) {
            process = new X86_64LinuxProcess(this, obj_file);
        } else {
            process = new I386LinuxProcess(this, obj_file);
        }
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == MIPS_ISA
    if (obj_file->getArch() != ObjectFile::Mips)
        fatal("Object file architecture does not match compiled ISA (MIPS).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new MipsLinuxProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == ARM_ISA
    ObjectFile::Arch arch = obj_file->getArch();
    if (arch != ObjectFile::Arm && arch != ObjectFile::Thumb &&
        arch != ObjectFile::Arm64)
        fatal("Object file architecture does not match compiled ISA (ARM).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (arch == ObjectFile::Arm64) {
            process = new ArmLinuxProcess64(this, obj_file,
                                            obj_file->getArch());
        } else {
            process = new ArmLinuxProcess32(this, obj_file,
                                            obj_file->getArch());
        }
        break;
      case ObjectFile::FreeBSD:
        if (arch == ObjectFile::Arm64) {
            process = new ArmFreebsdProcess64(this, obj_file,
                                              obj_file->getArch());
        } else {
            process = new ArmFreebsdProcess32(this, obj_file,
                                              obj_file->getArch());
        }
        break;
      case ObjectFile::LinuxArmOABI:
        fatal("M5 does not support ARM OABI binaries. Please recompile with an"
              " EABI compiler.");
      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == POWER_ISA
    if (obj_file->getArch() != ObjectFile::Power)
        fatal("Object file architecture does not match compiled ISA (Power).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new PowerLinuxProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == RISCV_ISA
    if (obj_file->getArch() != ObjectFile::Riscv)
        fatal("Object file architecture does not match compiled ISA (RISCV).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new RiscvLinuxProcess(this, obj_file);
        break;
      default:
        fatal("Unknown/unsupported operating system.");
    }
#else
#error "THE_ISA not set"
#endif

    if (process == NULL)
        fatal("Unknown error creating process object.");
    return process;
}

std::string
Process::fullPath(const std::string &file_name)
{
    if (file_name[0] == '/' || cwd.empty())
        return file_name;

    std::string full = cwd;

    if (cwd[cwd.size() - 1] != '/')
        full += '/';

    return full + file_name;
}
