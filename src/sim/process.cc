/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
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
 */

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <map>
#include <string>

#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/intmath.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "mem/multi_level_page_table.hh"
#include "mem/se_translating_port_proxy.hh"
#include "params/LiveProcess.hh"
#include "params/Process.hh"
#include "sim/debug.hh"
#include "sim/process.hh"
#include "sim/process_impl.hh"
#include "sim/stats.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/linux/process.hh"
#include "arch/alpha/tru64/process.hh"
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
#else
#error "THE_ISA not set"
#endif


using namespace std;
using namespace TheISA;

// current number of allocated processes
int num_processes = 0;

template<class IntType>

AuxVector<IntType>::AuxVector(IntType type, IntType val)
{
    a_type = TheISA::htog(type);
    a_val = TheISA::htog(val);
}

template struct AuxVector<uint32_t>;
template struct AuxVector<uint64_t>;

static int
openFile(const string& filename, int flags, mode_t mode)
{
    int sim_fd = open(filename.c_str(), flags, mode);
    if (sim_fd != -1)
        return sim_fd;
    fatal("Unable to open %s with mode %O", filename, mode);
}

static int
openInputFile(const string &filename)
{
    return openFile(filename, O_RDONLY, 0);
}

static int
openOutputFile(const string &filename)
{
    return openFile(filename, O_WRONLY | O_CREAT | O_TRUNC, 0664);
}

Process::Process(ProcessParams * params)
    : SimObject(params), system(params->system),
      brk_point(0), stack_base(0), stack_size(0), stack_min(0),
      max_stack_size(params->max_stack_size),
      next_thread_stack_base(0),
      M5_pid(system->allocatePID()),
      useArchPT(params->useArchPT),
      kvmInSE(params->kvmInSE),
      pTable(useArchPT ?
        static_cast<PageTableBase *>(new ArchPageTable(name(), M5_pid, system)) :
        static_cast<PageTableBase *>(new FuncPageTable(name(), M5_pid)) ),
      initVirtMem(system->getSystemPort(), this,
                  SETranslatingPortProxy::Always),
      fd_array(make_shared<array<FDEntry, NUM_FDS>>()),
      imap {{"",       -1},
            {"cin",    STDIN_FILENO},
            {"stdin",  STDIN_FILENO}},
      oemap{{"",       -1},
            {"cout",   STDOUT_FILENO},
            {"stdout", STDOUT_FILENO},
            {"cerr",   STDERR_FILENO},
            {"stderr", STDERR_FILENO}}
{
    int sim_fd;
    std::map<string,int>::iterator it;

    // Search through the input options and set fd if match is found;
    // otherwise, open an input file and seek to location.
    FDEntry *fde_stdin = getFDEntry(STDIN_FILENO);
    if ((it = imap.find(params->input)) != imap.end())
        sim_fd = it->second;
    else
        sim_fd = openInputFile(params->input);
    fde_stdin->set(sim_fd, params->input, O_RDONLY, -1, false);

    // Search through the output/error options and set fd if match is found;
    // otherwise, open an output file and seek to location.
    FDEntry *fde_stdout = getFDEntry(STDOUT_FILENO);
    if ((it = oemap.find(params->output)) != oemap.end())
        sim_fd = it->second;
    else
        sim_fd = openOutputFile(params->output);
    fde_stdout->set(sim_fd, params->output, O_WRONLY | O_CREAT | O_TRUNC,
                    0664, false);

    FDEntry *fde_stderr = getFDEntry(STDERR_FILENO);
    if (params->output == params->errout)
        // Reuse the same file descriptor if these match.
        sim_fd = fde_stdout->fd;
    else if ((it = oemap.find(params->errout)) != oemap.end())
        sim_fd = it->second;
    else
        sim_fd = openOutputFile(params->errout);
    fde_stderr->set(sim_fd, params->errout, O_WRONLY | O_CREAT | O_TRUNC,
                    0664, false);

    mmap_end = 0;
    nxm_start = nxm_end = 0;
    // other parameters will be initialized when the program is loaded
}


void
Process::regStats()
{
    using namespace Stats;

    num_syscalls
        .name(name() + ".num_syscalls")
        .desc("Number of system calls")
        ;
}

void
Process::inheritFDArray(Process *p)
{
    fd_array = p->fd_array;
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
    findFileOffsets();
    return DrainState::Drained;
}

int
Process::allocFD(int sim_fd, const string& filename, int flags, int mode,
                 bool pipe)
{
    for (int free_fd = 0; free_fd < fd_array->size(); free_fd++) {
        FDEntry *fde = getFDEntry(free_fd);
        if (fde->isFree()) {
            fde->set(sim_fd, filename, flags, mode, pipe);
            return free_fd;
        }
    }

    fatal("Out of target file descriptors");
}

void
Process::resetFDEntry(int tgt_fd)
{
    FDEntry *fde = getFDEntry(tgt_fd);
    assert(fde->fd > -1);

    fde->reset();
}

int
Process::getSimFD(int tgt_fd)
{
    FDEntry *entry = getFDEntry(tgt_fd);
    return entry ? entry->fd : -1;
}

FDEntry *
Process::getFDEntry(int tgt_fd)
{
    assert(0 <= tgt_fd && tgt_fd < fd_array->size());
    return &(*fd_array)[tgt_fd];
}

int
Process::getTgtFD(int sim_fd)
{
    for (int index = 0; index < fd_array->size(); index++)
        if ((*fd_array)[index].fd == sim_fd)
            return index;
    return -1;
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
Process::fixFileOffsets()
{
    auto seek = [] (FDEntry *fde)
    {
        if (lseek(fde->fd, fde->fileOffset, SEEK_SET) < 0)
            fatal("Unable to see to location in %s", fde->filename);
    };

    std::map<string,int>::iterator it;

    // Search through the input options and set fd if match is found;
    // otherwise, open an input file and seek to location.
    FDEntry *fde_stdin = getFDEntry(STDIN_FILENO);
    if ((it = imap.find(fde_stdin->filename)) != imap.end()) {
        fde_stdin->fd = it->second;
    } else {
        fde_stdin->fd = openInputFile(fde_stdin->filename);
        seek(fde_stdin);
    }

    // Search through the output/error options and set fd if match is found;
    // otherwise, open an output file and seek to location.
    FDEntry *fde_stdout = getFDEntry(STDOUT_FILENO);
    if ((it = oemap.find(fde_stdout->filename)) != oemap.end()) {
        fde_stdout->fd = it->second;
    } else {
        fde_stdout->fd = openOutputFile(fde_stdout->filename);
        seek(fde_stdout);
    }

    FDEntry *fde_stderr = getFDEntry(STDERR_FILENO);
    if (fde_stdout->filename == fde_stderr->filename) {
        // Reuse the same file descriptor if these match.
        fde_stderr->fd = fde_stdout->fd;
    } else if ((it = oemap.find(fde_stderr->filename)) != oemap.end()) {
        fde_stderr->fd = it->second;
    } else {
        fde_stderr->fd = openOutputFile(fde_stderr->filename);
        seek(fde_stderr);
    }

    for (int tgt_fd = 3; tgt_fd < fd_array->size(); tgt_fd++) {
        FDEntry *fde = getFDEntry(tgt_fd);
        if (fde->fd == -1)
            continue;

        if (fde->isPipe) {
            if (fde->filename == "PIPE-WRITE")
                continue;
            assert(fde->filename == "PIPE-READ");

            int fds[2];
            if (pipe(fds) < 0)
                fatal("Unable to create new pipe");

            fde->fd = fds[0];

            FDEntry *fde_write = getFDEntry(fde->readPipeSource);
            assert(fde_write->filename == "PIPE-WRITE");
            fde_write->fd = fds[1];
        } else {
            fde->fd = openFile(fde->filename.c_str(), fde->flags, fde->mode);
            seek(fde);
        }
    }
}

void
Process::findFileOffsets()
{
    for (auto& fde : *fd_array) {
        if (fde.fd != -1)
            fde.fileOffset = lseek(fde.fd, 0, SEEK_CUR);
    }
}

void
Process::setReadPipeSource(int read_pipe_fd, int source_fd)
{
    FDEntry *fde = getFDEntry(read_pipe_fd);
    assert(source_fd >= -1);
    fde->readPipeSource = source_fd;
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
    SERIALIZE_SCALAR(nxm_start);
    SERIALIZE_SCALAR(nxm_end);
    pTable->serialize(cp);
    for (int x = 0; x < fd_array->size(); x++) {
        (*fd_array)[x].serializeSection(cp, csprintf("FDEntry%d", x));
    }
    SERIALIZE_SCALAR(M5_pid);

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
    UNSERIALIZE_SCALAR(nxm_start);
    UNSERIALIZE_SCALAR(nxm_end);
    pTable->unserialize(cp);
    for (int x = 0; x < fd_array->size(); x++) {
        FDEntry *fde = getFDEntry(x);
        fde->unserializeSection(cp, csprintf("FDEntry%d", x));
    }
    fixFileOffsets();
    UNSERIALIZE_OPT_SCALAR(M5_pid);
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


////////////////////////////////////////////////////////////////////////
//
// LiveProcess member definitions
//
////////////////////////////////////////////////////////////////////////


LiveProcess::LiveProcess(LiveProcessParams *params, ObjectFile *_objFile)
    : Process(params), objFile(_objFile),
      argv(params->cmd), envp(params->env), cwd(params->cwd),
      executable(params->executable),
      __uid(params->uid), __euid(params->euid),
      __gid(params->gid), __egid(params->egid),
      __pid(params->pid), __ppid(params->ppid),
      drivers(params->drivers)
{

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
LiveProcess::syscall(int64_t callnum, ThreadContext *tc)
{
    num_syscalls++;

    SyscallDesc *desc = getDesc(callnum);
    if (desc == NULL)
        fatal("Syscall %d out of range", callnum);

    desc->doSyscall(callnum, this, tc);
}

IntReg
LiveProcess::getSyscallArg(ThreadContext *tc, int &i, int width)
{
    return getSyscallArg(tc, i);
}


EmulatedDriver *
LiveProcess::findDriver(std::string filename)
{
    for (EmulatedDriver *d : drivers) {
        if (d->match(filename))
            return d;
    }

    return NULL;
}

void
LiveProcess::updateBias()
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
LiveProcess::getInterpreter()
{
    return objFile->getInterpreter();
}


Addr
LiveProcess::getBias()
{
    ObjectFile *interp = getInterpreter();

    return interp ? interp->bias() : objFile->bias();
}


Addr
LiveProcess::getStartPC()
{
    ObjectFile *interp = getInterpreter();

    return interp ? interp->entryPoint() : objFile->entryPoint();
}


LiveProcess *
LiveProcess::create(LiveProcessParams * params)
{
    LiveProcess *process = NULL;

    // If not specified, set the executable parameter equal to the
    // simulated system's zeroth command line parameter
    if (params->executable == "") {
        params->executable = params->cmd[0];
    }

    ObjectFile *objFile = createObjectFile(params->executable);
    if (objFile == NULL) {
        fatal("Can't load object file %s", params->executable);
    }

#if THE_ISA == ALPHA_ISA
    if (objFile->getArch() != ObjectFile::Alpha)
        fatal("Object file architecture does not match compiled ISA (Alpha).");

    switch (objFile->getOpSys()) {
      case ObjectFile::Tru64:
        process = new AlphaTru64Process(params, objFile);
        break;

      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new AlphaLinuxProcess(params, objFile);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == SPARC_ISA
    if (objFile->getArch() != ObjectFile::SPARC64 &&
        objFile->getArch() != ObjectFile::SPARC32)
        fatal("Object file architecture does not match compiled ISA (SPARC).");
    switch (objFile->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (objFile->getArch() == ObjectFile::SPARC64) {
            process = new Sparc64LinuxProcess(params, objFile);
        } else {
            process = new Sparc32LinuxProcess(params, objFile);
        }
        break;


      case ObjectFile::Solaris:
        process = new SparcSolarisProcess(params, objFile);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == X86_ISA
    if (objFile->getArch() != ObjectFile::X86_64 &&
        objFile->getArch() != ObjectFile::I386)
        fatal("Object file architecture does not match compiled ISA (x86).");
    switch (objFile->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (objFile->getArch() == ObjectFile::X86_64) {
            process = new X86_64LinuxProcess(params, objFile);
        } else {
            process = new I386LinuxProcess(params, objFile);
        }
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == MIPS_ISA
    if (objFile->getArch() != ObjectFile::Mips)
        fatal("Object file architecture does not match compiled ISA (MIPS).");
    switch (objFile->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new MipsLinuxProcess(params, objFile);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == ARM_ISA
    ObjectFile::Arch arch = objFile->getArch();
    if (arch != ObjectFile::Arm && arch != ObjectFile::Thumb &&
        arch != ObjectFile::Arm64)
        fatal("Object file architecture does not match compiled ISA (ARM).");
    switch (objFile->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (arch == ObjectFile::Arm64) {
            process = new ArmLinuxProcess64(params, objFile,
                                            objFile->getArch());
        } else {
            process = new ArmLinuxProcess32(params, objFile,
                                            objFile->getArch());
        }
        break;
      case ObjectFile::FreeBSD:
        if (arch == ObjectFile::Arm64) {
            process = new ArmFreebsdProcess64(params, objFile,
                                              objFile->getArch());
        } else {
            process = new ArmFreebsdProcess32(params, objFile,
                                              objFile->getArch());
        }
        break;
      case ObjectFile::LinuxArmOABI:
        fatal("M5 does not support ARM OABI binaries. Please recompile with an"
              " EABI compiler.");
      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == POWER_ISA
    if (objFile->getArch() != ObjectFile::Power)
        fatal("Object file architecture does not match compiled ISA (Power).");
    switch (objFile->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new PowerLinuxProcess(params, objFile);
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

LiveProcess *
LiveProcessParams::create()
{
    return LiveProcess::create(this);
}
