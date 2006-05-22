/*
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
 */

#include <unistd.h>
#include <fcntl.h>

#include <string>

#include "base/intmath.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "config/full_system.hh"
#include "cpu/exec_context.hh"
#include "mem/page_table.hh"
#include "mem/physical.hh"
#include "mem/translating_port.hh"
#include "sim/builder.hh"
#include "sim/process.hh"
#include "sim/stats.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

//
// The purpose of this code is to fake the loader & syscall mechanism
// when there's no OS: thus there's no resone to use it in FULL_SYSTEM
// mode when we do have an OS
//
#if FULL_SYSTEM
#error "process.cc not compatible with FULL_SYSTEM"
#endif

// current number of allocated processes
int num_processes = 0;

Process::Process(const string &nm,
                 System *_system,
                 int stdin_fd, 	// initial I/O descriptors
                 int stdout_fd,
                 int stderr_fd)
    : SimObject(nm), system(_system)
{
    // initialize first 3 fds (stdin, stdout, stderr)
    fd_map[STDIN_FILENO] = stdin_fd;
    fd_map[STDOUT_FILENO] = stdout_fd;
    fd_map[STDERR_FILENO] = stderr_fd;

    // mark remaining fds as free
    for (int i = 3; i <= MAX_FD; ++i) {
        fd_map[i] = -1;
    }

    mmap_start = mmap_end = 0;
    nxm_start = nxm_end = 0;
    pTable = new PageTable(system);
    // other parameters will be initialized when the program is loaded
}


void
Process::regStats()
{
    using namespace Stats;

    num_syscalls
        .name(name() + ".PROG:num_syscalls")
        .desc("Number of system calls")
        ;
}

//
// static helper functions
//
int
Process::openInputFile(const string &filename)
{
    int fd = open(filename.c_str(), O_RDONLY);

    if (fd == -1) {
        perror(NULL);
        cerr << "unable to open \"" << filename << "\" for reading\n";
        fatal("can't open input file");
    }

    return fd;
}


int
Process::openOutputFile(const string &filename)
{
    int fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0774);

    if (fd == -1) {
        perror(NULL);
        cerr << "unable to open \"" << filename << "\" for writing\n";
        fatal("can't open output file");
    }

    return fd;
}


int
Process::registerExecContext(ExecContext *xc)
{
    // add to list
    int myIndex = execContexts.size();
    execContexts.push_back(xc);

    // return CPU number to caller
    return myIndex;
}

void
Process::startup()
{
    if (execContexts.empty())
        fatal("Process %s is not associated with any CPUs!\n", name());

    // first exec context for this process... initialize & enable
    ExecContext *xc = execContexts[0];

    // mark this context as active so it will start ticking.
    xc->activate(0);

    Port *mem_port;
    mem_port = system->physmem->getPort("functional");
    initVirtMem = new TranslatingPort(pTable, true);
    mem_port->setPeer(initVirtMem);
    initVirtMem->setPeer(mem_port);
}

void
Process::replaceExecContext(ExecContext *xc, int xcIndex)
{
    if (xcIndex >= execContexts.size()) {
        panic("replaceExecContext: bad xcIndex, %d >= %d\n",
              xcIndex, execContexts.size());
    }

    execContexts[xcIndex] = xc;
}

// map simulator fd sim_fd to target fd tgt_fd
void
Process::dup_fd(int sim_fd, int tgt_fd)
{
    if (tgt_fd < 0 || tgt_fd > MAX_FD)
        panic("Process::dup_fd tried to dup past MAX_FD (%d)", tgt_fd);

    fd_map[tgt_fd] = sim_fd;
}


// generate new target fd for sim_fd
int
Process::alloc_fd(int sim_fd)
{
    // in case open() returns an error, don't allocate a new fd
    if (sim_fd == -1)
        return -1;

    // find first free target fd
    for (int free_fd = 0; free_fd < MAX_FD; ++free_fd) {
        if (fd_map[free_fd] == -1) {
            fd_map[free_fd] = sim_fd;
            return free_fd;
        }
    }

    panic("Process::alloc_fd: out of file descriptors!");
}


// free target fd (e.g., after close)
void
Process::free_fd(int tgt_fd)
{
    if (fd_map[tgt_fd] == -1)
        warn("Process::free_fd: request to free unused fd %d", tgt_fd);

    fd_map[tgt_fd] = -1;
}


// look up simulator fd for given target fd
int
Process::sim_fd(int tgt_fd)
{
    if (tgt_fd > MAX_FD)
        return -1;

    return fd_map[tgt_fd];
}



//
// need to declare these here since there is no concrete Process type
// that can be constructed (i.e., no REGISTER_SIM_OBJECT() macro call,
// which is where these get declared for concrete types).
//
DEFINE_SIM_OBJECT_CLASS_NAME("Process", Process)


////////////////////////////////////////////////////////////////////////
//
// LiveProcess member definitions
//
////////////////////////////////////////////////////////////////////////


void
copyStringArray(vector<string> &strings, Addr array_ptr, Addr data_ptr,
                TranslatingPort* memPort)
{
    Addr data_ptr_swap;
    for (int i = 0; i < strings.size(); ++i) {
        data_ptr_swap = htog(data_ptr);
        memPort->writeBlob(array_ptr, (uint8_t*)&data_ptr_swap, sizeof(Addr));
        memPort->writeString(data_ptr, strings[i].c_str());
        array_ptr += sizeof(Addr);
        data_ptr += strings[i].size() + 1;
    }
    // add NULL terminator
    data_ptr = 0;

    memPort->writeBlob(array_ptr, (uint8_t*)&data_ptr, sizeof(Addr));
}

LiveProcess::LiveProcess(const string &nm, ObjectFile *_objFile,
                         System *_system,
                         int stdin_fd, int stdout_fd, int stderr_fd,
                         vector<string> &_argv, vector<string> &_envp)
    : Process(nm, _system, stdin_fd, stdout_fd, stderr_fd),
      objFile(_objFile), argv(_argv), envp(_envp)
{
    prog_fname = argv[0];

    // load up symbols, if any... these may be used for debugging or
    // profiling.
    if (!debugSymbolTable) {
        debugSymbolTable = new SymbolTable();
        if (!objFile->loadGlobalSymbols(debugSymbolTable) ||
            !objFile->loadLocalSymbols(debugSymbolTable)) {
            // didn't load any symbols
            delete debugSymbolTable;
            debugSymbolTable = NULL;
        }
    }
}

void
LiveProcess::argsInit(int intSize, int pageSize)
{
    Process::startup();

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    // Calculate how much space we need for arg & env arrays.
    int argv_array_size = intSize * (argv.size() + 1);
    int envp_array_size = intSize * (envp.size() + 1);
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }
    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }

    int space_needed =
        argv_array_size + envp_array_size + arg_data_size + env_data_size;
    // for SimpleScalar compatibility
    if (space_needed < 16384)
        space_needed = 16384;

    // set bottom of stack
    stack_min = stack_base - space_needed;
    // align it
    stack_min &= ~(intSize-1);
    stack_size = stack_base - stack_min;
    // map memory
    pTable->allocate(roundDown(stack_min, pageSize),
                     roundUp(stack_size, pageSize));

    // map out initial stack contents
    Addr argv_array_base = stack_min + intSize; // room for argc
    Addr envp_array_base = argv_array_base + argv_array_size;
    Addr arg_data_base = envp_array_base + envp_array_size;
    Addr env_data_base = arg_data_base + arg_data_size;

    // write contents to stack
    uint64_t argc = argv.size();
    if (intSize == 8)
        argc = htog((uint64_t)argc);
    else if (intSize == 4)
        argc = htog((uint32_t)argc);
    else
        panic("Unknown int size");

    initVirtMem->writeBlob(stack_min, (uint8_t*)&argc, intSize);

    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);
    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);

    execContexts[0]->setIntReg(ArgumentReg0, argc);
    execContexts[0]->setIntReg(ArgumentReg1, argv_array_base);
    execContexts[0]->setIntReg(StackPointerReg, stack_min);

    Addr prog_entry = objFile->entryPoint();
    execContexts[0]->setPC(prog_entry);
    execContexts[0]->setNextPC(prog_entry + sizeof(MachInst));
    execContexts[0]->setNextNPC(prog_entry + (2 * sizeof(MachInst)));

    num_processes++;
}

void
LiveProcess::syscall(int64_t callnum, ExecContext *xc)
{
    num_syscalls++;

    SyscallDesc *desc = getDesc(callnum);
    if (desc == NULL)
        fatal("Syscall %d out of range", callnum);

    desc->doSyscall(callnum, this, xc);
}

DEFINE_SIM_OBJECT_CLASS_NAME("LiveProcess", LiveProcess);
