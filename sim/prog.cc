/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
#include <stdio.h>

#include <string>

#include "main_memory.hh"
#include "prog.hh"

#include "eio.hh"
#include "thread.hh"
#include "fake_syscall.hh"
#include "loader.hh"
#include "exec_context.hh"
#include "smt.hh"

#include "statistics.hh"
#include "sim_stats.hh"

using namespace std;

//
// The purpose of this code is to fake the loader & syscall mechanism
// when there's no OS: thus there's no resone to use it in FULL_SYSTEM
// mode when we do have an OS
//
#ifdef FULL_SYSTEM
#error "prog.cc not compatible with FULL_SYSTEM"
#endif

// max allowable number of processes: should be no real cost to
// cranking this up if necessary
const int MAX_PROCESSES = 8;

// current number of allocated processes
int num_processes = 0;

Process::Process(const string &name,
                 int stdin_fd, 	// initial I/O descriptors
                 int stdout_fd,
                 int stderr_fd)
    : SimObject(name)
{
    // allocate memory space
    memory = new MainMemory(name + ".MainMem");

    // allocate initial register file
    init_regs = new RegFile;

    // initialize first 3 fds (stdin, stdout, stderr)
    fd_map[STDIN_FILENO] = stdin_fd;
    fd_map[STDOUT_FILENO] = stdout_fd;
    fd_map[STDERR_FILENO] = stderr_fd;

    // mark remaining fds as free
    for (int i = 3; i <= MAX_FD; ++i) {
        fd_map[i] = -1;
    }

    numCpus = 0;

    num_syscalls = 0;

    // other parameters will be initialized when the program is loaded
}

void
Process::regStats()
{
    using namespace Statistics;

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


void
Process::registerExecContext(ExecContext *ec)
{
    if (execContexts.empty()) {
        // first exec context for this process... initialize & enable

        // copy process's initial regs struct
        ec->regs = *init_regs;

        // mark this context as active
        ec->setStatus(ExecContext::Active);
    }
    else {
        ec->setStatus(ExecContext::Unallocated);
    }

    // add to list
    execContexts.push_back(ec);

    // increment available CPU count
    ++numCpus;
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
Process::open_fd(int sim_fd)
{
    int free_fd;

    // in case open() returns an error, don't allocate a new fd
    if (sim_fd == -1)
        return -1;

    // find first free target fd
    for (free_fd = 0; fd_map[free_fd] >= 0; ++free_fd) {
        if (free_fd == MAX_FD)
            panic("Process::open_fd: out of file descriptors!");
    }

    fd_map[free_fd] = sim_fd;

    return free_fd;
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
DEFINE_SIM_OBJECT_CLASS_NAME("Process object", Process)


////////////////////////////////////////////////////////////////////////
//
// LiveProcess member definitions
//
////////////////////////////////////////////////////////////////////////

LiveProcess::LiveProcess(const string &name,
                         int stdin_fd, int stdout_fd, int stderr_fd,
                         vector<string> &argv, vector<string> &envp)
    : Process(name, stdin_fd, stdout_fd, stderr_fd)
{
    smt_load_prog(argv, envp, init_regs, this);
}


void
LiveProcess::syscall(ExecContext *xc)
{
    num_syscalls++;

    fake_syscall(this, xc);
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(LiveProcess)

    VectorParam<string> cmd;
    Param<string> input;
    Param<string> output;
    VectorParam<string> env;

END_DECLARE_SIM_OBJECT_PARAMS(LiveProcess)


BEGIN_INIT_SIM_OBJECT_PARAMS(LiveProcess)

    INIT_PARAM(cmd, "command line (executable plus arguments)"),
    INIT_PARAM(input, "filename for stdin (dflt: use sim stdin)"),
    INIT_PARAM(output, "filename for stdout/stderr (dflt: use sim stdout)"),
    INIT_PARAM(env, "environment settings")

END_INIT_SIM_OBJECT_PARAMS(LiveProcess)


CREATE_SIM_OBJECT(LiveProcess)
{
    // initialize file descriptors to default: same as simulator
    int stdin_fd = input.isValid() ? Process::openInputFile(input) : 0;
    int stdout_fd = output.isValid() ? Process::openOutputFile(output) : 1;
    int stderr_fd = output.isValid() ? stdout_fd : 2;

    // dummy for default env
    vector<string> null_vec;

    //  We do this with "temp" because of the bogus compiler warning
    //  you get with g++ 2.95 -O if you just "return new LiveProcess(..."
    LiveProcess *temp = new LiveProcess(getInstanceName(),
                                        stdin_fd, stdout_fd, stderr_fd,
                                        cmd,
                                        env.isValid() ? env : null_vec);

    return temp;
}


REGISTER_SIM_OBJECT("LiveProcess", LiveProcess)
