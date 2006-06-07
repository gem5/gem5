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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __PROCESS_HH__
#define __PROCESS_HH__

//
// The purpose of this code is to fake the loader & syscall mechanism
// when there's no OS: thus there's no reason to use it in FULL_SYSTEM
// mode when we do have an OS.
//
#include "config/full_system.hh"

#if !FULL_SYSTEM

#include <vector>

#include "base/statistics.hh"
#include "sim/sim_object.hh"

class ThreadContext;
class SyscallDesc;
class PageTable;
class TranslatingPort;
class System;

void
copyStringArray(std::vector<std::string> &strings, Addr array_ptr,
        Addr data_ptr, TranslatingPort* memPort);

class Process : public SimObject
{
  public:

    /// Pointer to object representing the system this process is
    /// running on.
    System *system;

    // have we initialized a thread context from this process?  If
    // yes, subsequent contexts are assumed to be for dynamically
    // created threads and are not initialized.
    bool initialContextLoaded;

    // thread contexts associated with this process
    std::vector<ThreadContext *> threadContexts;

    // number of CPUs (esxec contexts, really) assigned to this process.
    unsigned int numCpus() { return threadContexts.size(); }

    // record of blocked context
    struct WaitRec
    {
        Addr waitChan;
        ThreadContext *waitingContext;

        WaitRec(Addr chan, ThreadContext *ctx)
            : waitChan(chan), waitingContext(ctx)
        {	}
    };

    // list of all blocked contexts
    std::list<WaitRec> waitList;

    Addr brk_point;		// top of the data segment

    Addr stack_base;		// stack segment base (highest address)
    unsigned stack_size;	// initial stack size
    Addr stack_min;		// lowest address accessed on the stack

    // addr to use for next stack region (for multithreaded apps)
    Addr next_thread_stack_base;

    // Base of region for mmaps (when user doesn't specify an address).
    Addr mmap_start;
    Addr mmap_end;

    // Base of region for nxm data
    Addr nxm_start;
    Addr nxm_end;

    std::string prog_fname;	// file name

    Stats::Scalar<> num_syscalls;	// number of syscalls executed


  protected:
    // constructor
    Process(const std::string &nm,
            System *_system,
            int stdin_fd, 	// initial I/O descriptors
            int stdout_fd,
            int stderr_fd);

    // post initialization startup
    virtual void startup();

  protected:
    /// Memory object for initialization (image loading)
    TranslatingPort *initVirtMem;

  public:
    PageTable *pTable;

  private:
    // file descriptor remapping support
    static const int MAX_FD = 256;	// max legal fd value
    int fd_map[MAX_FD+1];

  public:
    // static helper functions to generate file descriptors for constructor
    static int openInputFile(const std::string &filename);
    static int openOutputFile(const std::string &filename);

    // override of virtual SimObject method: register statistics
    virtual void regStats();

    // register a thread context for this process.
    // returns tc's cpu number (index into threadContexts[])
    int registerThreadContext(ThreadContext *tc);


    void replaceThreadContext(ThreadContext *tc, int tcIndex);

    // map simulator fd sim_fd to target fd tgt_fd
    void dup_fd(int sim_fd, int tgt_fd);

    // generate new target fd for sim_fd
    int alloc_fd(int sim_fd);

    // free target fd (e.g., after close)
    void free_fd(int tgt_fd);

    // look up simulator fd for given target fd
    int sim_fd(int tgt_fd);

    virtual void syscall(int64_t callnum, ThreadContext *tc) = 0;
};

//
// "Live" process with system calls redirected to host system
//
class ObjectFile;
class LiveProcess : public Process
{
  protected:
    ObjectFile *objFile;
    std::vector<std::string> argv;
    std::vector<std::string> envp;

    LiveProcess(const std::string &nm, ObjectFile *objFile,
                System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
                std::vector<std::string> &argv,
                std::vector<std::string> &envp);

    virtual void argsInit(int intSize, int pageSize);

  public:
    virtual void syscall(int64_t callnum, ThreadContext *tc);

    virtual SyscallDesc* getDesc(int callnum) = 0;
};


#endif // !FULL_SYSTEM

#endif // __PROCESS_HH__
