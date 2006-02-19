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

#include "arch/isa_traits.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "base/statistics.hh"
#include "base/trace.hh"

class ExecContext;
class FunctionalMemory;
class Process : public SimObject
{
  protected:
    typedef TheISA::Addr Addr;
    typedef TheISA::RegFile RegFile;
    typedef TheISA::MachInst MachInst;
  public:

    // have we initialized an execution context from this process?  If
    // yes, subsequent contexts are assumed to be for dynamically
    // created threads and are not initialized.
    bool initialContextLoaded;

    // execution contexts associated with this process
    std::vector<ExecContext *> execContexts;

    // number of CPUs (esxec contexts, really) assigned to this process.
    unsigned int numCpus() { return execContexts.size(); }

    // record of blocked context
    struct WaitRec
    {
        Addr waitChan;
        ExecContext *waitingContext;

        WaitRec(Addr chan, ExecContext *ctx)
            : waitChan(chan), waitingContext(ctx)
        {
        }
    };

    // list of all blocked contexts
    std::list<WaitRec> waitList;

    RegFile *init_regs;		// initial register contents

    Addr text_base;		// text (code) segment base
    unsigned text_size;		// text (code) size in bytes

    Addr data_base;		// initialized data segment base
    unsigned data_size;		// initialized data + bss size in bytes

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
    Addr prog_entry;		// entry point (initial PC)

    Stats::Scalar<> num_syscalls;	// number of syscalls executed


  protected:
    // constructor
    Process(const std::string &nm,
            int stdin_fd, 	// initial I/O descriptors
            int stdout_fd,
            int stderr_fd);

    // post initialization startup
    virtual void startup();

  protected:
    FunctionalMemory *memory;

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

    // register an execution context for this process.
    // returns xc's cpu number (index into execContexts[])
    int registerExecContext(ExecContext *xc);


    void replaceExecContext(ExecContext *xc, int xcIndex);

    // map simulator fd sim_fd to target fd tgt_fd
    void dup_fd(int sim_fd, int tgt_fd);

    // generate new target fd for sim_fd
    int alloc_fd(int sim_fd);

    // free target fd (e.g., after close)
    void free_fd(int tgt_fd);

    // look up simulator fd for given target fd
    int sim_fd(int tgt_fd);

    // is this a valid instruction fetch address?
    bool validInstAddr(Addr addr)
    {
        return (text_base <= addr &&
                addr < text_base + text_size &&
                !(addr & (sizeof(MachInst)-1)));
    }

    // is this a valid address? (used to filter data fetches)
    // note that we just assume stack size <= 16MB
    // this may be alpha-specific
    bool validDataAddr(Addr addr)
    {
        return ((data_base <= addr && addr < brk_point) ||
                (next_thread_stack_base <= addr && addr < stack_base) ||
                (text_base <= addr && addr < (text_base + text_size)) ||
                (mmap_start <= addr && addr < mmap_end) ||
                (nxm_start <= addr && addr < nxm_end));
    }

    virtual void syscall(ExecContext *xc) = 0;

    virtual FunctionalMemory *getMemory() { return memory; }
};

//
// "Live" process with system calls redirected to host system
//
class ObjectFile;
class LiveProcess : public Process
{
  protected:
    LiveProcess(const std::string &nm, ObjectFile *objFile,
                int stdin_fd, int stdout_fd, int stderr_fd,
                std::vector<std::string> &argv,
                std::vector<std::string> &envp);

  public:
    // this function is used to create the LiveProcess object, since
    // we can't tell which subclass of LiveProcess to use until we
    // open and look at the object file.
    static LiveProcess *create(const std::string &nm,
                               int stdin_fd, int stdout_fd, int stderr_fd,
                               std::string executable,
                               std::vector<std::string> &argv,
                               std::vector<std::string> &envp);
};


#endif // !FULL_SYSTEM

#endif // __PROCESS_HH__
