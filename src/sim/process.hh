/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
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

#include <inttypes.h>

#include <array>
#include <map>
#include <string>
#include <vector>

#include "arch/registers.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/fd_entry.hh"
#include "sim/sim_object.hh"

struct ProcessParams;

class EmulatedDriver;
class ObjectFile;
class PageTableBase;
class SyscallDesc;
class SyscallReturn;
class System;
class ThreadContext;

class Process : public SimObject
{
  public:
    struct WaitRec
    {
        Addr waitChan;
        ThreadContext *waitingContext;

        WaitRec(Addr chan, ThreadContext *ctx)
            : waitChan(chan), waitingContext(ctx)
        { }
    };

    Process(ProcessParams *params, ObjectFile *obj_file);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void initState() override;
    DrainState drain() override;

    void syscall(int64_t callnum, ThreadContext *tc);
    virtual TheISA::IntReg getSyscallArg(ThreadContext *tc, int &i) = 0;
    virtual TheISA::IntReg getSyscallArg(ThreadContext *tc, int &i, int width);
    virtual void setSyscallArg(ThreadContext *tc, int i,
                               TheISA::IntReg val) = 0;
    virtual void setSyscallReturn(ThreadContext *tc,
                                  SyscallReturn return_value) = 0;
    virtual SyscallDesc *getDesc(int callnum) = 0;

    inline uint64_t uid() { return _uid; }
    inline uint64_t euid() { return _euid; }
    inline uint64_t gid() { return _gid; }
    inline uint64_t egid() { return _egid; }
    inline uint64_t pid() { return _pid; }
    inline uint64_t ppid() { return _ppid; }

    const char *progName() const { return executable.c_str(); }
    std::string fullPath(const std::string &filename);
    std::string getcwd() const { return cwd; }

    /**
     * Find an emulated device driver.
     *
     * @param filename Name of the device (under /dev)
     * @return Pointer to driver object if found, else NULL
     */
    EmulatedDriver *findDriver(std::string filename);

    // This function acts as a callback to update the bias value in
    // the object file because the parameters needed to calculate the
    // bias are not available when the object file is created.
    void updateBias();
    Addr getBias();
    Addr getStartPC();
    ObjectFile *getInterpreter();

    // inherit file descriptor map from another process (necessary for clone)
    void inheritFDArray(Process *p);

    // override of virtual SimObject method: register statistics
    void regStats() override;

    // generate new target fd for sim_fd
    int allocFD(int sim_fd, const std::string& filename, int flags, int mode,
                bool pipe);

    // disassociate target fd with simulator fd and cleanup subsidiary fields
    void resetFDEntry(int tgt_fd);

    // look up simulator fd for given target fd
    int getSimFD(int tgt_fd);

    // look up fd entry for a given target fd
    FDEntry *getFDEntry(int tgt_fd);

    // look up target fd for given host fd
    // Assumes a 1:1 mapping between target file descriptor and host file
    // descriptor. Given the current API, this must be true given that it's
    // not possible to map multiple target file descriptors to the same host
    // file descriptor
    int getTgtFD(int sim_fd);

    // fix all offsets for currently open files and save them
    void fixFileOffsets();

    // find all offsets for currently open files and save them
    void findFileOffsets();

    // set the source of this read pipe for a checkpoint resume
    void setReadPipeSource(int read_pipe_fd, int source_fd);


    void allocateMem(Addr vaddr, int64_t size, bool clobber = false);

    /// Attempt to fix up a fault at vaddr by allocating a page on the stack.
    /// @return Whether the fault has been fixed.
    bool fixupStackFault(Addr vaddr);

    // After getting registered with system object, tell process which
    // system-wide context id it is assigned.
    void
    assignThreadContext(ContextID context_id)
    {
        contextIds.push_back(context_id);
    }

    // Find a free context to use
    ThreadContext *findFreeContext();

    /**
     * Does mmap region grow upward or downward from mmap_end?  Most
     * platforms grow downward, but a few (such as Alpha) grow upward
     * instead, so they can override this method to return false.
     */
    virtual bool mmapGrowsDown() const { return true; }

    /**
     * Maps a contiguous range of virtual addresses in this process's
     * address space to a contiguous range of physical addresses.
     * This function exists primarily to expose the map operation to
     * python, so that configuration scripts can set up mappings in SE mode.
     *
     * @param vaddr The starting virtual address of the range.
     * @param paddr The starting physical address of the range.
     * @param size The length of the range in bytes.
     * @param cacheable Specifies whether accesses are cacheable.
     * @return True if the map operation was successful.  (At this
     *           point in time, the map operation always succeeds.)
     */
    bool map(Addr vaddr, Addr paddr, int size, bool cacheable = true);

    // list of all blocked contexts
    std::list<WaitRec> waitList;

    // thread contexts associated with this process
    std::vector<ContextID> contextIds;

    // system object which owns this process
    System *system;

    Addr brk_point;              // top of the data segment
    Addr stack_base;             // stack segment base
    unsigned stack_size;         // initial stack size
    Addr stack_min;              // furthest address accessed from stack base
    Addr max_stack_size;         // the maximum size allowed for the stack
    Addr next_thread_stack_base; // addr for next region w/ multithreaded apps
    Addr mmap_end;               // base of automatic mmap region allocs

    Stats::Scalar num_syscalls;  // track how many system calls are executed

    bool useArchPT; // flag for using architecture specific page table
    bool kvmInSE;   // running KVM requires special initialization

    PageTableBase* pTable;

    SETranslatingPortProxy initVirtMem; // memory proxy for initial image load

    static const int NUM_FDS = 1024;

    // File descriptor remapping support.
    std::shared_ptr<std::array<FDEntry, NUM_FDS>> fd_array;

    // Standard file descriptor options for initialization and checkpoints.
    std::map<std::string, int> imap;
    std::map<std::string, int> oemap;

    ObjectFile *objFile;
    std::vector<std::string> argv;
    std::vector<std::string> envp;
    std::string cwd;
    std::string executable;

    // Id of the owner of the process
    uint64_t _uid;
    uint64_t _euid;
    uint64_t _gid;
    uint64_t _egid;

    // pid of the process and it's parent
    uint64_t _pid;
    uint64_t _ppid;

    // Emulated drivers available to this process
    std::vector<EmulatedDriver *> drivers;
};

#endif // __PROCESS_HH__
