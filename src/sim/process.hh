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

#include <array>
#include <string>
#include <vector>

#include "arch/registers.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/fd_entry.hh"
#include "sim/sim_object.hh"
#include "sim/syscallreturn.hh"

class PageTable;
struct ProcessParams;
struct LiveProcessParams;
class SyscallDesc;
class System;
class ThreadContext;
class EmulatedDriver;

template<class IntType>
struct AuxVector
{
    IntType a_type;
    IntType a_val;

    AuxVector()
    {}

    AuxVector(IntType type, IntType val);
};

class Process : public SimObject
{
  public:

    /// Pointer to object representing the system this process is
    /// running on.
    System *system;

    // thread contexts associated with this process
    std::vector<ContextID> contextIds;

    // number of CPUs (esxec contexts, really) assigned to this process.
    unsigned int numCpus() { return contextIds.size(); }

    // record of blocked context
    struct WaitRec
    {
        Addr waitChan;
        ThreadContext *waitingContext;

        WaitRec(Addr chan, ThreadContext *ctx)
            : waitChan(chan), waitingContext(ctx)
        {       }
    };

    // list of all blocked contexts
    std::list<WaitRec> waitList;

    Addr brk_point;             // top of the data segment

    Addr stack_base;            // stack segment base (highest address)
    unsigned stack_size;        // initial stack size
    Addr stack_min;             // lowest address accessed on the stack

    // The maximum size allowed for the stack.
    Addr max_stack_size;

    // addr to use for next stack region (for multithreaded apps)
    Addr next_thread_stack_base;

    // Base of region for mmaps (when user doesn't specify an address).
    Addr mmap_end;

    // Does mmap region grow upward or downward from mmap_end?  Most
    // platforms grow downward, but a few (such as Alpha) grow upward
    // instead, so they can override thie method to return false.
    virtual bool mmapGrowsDown() const { return true; }

    // Base of region for nxm data
    Addr nxm_start;
    Addr nxm_end;

    Stats::Scalar num_syscalls;       // number of syscalls executed

  protected:
    // constructor
    Process(ProcessParams *params);

    void initState() override;

    DrainState drain() override;

  public:

    //This id is assigned by m5 and is used to keep process' tlb entries
    //separated.
    uint64_t M5_pid;

    // flag for using architecture specific page table
    bool useArchPT;
    // running KvmCPU in SE mode requires special initialization
    bool kvmInSE;

    PageTableBase* pTable;

  protected:
    /// Memory proxy for initialization (image loading)
    SETranslatingPortProxy initVirtMem;

  private:
    static const int NUM_FDS = 1024;

    // File descriptor remapping support.
    std::shared_ptr<std::array<FDEntry, NUM_FDS>> fd_array;

    // Standard file descriptor options for initialization and checkpoints.
    std::map<std::string, int> imap;
    std::map<std::string, int> oemap;

  public:
    // inherit file descriptor map from another process (necessary for clone)
    void inheritFDArray(Process *p);

    // override of virtual SimObject method: register statistics
    void regStats() override;

    // After getting registered with system object, tell process which
    // system-wide context id it is assigned.
    void assignThreadContext(ContextID context_id)
    {
        contextIds.push_back(context_id);
    }

    // Find a free context to use
    ThreadContext *findFreeContext();

    // provide program name for debug messages
    virtual const char *progName() const { return "<unknown>"; }

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

    virtual void syscall(int64_t callnum, ThreadContext *tc) = 0;

    void allocateMem(Addr vaddr, int64_t size, bool clobber = false);

    /// Attempt to fix up a fault at vaddr by allocating a page on the stack.
    /// @return Whether the fault has been fixed.
    bool fixupStackFault(Addr vaddr);

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

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
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
    std::string cwd;
    std::string executable;

    LiveProcess(LiveProcessParams *params, ObjectFile *objFile);

    // Id of the owner of the process
    uint64_t __uid;
    uint64_t __euid;
    uint64_t __gid;
    uint64_t __egid;

    // pid of the process and it's parent
    uint64_t __pid;
    uint64_t __ppid;

    // Emulated drivers available to this process
    std::vector<EmulatedDriver *> drivers;

  public:

    enum AuxiliaryVectorType {
        M5_AT_NULL = 0,
        M5_AT_IGNORE = 1,
        M5_AT_EXECFD = 2,
        M5_AT_PHDR = 3,
        M5_AT_PHENT = 4,
        M5_AT_PHNUM = 5,
        M5_AT_PAGESZ = 6,
        M5_AT_BASE = 7,
        M5_AT_FLAGS = 8,
        M5_AT_ENTRY = 9,
        M5_AT_NOTELF = 10,
        M5_AT_UID = 11,
        M5_AT_EUID = 12,
        M5_AT_GID = 13,
        M5_AT_EGID = 14,
        // The following may be specific to Linux
        M5_AT_PLATFORM = 15,
        M5_AT_HWCAP = 16,
        M5_AT_CLKTCK = 17,

        M5_AT_SECURE = 23,
        M5_BASE_PLATFORM = 24,
        M5_AT_RANDOM = 25,

        M5_AT_EXECFN = 31,

        M5_AT_VECTOR_SIZE = 44
    };

    inline uint64_t uid() {return __uid;}
    inline uint64_t euid() {return __euid;}
    inline uint64_t gid() {return __gid;}
    inline uint64_t egid() {return __egid;}
    inline uint64_t pid() {return __pid;}
    inline uint64_t ppid() {return __ppid;}

    // provide program name for debug messages
    virtual const char *progName() const { return executable.c_str(); }

    std::string
    fullPath(const std::string &filename)
    {
        if (filename[0] == '/' || cwd.empty())
            return filename;

        std::string full = cwd;

        if (cwd[cwd.size() - 1] != '/')
            full += '/';

        return full + filename;
    }

    std::string getcwd() const { return cwd; }

    virtual void syscall(int64_t callnum, ThreadContext *tc);

    virtual TheISA::IntReg getSyscallArg(ThreadContext *tc, int &i) = 0;
    virtual TheISA::IntReg getSyscallArg(ThreadContext *tc, int &i, int width);
    virtual void setSyscallArg(ThreadContext *tc,
            int i, TheISA::IntReg val) = 0;
    virtual void setSyscallReturn(ThreadContext *tc,
            SyscallReturn return_value) = 0;

    virtual SyscallDesc *getDesc(int callnum) = 0;

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

    ObjectFile *getInterpreter();

    Addr getBias();
    Addr getStartPC();

    // this function is used to create the LiveProcess object, since
    // we can't tell which subclass of LiveProcess to use until we
    // open and look at the object file.
    static LiveProcess *create(LiveProcessParams *params);
};


#endif // __PROCESS_HH__
