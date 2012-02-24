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

#include <string>
#include <vector>

#include "arch/registers.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/sim_object.hh"
#include "sim/syscallreturn.hh"

class PageTable;
struct ProcessParams;
struct LiveProcessParams;
class SyscallDesc;
class System;
class ThreadContext;

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
    std::vector<int> contextIds;

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
    Addr mmap_start;
    Addr mmap_end;

    // Base of region for nxm data
    Addr nxm_start;
    Addr nxm_end;

    std::string prog_fname;     // file name

    Stats::Scalar num_syscalls;       // number of syscalls executed


  protected:
    // constructor
    Process(ProcessParams *params);

    virtual void initState();

  public:

    //This id is assigned by m5 and is used to keep process' tlb entries
    //separated.
    uint64_t M5_pid;

    PageTable* pTable;

    class FdMap
    {
      public:
        int fd;
        std::string filename;
        int mode;
        int flags;
        bool isPipe;
        int readPipeSource;
        uint64_t fileOffset;

        FdMap()
            : fd(-1), filename("NULL"), mode(0), flags(0),
              isPipe(false), readPipeSource(0), fileOffset(0)
        { }

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
    };

  protected:
    /// Memory proxy for initialization (image loading)
    SETranslatingPortProxy initVirtMem;

  private:
    // file descriptor remapping support
    static const int MAX_FD = 256;    // max legal fd value
    FdMap fd_map[MAX_FD+1];


  public:
    // static helper functions to generate file descriptors for constructor
    static int openInputFile(const std::string &filename);
    static int openOutputFile(const std::string &filename);

    // override of virtual SimObject method: register statistics
    virtual void regStats();

    // After getting registered with system object, tell process which
    // system-wide context id it is assigned.
    void assignThreadContext(int context_id)
    {
        contextIds.push_back(context_id);
    }

    // Find a free context to use
    ThreadContext *findFreeContext();

    // map simulator fd sim_fd to target fd tgt_fd
    void dup_fd(int sim_fd, int tgt_fd);

    // generate new target fd for sim_fd
    int alloc_fd(int sim_fd, std::string filename, int flags, int mode,
                 bool pipe);

    // free target fd (e.g., after close)
    void free_fd(int tgt_fd);

    // look up simulator fd for given target fd
    int sim_fd(int tgt_fd);

    // look up simulator fd_map object for a given target fd
    FdMap *sim_fd_obj(int tgt_fd);

    // fix all offsets for currently open files and save them
    void fix_file_offsets();

    // find all offsets for currently open files and save them
    void find_file_offsets();

    // set the source of this read pipe for a checkpoint resume
    void setReadPipeSource(int read_pipe_fd, int source_fd);

    virtual void syscall(int64_t callnum, ThreadContext *tc) = 0;

    void allocateMem(Addr vaddr, int64_t size, bool clobber = false);

    /// Attempt to fix up a fault at vaddr by allocating a page on the stack.
    /// @return Whether the fault has been fixed.
    bool fixupStackFault(Addr vaddr);

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
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

    LiveProcess(LiveProcessParams *params, ObjectFile *objFile);

    // Id of the owner of the process
    uint64_t __uid;
    uint64_t __euid;
    uint64_t __gid;
    uint64_t __egid;

    // pid of the process and it's parent
    uint64_t __pid;
    uint64_t __ppid;

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

    // this function is used to create the LiveProcess object, since
    // we can't tell which subclass of LiveProcess to use until we
    // open and look at the object file.
    static LiveProcess *create(LiveProcessParams *params);
};


#endif // __PROCESS_HH__
