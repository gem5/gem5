/*
 * Copyright (c) 2014-2016 Advanced Micro Devices, Inc.
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

#include <inttypes.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "base/loader/memory_image.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/fd_array.hh"
#include "sim/fd_entry.hh"
#include "sim/mem_state.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace loader
{
class ObjectFile;
} // namespace loader

struct ProcessParams;

class EmulatedDriver;
class EmulationPageTable;
class SEWorkload;
class SyscallDesc;
class SyscallReturn;
class System;
class ThreadContext;

class Process : public SimObject
{
  public:
    Process(const ProcessParams &params, EmulationPageTable *pTable,
            loader::ObjectFile *obj_file);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void init() override;
    void initState() override;
    DrainState drain() override;

    virtual void syscall(ThreadContext *tc) { numSyscalls++; }

    inline uint64_t uid() { return _uid; }
    inline uint64_t euid() { return _euid; }
    inline uint64_t gid() { return _gid; }
    inline uint64_t egid() { return _egid; }
    inline uint64_t pid() { return _pid; }
    inline uint64_t ppid() { return _ppid; }
    inline uint64_t pgid() { return _pgid; }
    inline void pgid(uint64_t pgid) { _pgid = pgid; }
    inline uint64_t tgid() { return _tgid; }

    const char *progName() const { return executable.c_str(); }

    /**
     * Find an emulated device driver.
     *
     * @param filename Name of the device (under /dev)
     * @return Pointer to driver object if found, else nullptr
     */
    EmulatedDriver *findDriver(std::string filename);

    // This function acts as a callback to update the bias value in
    // the object file because the parameters needed to calculate the
    // bias are not available when the object file is created.
    void updateBias();
    Addr getBias();
    Addr getStartPC();
    loader::ObjectFile *getInterpreter();

    // This function allocates physical memory as backing store, and then maps
    // it into the virtual address space of the process. The range of virtual
    // addresses being configured starts at the address "vaddr" and is of size
    // "size" bytes. If some part of this range of virtual addresses is already
    // configured, this function will error out unless "clobber" is set. If
    // clobber is set, then those existing mappings will be replaced.
    //
    // If the beginning or end of the virtual address range does not perfectly
    // align to page boundaries, it will be expanded in either direction until
    // it does. This function will therefore set up *at least* the range
    // requested, and may configure more if necessary.
    void allocateMem(Addr vaddr, int64_t size, bool clobber=false);

    /**
     * Unmap the given virtual address range and deallocate any physical
     * pages that it mapped to.
     */
    void deallocateMem(Addr vaddr, int64_t size);

    /// Attempt to fix up a fault at vaddr by allocating a page on the stack.
    /// @return Whether the fault has been fixed.
    bool fixupFault(Addr vaddr);

    // After getting registered with system object, tell process which
    // system-wide context id it is assigned.
    void
    assignThreadContext(ContextID context_id)
    {
        contextIds.push_back(context_id);
    }

    /**
     * After delegating a thread context to a child process
     * no longer should relate to the ThreadContext
     */
    void revokeThreadContext(int context_id);

    /**
     * Does mmap region grow upward or downward from mmapEnd?  Most
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
    bool map(Addr vaddr, Addr paddr, int64_t size, bool cacheable = true);

    void replicatePage(Addr vaddr, Addr new_paddr, ThreadContext *old_tc,
                       ThreadContext *new_tc, bool alloc_page);

    virtual void clone(ThreadContext *old_tc, ThreadContext *new_tc,
                       Process *new_p, RegVal flags);

    // thread contexts associated with this process
    std::vector<ContextID> contextIds;

    // system object which owns this process
    System *system;

    SEWorkload *seWorkload;

    // flag for using architecture specific page table
    bool useArchPT;
    // running KVM requires special initialization
    bool kvmInSE;
    // flag for using the process as a thread which shares page tables
    bool useForClone;

    /**
     * Whether to ensure that all newly allocated pages are zero-filled.
     * glibc malloc generally requires this. Disable at your own risk.
     */
    bool zeroPages;

    EmulationPageTable *pTable;

    // Memory proxy for initial image load.
    std::unique_ptr<SETranslatingPortProxy> initVirtMem;

    /**
     * Each instance of a Loader subclass will have a chance to try to load
     * an object file when tryLoaders is called. If they can't because they
     * aren't compatible with it (wrong arch, wrong OS, etc), then they
     * silently fail by returning nullptr so other loaders can try.
     */
    class Loader
    {
      public:
        Loader();

        /* Loader instances are singletons. */
        Loader(const Loader &) = delete;
        void operator=(const Loader &) = delete;

        virtual ~Loader() {}

        /**
         * Each subclass needs to implement this method. If the loader is
         * compatible with the passed in object file, it should return the
         * created Process object corresponding to it. If not, it should fail
         * silently and return nullptr. If there's a non-compatibliity related
         * error like file IO errors, etc., those should fail non-silently
         * with a panic or fail as normal.
         */
        virtual Process *load(const ProcessParams &params,
                              loader::ObjectFile *obj_file) = 0;
    };

    // Try all the Loader instance's "load" methods one by one until one is
    // successful. If none are, complain and fail.
    static Process *tryLoaders(const ProcessParams &params,
                               loader::ObjectFile *obj_file);

    loader::ObjectFile *objFile;
    loader::MemoryImage image;
    loader::MemoryImage interpImage;
    std::vector<std::string> argv;
    std::vector<std::string> envp;
    std::string executable;

    /**
     * Return an absolute path given a relative path paired with the current
     * working directory of the process running under simulation.
     *
     * @param path The relative path (generally a filename) that needs the
     * current working directory prepended.
     * @param host_fs A flag which determines whether to return a
     * path for the host filesystem or the filesystem of the process running
     * under simulation. Only matters if filesysem redirection is used to
     * replace files (or directories) that would normally appear via the
     * host filesystem.
     * @return String containing an absolute path.
     */
    std::string absolutePath(const std::string &path, bool host_fs);

    /**
     * Redirect file path if it matches any keys initialized by system object.
     * @param filename An input parameter containing either a relative path
     * or an absolute path. If given a relative path, the path will be
     * prepended to the current working directory of the simulation with
     * respect to the host filesystem.
     * @return String containing an absolute path.
     */
    std::string checkPathRedirect(const std::string &filename);

    /**
     * The cwd members are used to track changes to the current working
     * directory for the purpose of executing system calls which depend on
     * relative paths (i.e. open, chdir).
     *
     * The tgt member and host member may differ if the path for the current
     * working directory is redirected to point to a different location
     * (i.e. `cd /proc` should point to '$(gem5_repo)/m5out/fs/proc'
     * instead of '/proc').
     */
    std::string tgtCwd;
    std::string hostCwd;

    // Syscall emulation uname release.
    std::string release;

    // Id of the owner of the process
    uint64_t _uid;
    uint64_t _euid;
    uint64_t _gid;
    uint64_t _egid;

    // pid of the process and it's parent
    uint64_t _pid;
    uint64_t _ppid;
    uint64_t _pgid;
    uint64_t _tgid;

    // Emulated drivers available to this process
    std::vector<EmulatedDriver *> drivers;

    std::shared_ptr<FDArray> fds;

    bool *exitGroup;
    std::shared_ptr<MemState> memState;

    /**
     * Calls a futex wakeup at the address specified by this pointer when
     * this process exits.
     */
    uint64_t childClearTID;

    // Process was forked with SIGCHLD set.
    bool *sigchld;

    // Contexts to wake up when this thread exits or calls execve
    std::vector<ContextID> vforkContexts;

    // Track how many system calls are executed
    statistics::Scalar numSyscalls;
};

} // namespace gem5

#endif // __PROCESS_HH__
