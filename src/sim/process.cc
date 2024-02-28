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
 */

#include "sim/process.hh"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <climits>
#include <csignal>
#include <map>
#include <string>
#include <vector>

#include "base/intmath.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "mem/se_translating_port_proxy.hh"
#include "params/Process.hh"
#include "sim/emul_driver.hh"
#include "sim/fd_array.hh"
#include "sim/fd_entry.hh"
#include "sim/redirect_path.hh"
#include "sim/se_workload.hh"
#include "sim/syscall_desc.hh"
#include "sim/system.hh"

namespace gem5
{

namespace
{

typedef std::vector<Process::Loader *> LoaderList;

LoaderList &
process_loaders()
{
    static LoaderList loaders;
    return loaders;
}

} // anonymous namespace

Process::Loader::Loader()
{
    process_loaders().emplace_back(this);
}

Process *
Process::tryLoaders(const ProcessParams &params,
                    loader::ObjectFile *obj_file)
{
    for (auto &loader_it : process_loaders()) {
        Process *p = loader_it->load(params, obj_file);
        if (p)
            return p;
    }

    return nullptr;
}

static std::string
normalize(const std::string& directory)
{
    if (directory.back() != '/')
        return directory + '/';
    return directory;
}

Process::Process(const ProcessParams &params, EmulationPageTable *pTable,
                 loader::ObjectFile *obj_file)
    : SimObject(params), system(params.system),
      seWorkload(dynamic_cast<SEWorkload *>(system->workload)),
      useArchPT(params.useArchPT),
      kvmInSE(params.kvmInSE),
      useForClone(false),
      pTable(pTable),
      objFile(obj_file),
      argv(params.cmd), envp(params.env),
      executable(params.executable == "" ? params.cmd[0] : params.executable),
      tgtCwd(normalize(params.cwd)),
      hostCwd(checkPathRedirect(tgtCwd)),
      release(params.release),
      _uid(params.uid), _euid(params.euid),
      _gid(params.gid), _egid(params.egid),
      _pid(params.pid), _ppid(params.ppid),
      _pgid(params.pgid), drivers(params.drivers),
      fds(std::make_shared<FDArray>(
                  params.input, params.output, params.errout)),
      childClearTID(0),
      ADD_STAT(numSyscalls, statistics::units::Count::get(),
               "Number of system calls")
{
    fatal_if(!seWorkload, "Couldn't find appropriate workload object.");
    fatal_if(_pid >= System::maxPID, "_pid is too large: %d", _pid);

    auto ret_pair = system->PIDs.emplace(_pid);
    fatal_if(!ret_pair.second, "_pid %d is already used", _pid);

    /**
     * Linux bundles together processes into this concept called a thread
     * group. The thread group is responsible for recording which processes
     * behave as threads within a process context. The thread group leader
     * is the process who's tgid is equal to its pid. Other processes which
     * belong to the thread group, but do not lead the thread group, are
     * treated as child threads. These threads are created by the clone system
     * call with options specified to create threads (differing from the
     * options used to implement a fork). By default, set up the tgid/pid
     * with a new, equivalent value. If CLONE_THREAD is specified, patch
     * the tgid value with the old process' value.
     */
    _tgid = params.pid;

    exitGroup = new bool();
    sigchld = new bool();

    image = objFile->buildImage();

    if (loader::debugSymbolTable.empty())
        loader::debugSymbolTable = objFile->symtab();
}

void
Process::clone(ThreadContext *otc, ThreadContext *ntc,
               Process *np, RegVal flags)
{
#ifndef CLONE_VM
#define CLONE_VM 0
#endif
#ifndef CLONE_FILES
#define CLONE_FILES 0
#endif
#ifndef CLONE_THREAD
#define CLONE_THREAD 0
#endif
#ifndef CLONE_VFORK
#define CLONE_VFORK 0
#endif
    if (CLONE_VM & flags) {
        /**
         * Share the process memory address space between the new process
         * and the old process. Changes in one will be visible in the other
         * due to the pointer use.
         */
        delete np->pTable;
        np->pTable = pTable;

        np->memState = memState;
    } else {
        /**
         * Duplicate the process memory address space. The state needs to be
         * copied over (rather than using pointers to share everything).
         */
        typedef std::vector<std::pair<Addr,Addr>> MapVec;
        MapVec mappings;
        pTable->getMappings(&mappings);

        for (auto map : mappings) {
            Addr paddr, vaddr = map.first;
            bool alloc_page = !(np->pTable->translate(vaddr, paddr));
            np->replicatePage(vaddr, paddr, otc, ntc, alloc_page);
        }

        *np->memState = *memState;
    }

    if (CLONE_FILES & flags) {
        /**
         * The parent and child file descriptors are shared because the
         * two FDArray pointers are pointing to the same FDArray. Opening
         * and closing file descriptors will be visible to both processes.
         */
        np->fds = fds;
    } else {
        /**
         * Copy the file descriptors from the old process into the new
         * child process. The file descriptors entry can be opened and
         * closed independently of the other process being considered. The
         * host file descriptors are also dup'd so that the flags for the
         * host file descriptor is independent of the other process.
         */
        std::shared_ptr<FDArray> nfds = np->fds;
        for (int tgt_fd = 0; tgt_fd < fds->getSize(); tgt_fd++) {
            std::shared_ptr<FDEntry> this_fde = (*fds)[tgt_fd];
            if (!this_fde) {
                nfds->setFDEntry(tgt_fd, nullptr);
                continue;
            }
            nfds->setFDEntry(tgt_fd, this_fde->clone());

            auto this_hbfd = std::dynamic_pointer_cast<HBFDEntry>(this_fde);
            if (!this_hbfd)
                continue;

            int this_sim_fd = this_hbfd->getSimFD();
            if (this_sim_fd <= 2)
                continue;

            int np_sim_fd = dup(this_sim_fd);
            assert(np_sim_fd != -1);

            auto nhbfd = std::dynamic_pointer_cast<HBFDEntry>((*nfds)[tgt_fd]);
            nhbfd->setSimFD(np_sim_fd);
        }
    }

    if (CLONE_THREAD & flags) {
        np->_tgid = _tgid;
        delete np->exitGroup;
        np->exitGroup = exitGroup;
    }

    if (CLONE_VFORK & flags) {
        np->vforkContexts.push_back(otc->contextId());
    }

    np->argv.insert(np->argv.end(), argv.begin(), argv.end());
    np->envp.insert(np->envp.end(), envp.begin(), envp.end());
}

void
Process::revokeThreadContext(int context_id)
{
    std::vector<ContextID>::iterator it;
    for (it = contextIds.begin(); it != contextIds.end(); it++) {
        if (*it == context_id) {
            contextIds.erase(it);
            return;
        }
    }
    warn("Unable to find thread context to revoke");
}

void
Process::init()
{
    // Patch the ld_bias for dynamic executables.
    updateBias();

    if (objFile->getInterpreter())
        interpImage = objFile->getInterpreter()->buildImage();
}

void
Process::initState()
{
    if (contextIds.empty())
        fatal("Process %s is not associated with any HW contexts!\n", name());

    // first thread context for this process... initialize & enable
    ThreadContext *tc = system->threads[contextIds[0]];

    // mark this context as active so it will start ticking.
    tc->activate();

    pTable->initState();

    initVirtMem.reset(new SETranslatingPortProxy(
                tc, SETranslatingPortProxy::Always));

    // load object file into target memory
    image.write(*initVirtMem);
    interpImage.write(*initVirtMem);
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
    const auto page_size = pTable->pageSize();

    const Addr page_addr = roundDown(vaddr, page_size);

    // Check if the page has been mapped by other cores if not to clobber.
    // When running multithreaded programs in SE-mode with DerivO3CPU model,
    // there are cases where two or more cores have page faults on the same
    // page in nearby ticks. When the cores try to handle the faults at the
    // commit stage (also in nearby ticks/cycles), the first core will ask for
    // a physical page frame to map with the virtual page. Other cores can
    // return if the page has been mapped and `!clobber`.
    if (!clobber) {
        const EmulationPageTable::Entry *pte = pTable->lookup(page_addr);
        if (pte) {
            warn("Process::allocateMem: addr %#x already mapped\n", vaddr);
            return;
        }
    }

    const int npages = divCeil(size, page_size);
    const Addr paddr = seWorkload->allocPhysPages(npages);
    const Addr pages_size = npages * page_size;
    pTable->map(page_addr, paddr, pages_size,
                clobber ? EmulationPageTable::Clobber :
                          EmulationPageTable::MappingFlags(0));
}

void
Process::replicatePage(Addr vaddr, Addr new_paddr, ThreadContext *old_tc,
                       ThreadContext *new_tc, bool allocate_page)
{
    if (allocate_page)
        new_paddr = seWorkload->allocPhysPages(1);

    // Read from old physical page.
    uint8_t buf_p[pTable->pageSize()];
    SETranslatingPortProxy(old_tc).readBlob(vaddr, buf_p, sizeof(buf_p));

    // Create new mapping in process address space by clobbering existing
    // mapping (if any existed) and then write to the new physical page.
    bool clobber = true;
    pTable->map(vaddr, new_paddr, sizeof(buf_p), clobber);
    SETranslatingPortProxy(new_tc).writeBlob(vaddr, buf_p, sizeof(buf_p));
}

bool
Process::fixupFault(Addr vaddr)
{
    return memState->fixupFault(vaddr);
}

void
Process::serialize(CheckpointOut &cp) const
{
    memState->serialize(cp);
    pTable->serialize(cp);
    fds->serialize(cp);

    /**
     * Checkpoints for pipes, device drivers or sockets currently
     * do not work. Need to come back and fix them at a later date.
     */

    warn("Checkpoints for pipes, device drivers and sockets do not work.");
}

void
Process::unserialize(CheckpointIn &cp)
{
    memState->unserialize(cp);
    pTable->unserialize(cp);
    fds->unserialize(cp, this);

    /**
     * Checkpoints for pipes, device drivers or sockets currently
     * do not work. Need to come back and fix them at a later date.
     */
    warn("Checkpoints for pipes, device drivers and sockets do not work.");
    // The above returns a bool so that you could do something if you don't
    // find the param in the checkpoint if you wanted to, like set a default
    // but in this case we'll just stick with the instantiated value if not
    // found.
}

bool
Process::map(Addr vaddr, Addr paddr, int64_t size, bool cacheable)
{
    pTable->map(vaddr, paddr, size,
                cacheable ? EmulationPageTable::MappingFlags(0) :
                            EmulationPageTable::Uncacheable);
    return true;
}

EmulatedDriver *
Process::findDriver(std::string filename)
{
    for (EmulatedDriver *d : drivers) {
        if (d->match(filename))
            return d;
    }

    return nullptr;
}

std::string
Process::checkPathRedirect(const std::string &filename)
{
    // If the input parameter contains a relative path, convert it.
    // The target version of the current working directory is fine since
    // we immediately convert it using redirect paths into a host version.
    auto abs_path = absolutePath(filename, false);

    for (auto path : system->redirectPaths) {
        // Search through the redirect paths to see if a starting substring of
        // our path falls into any buckets which need to redirected.
        if (startswith(abs_path, path->appPath())) {
            std::string tail = abs_path.substr(path->appPath().size());

            // If this path needs to be redirected, search through a list
            // of targets to see if we can match a valid file (or directory).
            for (auto host_path : path->hostPaths()) {
                if (access((host_path + tail).c_str(), R_OK) == 0) {
                    // Return the valid match.
                    return host_path + tail;
                }
            }
            // The path needs to be redirected, but the file or directory
            // does not exist on the host filesystem. Return the first
            // host path as a default.
            return path->hostPaths()[0] + tail;
        }
    }

    // The path does not need to be redirected.
    return abs_path;
}

void
Process::updateBias()
{
    auto *interp = objFile->getInterpreter();

    if (!interp || !interp->relocatable())
        return;

    // Determine how large the interpreters footprint will be in the process
    // address space.
    Addr interp_mapsize = roundUp(interp->mapSize(), pTable->pageSize());

    // We are allocating the memory area; set the bias to the lowest address
    // in the allocated memory region.
    Addr mmap_end = memState->getMmapEnd();
    Addr ld_bias = mmapGrowsDown() ? mmap_end - interp_mapsize : mmap_end;

    // Adjust the process mmap area to give the interpreter room; the real
    // execve system call would just invoke the kernel's internal mmap
    // functions to make these adjustments.
    mmap_end = mmapGrowsDown() ? ld_bias : mmap_end + interp_mapsize;
    memState->setMmapEnd(mmap_end);

    interp->updateBias(ld_bias);
}

loader::ObjectFile *
Process::getInterpreter()
{
    return objFile->getInterpreter();
}

Addr
Process::getBias()
{
    auto *interp = getInterpreter();

    return interp ? interp->bias() : objFile->bias();
}

Addr
Process::getStartPC()
{
    auto *interp = getInterpreter();

    return interp ? interp->entryPoint() : objFile->entryPoint();
}

std::string
Process::absolutePath(const std::string &filename, bool host_filesystem)
{
    if (filename.empty() || startswith(filename, "/"))
        return filename;

    // Construct the absolute path given the current working directory for
    // either the host filesystem or target filesystem. The distinction only
    // matters if filesystem redirection is utilized in the simulation.
    auto path_base = std::string();
    if (host_filesystem) {
        path_base = hostCwd;
        assert(!hostCwd.empty());
    } else {
        path_base = tgtCwd;
        assert(!tgtCwd.empty());
    }

    // Add a trailing '/' if the current working directory did not have one.
    path_base = normalize(path_base);

    // Append the filename onto the current working path.
    auto absolute_path = path_base + filename;

    return absolute_path;
}

Process *
ProcessParams::create() const
{
    // If not specified, set the executable parameter equal to the
    // simulated system's zeroth command line parameter
    const std::string &exec = (executable == "") ? cmd[0] : executable;

    auto *obj_file = loader::createObjectFile(exec);
    fatal_if(!obj_file, "Cannot load object file %s.", exec);

    Process *process = Process::tryLoaders(*this, obj_file);
    fatal_if(!process, "Unknown error creating process object.");

    return process;
}

} // namespace gem5
