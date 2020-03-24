/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "sim/syscall_emul.hh"

#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>

#include "arch/utility.hh"
#include "base/chunk_generator.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "dev/net/dist_iface.hh"
#include "mem/page_table.hh"
#include "sim/byteswap.hh"
#include "sim/process.hh"
#include "sim/sim_exit.hh"
#include "sim/syscall_debug_macros.hh"
#include "sim/syscall_desc.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

void
warnUnsupportedOS(std::string syscall_name)
{
    warn("Cannot invoke %s on host operating system.", syscall_name);
}

SyscallReturn
unimplementedFunc(SyscallDesc *desc, ThreadContext *tc)
{
    fatal("syscall %s (#%d) unimplemented.", desc->name(), desc->num());
}


SyscallReturn
ignoreFunc(SyscallDesc *desc, ThreadContext *tc)
{
    warn("ignoring syscall %s(...)", desc->name());
    return 0;
}

SyscallReturn
ignoreWarnOnceFunc(SyscallDesc *desc, ThreadContext *tc)
{
    static std::unordered_map<SyscallDesc *, bool> bool_map;

    bool &warned = bool_map[desc];
    if (!warned) {
        warn("ignoring syscall %s(...)\n"
             "      (further warnings will be suppressed)", desc->name());
        warned = true;
    }

    return 0;
}

static void
exitFutexWake(ThreadContext *tc, Addr addr, uint64_t tgid)
{
    // Clear value at address pointed to by thread's childClearTID field.
    BufferArg ctidBuf(addr, sizeof(long));
    long *ctid = (long *)ctidBuf.bufferPtr();
    *ctid = 0;
    ctidBuf.copyOut(tc->getVirtProxy());

    FutexMap &futex_map = tc->getSystemPtr()->futexMap;
    // Wake one of the waiting threads.
    futex_map.wakeup(addr, tgid, 1);
}

static SyscallReturn
exitImpl(SyscallDesc *desc, ThreadContext *tc, bool group, int status)
{
    auto p = tc->getProcessPtr();

    System *sys = tc->getSystemPtr();

    if (group)
        *p->exitGroup = true;

    if (p->childClearTID)
        exitFutexWake(tc, p->childClearTID, p->tgid());

    bool last_thread = true;
    Process *parent = nullptr, *tg_lead = nullptr;
    for (int i = 0; last_thread && i < sys->numContexts(); i++) {
        Process *walk;
        if (!(walk = sys->threadContexts[i]->getProcessPtr()))
            continue;

        /**
         * Threads in a thread group require special handing. For instance,
         * we send the SIGCHLD signal so that it appears that it came from
         * the head of the group. We also only delete file descriptors if
         * we are the last thread in the thread group.
         */
        if (walk->pid() == p->tgid())
            tg_lead = walk;

        if ((sys->threadContexts[i]->status() != ThreadContext::Halted) &&
            (sys->threadContexts[i]->status() != ThreadContext::Halting) &&
            (walk != p)) {
            /**
             * Check if we share thread group with the pointer; this denotes
             * that we are not the last thread active in the thread group.
             * Note that setting this to false also prevents further
             * iterations of the loop.
             */
            if (walk->tgid() == p->tgid()) {
                /**
                 * If p is trying to exit_group and both walk and p are in
                 * the same thread group (i.e., sharing the same tgid),
                 * we need to halt walk's thread context. After all threads
                 * except p are halted, p becomes the last thread in the
                 * group.
                 *
                 * If p is not doing exit_group and there exists another
                 * active thread context in the group, last_thread is
                 * set to false to prevent the parent thread from killing
                 * all threads in the group.
                 */
                if (*(p->exitGroup)) {
                    sys->threadContexts[i]->halt();
                } else {
                    last_thread = false;
                }
            }

            /**
             * A corner case exists which involves execve(). After execve(),
             * the execve will enable SIGCHLD in the process. The problem
             * occurs when the exiting process is the root process in the
             * system; there is no parent to receive the signal. We obviate
             * this problem by setting the root process' ppid to zero in the
             * Python configuration files. We really should handle the
             * root/execve specific case more gracefully.
             */
            if (*p->sigchld && (p->ppid() != 0) && (walk->pid() == p->ppid()))
                parent = walk;
        }
    }

    if (last_thread) {
        if (parent) {
            assert(tg_lead);
            sys->signalList.push_back(BasicSignal(tg_lead, parent, SIGCHLD));
        }

        /**
         * Run though FD array of the exiting process and close all file
         * descriptors except for the standard file descriptors.
         * (The standard file descriptors are shared with gem5.)
         */
        for (int i = 0; i < p->fds->getSize(); i++) {
            if ((*p->fds)[i])
                p->fds->closeFDEntry(i);
        }
    }

    tc->halt();

    /**
     * check to see if there is no more active thread in the system. If so,
     * exit the simulation loop
     */
    int activeContexts = 0;
    for (auto &system: sys->systemList)
        activeContexts += system->numRunningContexts();

    if (activeContexts == 0) {
        /**
         * Even though we are terminating the final thread context, dist-gem5
         * requires the simulation to remain active and provide
         * synchronization messages to the switch process. So we just halt
         * the last thread context and return. The simulation will be
         * terminated by dist-gem5 in a coordinated manner once all nodes
         * have signaled their readiness to exit. For non dist-gem5
         * simulations, readyToExit() always returns true.
         */
        if (!DistIface::readyToExit(0)) {
            return status;
        }

        exitSimLoop("exiting with last active thread context", status & 0xff);
        return status;
    }

    return status;
}

SyscallReturn
exitFunc(SyscallDesc *desc, ThreadContext *tc, int status)
{
    return exitImpl(desc, tc, false, status);
}

SyscallReturn
exitGroupFunc(SyscallDesc *desc, ThreadContext *tc, int status)
{
    return exitImpl(desc, tc, true, status);
}

SyscallReturn
getpagesizeFunc(SyscallDesc *desc, ThreadContext *tc)
{
    return (int)PageBytes;
}


SyscallReturn
brkFunc(SyscallDesc *desc, ThreadContext *tc, Addr new_brk)
{
    // change brk addr to first arg
    auto p = tc->getProcessPtr();

    std::shared_ptr<MemState> mem_state = p->memState;
    Addr brk_point = mem_state->getBrkPoint();

    // in Linux at least, brk(0) returns the current break value
    // (note that the syscall and the glibc function have different behavior)
    if (new_brk == 0 || (new_brk == brk_point))
        return brk_point;

    mem_state->updateBrkRegion(brk_point, new_brk);

    DPRINTF_SYSCALL(Verbose, "brk: break point changed to: %#X\n",
                    mem_state->getBrkPoint());

    return mem_state->getBrkPoint();
}

SyscallReturn
setTidAddressFunc(SyscallDesc *desc, ThreadContext *tc, uint64_t tidPtr)
{
    auto process = tc->getProcessPtr();

    process->childClearTID = tidPtr;
    return process->pid();
}

SyscallReturn
closeFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd)
{
    auto p = tc->getProcessPtr();
    return p->fds->closeFDEntry(tgt_fd);
}

SyscallReturn
lseekFunc(SyscallDesc *desc, ThreadContext *tc,
          int tgt_fd, uint64_t offs, int whence)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    off_t result = lseek(sim_fd, offs, whence);

    return (result == (off_t)-1) ? -errno : result;
}


SyscallReturn
_llseekFunc(SyscallDesc *desc, ThreadContext *tc,
            int tgt_fd, uint64_t offset_high, uint32_t offset_low,
            Addr result_ptr, int whence)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    uint64_t offset = (offset_high << 32) | offset_low;

    uint64_t result = lseek(sim_fd, offset, whence);
    result = htog(result, tc->getSystemPtr()->getGuestByteOrder());

    if (result == (off_t)-1)
        return -errno;
    // Assuming that the size of loff_t is 64 bits on the target platform
    BufferArg result_buf(result_ptr, sizeof(result));
    memcpy(result_buf.bufferPtr(), &result, sizeof(result));
    result_buf.copyOut(tc->getVirtProxy());
    return 0;
}


SyscallReturn
munmapFunc(SyscallDesc *desc, ThreadContext *tc, Addr start, size_t length)
{
    // Even if the system is currently not capable of recycling physical
    // pages, there is no reason we can't unmap them so that we trigger
    // appropriate seg faults when the application mistakenly tries to
    // access them again.
    auto p = tc->getProcessPtr();

    if (start & (tc->getSystemPtr()->getPageBytes() - 1) || !length) {
        return -EINVAL;
    }

    length = roundUp(length, tc->getSystemPtr()->getPageBytes());

    p->memState->unmapRegion(start, length);

    return 0;
}


const char *hostname = "m5.eecs.umich.edu";

SyscallReturn
gethostnameFunc(SyscallDesc *desc, ThreadContext *tc,
                Addr buf_ptr, int name_len)
{
    BufferArg name(buf_ptr, name_len);
    strncpy((char *)name.bufferPtr(), hostname, name_len);
    name.copyOut(tc->getVirtProxy());
    return 0;
}

SyscallReturn
getcwdFunc(SyscallDesc *desc, ThreadContext *tc,
           Addr buf_ptr, unsigned long size)
{
    int result = 0;
    auto p = tc->getProcessPtr();
    BufferArg buf(buf_ptr, size);

    // Is current working directory defined?
    string cwd = p->tgtCwd;
    if (!cwd.empty()) {
        if (cwd.length() >= size) {
            // Buffer too small
            return -ERANGE;
        }
        strncpy((char *)buf.bufferPtr(), cwd.c_str(), size);
        result = cwd.length();
    } else {
        if (getcwd((char *)buf.bufferPtr(), size)) {
            result = strlen((char *)buf.bufferPtr());
        } else {
            result = -1;
        }
    }

    buf.copyOut(tc->getVirtProxy());

    return (result == -1) ? -errno : result;
}

SyscallReturn
readlinkFunc(SyscallDesc *desc, ThreadContext *tc,
             Addr pathname, Addr buf_ptr, size_t bufsiz)
{
    string path;
    auto p = tc->getProcessPtr();

    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = p->checkPathRedirect(path);

    BufferArg buf(buf_ptr, bufsiz);

    int result = -1;
    if (path != "/proc/self/exe") {
        result = readlink(path.c_str(), (char *)buf.bufferPtr(), bufsiz);
    } else {
        // Emulate readlink() called on '/proc/self/exe' should return the
        // absolute path of the binary running in the simulated system (the
        // Process' executable). It is possible that using this path in
        // the simulated system will result in unexpected behavior if:
        //  1) One binary runs another (e.g., -c time -o "my_binary"), and
        //     called binary calls readlink().
        //  2) The host's full path to the running benchmark changes from one
        //     simulation to another. This can result in different simulated
        //     performance since the simulated system will process the binary
        //     path differently, even if the binary itself does not change.

        // Get the absolute canonical path to the running application
        char real_path[PATH_MAX];
        char *check_real_path = realpath(p->progName(), real_path);
        if (!check_real_path) {
            fatal("readlink('/proc/self/exe') unable to resolve path to "
                  "executable: %s", p->progName());
        }
        strncpy((char*)buf.bufferPtr(), real_path, bufsiz);
        size_t real_path_len = strlen(real_path);
        if (real_path_len > bufsiz) {
            // readlink will truncate the contents of the
            // path to ensure it is no more than bufsiz
            result = bufsiz;
        } else {
            result = real_path_len;
        }

        // Issue a warning about potential unexpected results
        warn_once("readlink() called on '/proc/self/exe' may yield unexpected "
                  "results in various settings.\n      Returning '%s'\n",
                  (char*)buf.bufferPtr());
    }

    buf.copyOut(tc->getVirtProxy());

    return (result == -1) ? -errno : result;
}

SyscallReturn
unlinkFunc(SyscallDesc *desc, ThreadContext *tc, Addr pathname)
{
    string path;
    auto p = tc->getProcessPtr();

    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    path = p->checkPathRedirect(path);

    int result = unlink(path.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
linkFunc(SyscallDesc *desc, ThreadContext *tc,
         Addr pathname, Addr new_pathname)
{
    string path;
    string new_path;
    auto p = tc->getProcessPtr();

    auto &virt_mem = tc->getVirtProxy();
    if (!virt_mem.tryReadString(path, pathname))
        return -EFAULT;
    if (!virt_mem.tryReadString(new_path, new_pathname))
        return -EFAULT;

    path = p->absolutePath(path, true);
    new_path = p->absolutePath(new_path, true);

    int result = link(path.c_str(), new_path.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
symlinkFunc(SyscallDesc *desc, ThreadContext *tc,
            Addr pathname, Addr new_pathname)
{
    string path;
    string new_path;
    auto p = tc->getProcessPtr();

    auto &virt_mem = tc->getVirtProxy();
    if (!virt_mem.tryReadString(path, pathname))
        return -EFAULT;
    if (!virt_mem.tryReadString(new_path, new_pathname))
        return -EFAULT;

    path = p->absolutePath(path, true);
    new_path = p->absolutePath(new_path, true);

    int result = symlink(path.c_str(), new_path.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
mkdirFunc(SyscallDesc *desc, ThreadContext *tc, Addr pathname, mode_t mode)
{
    auto p = tc->getProcessPtr();
    std::string path;
    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    path = p->checkPathRedirect(path);

    auto result = mkdir(path.c_str(), mode);
    return (result == -1) ? -errno : result;
}

SyscallReturn
renameFunc(SyscallDesc *desc, ThreadContext *tc, Addr oldpath, Addr newpath)
{
    auto p = tc->getProcessPtr();

    string old_name;
    if (!tc->getVirtProxy().tryReadString(old_name, oldpath))
        return -EFAULT;

    string new_name;
    if (!tc->getVirtProxy().tryReadString(new_name, newpath))
        return -EFAULT;

    // Adjust path for cwd and redirection
    old_name = p->checkPathRedirect(old_name);
    new_name = p->checkPathRedirect(new_name);

    int64_t result = rename(old_name.c_str(), new_name.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
truncateFunc(SyscallDesc *desc, ThreadContext *tc, Addr pathname, off_t length)
{
    string path;
    auto p = tc->getProcessPtr();

    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = p->checkPathRedirect(path);

    int result = truncate(path.c_str(), length);
    return (result == -1) ? -errno : result;
}

SyscallReturn
ftruncateFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd, off_t length)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    int result = ftruncate(sim_fd, length);
    return (result == -1) ? -errno : result;
}

SyscallReturn
truncate64Func(SyscallDesc *desc, ThreadContext *tc,
               Addr pathname, int64_t length)
{
    auto process = tc->getProcessPtr();
    string path;

    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = process->checkPathRedirect(path);

#if NO_STAT64
    int result = truncate(path.c_str(), length);
#else
    int result = truncate64(path.c_str(), length);
#endif
    return (result == -1) ? -errno : result;
}

SyscallReturn
ftruncate64Func(SyscallDesc *desc, ThreadContext *tc,
                int tgt_fd, int64_t length)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

#if NO_STAT64
    int result = ftruncate(sim_fd, length);
#else
    int result = ftruncate64(sim_fd, length);
#endif
    return (result == -1) ? -errno : result;
}

SyscallReturn
umaskFunc(SyscallDesc *desc, ThreadContext *tc)
{
    // Letting the simulated program change the simulator's umask seems like
    // a bad idea.  Compromise by just returning the current umask but not
    // changing anything.
    mode_t oldMask = umask(0);
    umask(oldMask);
    return (int)oldMask;
}

SyscallReturn
chownFunc(SyscallDesc *desc, ThreadContext *tc,
          Addr pathname, uint32_t owner, uint32_t group)
{
    string path;
    auto p = tc->getProcessPtr();

    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    /* XXX endianess */
    uid_t hostOwner = owner;
    gid_t hostGroup = group;

    // Adjust path for cwd and redirection
    path = p->checkPathRedirect(path);

    int result = chown(path.c_str(), hostOwner, hostGroup);
    return (result == -1) ? -errno : result;
}

SyscallReturn
fchownFunc(SyscallDesc *desc, ThreadContext *tc,
           int tgt_fd, uint32_t owner, uint32_t group)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    /* XXX endianess */
    uid_t hostOwner = owner;
    gid_t hostGroup = group;

    int result = fchown(sim_fd, hostOwner, hostGroup);
    return (result == -1) ? -errno : result;
}

/**
 * FIXME: The file description is not shared among file descriptors created
 * with dup. Really, it's difficult to maintain fields like file offset or
 * flags since an update to such a field won't be reflected in the metadata
 * for the fd entries that we maintain for checkpoint restoration.
 */
SyscallReturn
dupFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd)
{
    auto p = tc->getProcessPtr();

    auto old_hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!old_hbfdp)
        return -EBADF;
    int sim_fd = old_hbfdp->getSimFD();

    int result = dup(sim_fd);
    if (result == -1)
        return -errno;

    auto new_hbfdp = std::dynamic_pointer_cast<HBFDEntry>(old_hbfdp->clone());
    new_hbfdp->setSimFD(result);
    new_hbfdp->setCOE(false);
    return p->fds->allocFD(new_hbfdp);
}

SyscallReturn
dup2Func(SyscallDesc *desc, ThreadContext *tc, int old_tgt_fd, int new_tgt_fd)
{
    auto p = tc->getProcessPtr();
    auto old_hbp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[old_tgt_fd]);
    if (!old_hbp)
        return -EBADF;
    int old_sim_fd = old_hbp->getSimFD();

    /**
     * We need a valid host file descriptor number to be able to pass into
     * the second parameter for dup2 (newfd), but we don't know what the
     * viable numbers are; we execute the open call to retrieve one.
     */
    int res_fd = dup2(old_sim_fd, open("/dev/null", O_RDONLY));
    if (res_fd == -1)
        return -errno;

    auto new_hbp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[new_tgt_fd]);
    if (new_hbp)
        p->fds->closeFDEntry(new_tgt_fd);
    new_hbp = std::dynamic_pointer_cast<HBFDEntry>(old_hbp->clone());
    new_hbp->setSimFD(res_fd);
    new_hbp->setCOE(false);

    return p->fds->allocFD(new_hbp);
}

SyscallReturn
fcntlFunc(SyscallDesc *desc, ThreadContext *tc,
          int tgt_fd, int cmd, GuestABI::VarArgs<int> varargs)
{
    auto p = tc->getProcessPtr();

    auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!hbfdp)
        return -EBADF;
    int sim_fd = hbfdp->getSimFD();

    int coe = hbfdp->getCOE();

    switch (cmd) {
      case F_GETFD:
        return coe & FD_CLOEXEC;

      case F_SETFD: {
        int arg = varargs.get<int>();
        arg ? hbfdp->setCOE(true) : hbfdp->setCOE(false);
        return 0;
      }

      // Rely on the host to maintain the file status flags for this file
      // description rather than maintain it ourselves. Admittedly, this
      // is suboptimal (and possibly error prone), but it is difficult to
      // maintain the flags by tracking them across the different descriptors
      // (that refer to this file description) caused by clone, dup, and
      // subsequent fcntls.
      case F_GETFL:
      case F_SETFL: {
        int arg = varargs.get<int>();
        int rv = fcntl(sim_fd, cmd, arg);
        return (rv == -1) ? -errno : rv;
      }

      default:
        warn("fcntl: unsupported command %d\n", cmd);
        return 0;
    }
}

SyscallReturn
fcntl64Func(SyscallDesc *desc, ThreadContext *tc, int tgt_fd, int cmd)
{
    auto p = tc->getProcessPtr();

    auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!hbfdp)
        return -EBADF;
    int sim_fd = hbfdp->getSimFD();

    switch (cmd) {
      case 33: //F_GETLK64
        warn("fcntl64(%d, F_GETLK64) not supported, error returned\n", tgt_fd);
        return -EMFILE;

      case 34: // F_SETLK64
      case 35: // F_SETLKW64
        warn("fcntl64(%d, F_SETLK(W)64) not supported, error returned\n",
             tgt_fd);
        return -EMFILE;

      default:
        // not sure if this is totally valid, but we'll pass it through
        // to the underlying OS
        warn("fcntl64(%d, %d) passed through to host\n", tgt_fd, cmd);
        return fcntl(sim_fd, cmd);
    }
}

SyscallReturn
pipePseudoFunc(SyscallDesc *desc, ThreadContext *tc)
{
    return pipe2Func(desc, tc, 0, 0);
}

SyscallReturn
pipeFunc(SyscallDesc *desc, ThreadContext *tc, Addr tgt_addr)
{
    return pipe2Func(desc, tc, tgt_addr, 0);
}

SyscallReturn
pipe2Func(SyscallDesc *desc, ThreadContext *tc, Addr tgt_addr, int flags)
{
    auto p = tc->getProcessPtr();

    int sim_fds[2], tgt_fds[2];

    int pipe_retval = pipe(sim_fds);
    if (pipe_retval == -1)
        return -errno;

    auto rend = PipeFDEntry::EndType::read;
    auto rpfd = std::make_shared<PipeFDEntry>(sim_fds[0], O_WRONLY, rend);
    tgt_fds[0] = p->fds->allocFD(rpfd);
    int sim_fd_rpfd = rpfd->getSimFD();

    auto wend = PipeFDEntry::EndType::write;
    auto wpfd = std::make_shared<PipeFDEntry>(sim_fds[1], O_RDONLY, wend);
    tgt_fds[1] = p->fds->allocFD(wpfd);
    int sim_fd_wpfd = wpfd->getSimFD();

    /**
     * Now patch the read object to record the target file descriptor chosen
     * as the write end of the pipe.
     */
    rpfd->setPipeReadSource(tgt_fds[1]);

    /**
     * On some architectures, it's possible to use more than one register for
     * a return value. In those cases, pipe returns its values rather than
     * write them into a buffer.
     */
    if (tgt_addr == 0)
        return SyscallReturn(tgt_fds[0], tgt_fds[1]);

    /**
     * Copy the target file descriptors into buffer space and then copy
     * the buffer space back into the target address space.
     */
    BufferArg tgt_handle(tgt_addr, sizeof(int[2]));
    int *buf_ptr = (int*)tgt_handle.bufferPtr();
    buf_ptr[0] = tgt_fds[0];
    buf_ptr[1] = tgt_fds[1];
    tgt_handle.copyOut(tc->getVirtProxy());

    if (flags) {
        // pipe2 only uses O_NONBLOCK, O_CLOEXEC, and (O_NONBLOCK | O_CLOEXEC)
        // if flags set to anything else, return EINVAL
        if ((flags != O_CLOEXEC) && (flags != O_NONBLOCK) &&
            (flags != (O_CLOEXEC | O_NONBLOCK))) {
            return -EINVAL;
        }

        /*
          If O_NONBLOCK is passed in as a flag to pipe2, set O_NONBLOCK file
          status flag for two new open file descriptors.
        */
        if (flags & O_NONBLOCK) {
            /*
              O_NONBLOCK is set when the programmer wants to avoid a separate
              call(s) to fcntl in their code, so mirror the fcntl
              implementation for handling file descriptors -- rely on host to
              maintain file status flags.
            */
            if (fcntl(sim_fd_rpfd, F_SETFL, O_NONBLOCK)) {
                return -errno;
            }
            if (fcntl(sim_fd_wpfd, F_SETFL, O_NONBLOCK)) {
                return -errno;
            }
        }

        /*
          If O_CLOEXEC is passed in as a flag to pipe2, set close-on-exec
          (FD_CLOEXEC) file status flag for two new open file descriptors.
        */
        if (flags & O_CLOEXEC) {
            rpfd->setCOE(true);
            wpfd->setCOE(true);
        }
    }

    return 0;
}

SyscallReturn
getpgrpFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->pgid();
}

SyscallReturn
setpgidFunc(SyscallDesc *desc, ThreadContext *tc, int pid, int pgid)
{
    auto process = tc->getProcessPtr();

    if (pgid < 0)
        return -EINVAL;

    if (pid == 0) {
        process->pgid(process->pid());
        return 0;
    }

    Process *matched_ph = nullptr;
    System *sysh = tc->getSystemPtr();

    // Retrieves process pointer from active/suspended thread contexts.
    for (int i = 0; i < sysh->numContexts(); i++) {
        if (sysh->threadContexts[i]->status() != ThreadContext::Halted) {
            Process *temp_h = sysh->threadContexts[i]->getProcessPtr();
            Process *walk_ph = (Process*)temp_h;

            if (walk_ph && walk_ph->pid() == process->pid())
                matched_ph = walk_ph;
        }
    }

    assert(matched_ph);
    matched_ph->pgid((pgid == 0) ? matched_ph->pid() : pgid);

    return 0;
}


SyscallReturn
getpidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->tgid();
}

SyscallReturn
gettidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->pid();
}

SyscallReturn
getppidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->ppid();
}

SyscallReturn
getuidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->uid();              // UID
}

SyscallReturn
geteuidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->euid();             // UID
}

SyscallReturn
getgidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->gid();
}

SyscallReturn
getegidFunc(SyscallDesc *desc, ThreadContext *tc)
{
    auto process = tc->getProcessPtr();
    return process->egid();
}

SyscallReturn
fallocateFunc(SyscallDesc *desc, ThreadContext *tc,
              int tgt_fd, int mode, off_t offset, off_t len)
{
#if defined(__linux__)
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    int result = fallocate(sim_fd, mode, offset, len);
    if (result < 0)
        return -errno;
    return 0;
#else
    warnUnsupportedOS("fallocate");
    return -1;
#endif
}

SyscallReturn
accessFunc(SyscallDesc *desc, ThreadContext *tc,
           Addr pathname, mode_t mode)
{
    string path;
    auto p = tc->getProcessPtr();
    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = p->checkPathRedirect(path);

    int result = access(path.c_str(), mode);
    return (result == -1) ? -errno : result;
}

SyscallReturn
mknodFunc(SyscallDesc *desc, ThreadContext *tc,
          Addr pathname, mode_t mode, dev_t dev)
{
    auto p = tc->getProcessPtr();
    std::string path;
    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    path = p->checkPathRedirect(path);

    auto result = mknod(path.c_str(), mode, dev);
    return (result == -1) ? -errno : result;
}

SyscallReturn
chdirFunc(SyscallDesc *desc, ThreadContext *tc, Addr pathname)
{
    auto p = tc->getProcessPtr();
    std::string path;
    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    std::string tgt_cwd;
    if (startswith(path, "/")) {
        tgt_cwd = path;
    } else {
        char buf[PATH_MAX];
        tgt_cwd = realpath((p->tgtCwd + "/" + path).c_str(), buf);
    }
    std::string host_cwd = p->checkPathRedirect(tgt_cwd);

    int result = chdir(host_cwd.c_str());

    if (result == -1)
        return -errno;

    p->hostCwd = host_cwd;
    p->tgtCwd = tgt_cwd;
    return result;
}

SyscallReturn
rmdirFunc(SyscallDesc *desc, ThreadContext *tc, Addr pathname)
{
    auto p = tc->getProcessPtr();
    std::string path;
    if (!tc->getVirtProxy().tryReadString(path, pathname))
        return -EFAULT;

    path = p->checkPathRedirect(path);

    auto result = rmdir(path.c_str());
    return (result == -1) ? -errno : result;
}

#if defined(SYS_getdents) || defined(SYS_getdents64)
template<typename DE, int SYS_NUM>
static SyscallReturn
getdentsImpl(SyscallDesc *desc, ThreadContext *tc,
             int tgt_fd, Addr buf_ptr, unsigned count)
{
    auto p = tc->getProcessPtr();

    auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!hbfdp)
        return -EBADF;
    int sim_fd = hbfdp->getSimFD();

    BufferArg buf_arg(buf_ptr, count);
    auto status = syscall(SYS_NUM, sim_fd, buf_arg.bufferPtr(), count);

    if (status == -1)
        return -errno;

    unsigned traversed = 0;
    while (traversed < status) {
        DE *buffer = (DE*)((Addr)buf_arg.bufferPtr() + traversed);

        auto host_reclen = buffer->d_reclen;

        /**
         * Convert the byte ordering from the host to the target before
         * passing the data back into the target's address space to preserve
         * endianness.
         */
        const ByteOrder bo = tc->getSystemPtr()->getGuestByteOrder();
        buffer->d_ino = htog(buffer->d_ino, bo);
        buffer->d_off = htog(buffer->d_off, bo);
        buffer->d_reclen = htog(buffer->d_reclen, bo);

        traversed += host_reclen;
    }

    buf_arg.copyOut(tc->getVirtProxy());
    return status;
}
#endif

#if defined(SYS_getdents)
SyscallReturn
getdentsFunc(SyscallDesc *desc, ThreadContext *tc,
             int tgt_fd, Addr buf_ptr, unsigned count)
{
    typedef struct linux_dirent {
        unsigned long d_ino;
        unsigned long d_off;
        unsigned short d_reclen;
        char dname[];
    } LinDent;

    return getdentsImpl<LinDent, SYS_getdents>(desc, tc,
                                               tgt_fd, buf_ptr, count);
}
#endif

#if defined(SYS_getdents64)
SyscallReturn
getdents64Func(SyscallDesc *desc, ThreadContext *tc,
               int tgt_fd, Addr buf_ptr, unsigned count)
{
    typedef struct linux_dirent64 {
        ino64_t d_ino;
        off64_t d_off;
        unsigned short d_reclen;
        char dname[];
    } LinDent64;

    return getdentsImpl<LinDent64, SYS_getdents64>(desc, tc,
                                                   tgt_fd, buf_ptr, count);
}
#endif

SyscallReturn
shutdownFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd, int how)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    int retval = shutdown(sim_fd, how);

    return (retval == -1) ? -errno : retval;
}

SyscallReturn
bindFunc(SyscallDesc *desc, ThreadContext *tc,
         int tgt_fd, Addr buf_ptr, int addrlen)
{
    auto p = tc->getProcessPtr();

    BufferArg bufSock(buf_ptr, addrlen);
    bufSock.copyIn(tc->getVirtProxy());

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    int status = ::bind(sim_fd,
                        (struct sockaddr *)bufSock.bufferPtr(),
                        addrlen);

    return (status == -1) ? -errno : status;
}

SyscallReturn
listenFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd, int backlog)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    int status = listen(sim_fd, backlog);

    return (status == -1) ? -errno : status;
}

SyscallReturn
connectFunc(SyscallDesc *desc, ThreadContext *tc,
            int tgt_fd, Addr buf_ptr, int addrlen)
{
    auto p = tc->getProcessPtr();

    BufferArg addr(buf_ptr, addrlen);
    addr.copyIn(tc->getVirtProxy());

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    int status = connect(sim_fd,
                         (struct sockaddr *)addr.bufferPtr(),
                         (socklen_t)addrlen);

    return (status == -1) ? -errno : status;
}

SyscallReturn
recvfromFunc(SyscallDesc *desc, ThreadContext *tc,
             int tgt_fd, Addr bufrPtr, size_t bufrLen, int flags,
             Addr addrPtr, Addr addrlenPtr)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    // Reserve buffer space.
    BufferArg bufrBuf(bufrPtr, bufrLen);

    // Get address length.
    socklen_t addrLen = 0;
    if (addrlenPtr != 0) {
        // Read address length parameter.
        BufferArg addrlenBuf(addrlenPtr, sizeof(socklen_t));
        addrlenBuf.copyIn(tc->getVirtProxy());
        addrLen = *((socklen_t *)addrlenBuf.bufferPtr());
    }

    struct sockaddr sa, *sap = NULL;
    if (addrLen != 0) {
        BufferArg addrBuf(addrPtr, addrLen);
        addrBuf.copyIn(tc->getVirtProxy());
        memcpy(&sa, (struct sockaddr *)addrBuf.bufferPtr(),
               sizeof(struct sockaddr));
        sap = &sa;
    }

    ssize_t recvd_size = recvfrom(sim_fd,
                                  (void *)bufrBuf.bufferPtr(),
                                  bufrLen, flags, sap, (socklen_t *)&addrLen);

    if (recvd_size == -1)
        return -errno;

    // Pass the received data out.
    bufrBuf.copyOut(tc->getVirtProxy());

    // Copy address to addrPtr and pass it on.
    if (sap != NULL) {
        BufferArg addrBuf(addrPtr, addrLen);
        memcpy(addrBuf.bufferPtr(), sap, sizeof(sa));
        addrBuf.copyOut(tc->getVirtProxy());
    }

    // Copy len to addrlenPtr and pass it on.
    if (addrLen != 0) {
        BufferArg addrlenBuf(addrlenPtr, sizeof(socklen_t));
        *(socklen_t *)addrlenBuf.bufferPtr() = addrLen;
        addrlenBuf.copyOut(tc->getVirtProxy());
    }

    return recvd_size;
}

SyscallReturn
sendtoFunc(SyscallDesc *desc, ThreadContext *tc,
           int tgt_fd, Addr bufrPtr, size_t bufrLen, int flags,
           Addr addrPtr, socklen_t addrLen)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    // Reserve buffer space.
    BufferArg bufrBuf(bufrPtr, bufrLen);
    bufrBuf.copyIn(tc->getVirtProxy());

    struct sockaddr sa, *sap = nullptr;
    memset(&sa, 0, sizeof(sockaddr));
    if (addrLen != 0) {
        BufferArg addrBuf(addrPtr, addrLen);
        addrBuf.copyIn(tc->getVirtProxy());
        memcpy(&sa, (sockaddr*)addrBuf.bufferPtr(), addrLen);
        sap = &sa;
    }

    ssize_t sent_size = sendto(sim_fd,
                               (void *)bufrBuf.bufferPtr(),
                               bufrLen, flags, sap, (socklen_t)addrLen);

    return (sent_size == -1) ? -errno : sent_size;
}

SyscallReturn
recvmsgFunc(SyscallDesc *desc, ThreadContext *tc,
            int tgt_fd, Addr msgPtr, int flags)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

     /**
      *  struct msghdr {
      *     void         *msg_name;       // optional address
      *    socklen_t     msg_namelen;    // size of address
      *    struct iovec *msg_iov;        // iovec array
      *    size_t        msg_iovlen;     // number entries in msg_iov
      *    i                             // entries correspond to buffer
      *    void         *msg_control;    // ancillary data
      *    size_t        msg_controllen; // ancillary data buffer len
      *    int           msg_flags;      // flags on received message
      *  };
      *
      *  struct iovec {
      *    void  *iov_base;              // starting address
      *    size_t iov_len;               // number of bytes to transfer
      *  };
      */

    /**
     * The plan with this system call is to replace all of the pointers in the
     * structure and the substructure with BufferArg class pointers. We will
     * copy every field from the structures into our BufferArg classes.
     */
    BufferArg msgBuf(msgPtr, sizeof(struct msghdr));
    msgBuf.copyIn(tc->getVirtProxy());
    struct msghdr *msgHdr = (struct msghdr *)msgBuf.bufferPtr();

    /**
     * We will use these address place holders to retain the pointers which
     * we are going to replace with our own buffers in our simulator address
     * space.
     */
    Addr msg_name_phold = 0;
    Addr msg_iov_phold = 0;
    Addr iovec_base_phold[msgHdr->msg_iovlen];
    Addr msg_control_phold = 0;

    /**
     * Record msg_name pointer then replace with buffer pointer.
     */
    BufferArg *nameBuf = NULL;
    if (msgHdr->msg_name) {
        /*1*/msg_name_phold = (Addr)msgHdr->msg_name;
        /*2*/nameBuf = new BufferArg(msg_name_phold, msgHdr->msg_namelen);
        /*3*/nameBuf->copyIn(tc->getVirtProxy());
        /*4*/msgHdr->msg_name = nameBuf->bufferPtr();
    }

    /**
     * Record msg_iov pointer then replace with buffer pointer. Also, setup
     * an array of buffer pointers for the iovec structs record and replace
     * their pointers with buffer pointers.
     */
    BufferArg *iovBuf = NULL;
    BufferArg *iovecBuf[msgHdr->msg_iovlen];
    for (int i = 0; i < msgHdr->msg_iovlen; i++) {
        iovec_base_phold[i] = 0;
        iovecBuf[i] = NULL;
    }

    if (msgHdr->msg_iov) {
        /*1*/msg_iov_phold = (Addr)msgHdr->msg_iov;
        /*2*/iovBuf = new BufferArg(msg_iov_phold, msgHdr->msg_iovlen *
                                    sizeof(struct iovec));
        /*3*/iovBuf->copyIn(tc->getVirtProxy());
        for (int i = 0; i < msgHdr->msg_iovlen; i++) {
            if (((struct iovec *)iovBuf->bufferPtr())[i].iov_base) {
                /*1*/iovec_base_phold[i] =
                     (Addr)((struct iovec *)iovBuf->bufferPtr())[i].iov_base;
                /*2*/iovecBuf[i] = new BufferArg(iovec_base_phold[i],
                     ((struct iovec *)iovBuf->bufferPtr())[i].iov_len);
                /*3*/iovecBuf[i]->copyIn(tc->getVirtProxy());
                /*4*/((struct iovec *)iovBuf->bufferPtr())[i].iov_base =
                     iovecBuf[i]->bufferPtr();
            }
        }
        /*4*/msgHdr->msg_iov = (struct iovec *)iovBuf->bufferPtr();
    }

    /**
     * Record msg_control pointer then replace with buffer pointer.
     */
    BufferArg *controlBuf = NULL;
    if (msgHdr->msg_control) {
        /*1*/msg_control_phold = (Addr)msgHdr->msg_control;
        /*2*/controlBuf = new BufferArg(msg_control_phold,
                                        CMSG_ALIGN(msgHdr->msg_controllen));
        /*3*/controlBuf->copyIn(tc->getVirtProxy());
        /*4*/msgHdr->msg_control = controlBuf->bufferPtr();
    }

    ssize_t recvd_size = recvmsg(sim_fd, msgHdr, flags);

    if (recvd_size < 0)
        return -errno;

    if (msgHdr->msg_name) {
        nameBuf->copyOut(tc->getVirtProxy());
        delete(nameBuf);
        msgHdr->msg_name = (void *)msg_name_phold;
    }

    if (msgHdr->msg_iov) {
        for (int i = 0; i< msgHdr->msg_iovlen; i++) {
            if (((struct iovec *)iovBuf->bufferPtr())[i].iov_base) {
                iovecBuf[i]->copyOut(tc->getVirtProxy());
                delete iovecBuf[i];
                ((struct iovec *)iovBuf->bufferPtr())[i].iov_base =
                (void *)iovec_base_phold[i];
            }
        }
        iovBuf->copyOut(tc->getVirtProxy());
        delete iovBuf;
        msgHdr->msg_iov = (struct iovec *)msg_iov_phold;
    }

    if (msgHdr->msg_control) {
        controlBuf->copyOut(tc->getVirtProxy());
        delete(controlBuf);
        msgHdr->msg_control = (void *)msg_control_phold;
    }

    msgBuf.copyOut(tc->getVirtProxy());

    return recvd_size;
}

SyscallReturn
sendmsgFunc(SyscallDesc *desc, ThreadContext *tc,
            int tgt_fd, Addr msgPtr, int flags)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    /**
     * Reserve buffer space.
     */
    BufferArg msgBuf(msgPtr, sizeof(struct msghdr));
    msgBuf.copyIn(tc->getVirtProxy());
    struct msghdr msgHdr = *((struct msghdr *)msgBuf.bufferPtr());

    /**
     * Assuming msgHdr.msg_iovlen >= 1, then there is no point calling
     * recvmsg without a buffer.
     */
    struct iovec *iovPtr = msgHdr.msg_iov;
    BufferArg iovBuf((Addr)iovPtr, sizeof(struct iovec) * msgHdr.msg_iovlen);
    iovBuf.copyIn(tc->getVirtProxy());
    struct iovec *iov = (struct iovec *)iovBuf.bufferPtr();
    msgHdr.msg_iov = iov;

    /**
     * Cannot instantiate buffers till inside the loop.
     * Create array to hold buffer addresses, to be used during copyIn of
     * send data.
     */
    BufferArg **bufferArray = (BufferArg **)malloc(msgHdr.msg_iovlen
                                                   * sizeof(BufferArg *));

    /**
     * Iterate through the iovec structures:
     * Get the base buffer addreses, reserve iov_len amount of space for each.
     * Put the buf address into the bufferArray for later retrieval.
     */
    for (int iovIndex = 0 ; iovIndex < msgHdr.msg_iovlen; iovIndex++) {
        Addr basePtr = (Addr) iov[iovIndex].iov_base;
        bufferArray[iovIndex] = new BufferArg(basePtr, iov[iovIndex].iov_len);
        bufferArray[iovIndex]->copyIn(tc->getVirtProxy());
        iov[iovIndex].iov_base = bufferArray[iovIndex]->bufferPtr();
    }

    ssize_t sent_size = sendmsg(sim_fd, &msgHdr, flags);
    int local_errno = errno;

    /**
     * Free dynamically allocated memory.
     */
    for (int iovIndex = 0 ; iovIndex < msgHdr.msg_iovlen; iovIndex++) {
        BufferArg *baseBuf = ( BufferArg *)bufferArray[iovIndex];
        delete(baseBuf);
    }

    /**
     * Malloced above.
     */
    free(bufferArray);

    return (sent_size < 0) ? -local_errno : sent_size;
}

SyscallReturn
getsockoptFunc(SyscallDesc *desc, ThreadContext *tc,
               int tgt_fd, int level, int optname, Addr valPtr, Addr lenPtr)
{
    // union of all possible return value types from getsockopt
    union val {
        int i_val;
        long l_val;
        struct linger linger_val;
        struct timeval timeval_val;
    } val;

    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    socklen_t len = sizeof(val);
    int status = getsockopt(sim_fd, level, optname, &val, &len);

    if (status == -1)
        return -errno;

    // copy val to valPtr and pass it on
    BufferArg valBuf(valPtr, sizeof(val));
    memcpy(valBuf.bufferPtr(), &val, sizeof(val));
    valBuf.copyOut(tc->getVirtProxy());

    // copy len to lenPtr and pass  it on
    BufferArg lenBuf(lenPtr, sizeof(len));
    memcpy(lenBuf.bufferPtr(), &len, sizeof(len));
    lenBuf.copyOut(tc->getVirtProxy());

    return status;
}

SyscallReturn
getsocknameFunc(SyscallDesc *desc, ThreadContext *tc,
                int tgt_fd, Addr addrPtr, Addr lenPtr)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    // lenPtr is an in-out paramenter:
    // sending the address length in, conveying the final length out

    // Read in the value of len from the passed pointer.
    BufferArg lenBuf(lenPtr, sizeof(socklen_t));
    lenBuf.copyIn(tc->getVirtProxy());
    socklen_t len = *(socklen_t *)lenBuf.bufferPtr();

    struct sockaddr sa;
    int status = getsockname(sim_fd, &sa, &len);

    if (status == -1)
        return -errno;

    // Copy address to addrPtr and pass it on.
    BufferArg addrBuf(addrPtr, sizeof(sa));
    memcpy(addrBuf.bufferPtr(), &sa, sizeof(sa));
    addrBuf.copyOut(tc->getVirtProxy());

    // Copy len to lenPtr and pass  it on.
    *(socklen_t *)lenBuf.bufferPtr() = len;
    lenBuf.copyOut(tc->getVirtProxy());

    return status;
}

SyscallReturn
getpeernameFunc(SyscallDesc *desc, ThreadContext *tc,
                int tgt_fd, Addr sockAddrPtr, Addr addrlenPtr)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    BufferArg bufAddrlen(addrlenPtr, sizeof(unsigned));
    bufAddrlen.copyIn(tc->getVirtProxy());
    BufferArg bufSock(sockAddrPtr, *(unsigned *)bufAddrlen.bufferPtr());

    int retval = getpeername(sim_fd,
                             (struct sockaddr *)bufSock.bufferPtr(),
                             (unsigned *)bufAddrlen.bufferPtr());

    if (retval != -1) {
        bufSock.copyOut(tc->getVirtProxy());
        bufAddrlen.copyOut(tc->getVirtProxy());
    }

    return (retval == -1) ? -errno : retval;
}

SyscallReturn
setsockoptFunc(SyscallDesc *desc, ThreadContext *tc,
               int tgt_fd, int level, int optname, Addr valPtr, socklen_t len)
{
    auto p = tc->getProcessPtr();

    BufferArg valBuf(valPtr, len);
    valBuf.copyIn(tc->getVirtProxy());

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    int status = setsockopt(sim_fd, level, optname,
                            (struct sockaddr *)valBuf.bufferPtr(), len);

    return (status == -1) ? -errno : status;
}

