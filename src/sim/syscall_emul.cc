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
 *
 * Authors: Steve Reinhardt
 *          Ali Saidi
 */

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <string>

#include "arch/utility.hh"
#include "base/chunk_generator.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/SyscallBase.hh"
#include "debug/SyscallVerbose.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#include "sim/sim_exit.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

void
SyscallDesc::doSyscall(int callnum, LiveProcess *process, ThreadContext *tc)
{
    if (DTRACE(SyscallBase)) {
        int index = 0;
        IntReg arg[6] M5_VAR_USED;

        // we can't just put the calls to getSyscallArg() in the
        // DPRINTF arg list, because C++ doesn't guarantee their order
        for (int i = 0; i < 6; ++i)
            arg[i] = process->getSyscallArg(tc, index);

        // Linux supports up to six system call arguments through registers
        // so we want to print all six. Check to the relevant man page to
        // verify how many are actually used by a given system call.
        DPRINTF_SYSCALL(Base,
                        "%s called w/arguments %d, %d, %d, %d, %d, %d\n",
                        name, arg[0], arg[1], arg[2], arg[3], arg[4],
                        arg[5]);
    }

    SyscallReturn retval = (*funcPtr)(this, callnum, process, tc);

    if (retval.needsRetry()) {
        DPRINTF_SYSCALL(Base, "%s needs retry\n", name);
    } else {
        DPRINTF_SYSCALL(Base, "%s returns %d\n", name,
                        retval.encodedValue());
    }

    if (!(flags & SyscallDesc::SuppressReturnValue) && !retval.needsRetry())
        process->setSyscallReturn(tc, retval);
}


SyscallReturn
unimplementedFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
                  ThreadContext *tc)
{
    fatal("syscall %s (#%d) unimplemented.", desc->name, callnum);

    return 1;
}


SyscallReturn
ignoreFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    int index = 0;
    const char *extra_text = "";

    if (desc->warnOnce()) {
        if (desc->warned)
            return 0;

        desc->warned = true;
        extra_text = "\n      (further warnings will be suppressed)";
    }

    warn("ignoring syscall %s(%d, ...)%s", desc->name,
         process->getSyscallArg(tc, index), extra_text);

    return 0;
}


SyscallReturn
exitFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
         ThreadContext *tc)
{
    if (process->system->numRunningContexts() == 1) {
        // Last running context... exit simulator
        int index = 0;
        exitSimLoop("target called exit()",
                    process->getSyscallArg(tc, index) & 0xff);
    } else {
        // other running threads... just halt this one
        tc->halt();
    }

    return 1;
}


SyscallReturn
exitGroupFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
              ThreadContext *tc)
{
    // halt all threads belonging to this process
    for (auto i: process->contextIds) {
        process->system->getThreadContext(i)->halt();
    }

    if (!process->system->numRunningContexts()) {
        // all threads belonged to this process... exit simulator
        int index = 0;
        exitSimLoop("target called exit()",
                    process->getSyscallArg(tc, index) & 0xff);
    }

    return 1;
}


SyscallReturn
getpagesizeFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    return (int)PageBytes;
}


SyscallReturn
brkFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    // change brk addr to first arg
    int index = 0;
    Addr new_brk = p->getSyscallArg(tc, index);

    // in Linux at least, brk(0) returns the current break value
    // (note that the syscall and the glibc function have different behavior)
    if (new_brk == 0)
        return p->brk_point;

    if (new_brk > p->brk_point) {
        // might need to allocate some new pages
        for (ChunkGenerator gen(p->brk_point, new_brk - p->brk_point,
                                PageBytes); !gen.done(); gen.next()) {
            if (!p->pTable->translate(gen.addr()))
                p->allocateMem(roundDown(gen.addr(), PageBytes), PageBytes);

            // if the address is already there, zero it out
            else {
                uint8_t zero  = 0;
                SETranslatingPortProxy &tp = tc->getMemProxy();

                // split non-page aligned accesses
                Addr next_page = roundUp(gen.addr(), PageBytes);
                uint32_t size_needed = next_page - gen.addr();
                tp.memsetBlob(gen.addr(), zero, size_needed);
                if (gen.addr() + PageBytes > next_page &&
                    next_page < new_brk &&
                    p->pTable->translate(next_page))
                {
                    size_needed = PageBytes - size_needed;
                    tp.memsetBlob(next_page, zero, size_needed);
                }
            }
        }
    }

    p->brk_point = new_brk;
    DPRINTF_SYSCALL(Verbose, "brk: break point changed to: %#X\n",
                    p->brk_point);
    return p->brk_point;
}


SyscallReturn
closeFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = p->getSyscallArg(tc, index);

    int sim_fd = p->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    int status = 0;
    if (sim_fd > 2)
        status = close(sim_fd);
    if (status >= 0)
        p->resetFDEntry(tgt_fd);
    return status;
}


SyscallReturn
readFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = p->getSyscallArg(tc, index);
    Addr bufPtr = p->getSyscallArg(tc, index);
    int nbytes = p->getSyscallArg(tc, index);
    BufferArg bufArg(bufPtr, nbytes);

    int sim_fd = p->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    int bytes_read = read(sim_fd, bufArg.bufferPtr(), nbytes);

    if (bytes_read > 0)
        bufArg.copyOut(tc->getMemProxy());

    return bytes_read;
}

SyscallReturn
writeFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = p->getSyscallArg(tc, index);
    Addr bufPtr = p->getSyscallArg(tc, index);
    int nbytes = p->getSyscallArg(tc, index);
    BufferArg bufArg(bufPtr, nbytes);

    int sim_fd = p->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    bufArg.copyIn(tc->getMemProxy());

    int bytes_written = write(sim_fd, bufArg.bufferPtr(), nbytes);

    fsync(sim_fd);

    return bytes_written;
}


SyscallReturn
lseekFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = p->getSyscallArg(tc, index);
    uint64_t offs = p->getSyscallArg(tc, index);
    int whence = p->getSyscallArg(tc, index);

    int sim_fd = p->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    off_t result = lseek(sim_fd, offs, whence);

    return (result == (off_t)-1) ? -errno : result;
}


SyscallReturn
_llseekFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = p->getSyscallArg(tc, index);
    uint64_t offset_high = p->getSyscallArg(tc, index);
    uint32_t offset_low = p->getSyscallArg(tc, index);
    Addr result_ptr = p->getSyscallArg(tc, index);
    int whence = p->getSyscallArg(tc, index);

    int sim_fd = p->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    uint64_t offset = (offset_high << 32) | offset_low;

    uint64_t result = lseek(sim_fd, offset, whence);
    result = TheISA::htog(result);

    if (result == (off_t)-1)
        return -errno;
    // Assuming that the size of loff_t is 64 bits on the target platform
    BufferArg result_buf(result_ptr, sizeof(result));
    memcpy(result_buf.bufferPtr(), &result, sizeof(result));
    result_buf.copyOut(tc->getMemProxy());
    return 0;
}


SyscallReturn
munmapFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    // With mmap more fully implemented, it might be worthwhile to bite
    // the bullet and implement munmap. Should allow us to reuse simulated
    // memory.
    return 0;
}


const char *hostname = "m5.eecs.umich.edu";

SyscallReturn
gethostnameFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    int index = 0;
    Addr bufPtr = p->getSyscallArg(tc, index);
    int name_len = p->getSyscallArg(tc, index);
    BufferArg name(bufPtr, name_len);

    strncpy((char *)name.bufferPtr(), hostname, name_len);

    name.copyOut(tc->getMemProxy());

    return 0;
}

SyscallReturn
getcwdFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    int result = 0;
    int index = 0;
    Addr bufPtr = p->getSyscallArg(tc, index);
    unsigned long size = p->getSyscallArg(tc, index);
    BufferArg buf(bufPtr, size);

    // Is current working directory defined?
    string cwd = p->getcwd();
    if (!cwd.empty()) {
        if (cwd.length() >= size) {
            // Buffer too small
            return -ERANGE;
        }
        strncpy((char *)buf.bufferPtr(), cwd.c_str(), size);
        result = cwd.length();
    } else {
        if (getcwd((char *)buf.bufferPtr(), size) != NULL) {
            result = strlen((char *)buf.bufferPtr());
        } else {
            result = -1;
        }
    }

    buf.copyOut(tc->getMemProxy());

    return (result == -1) ? -errno : result;
}

/// Target open() handler.
SyscallReturn
readlinkFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
         ThreadContext *tc)
{
    return readlinkFunc(desc, callnum, process, tc, 0);
}

SyscallReturn
readlinkFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc,
        int index)
{
    string path;

    if (!tc->getMemProxy().tryReadString(path, p->getSyscallArg(tc, index)))
        return -EFAULT;

    // Adjust path for current working directory
    path = p->fullPath(path);

    Addr bufPtr = p->getSyscallArg(tc, index);
    size_t bufsiz = p->getSyscallArg(tc, index);

    BufferArg buf(bufPtr, bufsiz);

    int result = -1;
    if (path != "/proc/self/exe") {
        result = readlink(path.c_str(), (char *)buf.bufferPtr(), bufsiz);
    } else {
        // Emulate readlink() called on '/proc/self/exe' should return the
        // absolute path of the binary running in the simulated system (the
        // LiveProcess' executable). It is possible that using this path in
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

    buf.copyOut(tc->getMemProxy());

    return (result == -1) ? -errno : result;
}

SyscallReturn
unlinkFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    return unlinkHelper(desc, num, p, tc, 0);
}

SyscallReturn
unlinkHelper(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc,
           int index)
{
    string path;

    if (!tc->getMemProxy().tryReadString(path, p->getSyscallArg(tc, index)))
        return -EFAULT;

    // Adjust path for current working directory
    path = p->fullPath(path);

    int result = unlink(path.c_str());
    return (result == -1) ? -errno : result;
}


SyscallReturn
mkdirFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    string path;

    int index = 0;
    if (!tc->getMemProxy().tryReadString(path, p->getSyscallArg(tc, index)))
        return -EFAULT;

    // Adjust path for current working directory
    path = p->fullPath(path);

    mode_t mode = p->getSyscallArg(tc, index);

    int result = mkdir(path.c_str(), mode);
    return (result == -1) ? -errno : result;
}

SyscallReturn
renameFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    string old_name;

    int index = 0;
    if (!tc->getMemProxy().tryReadString(old_name, p->getSyscallArg(tc, index)))
        return -EFAULT;

    string new_name;

    if (!tc->getMemProxy().tryReadString(new_name, p->getSyscallArg(tc, index)))
        return -EFAULT;

    // Adjust path for current working directory
    old_name = p->fullPath(old_name);
    new_name = p->fullPath(new_name);

    int64_t result = rename(old_name.c_str(), new_name.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
truncateFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    string path;

    int index = 0;
    if (!tc->getMemProxy().tryReadString(path, p->getSyscallArg(tc, index)))
        return -EFAULT;

    off_t length = p->getSyscallArg(tc, index);

    // Adjust path for current working directory
    path = p->fullPath(path);

    int result = truncate(path.c_str(), length);
    return (result == -1) ? -errno : result;
}

SyscallReturn
ftruncateFunc(SyscallDesc *desc, int num,
              LiveProcess *process, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = process->getSyscallArg(tc, index);
    off_t length = process->getSyscallArg(tc, index);

    int sim_fd = process->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    int result = ftruncate(sim_fd, length);
    return (result == -1) ? -errno : result;
}

SyscallReturn
truncate64Func(SyscallDesc *desc, int num,
                LiveProcess *process, ThreadContext *tc)
{
    int index = 0;
    string path;

    if (!tc->getMemProxy().tryReadString(path, process->getSyscallArg(tc, index)))
       return -EFAULT;

    int64_t length = process->getSyscallArg(tc, index, 64);

    // Adjust path for current working directory
    path = process->fullPath(path);

#if NO_STAT64
    int result = truncate(path.c_str(), length);
#else
    int result = truncate64(path.c_str(), length);
#endif
    return (result == -1) ? -errno : result;
}

SyscallReturn
ftruncate64Func(SyscallDesc *desc, int num,
                LiveProcess *process, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = process->getSyscallArg(tc, index);
    int64_t length = process->getSyscallArg(tc, index, 64);

    int sim_fd = process->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

#if NO_STAT64
    int result = ftruncate(sim_fd, length);
#else
    int result = ftruncate64(sim_fd, length);
#endif
    return (result == -1) ? -errno : result;
}

SyscallReturn
umaskFunc(SyscallDesc *desc, int num, LiveProcess *process, ThreadContext *tc)
{
    // Letting the simulated program change the simulator's umask seems like
    // a bad idea.  Compromise by just returning the current umask but not
    // changing anything.
    mode_t oldMask = umask(0);
    umask(oldMask);
    return (int)oldMask;
}

SyscallReturn
chownFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    string path;

    int index = 0;
    if (!tc->getMemProxy().tryReadString(path, p->getSyscallArg(tc, index)))
        return -EFAULT;

    /* XXX endianess */
    uint32_t owner = p->getSyscallArg(tc, index);
    uid_t hostOwner = owner;
    uint32_t group = p->getSyscallArg(tc, index);
    gid_t hostGroup = group;

    // Adjust path for current working directory
    path = p->fullPath(path);

    int result = chown(path.c_str(), hostOwner, hostGroup);
    return (result == -1) ? -errno : result;
}

SyscallReturn
fchownFunc(SyscallDesc *desc, int num, LiveProcess *process, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = process->getSyscallArg(tc, index);

    int sim_fd = process->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    /* XXX endianess */
    uint32_t owner = process->getSyscallArg(tc, index);
    uid_t hostOwner = owner;
    uint32_t group = process->getSyscallArg(tc, index);
    gid_t hostGroup = group;

    int result = fchown(sim_fd, hostOwner, hostGroup);
    return (result == -1) ? -errno : result;
}


SyscallReturn
dupFunc(SyscallDesc *desc, int num, LiveProcess *process, ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = process->getSyscallArg(tc, index);

    int sim_fd = process->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    FDEntry *fde = process->getFDEntry(tgt_fd);

    int result = dup(sim_fd);
    return (result == -1) ? -errno :
        process->allocFD(result, fde->filename, fde->flags, fde->mode, false);
}


SyscallReturn
fcntlFunc(SyscallDesc *desc, int num, LiveProcess *process,
          ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = process->getSyscallArg(tc, index);

    int sim_fd = process->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    int cmd = process->getSyscallArg(tc, index);
    switch (cmd) {
      case 0: // F_DUPFD
        // if we really wanted to support this, we'd need to do it
        // in the target fd space.
        warn("fcntl(%d, F_DUPFD) not supported, error returned\n", tgt_fd);
        return -EMFILE;

      case 1: // F_GETFD (get close-on-exec flag)
      case 2: // F_SETFD (set close-on-exec flag)
        return 0;

      case 3: // F_GETFL (get file flags)
      case 4: // F_SETFL (set file flags)
        // not sure if this is totally valid, but we'll pass it through
        // to the underlying OS
        warn("fcntl(%d, %d) passed through to host\n", tgt_fd, cmd);
        return fcntl(sim_fd, cmd);
        // return 0;

      case 7: // F_GETLK  (get lock)
      case 8: // F_SETLK  (set lock)
      case 9: // F_SETLKW (set lock and wait)
        // don't mess with file locking... just act like it's OK
        warn("File lock call (fcntl(%d, %d)) ignored.\n", tgt_fd, cmd);
        return 0;

      default:
        warn("Unknown fcntl command %d\n", cmd);
        return 0;
    }
}

SyscallReturn
fcntl64Func(SyscallDesc *desc, int num, LiveProcess *process,
            ThreadContext *tc)
{
    int index = 0;
    int tgt_fd = process->getSyscallArg(tc, index);

    int sim_fd = process->getSimFD(tgt_fd);
    if (sim_fd < 0)
        return -EBADF;

    int cmd = process->getSyscallArg(tc, index);
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
        // return 0;
    }
}

SyscallReturn
pipePseudoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
         ThreadContext *tc)
{
    int fds[2], sim_fds[2];
    int pipe_retval = pipe(fds);

    if (pipe_retval < 0) {
        // error
        return pipe_retval;
    }

    sim_fds[0] = process->allocFD(fds[0], "PIPE-READ", O_WRONLY, -1, true);
    sim_fds[1] = process->allocFD(fds[1], "PIPE-WRITE", O_RDONLY, -1, true);

    process->setReadPipeSource(sim_fds[0], sim_fds[1]);
    // Alpha Linux convention for pipe() is that fd[0] is returned as
    // the return value of the function, and fd[1] is returned in r20.
    tc->setIntReg(SyscallPseudoReturnReg, sim_fds[1]);
    return sim_fds[0];
}


SyscallReturn
getpidPseudoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    // Make up a PID.  There's no interprocess communication in
    // fake_syscall mode, so there's no way for a process to know it's
    // not getting a unique value.

    tc->setIntReg(SyscallPseudoReturnReg, process->ppid());
    return process->pid();
}


SyscallReturn
getuidPseudoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    // Make up a UID and EUID... it shouldn't matter, and we want the
    // simulation to be deterministic.

    // EUID goes in r20.
    tc->setIntReg(SyscallPseudoReturnReg, process->euid()); //EUID
    return process->uid();              // UID
}


SyscallReturn
getgidPseudoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    // Get current group ID.  EGID goes in r20.
    tc->setIntReg(SyscallPseudoReturnReg, process->egid()); //EGID
    return process->gid();
}


SyscallReturn
setuidFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    // can't fathom why a benchmark would call this.
    int index = 0;
    warn("Ignoring call to setuid(%d)\n", process->getSyscallArg(tc, index));
    return 0;
}

SyscallReturn
getpidFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    // Make up a PID.  There's no interprocess communication in
    // fake_syscall mode, so there's no way for a process to know it's
    // not getting a unique value.

    tc->setIntReg(SyscallPseudoReturnReg, process->ppid()); //PID
    return process->pid();
}

SyscallReturn
getppidFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    return process->ppid();
}

SyscallReturn
getuidFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    return process->uid();              // UID
}

SyscallReturn
geteuidFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    return process->euid();             // UID
}

SyscallReturn
getgidFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    return process->gid();
}

SyscallReturn
getegidFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    return process->egid();
}


SyscallReturn
cloneFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
           ThreadContext *tc)
{
    int index = 0;
    IntReg flags = process->getSyscallArg(tc, index);
    IntReg newStack = process->getSyscallArg(tc, index);

    DPRINTF(SyscallVerbose, "In sys_clone:\n");
    DPRINTF(SyscallVerbose, " Flags=%llx\n", flags);
    DPRINTF(SyscallVerbose, " Child stack=%llx\n", newStack);


    if (flags != 0x10f00) {
        warn("This sys_clone implementation assumes flags "
             "CLONE_VM|CLONE_FS|CLONE_FILES|CLONE_SIGHAND|CLONE_THREAD "
             "(0x10f00), and may not work correctly with given flags "
             "0x%llx\n", flags);
    }

    ThreadContext* ctc; // child thread context
    if ( ( ctc = process->findFreeContext() ) != NULL ) {
        DPRINTF(SyscallVerbose, " Found unallocated thread context\n");

        ctc->clearArchRegs();

        // Arch-specific cloning code
        #if THE_ISA == ALPHA_ISA or THE_ISA == X86_ISA
            // Cloning the misc. regs for these archs is enough
            TheISA::copyMiscRegs(tc, ctc);
        #elif THE_ISA == SPARC_ISA
            TheISA::copyRegs(tc, ctc);

            // TODO: Explain what this code actually does :-)
            ctc->setIntReg(NumIntArchRegs + 6, 0);
            ctc->setIntReg(NumIntArchRegs + 4, 0);
            ctc->setIntReg(NumIntArchRegs + 3, NWindows - 2);
            ctc->setIntReg(NumIntArchRegs + 5, NWindows);
            ctc->setMiscReg(MISCREG_CWP, 0);
            ctc->setIntReg(NumIntArchRegs + 7, 0);
            ctc->setMiscRegNoEffect(MISCREG_TL, 0);
            ctc->setMiscReg(MISCREG_ASI, ASI_PRIMARY);

            for (int y = 8; y < 32; y++)
                ctc->setIntReg(y, tc->readIntReg(y));
        #elif THE_ISA == ARM_ISA
            TheISA::copyRegs(tc, ctc);
        #else
            fatal("sys_clone is not implemented for this ISA\n");
        #endif

        // Set up stack register
        ctc->setIntReg(TheISA::StackPointerReg, newStack);

        // Set up syscall return values in parent and child
        ctc->setIntReg(ReturnValueReg, 0); // return value, child

        // Alpha needs SyscallSuccessReg=0 in child
        #if THE_ISA == ALPHA_ISA
            ctc->setIntReg(TheISA::SyscallSuccessReg, 0);
        #endif

        // In SPARC/Linux, clone returns 0 on pseudo-return register if
        // parent, non-zero if child
        #if THE_ISA == SPARC_ISA
            tc->setIntReg(TheISA::SyscallPseudoReturnReg, 0);
            ctc->setIntReg(TheISA::SyscallPseudoReturnReg, 1);
        #endif

        ctc->pcState(tc->nextInstAddr());

        ctc->activate();

        // Should return nonzero child TID in parent's syscall return register,
        // but for our pthread library any non-zero value will work
        return 1;
    } else {
        fatal("Called sys_clone, but no unallocated thread contexts found!\n");
        return 0;
    }
}

SyscallReturn
accessFunc(SyscallDesc *desc, int callnum, LiveProcess *p, ThreadContext *tc,
        int index)
{
    string path;
    if (!tc->getMemProxy().tryReadString(path, p->getSyscallArg(tc, index)))
        return -EFAULT;

    // Adjust path for current working directory
    path = p->fullPath(path);

    mode_t mode = p->getSyscallArg(tc, index);

    int result = access(path.c_str(), mode);
    return (result == -1) ? -errno : result;
}

SyscallReturn
accessFunc(SyscallDesc *desc, int callnum, LiveProcess *p, ThreadContext *tc)
{
    return accessFunc(desc, callnum, p, tc, 0);
}

