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

#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <iostream>

#include "sim/syscall_emul.hh"
#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "cpu/base.hh"
#include "sim/process.hh"

#include "sim/sim_events.hh"

using namespace std;
using namespace TheISA;

void
SyscallDesc::doSyscall(int callnum, Process *process, ExecContext *xc)
{
    DPRINTFR(SyscallVerbose, "%s: syscall %s called\n",
             xc->getCpuPtr()->name(), name);

    SyscallReturn retval = (*funcPtr)(this, callnum, process, xc);

    DPRINTFR(SyscallVerbose, "%s: syscall %s returns %d\n",
             xc->getCpuPtr()->name(), name, retval.value());

    if (!(flags & SyscallDesc::SuppressReturnValue))
        xc->setSyscallReturn(retval);
}


SyscallReturn
unimplementedFunc(SyscallDesc *desc, int callnum, Process *process,
                  ExecContext *xc)
{
    fatal("syscall %s (#%d) unimplemented.", desc->name, callnum);
}


SyscallReturn
ignoreFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    warn("ignoring syscall %s(%d, %d, ...)", desc->name,
         xc->getSyscallArg(0), xc->getSyscallArg(1));

    return 0;
}


SyscallReturn
exitFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    new SimExitEvent("target called exit()", xc->getSyscallArg(0) & 0xff);

    return 1;
}


SyscallReturn
getpagesizeFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    return (int)VMPageSize;
}


SyscallReturn
obreakFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    // change brk addr to first arg
    Addr new_brk = xc->getSyscallArg(0);
    if (new_brk != 0)
    {
        p->brk_point = xc->getSyscallArg(0);
    }
    DPRINTF(SyscallVerbose, "Break Point changed to: %#X\n", p->brk_point);
    return p->brk_point;
}


SyscallReturn
closeFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    int target_fd = xc->getSyscallArg(0);
    int status = close(p->sim_fd(target_fd));
    if (status >= 0)
        p->free_fd(target_fd);
    return status;
}


SyscallReturn
readFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    int fd = p->sim_fd(xc->getSyscallArg(0));
    int nbytes = xc->getSyscallArg(2);
    BufferArg bufArg(xc->getSyscallArg(1), nbytes);

    int bytes_read = read(fd, bufArg.bufferPtr(), nbytes);

    if (bytes_read != -1)
        bufArg.copyOut(xc->getMemPort());

    return bytes_read;
}

SyscallReturn
writeFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    int fd = p->sim_fd(xc->getSyscallArg(0));
    int nbytes = xc->getSyscallArg(2);
    BufferArg bufArg(xc->getSyscallArg(1), nbytes);

    bufArg.copyIn(xc->getMemPort());

    int bytes_written = write(fd, bufArg.bufferPtr(), nbytes);

    fsync(fd);

    return bytes_written;
}


SyscallReturn
lseekFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    int fd = p->sim_fd(xc->getSyscallArg(0));
    uint64_t offs = xc->getSyscallArg(1);
    int whence = xc->getSyscallArg(2);

    off_t result = lseek(fd, offs, whence);

    return (result == (off_t)-1) ? -errno : result;
}


SyscallReturn
munmapFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    // given that we don't really implement mmap, munmap is really easy
    return 0;
}


const char *hostname = "m5.eecs.umich.edu";

SyscallReturn
gethostnameFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    int name_len = xc->getSyscallArg(1);
    BufferArg name(xc->getSyscallArg(0), name_len);

    strncpy((char *)name.bufferPtr(), hostname, name_len);

    name.copyOut(xc->getMemPort());

    return 0;
}

SyscallReturn
unlinkFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    string path;

    if (!xc->getMemPort()->tryReadStringFunctional(path, xc->getSyscallArg(0)))
        return (TheISA::IntReg)-EFAULT;

    int result = unlink(path.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
renameFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    string old_name;

    if (!xc->getMemPort()->tryReadStringFunctional(old_name, xc->getSyscallArg(0)))
        return -EFAULT;

    string new_name;

    if (!xc->getMemPort()->tryReadStringFunctional(new_name, xc->getSyscallArg(1)))
        return -EFAULT;

    int64_t result = rename(old_name.c_str(), new_name.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
truncateFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    string path;

    if (!xc->getMemPort()->tryReadStringFunctional(path, xc->getSyscallArg(0)))
        return -EFAULT;

    off_t length = xc->getSyscallArg(1);

    int result = truncate(path.c_str(), length);
    return (result == -1) ? -errno : result;
}

SyscallReturn
ftruncateFunc(SyscallDesc *desc, int num, Process *process, ExecContext *xc)
{
    int fd = process->sim_fd(xc->getSyscallArg(0));

    if (fd < 0)
        return -EBADF;

    off_t length = xc->getSyscallArg(1);

    int result = ftruncate(fd, length);
    return (result == -1) ? -errno : result;
}

SyscallReturn
chownFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    string path;

    if (!xc->getMemPort()->tryReadStringFunctional(path, xc->getSyscallArg(0)))
        return -EFAULT;

    /* XXX endianess */
    uint32_t owner = xc->getSyscallArg(1);
    uid_t hostOwner = owner;
    uint32_t group = xc->getSyscallArg(2);
    gid_t hostGroup = group;

    int result = chown(path.c_str(), hostOwner, hostGroup);
    return (result == -1) ? -errno : result;
}

SyscallReturn
fchownFunc(SyscallDesc *desc, int num, Process *process, ExecContext *xc)
{
    int fd = process->sim_fd(xc->getSyscallArg(0));

    if (fd < 0)
        return -EBADF;

    /* XXX endianess */
    uint32_t owner = xc->getSyscallArg(1);
    uid_t hostOwner = owner;
    uint32_t group = xc->getSyscallArg(2);
    gid_t hostGroup = group;

    int result = fchown(fd, hostOwner, hostGroup);
    return (result == -1) ? -errno : result;
}


SyscallReturn
fcntlFunc(SyscallDesc *desc, int num, Process *process,
          ExecContext *xc)
{
    int fd = xc->getSyscallArg(0);

    if (fd < 0 || process->sim_fd(fd) < 0)
        return -EBADF;

    int cmd = xc->getSyscallArg(1);
    switch (cmd) {
      case 0: // F_DUPFD
        // if we really wanted to support this, we'd need to do it
        // in the target fd space.
        warn("fcntl(%d, F_DUPFD) not supported, error returned\n", fd);
        return -EMFILE;

      case 1: // F_GETFD (get close-on-exec flag)
      case 2: // F_SETFD (set close-on-exec flag)
        return 0;

      case 3: // F_GETFL (get file flags)
      case 4: // F_SETFL (set file flags)
        // not sure if this is totally valid, but we'll pass it through
        // to the underlying OS
        warn("fcntl(%d, %d) passed through to host\n", fd, cmd);
        return fcntl(process->sim_fd(fd), cmd);
        // return 0;

      case 7: // F_GETLK  (get lock)
      case 8: // F_SETLK  (set lock)
      case 9: // F_SETLKW (set lock and wait)
        // don't mess with file locking... just act like it's OK
        warn("File lock call (fcntl(%d, %d)) ignored.\n", fd, cmd);
        return 0;

      default:
        warn("Unknown fcntl command %d\n", cmd);
        return 0;
    }
}

SyscallReturn
pipePseudoFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    int fds[2], sim_fds[2];
    int pipe_retval = pipe(fds);

    if (pipe_retval < 0) {
        // error
        return pipe_retval;
    }

    sim_fds[0] = process->alloc_fd(fds[0]);
    sim_fds[1] = process->alloc_fd(fds[1]);

    // Alpha Linux convention for pipe() is that fd[0] is returned as
    // the return value of the function, and fd[1] is returned in r20.
    xc->setIntReg(SyscallPseudoReturnReg, sim_fds[1]);
    return sim_fds[0];
}


SyscallReturn
getpidPseudoFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // Make up a PID.  There's no interprocess communication in
    // fake_syscall mode, so there's no way for a process to know it's
    // not getting a unique value.

    xc->setIntReg(SyscallPseudoReturnReg, 99);
    return 100;
}


SyscallReturn
getuidPseudoFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // Make up a UID and EUID... it shouldn't matter, and we want the
    // simulation to be deterministic.

    // EUID goes in r20.
    xc->setIntReg(SyscallPseudoReturnReg, 100); //EUID
    return 100;		// UID
}


SyscallReturn
getgidPseudoFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // Get current group ID.  EGID goes in r20.
    xc->setIntReg(SyscallPseudoReturnReg, 100); //EGID
    return 100;
}


SyscallReturn
setuidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // can't fathom why a benchmark would call this.
    warn("Ignoring call to setuid(%d)\n", xc->getSyscallArg(0));
    return 0;
}

SyscallReturn
getpidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // Make up a PID.  There's no interprocess communication in
    // fake_syscall mode, so there's no way for a process to know it's
    // not getting a unique value.

    xc->setIntReg(SyscallPseudoReturnReg, 99); //PID
    return 100;
}

SyscallReturn
getppidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    return 99;
}

SyscallReturn
getuidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    return 100;		// UID
}

SyscallReturn
geteuidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    return 100;		// UID
}

SyscallReturn
getgidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    return 100;
}

SyscallReturn
getegidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    return 100;
}


