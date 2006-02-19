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
             xc->cpu->name(), name);

    SyscallReturn retval = (*funcPtr)(this, callnum, process, xc);

    DPRINTFR(SyscallVerbose, "%s: syscall %s returns %d\n",
             xc->cpu->name(), name, retval.value());

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
    new SimExitEvent("syscall caused exit", xc->getSyscallArg(0) & 0xff);

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
        bufArg.copyOut(xc->mem);

    return bytes_read;
}

SyscallReturn
writeFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    int fd = p->sim_fd(xc->getSyscallArg(0));
    int nbytes = xc->getSyscallArg(2);
    BufferArg bufArg(xc->getSyscallArg(1), nbytes);

    bufArg.copyIn(xc->mem);

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

    name.copyOut(xc->mem);

    return 0;
}

SyscallReturn
unlinkFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    string path;

    if (xc->mem->readString(path, xc->getSyscallArg(0)) != NoFault)
        return (TheISA::IntReg)-EFAULT;

    int result = unlink(path.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
renameFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    string old_name;

    if (xc->mem->readString(old_name, xc->getSyscallArg(0)) != NoFault)
        return -EFAULT;

    string new_name;

    if (xc->mem->readString(new_name, xc->getSyscallArg(1)) != NoFault)
        return -EFAULT;

    int64_t result = rename(old_name.c_str(), new_name.c_str());
    return (result == -1) ? -errno : result;
}

SyscallReturn
truncateFunc(SyscallDesc *desc, int num, Process *p, ExecContext *xc)
{
    string path;

    if (xc->mem->readString(path, xc->getSyscallArg(0)) != NoFault)
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

    if (xc->mem->readString(path, xc->getSyscallArg(0)) != NoFault)
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
