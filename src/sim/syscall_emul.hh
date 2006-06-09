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
 *          Kevin Lim
 */

#ifndef __SIM_SYSCALL_EMUL_HH__
#define __SIM_SYSCALL_EMUL_HH__

#define BSD_HOST (defined(__APPLE__) || defined(__OpenBSD__) || \
                  defined(__FreeBSD__))

///
/// @file syscall_emul.hh
///
/// This file defines objects used to emulate syscalls from the target
/// application on the host machine.

#include <errno.h>
#include <string>
#ifdef __CYGWIN32__
#include <sys/fcntl.h>	// for O_BINARY
#endif
#include <sys/uio.h>

#include "arch/isa_traits.hh"	// for Addr
#include "base/chunk_generator.hh"
#include "base/intmath.hh"	// for RoundUp
#include "base/misc.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/translating_port.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"

///
/// System call descriptor.
///
class SyscallDesc {

  public:

    /// Typedef for target syscall handler functions.
    typedef SyscallReturn (*FuncPtr)(SyscallDesc *, int num,
                           Process *, ThreadContext *);

    const char *name;	//!< Syscall name (e.g., "open").
    FuncPtr funcPtr;	//!< Pointer to emulation function.
    int flags;		//!< Flags (see Flags enum).

    /// Flag values for controlling syscall behavior.
    enum Flags {
        /// Don't set return regs according to funcPtr return value.
        /// Used for syscalls with non-standard return conventions
        /// that explicitly set the ThreadContext regs (e.g.,
        /// sigreturn).
        SuppressReturnValue = 1
    };

    /// Constructor.
    SyscallDesc(const char *_name, FuncPtr _funcPtr, int _flags = 0)
        : name(_name), funcPtr(_funcPtr), flags(_flags)
    {
    }

    /// Emulate the syscall.  Public interface for calling through funcPtr.
    void doSyscall(int callnum, Process *proc, ThreadContext *tc);
};


class BaseBufferArg {

  public:

    BaseBufferArg(Addr _addr, int _size) : addr(_addr), size(_size)
    {
        bufPtr = new uint8_t[size];
        // clear out buffer: in case we only partially populate this,
        // and then do a copyOut(), we want to make sure we don't
        // introduce any random junk into the simulated address space
        memset(bufPtr, 0, size);
    }

    virtual ~BaseBufferArg() { delete [] bufPtr; }

    //
    // copy data into simulator space (read from target memory)
    //
    virtual bool copyIn(TranslatingPort *memport)
    {
        memport->readBlob(addr, bufPtr, size);
        return true;	// no EFAULT detection for now
    }

    //
    // copy data out of simulator space (write to target memory)
    //
    virtual bool copyOut(TranslatingPort *memport)
    {
        memport->writeBlob(addr, bufPtr, size);
        return true;	// no EFAULT detection for now
    }

  protected:
    Addr addr;
    int size;
    uint8_t *bufPtr;
};


class BufferArg : public BaseBufferArg
{
  public:
    BufferArg(Addr _addr, int _size) : BaseBufferArg(_addr, _size) { }
    void *bufferPtr()	{ return bufPtr; }
};

template <class T>
class TypedBufferArg : public BaseBufferArg
{
  public:
    // user can optionally specify a specific number of bytes to
    // allocate to deal with those structs that have variable-size
    // arrays at the end
    TypedBufferArg(Addr _addr, int _size = sizeof(T))
        : BaseBufferArg(_addr, _size)
    { }

    // type case
    operator T*() { return (T *)bufPtr; }

    // dereference operators
    T &operator*()	 { return *((T *)bufPtr); }
    T* operator->()	 { return (T *)bufPtr; }
    T &operator[](int i) { return ((T *)bufPtr)[i]; }
};

//////////////////////////////////////////////////////////////////////
//
// The following emulation functions are generic enough that they
// don't need to be recompiled for different emulated OS's.  They are
// defined in sim/syscall_emul.cc.
//
//////////////////////////////////////////////////////////////////////


/// Handler for unimplemented syscalls that we haven't thought about.
SyscallReturn unimplementedFunc(SyscallDesc *desc, int num,
                                Process *p, ThreadContext *tc);

/// Handler for unimplemented syscalls that we never intend to
/// implement (signal handling, etc.) and should not affect the correct
/// behavior of the program.  Print a warning only if the appropriate
/// trace flag is enabled.  Return success to the target program.
SyscallReturn ignoreFunc(SyscallDesc *desc, int num,
                         Process *p, ThreadContext *tc);

/// Target exit() handler: terminate simulation.
SyscallReturn exitFunc(SyscallDesc *desc, int num,
                       Process *p, ThreadContext *tc);

/// Target getpagesize() handler.
SyscallReturn getpagesizeFunc(SyscallDesc *desc, int num,
                              Process *p, ThreadContext *tc);

/// Target obreak() handler: set brk address.
SyscallReturn obreakFunc(SyscallDesc *desc, int num,
                         Process *p, ThreadContext *tc);

/// Target close() handler.
SyscallReturn closeFunc(SyscallDesc *desc, int num,
                        Process *p, ThreadContext *tc);

/// Target read() handler.
SyscallReturn readFunc(SyscallDesc *desc, int num,
                       Process *p, ThreadContext *tc);

/// Target write() handler.
SyscallReturn writeFunc(SyscallDesc *desc, int num,
                        Process *p, ThreadContext *tc);

/// Target lseek() handler.
SyscallReturn lseekFunc(SyscallDesc *desc, int num,
                        Process *p, ThreadContext *tc);

/// Target munmap() handler.
SyscallReturn munmapFunc(SyscallDesc *desc, int num,
                         Process *p, ThreadContext *tc);

/// Target gethostname() handler.
SyscallReturn gethostnameFunc(SyscallDesc *desc, int num,
                              Process *p, ThreadContext *tc);

/// Target unlink() handler.
SyscallReturn unlinkFunc(SyscallDesc *desc, int num,
                         Process *p, ThreadContext *tc);

/// Target rename() handler.
SyscallReturn renameFunc(SyscallDesc *desc, int num,
                         Process *p, ThreadContext *tc);


/// Target truncate() handler.
SyscallReturn truncateFunc(SyscallDesc *desc, int num,
                           Process *p, ThreadContext *tc);


/// Target ftruncate() handler.
SyscallReturn ftruncateFunc(SyscallDesc *desc, int num,
                            Process *p, ThreadContext *tc);


/// Target chown() handler.
SyscallReturn chownFunc(SyscallDesc *desc, int num,
                        Process *p, ThreadContext *tc);


/// Target fchown() handler.
SyscallReturn fchownFunc(SyscallDesc *desc, int num,
                         Process *p, ThreadContext *tc);

/// Target fnctl() handler.
SyscallReturn fcntlFunc(SyscallDesc *desc, int num,
                        Process *process, ThreadContext *tc);

/// Target fcntl64() handler.
SyscallReturn fcntl64Func(SyscallDesc *desc, int num,
                        Process *process, ThreadContext *tc);

/// Target setuid() handler.
SyscallReturn setuidFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target getpid() handler.
SyscallReturn getpidFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target getuid() handler.
SyscallReturn getuidFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target getgid() handler.
SyscallReturn getgidFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target getppid() handler.
SyscallReturn getppidFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target geteuid() handler.
SyscallReturn geteuidFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target getegid() handler.
SyscallReturn getegidFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);



/// Pseudo Funcs  - These functions use a different return convension,
/// returning a second value in a register other than the normal return register
SyscallReturn pipePseudoFunc(SyscallDesc *desc, int num,
                             Process *process, ThreadContext *tc);

/// Target getpidPseudo() handler.
SyscallReturn getpidPseudoFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target getuidPseudo() handler.
SyscallReturn getuidPseudoFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);

/// Target getgidPseudo() handler.
SyscallReturn getgidPseudoFunc(SyscallDesc *desc, int num,
                               Process *p, ThreadContext *tc);


/// This struct is used to build an target-OS-dependent table that
/// maps the target's open() flags to the host open() flags.
struct OpenFlagTransTable {
    int tgtFlag;	//!< Target system flag value.
    int hostFlag;	//!< Corresponding host system flag value.
};



/// A readable name for 1,000,000, for converting microseconds to seconds.
const int one_million = 1000000;

/// Approximate seconds since the epoch (1/1/1970).  About a billion,
/// by my reckoning.  We want to keep this a constant (not use the
/// real-world time) to keep simulations repeatable.
const unsigned seconds_since_epoch = 1000000000;

/// Helper function to convert current elapsed time to seconds and
/// microseconds.
template <class T1, class T2>
void
getElapsedTime(T1 &sec, T2 &usec)
{
    int elapsed_usecs = curTick / Clock::Int::us;
    sec = elapsed_usecs / one_million;
    usec = elapsed_usecs % one_million;
}

//////////////////////////////////////////////////////////////////////
//
// The following emulation functions are generic, but need to be
// templated to account for differences in types, constants, etc.
//
//////////////////////////////////////////////////////////////////////

/// Target ioctl() handler.  For the most part, programs call ioctl()
/// only to find out if their stdout is a tty, to determine whether to
/// do line or block buffering.
template <class OS>
SyscallReturn
ioctlFunc(SyscallDesc *desc, int callnum, Process *process,
          ThreadContext *tc)
{
    int fd = tc->getSyscallArg(0);
    unsigned req = tc->getSyscallArg(1);

    DPRINTF(SyscallVerbose, "ioctl(%d, 0x%x, ...)\n", fd, req);

    if (fd < 0 || process->sim_fd(fd) < 0) {
        // doesn't map to any simulator fd: not a valid target fd
        return -EBADF;
    }

    switch (req) {
      case OS::TIOCISATTY:
      case OS::TIOCGETP:
      case OS::TIOCSETP:
      case OS::TIOCSETN:
      case OS::TIOCSETC:
      case OS::TIOCGETC:
      case OS::TIOCGETS:
      case OS::TIOCGETA:
        return -ENOTTY;

      default:
        fatal("Unsupported ioctl call: ioctl(%d, 0x%x, ...) @ 0x%llx\n",
              fd, req, tc->readPC());
    }
}

/// Target open() handler.
template <class OS>
SyscallReturn
openFunc(SyscallDesc *desc, int callnum, Process *process,
         ThreadContext *tc)
{
    std::string path;

    if (!tc->getMemPort()->tryReadString(path, tc->getSyscallArg(0)))
        return -EFAULT;

    if (path == "/dev/sysdev0") {
        // This is a memory-mapped high-resolution timer device on Alpha.
        // We don't support it, so just punt.
        warn("Ignoring open(%s, ...)\n", path);
        return -ENOENT;
    }

    int tgtFlags = tc->getSyscallArg(1);
    int mode = tc->getSyscallArg(2);
    int hostFlags = 0;

    // translate open flags
    for (int i = 0; i < OS::NUM_OPEN_FLAGS; i++) {
        if (tgtFlags & OS::openFlagTable[i].tgtFlag) {
            tgtFlags &= ~OS::openFlagTable[i].tgtFlag;
            hostFlags |= OS::openFlagTable[i].hostFlag;
        }
    }

    // any target flags left?
    if (tgtFlags != 0)
        warn("Syscall: open: cannot decode flags 0x%x", tgtFlags);

#ifdef __CYGWIN32__
    hostFlags |= O_BINARY;
#endif

    DPRINTF(SyscallVerbose, "opening file %s\n", path.c_str());

    // open the file
    int fd = open(path.c_str(), hostFlags, mode);

    return (fd == -1) ? -errno : process->alloc_fd(fd);
}


/// Target chmod() handler.
template <class OS>
SyscallReturn
chmodFunc(SyscallDesc *desc, int callnum, Process *process,
          ThreadContext *tc)
{
    std::string path;

    if (!tc->getMemPort()->tryReadString(path, tc->getSyscallArg(0)))
        return -EFAULT;

    uint32_t mode = tc->getSyscallArg(1);
    mode_t hostMode = 0;

    // XXX translate mode flags via OS::something???
    hostMode = mode;

    // do the chmod
    int result = chmod(path.c_str(), hostMode);
    if (result < 0)
        return -errno;

    return 0;
}


/// Target fchmod() handler.
template <class OS>
SyscallReturn
fchmodFunc(SyscallDesc *desc, int callnum, Process *process,
           ThreadContext *tc)
{
    int fd = tc->getSyscallArg(0);
    if (fd < 0 || process->sim_fd(fd) < 0) {
        // doesn't map to any simulator fd: not a valid target fd
        return -EBADF;
    }

    uint32_t mode = tc->getSyscallArg(1);
    mode_t hostMode = 0;

    // XXX translate mode flags via OS::someting???
    hostMode = mode;

    // do the fchmod
    int result = fchmod(process->sim_fd(fd), hostMode);
    if (result < 0)
        return -errno;

    return 0;
}


/// Target stat() handler.
template <class OS>
SyscallReturn
statFunc(SyscallDesc *desc, int callnum, Process *process,
         ThreadContext *tc)
{
    std::string path;

    if (!tc->getMemPort()->tryReadString(path, tc->getSyscallArg(0)))
    return -EFAULT;

    struct stat hostBuf;
    int result = stat(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    OS::copyOutStatBuf(tc->getMemPort(), tc->getSyscallArg(1), &hostBuf);

    return 0;
}


/// Target fstat64() handler.
template <class OS>
SyscallReturn
fstat64Func(SyscallDesc *desc, int callnum, Process *process,
            ThreadContext *tc)
{
    int fd = tc->getSyscallArg(0);
    if (fd < 0 || process->sim_fd(fd) < 0) {
        // doesn't map to any simulator fd: not a valid target fd
        return -EBADF;
    }

#if BSD_HOST
    struct stat  hostBuf;
    int result = fstat(process->sim_fd(fd), &hostBuf);
#else
    struct stat64  hostBuf;
    int result = fstat64(process->sim_fd(fd), &hostBuf);
#endif

    if (result < 0)
        return -errno;

    OS::copyOutStat64Buf(tc->getMemPort(), fd, tc->getSyscallArg(1), &hostBuf);

    return 0;
}


/// Target lstat() handler.
template <class OS>
SyscallReturn
lstatFunc(SyscallDesc *desc, int callnum, Process *process,
          ThreadContext *tc)
{
    std::string path;

    if (!tc->getMemPort()->tryReadString(path, tc->getSyscallArg(0)))
      return -EFAULT;

    struct stat hostBuf;
    int result = lstat(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    OS::copyOutStatBuf(tc->getMemPort(), tc->getSyscallArg(1), &hostBuf);

    return 0;
}

/// Target lstat64() handler.
template <class OS>
SyscallReturn
lstat64Func(SyscallDesc *desc, int callnum, Process *process,
            ThreadContext *tc)
{
    std::string path;

    if (!tc->getMemPort()->tryReadString(path, tc->getSyscallArg(0)))
      return -EFAULT;

#if BSD_HOST
    struct stat hostBuf;
    int result = lstat(path.c_str(), &hostBuf);
#else
    struct stat64 hostBuf;
    int result = lstat64(path.c_str(), &hostBuf);
#endif

    if (result < 0)
        return -errno;

    OS::copyOutStat64Buf(tc->getMemPort(), -1, tc->getSyscallArg(1), &hostBuf);

    return 0;
}

/// Target fstat() handler.
template <class OS>
SyscallReturn
fstatFunc(SyscallDesc *desc, int callnum, Process *process,
          ThreadContext *tc)
{
    int fd = process->sim_fd(tc->getSyscallArg(0));

    DPRINTF(SyscallVerbose, "fstat(%d, ...)\n", fd);

    if (fd < 0)
        return -EBADF;

    struct stat hostBuf;
    int result = fstat(fd, &hostBuf);

    if (result < 0)
        return -errno;

    OS::copyOutStatBuf(tc->getMemPort(), tc->getSyscallArg(1), &hostBuf);

    return 0;
}


/// Target statfs() handler.
template <class OS>
SyscallReturn
statfsFunc(SyscallDesc *desc, int callnum, Process *process,
           ThreadContext *tc)
{
    std::string path;

    if (!tc->getMemPort()->tryReadString(path, tc->getSyscallArg(0)))
      return -EFAULT;

    struct statfs hostBuf;
    int result = statfs(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    OS::copyOutStatfsBuf(tc->getMemPort(), tc->getSyscallArg(1), &hostBuf);

    return 0;
}


/// Target fstatfs() handler.
template <class OS>
SyscallReturn
fstatfsFunc(SyscallDesc *desc, int callnum, Process *process,
            ThreadContext *tc)
{
    int fd = process->sim_fd(tc->getSyscallArg(0));

    if (fd < 0)
        return -EBADF;

    struct statfs hostBuf;
    int result = fstatfs(fd, &hostBuf);

    if (result < 0)
        return -errno;

    OS::copyOutStatfsBuf(tc->getMemPort(), tc->getSyscallArg(1), &hostBuf);

    return 0;
}


/// Target writev() handler.
template <class OS>
SyscallReturn
writevFunc(SyscallDesc *desc, int callnum, Process *process,
           ThreadContext *tc)
{
    int fd = tc->getSyscallArg(0);
    if (fd < 0 || process->sim_fd(fd) < 0) {
        // doesn't map to any simulator fd: not a valid target fd
        return -EBADF;
    }

    TranslatingPort *p = tc->getMemPort();
    uint64_t tiov_base = tc->getSyscallArg(1);
    size_t count = tc->getSyscallArg(2);
    struct iovec hiov[count];
    for (int i = 0; i < count; ++i)
    {
        typename OS::tgt_iovec tiov;

        p->readBlob(tiov_base + i*sizeof(typename OS::tgt_iovec),
                    (uint8_t*)&tiov, sizeof(typename OS::tgt_iovec));
        hiov[i].iov_len = gtoh(tiov.iov_len);
        hiov[i].iov_base = new char [hiov[i].iov_len];
        p->readBlob(gtoh(tiov.iov_base), (uint8_t *)hiov[i].iov_base,
                    hiov[i].iov_len);
    }

    int result = writev(process->sim_fd(fd), hiov, count);

    for (int i = 0; i < count; ++i)
    {
        delete [] (char *)hiov[i].iov_base;
    }

    if (result < 0)
        return -errno;

    return 0;
}


/// Target mmap() handler.
///
/// We don't really handle mmap().  If the target is mmaping an
/// anonymous region or /dev/zero, we can get away with doing basically
/// nothing (since memory is initialized to zero and the simulator
/// doesn't really check addresses anyway).  Always print a warning,
/// since this could be seriously broken if we're not mapping
/// /dev/zero.
//
/// Someday we should explicitly check for /dev/zero in open, flag the
/// file descriptor, and fail (or implement!) a non-anonymous mmap to
/// anything else.
template <class OS>
SyscallReturn
mmapFunc(SyscallDesc *desc, int num, Process *p, ThreadContext *tc)
{
    Addr start = tc->getSyscallArg(0);
    uint64_t length = tc->getSyscallArg(1);
    // int prot = tc->getSyscallArg(2);
    int flags = tc->getSyscallArg(3);
    // int fd = p->sim_fd(tc->getSyscallArg(4));
    // int offset = tc->getSyscallArg(5);

    if ((start  % TheISA::VMPageSize) != 0 ||
        (length % TheISA::VMPageSize) != 0) {
        warn("mmap failing: arguments not page-aligned: "
             "start 0x%x length 0x%x",
             start, length);
        return -EINVAL;
    }

    if (start != 0) {
        warn("mmap: ignoring suggested map address 0x%x, using 0x%x",
             start, p->mmap_end);
    }

    // pick next address from our "mmap region"
    start = p->mmap_end;
    p->pTable->allocate(start, length);
    p->mmap_end += length;

    if (!(flags & OS::TGT_MAP_ANONYMOUS)) {
        warn("allowing mmap of file @ fd %d. "
             "This will break if not /dev/zero.", tc->getSyscallArg(4));
    }

    return start;
}

/// Target getrlimit() handler.
template <class OS>
SyscallReturn
getrlimitFunc(SyscallDesc *desc, int callnum, Process *process,
        ThreadContext *tc)
{
    unsigned resource = tc->getSyscallArg(0);
    TypedBufferArg<typename OS::rlimit> rlp(tc->getSyscallArg(1));

    switch (resource) {
        case OS::TGT_RLIMIT_STACK:
            // max stack size in bytes: make up a number (2MB for now)
            rlp->rlim_cur = rlp->rlim_max = 8 * 1024 * 1024;
            rlp->rlim_cur = htog(rlp->rlim_cur);
            rlp->rlim_max = htog(rlp->rlim_max);
            break;

        default:
            std::cerr << "getrlimitFunc: unimplemented resource " << resource
                << std::endl;
            abort();
            break;
    }

    rlp.copyOut(tc->getMemPort());
    return 0;
}

/// Target gettimeofday() handler.
template <class OS>
SyscallReturn
gettimeofdayFunc(SyscallDesc *desc, int callnum, Process *process,
        ThreadContext *tc)
{
    TypedBufferArg<typename OS::timeval> tp(tc->getSyscallArg(0));

    getElapsedTime(tp->tv_sec, tp->tv_usec);
    tp->tv_sec += seconds_since_epoch;
    tp->tv_sec = htog(tp->tv_sec);
    tp->tv_usec = htog(tp->tv_usec);

    tp.copyOut(tc->getMemPort());

    return 0;
}


/// Target utimes() handler.
template <class OS>
SyscallReturn
utimesFunc(SyscallDesc *desc, int callnum, Process *process,
           ThreadContext *tc)
{
    std::string path;

    if (!tc->getMemPort()->tryReadString(path, tc->getSyscallArg(0)))
      return -EFAULT;

    TypedBufferArg<typename OS::timeval [2]> tp(tc->getSyscallArg(1));
    tp.copyIn(tc->getMemPort());

    struct timeval hostTimeval[2];
    for (int i = 0; i < 2; ++i)
    {
        hostTimeval[i].tv_sec = gtoh((*tp)[i].tv_sec);
        hostTimeval[i].tv_usec = gtoh((*tp)[i].tv_usec);
    }
    int result = utimes(path.c_str(), hostTimeval);

    if (result < 0)
        return -errno;

    return 0;
}
/// Target getrusage() function.
template <class OS>
SyscallReturn
getrusageFunc(SyscallDesc *desc, int callnum, Process *process,
              ThreadContext *tc)
{
    int who = tc->getSyscallArg(0);	// THREAD, SELF, or CHILDREN
    TypedBufferArg<typename OS::rusage> rup(tc->getSyscallArg(1));

    if (who != OS::TGT_RUSAGE_SELF) {
        // don't really handle THREAD or CHILDREN, but just warn and
        // plow ahead
        warn("getrusage() only supports RUSAGE_SELF.  Parameter %d ignored.",
             who);
    }

    getElapsedTime(rup->ru_utime.tv_sec, rup->ru_utime.tv_usec);
    rup->ru_utime.tv_sec = htog(rup->ru_utime.tv_sec);
    rup->ru_utime.tv_usec = htog(rup->ru_utime.tv_usec);

    rup->ru_stime.tv_sec = 0;
    rup->ru_stime.tv_usec = 0;
    rup->ru_maxrss = 0;
    rup->ru_ixrss = 0;
    rup->ru_idrss = 0;
    rup->ru_isrss = 0;
    rup->ru_minflt = 0;
    rup->ru_majflt = 0;
    rup->ru_nswap = 0;
    rup->ru_inblock = 0;
    rup->ru_oublock = 0;
    rup->ru_msgsnd = 0;
    rup->ru_msgrcv = 0;
    rup->ru_nsignals = 0;
    rup->ru_nvcsw = 0;
    rup->ru_nivcsw = 0;

    rup.copyOut(tc->getMemPort());

    return 0;
}




#endif // __SIM_SYSCALL_EMUL_HH__
