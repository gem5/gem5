/*
 * Copyright (c) 2012-2013, 2015, 2019-2021 Arm Limited
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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

#ifndef __SIM_SYSCALL_EMUL_HH__
#define __SIM_SYSCALL_EMUL_HH__

#if (defined(__APPLE__) || defined(__OpenBSD__) ||      \
     defined(__FreeBSD__) || defined(__CYGWIN__) ||     \
     defined(__NetBSD__))
#define NO_STAT64 1
#else
#define NO_STAT64 0
#endif

///
/// @file syscall_emul.hh
///
/// This file defines objects used to emulate syscalls from the target
/// application on the host machine.

#if defined(__linux__)
#include <sched.h>
#include <sys/eventfd.h>
#include <sys/statfs.h>

#else
#include <sys/mount.h>

#endif

#ifdef __CYGWIN32__
#include <sys/fcntl.h>

#endif
#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cerrno>
#include <memory>
#include <string>

#include "arch/generic/tlb.hh"
#include "base/intmath.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "mem/page_table.hh"
#include "mem/se_translating_port_proxy.hh"
#include "params/Process.hh"
#include "sim/emul_driver.hh"
#include "sim/futex_map.hh"
#include "sim/guest_abi.hh"
#include "sim/process.hh"
#include "sim/proxy_ptr.hh"
#include "sim/syscall_debug_macros.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul_buf.hh"
#include "sim/syscall_return.hh"

#if defined(__APPLE__) && defined(__MACH__) && !defined(CMSG_ALIGN)
#define CMSG_ALIGN(len) (((len) + sizeof(size_t) - 1) & ~(sizeof(size_t) - 1))
#elif defined(__FreeBSD__) && !defined(CMSG_ALIGN)
#define CMSG_ALIGN(n) _ALIGN(n)
#endif

namespace gem5
{

//////////////////////////////////////////////////////////////////////
//
// The following emulation functions are generic enough that they
// don't need to be recompiled for different emulated OS's.  They are
// defined in sim/syscall_emul.cc.
//
//////////////////////////////////////////////////////////////////////

void warnUnsupportedOS(std::string syscall_name);

/// Handler for unimplemented syscalls that we haven't thought about.
SyscallReturn unimplementedFunc(SyscallDesc *desc, ThreadContext *tc);

/// Handler for unimplemented syscalls that we never intend to
/// implement (signal handling, etc.) and should not affect the correct
/// behavior of the program.  Prints a warning.  Return success to the target
/// program.
SyscallReturn ignoreFunc(SyscallDesc *desc, ThreadContext *tc);
/// Like above, but only prints a warning once per syscall desc it's used with.
SyscallReturn
ignoreWarnOnceFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target exit() handler: terminate current context.
SyscallReturn exitFunc(SyscallDesc *desc, ThreadContext *tc, int status);

/// Target exit_group() handler: terminate simulation. (exit all threads)
SyscallReturn exitGroupFunc(SyscallDesc *desc, ThreadContext *tc, int status);

/// Target set_tid_address() handler.
SyscallReturn setTidAddressFunc(SyscallDesc *desc, ThreadContext *tc,
                                uint64_t tidPtr);

/// Target getpagesize() handler.
SyscallReturn getpagesizeFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target brk() handler: set brk address.
SyscallReturn brkFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> new_brk);

/// Target close() handler.
SyscallReturn closeFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd);

/// Target lseek() handler.
SyscallReturn lseekFunc(SyscallDesc *desc, ThreadContext *tc,
                        int tgt_fd, uint64_t offs, int whence);

/// Target _llseek() handler.
SyscallReturn _llseekFunc(SyscallDesc *desc, ThreadContext *tc,
                          int tgt_fd, uint64_t offset_high,
                          uint32_t offset_low, VPtr<> result_ptr, int whence);

/// Target shutdown() handler.
SyscallReturn shutdownFunc(SyscallDesc *desc, ThreadContext *tc,
                           int tgt_fd, int how);

/// Target gethostname() handler.
SyscallReturn gethostnameFunc(SyscallDesc *desc, ThreadContext *tc,
                              VPtr<> buf_ptr, int name_len);

/// Target getcwd() handler.
SyscallReturn getcwdFunc(SyscallDesc *desc, ThreadContext *tc,
                         VPtr<> buf_ptr, unsigned long size);

/// Target unlink() handler.
SyscallReturn unlinkFunc(SyscallDesc *desc, ThreadContext *tc,
                         VPtr<> pathname);
SyscallReturn unlinkImpl(SyscallDesc *desc, ThreadContext *tc,
                         std::string path);

/// Target link() handler
SyscallReturn linkFunc(SyscallDesc *desc, ThreadContext *tc,
                       VPtr<> pathname, VPtr<> new_pathname);

/// Target symlink() handler.
SyscallReturn symlinkFunc(SyscallDesc *desc, ThreadContext *tc,
                          VPtr<> pathname, VPtr<> new_pathname);

/// Target mkdir() handler.
SyscallReturn mkdirFunc(SyscallDesc *desc, ThreadContext *tc,
                        VPtr<> pathname, mode_t mode);
SyscallReturn mkdirImpl(SyscallDesc *desc, ThreadContext *tc,
                        std::string path, mode_t mode);

/// Target mknod() handler.
SyscallReturn mknodFunc(SyscallDesc *desc, ThreadContext *tc,
                        VPtr<> pathname, mode_t mode, dev_t dev);
SyscallReturn mknodImpl(SyscallDesc *desc, ThreadContext *tc,
                        std::string path, mode_t mode, dev_t dev);

/// Target chdir() handler.
SyscallReturn chdirFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> pathname);

// Target rmdir() handler.
SyscallReturn rmdirFunc(SyscallDesc *desc, ThreadContext *tc,
                        VPtr<> pathname);
SyscallReturn rmdirImpl(SyscallDesc *desc, ThreadContext *tc,
                        std::string path);

/// Target rename() handler.
SyscallReturn renameFunc(SyscallDesc *desc, ThreadContext *tc,
                         VPtr<> oldpath, VPtr<> newpath);
SyscallReturn renameImpl(SyscallDesc *desc, ThreadContext *tc,
                         std::string oldpath, std::string newpath);

/// Target truncate64() handler.
SyscallReturn truncate64Func(SyscallDesc *desc, ThreadContext *tc,
                             VPtr<> pathname, int64_t length);

/// Target ftruncate64() handler.
SyscallReturn ftruncate64Func(SyscallDesc *desc, ThreadContext *tc,
                              int tgt_fd, int64_t length);

/// Target umask() handler.
SyscallReturn umaskFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target gettid() handler.
SyscallReturn gettidFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target chown() handler.
SyscallReturn chownFunc(SyscallDesc *desc, ThreadContext *tc,
                        VPtr<> pathname, uint32_t owner, uint32_t group);
SyscallReturn chownImpl(SyscallDesc *desc, ThreadContext *tc,
                        std::string path, uint32_t owner, uint32_t group);

/// Target getpgrpFunc() handler.
SyscallReturn getpgrpFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target setpgid() handler.
SyscallReturn setpgidFunc(SyscallDesc *desc, ThreadContext *tc,
                          int pid, int pgid);

/// Target fchown() handler.
SyscallReturn fchownFunc(SyscallDesc *desc, ThreadContext *tc,
                         int tgt_fd, uint32_t owner, uint32_t group);

/// Target dup() handler.
SyscallReturn dupFunc(SyscallDesc *desc, ThreadContext *tc,
                      int tgt_fd);

/// Target dup2() handler.
SyscallReturn dup2Func(SyscallDesc *desc, ThreadContext *tc,
                       int old_tgt_fd, int new_tgt_fd);

/// Target fcntl() handler.
SyscallReturn fcntlFunc(SyscallDesc *desc, ThreadContext *tc,
                        int tgt_fd, int cmd, guest_abi::VarArgs<int> varargs);

/// Target fcntl64() handler.
SyscallReturn fcntl64Func(SyscallDesc *desc, ThreadContext *tc,
                          int tgt_fd, int cmd);

/// Target pipe() handler.
SyscallReturn pipeFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> tgt_addr);

/// Target pipe() handler.
SyscallReturn pipe2Func(SyscallDesc *desc, ThreadContext *tc,
                        VPtr<> tgt_addr, int flags);

/// Target getpid() handler.
SyscallReturn getpidFunc(SyscallDesc *desc, ThreadContext *tc);

// Target getpeername() handler.
SyscallReturn getpeernameFunc(SyscallDesc *desc, ThreadContext *tc,
                              int tgt_fd, VPtr<> sockAddrPtr,
                              VPtr<> addrlenPtr);

// Target bind() handler.
SyscallReturn bindFunc(SyscallDesc *desc, ThreadContext *tc,
                       int tgt_fd, VPtr<> buf_ptr, int addrlen);

// Target listen() handler.
SyscallReturn listenFunc(SyscallDesc *desc, ThreadContext *tc,
                         int tgt_fd, int backlog);

// Target connect() handler.
SyscallReturn connectFunc(SyscallDesc *desc, ThreadContext *tc,
                          int tgt_fd, VPtr<> buf_ptr, int addrlen);

#if defined(SYS_getdents)
// Target getdents() handler.
SyscallReturn getdentsFunc(SyscallDesc *desc, ThreadContext *tc,
                           int tgt_fd, VPtr<> buf_ptr, unsigned count);
#endif

#if defined(SYS_getdents64)
// Target getdents() handler.
SyscallReturn getdents64Func(SyscallDesc *desc, ThreadContext *tc,
                             int tgt_fd, VPtr<> buf_ptr, unsigned count);
#endif

// Target recvmsg() handler.
SyscallReturn recvmsgFunc(SyscallDesc *desc, ThreadContext *tc,
                          int tgt_fd, VPtr<> msgPtr, int flags);

// Target sendmsg() handler.
SyscallReturn sendmsgFunc(SyscallDesc *desc, ThreadContext *tc,
                          int tgt_fd, VPtr<> msgPtr, int flags);

// Target getuid() handler.
SyscallReturn getuidFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target getgid() handler.
SyscallReturn getgidFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target getppid() handler.
SyscallReturn getppidFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target geteuid() handler.
SyscallReturn geteuidFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target getegid() handler.
SyscallReturn getegidFunc(SyscallDesc *desc, ThreadContext *tc);

/// Target access() handler
SyscallReturn accessFunc(SyscallDesc *desc, ThreadContext *tc,
                         VPtr<> pathname, mode_t mode);
SyscallReturn accessImpl(SyscallDesc *desc, ThreadContext *tc,
                         std::string path, mode_t mode);

// Target getsockopt() handler.
SyscallReturn getsockoptFunc(SyscallDesc *desc, ThreadContext *tc,
                             int tgt_fd, int level, int optname,
                             VPtr<> valPtr, VPtr<> lenPtr);

// Target setsockopt() handler.
SyscallReturn setsockoptFunc(SyscallDesc *desc, ThreadContext *tc,
                             int tgt_fd, int level, int optname,
                             VPtr<> valPtr, socklen_t len);

SyscallReturn getcpuFunc(SyscallDesc *desc, ThreadContext *tc,
                         VPtr<uint32_t> cpu, VPtr<uint32_t> node,
                         VPtr<uint32_t> tcache);

// Target getsockname() handler.
SyscallReturn getsocknameFunc(SyscallDesc *desc, ThreadContext *tc,
                              int tgt_fd, VPtr<> addrPtr, VPtr<> lenPtr);

template <class OS>
SyscallReturn
atSyscallPath(ThreadContext *tc, int dirfd, std::string &path)
{
    // If pathname is absolute, then dirfd is ignored.
    if (dirfd != OS::TGT_AT_FDCWD && !startswith(path, "/")) {
        auto process = tc->getProcessPtr();

        std::shared_ptr<FDEntry> fdep = ((*process->fds)[dirfd]);
        auto ffdp = std::dynamic_pointer_cast<FileFDEntry>(fdep);
        if (!ffdp)
            return -EBADF;

        if (path.empty())
            path = ffdp->getFileName();
        else
            path = ffdp->getFileName() + "/" + path;
    }

    return 0;
}

/// Futex system call
/// Implemented by Daniel Sanchez
/// Used by printf's in multi-threaded apps
template <class OS>
SyscallReturn
futexFunc(SyscallDesc *desc, ThreadContext *tc,
        VPtr<> uaddr, int op, int val, int timeout, VPtr<> uaddr2, int val3)
{
    auto process = tc->getProcessPtr();

    /*
     * Unsupported option that does not affect the correctness of the
     * application. This is a performance optimization utilized by Linux.
     */
    op &= ~OS::TGT_FUTEX_PRIVATE_FLAG;
    op &= ~OS::TGT_FUTEX_CLOCK_REALTIME_FLAG;

    FutexMap &futex_map = tc->getSystemPtr()->futexMap;

    if (OS::TGT_FUTEX_WAIT == op || OS::TGT_FUTEX_WAIT_BITSET == op) {
        // Ensure futex system call accessed atomically.
        BufferArg buf(uaddr, sizeof(int));
        buf.copyIn(SETranslatingPortProxy(tc));
        int mem_val = *(int*)buf.bufferPtr();

        /*
         * The value in memory at uaddr is not equal with the expected val
         * (a different thread must have changed it before the system call was
         * invoked). In this case, we need to throw an error.
         */
        if (val != mem_val)
            return -OS::TGT_EWOULDBLOCK;

        if (OS::TGT_FUTEX_WAIT == op) {
            futex_map.suspend(uaddr, process->tgid(), tc);
        } else {
            futex_map.suspend_bitset(uaddr, process->tgid(), tc, val3);
        }

        return 0;
    } else if (OS::TGT_FUTEX_WAKE == op) {
        return futex_map.wakeup(uaddr, process->tgid(), val);
    } else if (OS::TGT_FUTEX_WAKE_BITSET == op) {
        return futex_map.wakeup_bitset(uaddr, process->tgid(), val3);
    } else if (OS::TGT_FUTEX_REQUEUE == op ||
               OS::TGT_FUTEX_CMP_REQUEUE == op) {

        // Ensure futex system call accessed atomically.
        BufferArg buf(uaddr, sizeof(int));
        buf.copyIn(SETranslatingPortProxy(tc));
        int mem_val = *(int*)buf.bufferPtr();
        /*
         * For CMP_REQUEUE, the whole operation is only started only if
         * val3 is still the value of the futex pointed to by uaddr.
         */
        if (OS::TGT_FUTEX_CMP_REQUEUE && val3 != mem_val)
            return -OS::TGT_EWOULDBLOCK;
        return futex_map.requeue(uaddr, process->tgid(), val, timeout, uaddr2);
    } else if (OS::TGT_FUTEX_WAKE_OP == op) {
        /*
         * The FUTEX_WAKE_OP operation is equivalent to executing the
         * following code atomically and totally ordered with respect to
         * other futex operations on any of the two supplied futex words:
         *
         *   int oldval = *(int *) addr2;
         *   *(int *) addr2 = oldval op oparg;
         *   futex(addr1, FUTEX_WAKE, val, 0, 0, 0);
         *   if (oldval cmp cmparg)
         *        futex(addr2, FUTEX_WAKE, val2, 0, 0, 0);
         *
         * (op, oparg, cmp, cmparg are encoded in val3)
         *
         * +---+---+-----------+-----------+
         * |op |cmp|   oparg   |  cmparg   |
         * +---+---+-----------+-----------+
         *   4   4       12          12    <== # of bits
         *
         * reference: http://man7.org/linux/man-pages/man2/futex.2.html
         *
         */
        // get value from simulated-space
        BufferArg buf(uaddr2, sizeof(int));
        buf.copyIn(SETranslatingPortProxy(tc));
        int oldval = *(int*)buf.bufferPtr();
        int newval = oldval;
        // extract op, oparg, cmp, cmparg from val3
        int wake_cmparg =  val3 & 0xfff;
        int wake_oparg  = (val3 & 0xfff000)   >> 12;
        int wake_cmp    = (val3 & 0xf000000)  >> 24;
        int wake_op     = (val3 & 0xf0000000) >> 28;
        if ((wake_op & OS::TGT_FUTEX_OP_ARG_SHIFT) >> 3 == 1)
            wake_oparg = (1 << wake_oparg);
        wake_op &= ~OS::TGT_FUTEX_OP_ARG_SHIFT;
        // perform operation on the value of the second futex
        if (wake_op == OS::TGT_FUTEX_OP_SET)
            newval = wake_oparg;
        else if (wake_op == OS::TGT_FUTEX_OP_ADD)
            newval += wake_oparg;
        else if (wake_op == OS::TGT_FUTEX_OP_OR)
            newval |= wake_oparg;
        else if (wake_op == OS::TGT_FUTEX_OP_ANDN)
            newval &= ~wake_oparg;
        else if (wake_op == OS::TGT_FUTEX_OP_XOR)
            newval ^= wake_oparg;
        // copy updated value back to simulated-space
        *(int*)buf.bufferPtr() = newval;
        buf.copyOut(SETranslatingPortProxy(tc));
        // perform the first wake-up
        int woken1 = futex_map.wakeup(uaddr, process->tgid(), val);
        int woken2 = 0;
        // calculate the condition of the second wake-up
        bool is_wake2 = false;
        if (wake_cmp == OS::TGT_FUTEX_OP_CMP_EQ)
            is_wake2 = oldval == wake_cmparg;
        else if (wake_cmp == OS::TGT_FUTEX_OP_CMP_NE)
            is_wake2 = oldval != wake_cmparg;
        else if (wake_cmp == OS::TGT_FUTEX_OP_CMP_LT)
            is_wake2 = oldval < wake_cmparg;
        else if (wake_cmp == OS::TGT_FUTEX_OP_CMP_LE)
            is_wake2 = oldval <= wake_cmparg;
        else if (wake_cmp == OS::TGT_FUTEX_OP_CMP_GT)
            is_wake2 = oldval > wake_cmparg;
        else if (wake_cmp == OS::TGT_FUTEX_OP_CMP_GE)
            is_wake2 = oldval >= wake_cmparg;
        // perform the second wake-up
        if (is_wake2)
            woken2 = futex_map.wakeup(uaddr2, process->tgid(), timeout);

        return woken1 + woken2;
    }
    warn("futex: op %d not implemented; ignoring.", op);
    return -ENOSYS;
}

/// Pseudo Funcs  - These functions use a different return convension,
/// returning a second value in a register other than the normal return register
SyscallReturn pipePseudoFunc(SyscallDesc *desc, ThreadContext *tc);


/// Approximate seconds since the epoch (1/1/1970).  About a billion,
/// by my reckoning.  We want to keep this a constant (not use the
/// real-world time) to keep simulations repeatable.
const unsigned seconds_since_epoch = 1000 * 1000 * 1000;

/// Helper function to convert current elapsed time to seconds and
/// microseconds.
template <class T1, class T2>
void
getElapsedTimeMicro(T1 &sec, T2 &usec)
{
    static const int OneMillion = 1000 * 1000;

    uint64_t elapsed_usecs = curTick() / sim_clock::as_int::us;
    sec = elapsed_usecs / OneMillion;
    usec = elapsed_usecs % OneMillion;
}

/// Helper function to convert current elapsed time to seconds and
/// nanoseconds.
template <class T1, class T2>
void
getElapsedTimeNano(T1 &sec, T2 &nsec)
{
    static const int OneBillion = 1000 * 1000 * 1000;

    uint64_t elapsed_nsecs = curTick() / sim_clock::as_int::ns;
    sec = elapsed_nsecs / OneBillion;
    nsec = elapsed_nsecs % OneBillion;
}

//////////////////////////////////////////////////////////////////////
//
// The following emulation functions are generic, but need to be
// templated to account for differences in types, constants, etc.
//
//////////////////////////////////////////////////////////////////////

    typedef struct statfs hst_statfs;
#if NO_STAT64
    typedef struct stat hst_stat;
    typedef struct stat hst_stat64;
#else
    typedef struct stat hst_stat;
    typedef struct stat64 hst_stat64;
#endif

//// Helper function to convert a host stat buffer to a target stat
//// buffer.  Also copies the target buffer out to the simulated
//// memory space.  Used by stat(), fstat(), and lstat().

template <typename OS, typename TgtStatPtr, typename HostStatPtr>
void
copyOutStatBuf(TgtStatPtr tgt, HostStatPtr host, bool fakeTTY=false)
{
    constexpr ByteOrder bo = OS::byteOrder;

    if (fakeTTY)
        tgt->st_dev = 0xA;
    else
        tgt->st_dev = host->st_dev;
    tgt->st_dev = htog(tgt->st_dev, bo);
    tgt->st_ino = host->st_ino;
    tgt->st_ino = htog(tgt->st_ino, bo);
    tgt->st_mode = host->st_mode;
    if (fakeTTY) {
        // Claim to be a character device
        tgt->st_mode &= ~S_IFMT;    // Clear S_IFMT
        tgt->st_mode |= S_IFCHR;    // Set S_IFCHR
    }
    tgt->st_mode = htog(tgt->st_mode, bo);
    tgt->st_nlink = host->st_nlink;
    tgt->st_nlink = htog(tgt->st_nlink, bo);
    tgt->st_uid = host->st_uid;
    tgt->st_uid = htog(tgt->st_uid, bo);
    tgt->st_gid = host->st_gid;
    tgt->st_gid = htog(tgt->st_gid, bo);
    if (fakeTTY)
        tgt->st_rdev = 0x880d;
    else
        tgt->st_rdev = host->st_rdev;
    tgt->st_rdev = htog(tgt->st_rdev, bo);
    tgt->st_size = host->st_size;
    tgt->st_size = htog(tgt->st_size, bo);
    tgt->st_atimeX = host->st_atime;
    tgt->st_atimeX = htog(tgt->st_atimeX, bo);
    tgt->st_mtimeX = host->st_mtime;
    tgt->st_mtimeX = htog(tgt->st_mtimeX, bo);
    tgt->st_ctimeX = host->st_ctime;
    tgt->st_ctimeX = htog(tgt->st_ctimeX, bo);
    // Force the block size to be 8KB. This helps to ensure buffered io works
    // consistently across different hosts.
    tgt->st_blksize = 0x2000;
    tgt->st_blksize = htog(tgt->st_blksize, bo);
    tgt->st_blocks = host->st_blocks;
    tgt->st_blocks = htog(tgt->st_blocks, bo);
}

// Same for stat64

template <typename OS, typename TgtStatPtr, typename HostStatPtr>
void
copyOutStat64Buf(TgtStatPtr tgt, HostStatPtr host,
                 bool fakeTTY=false)
{
    copyOutStatBuf<OS>(tgt, host, fakeTTY);
#if defined(STAT_HAVE_NSEC)
    constexpr ByteOrder bo = OS::byteOrder;

    tgt->st_atime_nsec = host->st_atime_nsec;
    tgt->st_atime_nsec = htog(tgt->st_atime_nsec, bo);
    tgt->st_mtime_nsec = host->st_mtime_nsec;
    tgt->st_mtime_nsec = htog(tgt->st_mtime_nsec, bo);
    tgt->st_ctime_nsec = host->st_ctime_nsec;
    tgt->st_ctime_nsec = htog(tgt->st_ctime_nsec, bo);
#else
    tgt->st_atime_nsec = 0;
    tgt->st_mtime_nsec = 0;
    tgt->st_ctime_nsec = 0;
#endif
}

template <class OS, typename TgtStatPtr, typename HostStatPtr>
void
copyOutStatfsBuf(TgtStatPtr tgt, HostStatPtr host)
{
    constexpr ByteOrder bo = OS::byteOrder;

    tgt->f_type = htog(host->f_type, bo);
#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
    tgt->f_bsize = htog(host->f_iosize, bo);
#else
    tgt->f_bsize = htog(host->f_bsize, bo);
#endif
    tgt->f_blocks = htog(host->f_blocks, bo);
    tgt->f_bfree = htog(host->f_bfree, bo);
    tgt->f_bavail = htog(host->f_bavail, bo);
    tgt->f_files = htog(host->f_files, bo);
    tgt->f_ffree = htog(host->f_ffree, bo);
    memcpy(&tgt->f_fsid, &host->f_fsid, sizeof(host->f_fsid));
#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
    tgt->f_namelen = htog(host->f_namemax, bo);
    tgt->f_frsize = htog(host->f_bsize, bo);
#elif defined(__APPLE__)
    tgt->f_namelen = 0;
    tgt->f_frsize = 0;
#else
    tgt->f_namelen = htog(host->f_namelen, bo);
    tgt->f_frsize = htog(host->f_frsize, bo);
#endif
#if defined(__linux__)
    memcpy(&tgt->f_spare, &host->f_spare,
            std::min(sizeof(host->f_spare), sizeof(tgt->f_spare)));
#else
    /*
     * The fields are different sizes per OS. Don't bother with
     * f_spare or f_reserved on non-Linux for now.
     */
    memset(&tgt->f_spare, 0, sizeof(tgt->f_spare));
#endif
}

/// Target ioctl() handler.  For the most part, programs call ioctl()
/// only to find out if their stdout is a tty, to determine whether to
/// do line or block buffering.  We always claim that output fds are
/// not TTYs to provide repeatable results.
template <class OS>
SyscallReturn
ioctlFunc(SyscallDesc *desc, ThreadContext *tc,
          int tgt_fd, unsigned req, VPtr<> addr)
{
    auto p = tc->getProcessPtr();

    DPRINTF_SYSCALL(Verbose, "ioctl(%d, 0x%x, ...)\n", tgt_fd, req);

    if (OS::isTtyReq(req))
        return -ENOTTY;

    auto dfdp = std::dynamic_pointer_cast<DeviceFDEntry>((*p->fds)[tgt_fd]);
    if (dfdp) {
        EmulatedDriver *emul_driver = dfdp->getDriver();
        if (emul_driver)
            return emul_driver->ioctl(tc, req, addr);
    }

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (sfdp) {
        int status;

        switch (req) {
          case SIOCGIFCONF: {
            BufferArg conf_arg(addr, sizeof(ifconf));
            conf_arg.copyIn(SETranslatingPortProxy(tc));

            ifconf *conf = (ifconf*)conf_arg.bufferPtr();
            Addr ifc_buf_addr = (Addr)conf->ifc_buf;
            BufferArg ifc_buf_arg(ifc_buf_addr, conf->ifc_len);
            ifc_buf_arg.copyIn(SETranslatingPortProxy(tc));

            conf->ifc_buf = (char*)ifc_buf_arg.bufferPtr();

            status = ioctl(sfdp->getSimFD(), req, conf_arg.bufferPtr());
            if (status != -1) {
                conf->ifc_buf = (char*)ifc_buf_addr;
                ifc_buf_arg.copyOut(SETranslatingPortProxy(tc));
                conf_arg.copyOut(SETranslatingPortProxy(tc));
            }

            return status;
          }
          case SIOCGIFFLAGS:
#if defined(__linux__)
          case SIOCGIFINDEX:
#endif
          case SIOCGIFNETMASK:
          case SIOCGIFADDR:
#if defined(__linux__)
          case SIOCGIFHWADDR:
#endif
          case SIOCGIFMTU: {
            BufferArg req_arg(addr, sizeof(ifreq));
            req_arg.copyIn(SETranslatingPortProxy(tc));

            status = ioctl(sfdp->getSimFD(), req, req_arg.bufferPtr());
            if (status != -1)
                req_arg.copyOut(SETranslatingPortProxy(tc));
            return status;
          }
        }
    }

    /**
     * For lack of a better return code, return ENOTTY. Ideally, we should
     * return something better here, but at least we issue the warning.
     */
    warn("Unsupported ioctl call (return ENOTTY): ioctl(%d, 0x%x, ...) @ \n",
         tgt_fd, req, tc->pcState());
    return -ENOTTY;
}

/// Target open() handler.
template <class OS>
SyscallReturn
openatFunc(SyscallDesc *desc, ThreadContext *tc,
           int tgt_dirfd, VPtr<> pathname, int tgt_flags, int mode)
{
    auto p = tc->getProcessPtr();

    /**
     * Retrieve the simulated process' memory proxy and then read in the path
     * string from that memory space into the host's working memory space.
     */
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

#ifdef __CYGWIN32__
    int host_flags = O_BINARY;
#else
    int host_flags = 0;
#endif
    /**
     * Translate target flags into host flags. Flags exist which are not
     * ported between architectures which can cause check failures.
     */
    for (const auto &p: OS::openFlagTable) {
        if (tgt_flags & p.first) {
            tgt_flags &= ~p.first;
            host_flags |= p.second;
        }
    }
    warn_if(tgt_flags, "%s: cannot decode flags %#x", desc->name(), tgt_flags);

#ifdef __CYGWIN32__
    host_flags |= O_BINARY;
#endif

    /**
     * If the simulated process called open or openat with AT_FDCWD specified,
     * take the current working directory value which was passed into the
     * process class as a Python parameter and append the current path to
     * create a full path.
     * Otherwise, openat with a valid target directory file descriptor has
     * been called. If the path option, which was passed in as a parameter,
     * is not absolute, retrieve the directory file descriptor's path and
     * prepend it to the path passed in as a parameter.
     * In every case, we should have a full path (which is relevant to the
     * host) to work with after this block has been passed.
     */
    std::string redir_path = path;
    std::string abs_path = path;
    if (tgt_dirfd == OS::TGT_AT_FDCWD) {
        abs_path = p->absolutePath(path, true);
        redir_path = p->checkPathRedirect(path);
    } else if (!startswith(path, "/")) {
        std::shared_ptr<FDEntry> fdep = ((*p->fds)[tgt_dirfd]);
        auto ffdp = std::dynamic_pointer_cast<FileFDEntry>(fdep);
        if (!ffdp)
            return -EBADF;
        abs_path = ffdp->getFileName() + path;
        redir_path = p->checkPathRedirect(abs_path);
    }

    /**
     * Since this is an emulated environment, we create pseudo file
     * descriptors for device requests that have been registered with
     * the process class through Python; this allows us to create a file
     * descriptor for subsequent ioctl or mmap calls.
     */
    if (startswith(abs_path, "/dev/")) {
        std::string filename = abs_path.substr(strlen("/dev/"));
        EmulatedDriver *drv = p->findDriver(filename);
        if (drv) {
            DPRINTF_SYSCALL(Verbose, "%s: passing call to "
                            "driver open with path[%s]\n",
                            desc->name(), abs_path.c_str());
            return drv->open(tc, mode, host_flags);
        }
        /**
         * Fall through here for pass through to host devices, such
         * as /dev/zero
         */
    }

    /**
     * We make several attempts resolve a call to open.
     *
     * 1) Resolve any path redirection before hand. This will set the path
     * up with variable 'redir_path' which may contain a modified path or
     * the original path value. This should already be done in prior code.
     * 2) Try to handle the access using 'special_paths'. Some special_paths
     * and files cannot be called on the host and need to be handled as
     * special cases inside the simulator. These special_paths are handled by
     * C++ routines to provide output back to userspace.
     * 3) If the full path that was created above does not match any of the
     * special cases, pass it through to the open call on the __HOST__ to let
     * the host open the file on our behalf. Again, the openImpl tries to
     * USE_THE_HOST_FILESYSTEM_OPEN (with a possible redirection to the
     * faux-filesystem files). The faux-filesystem is dynamically created
     * during simulator configuration using Python functions.
     * 4) If the host cannot open the file, the open attempt failed in "3)".
     * Return the host's error code back through the system call to the
     * simulated process. If running a debug trace, also notify the user that
     * the open call failed.
     *
     * Any success will set sim_fd to something other than -1 and skip the
     * next conditions effectively bypassing them.
     */
    int sim_fd = -1;
    std::string used_path;
    std::vector<std::string> special_paths =
            { "/proc/meminfo/", "/system/", "/platform/", "/etc/passwd",
              "/proc/self/maps", "/dev/urandom",
              "/sys/devices/system/cpu/online" };
    for (auto entry : special_paths) {
        if (startswith(path, entry)) {
            sim_fd = OS::openSpecialFile(abs_path, p, tc);
            used_path = abs_path;
        }
    }
    if (sim_fd == -1) {
        sim_fd = open(redir_path.c_str(), host_flags, mode);
        used_path = redir_path;
    }
    if (sim_fd == -1) {
        int local = -errno;
        DPRINTF_SYSCALL(Verbose, "%s: failed -> path:%s "
                        "(inferred from:%s)\n", desc->name(),
                        used_path.c_str(), path.c_str());
        return local;
    }

    /**
     * The file was opened successfully and needs to be recorded in the
     * process' file descriptor array so that it can be retrieved later.
     * The target file descriptor that is chosen will be the lowest unused
     * file descriptor.
     * Return the indirect target file descriptor back to the simulated
     * process to act as a handle for the opened file.
     */
    auto ffdp = std::make_shared<FileFDEntry>(sim_fd, host_flags, path, 0);
    // Record the file mode for checkpoint restoring
    ffdp->setFileMode(mode);
    int tgt_fd = p->fds->allocFD(ffdp);
    DPRINTF_SYSCALL(Verbose, "%s: sim_fd[%d], target_fd[%d] -> path:%s\n"
                    "(inferred from:%s)\n", desc->name(),
                    sim_fd, tgt_fd, used_path.c_str(), path.c_str());
    return tgt_fd;
}

/// Target open() handler.
template <class OS>
SyscallReturn
openFunc(SyscallDesc *desc, ThreadContext *tc,
         VPtr<> pathname, int tgt_flags, int mode)
{
    return openatFunc<OS>(
            desc, tc, OS::TGT_AT_FDCWD, pathname, tgt_flags, mode);
}

/// Target unlinkat() handler.
template <class OS>
SyscallReturn
unlinkatFunc(SyscallDesc *desc, ThreadContext *tc,
             int dirfd, VPtr<> pathname, int flags)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    if (flags & OS::TGT_AT_REMOVEDIR) {
        return rmdirImpl(desc, tc, path);
    } else {
        return unlinkImpl(desc, tc, path);
    }
}

/// Target facessat() handler
template <class OS>
SyscallReturn
faccessatFunc(SyscallDesc *desc, ThreadContext *tc,
              int dirfd, VPtr<> pathname, int mode)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    return accessImpl(desc, tc, path, mode);
}

/// Target readlinkat() handler
template <class OS>
SyscallReturn
readlinkatFunc(SyscallDesc *desc, ThreadContext *tc,
               int dirfd, VPtr<> pathname, VPtr<> buf_ptr,
               typename OS::size_t bufsiz)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    auto p = tc->getProcessPtr();

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
        typename OS::size_t real_path_len = strlen(real_path);
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

    buf.copyOut(SETranslatingPortProxy(tc));

    return (result == -1) ? -errno : result;
}

/// Target readlink() handler
template <class OS>
SyscallReturn
readlinkFunc(SyscallDesc *desc, ThreadContext *tc,
             VPtr<> pathname, VPtr<> buf_ptr,
             typename OS::size_t bufsiz)
{
    return readlinkatFunc<OS>(desc, tc, OS::TGT_AT_FDCWD,
        pathname, buf_ptr, bufsiz);
}

/// Target renameat() handler.
template <class OS>
SyscallReturn
renameatFunc(SyscallDesc *desc, ThreadContext *tc,
             int olddirfd, VPtr<> oldpath, int newdirfd, VPtr<> newpath)
{
    SETranslatingPortProxy proxy(tc);
    std::string old_name;
    if (!proxy.tryReadString(old_name, oldpath))
        return -EFAULT;

    std::string new_name;
    if (!proxy.tryReadString(new_name, newpath))
        return -EFAULT;

    // Modifying old_name from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, olddirfd, old_name); !res.successful()) {
        return res;
    }

    // Modifying new_name from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, newdirfd, new_name); !res.successful()) {
        return res;
    }

    return renameImpl(desc, tc, old_name, new_name);
}

/// Target fchownat() handler
template <class OS>
SyscallReturn
fchownatFunc(SyscallDesc *desc, ThreadContext *tc,
             int dirfd, VPtr<> pathname, uint32_t owner, uint32_t group,
             int flags)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    return chownImpl(desc, tc, path, owner, group);
}

/// Target mkdirat() handler
template <class OS>
SyscallReturn
mkdiratFunc(SyscallDesc *desc, ThreadContext *tc,
            int dirfd, VPtr<> pathname, mode_t mode)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    return mkdirImpl(desc, tc, path, mode);
}

/// Target mknodat() handler
template <class OS>
SyscallReturn
mknodatFunc(SyscallDesc *desc, ThreadContext *tc,
            int dirfd, VPtr<> pathname, mode_t mode, dev_t dev)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    return mknodImpl(desc, tc, path, mode, dev);
}

/// Target sysinfo() handler.
template <class OS>
SyscallReturn
sysinfoFunc(SyscallDesc *desc, ThreadContext *tc,
            VPtr<typename OS::tgt_sysinfo> sysinfo)
{
    auto process = tc->getProcessPtr();

    sysinfo->uptime = seconds_since_epoch;
    sysinfo->totalram = process->system->memSize();
    sysinfo->mem_unit = 1;

    return 0;
}

/// Target chmod() handler.
template <class OS>
SyscallReturn
fchmodatFunc(SyscallDesc *desc, ThreadContext *tc,
             int dirfd, VPtr<> pathname, mode_t mode)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    mode_t hostMode = 0;

    // XXX translate mode flags via OS::something???
    hostMode = mode;

    auto process = tc->getProcessPtr();
    // Adjust path for cwd and redirection
    path = process->checkPathRedirect(path);

    // do the chmod
    int result = chmod(path.c_str(), hostMode);
    if (result < 0)
        return -errno;

    return 0;
}

/// Target chmod() handler.
template <class OS>
SyscallReturn
chmodFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> pathname, mode_t mode)
{
    return fchmodatFunc<OS>(desc, tc, OS::TGT_AT_FDCWD, pathname, mode);
}

template <class OS>
SyscallReturn
pollFunc(SyscallDesc *desc, ThreadContext *tc,
         VPtr<> fdsPtr, int nfds, int tmout)
{
    auto p = tc->getProcessPtr();

    BufferArg fdsBuf(fdsPtr, sizeof(struct pollfd) * nfds);
    fdsBuf.copyIn(SETranslatingPortProxy(tc));

    /**
     * Record the target file descriptors in a local variable. We need to
     * replace them with host file descriptors but we need a temporary copy
     * for later. Afterwards, replace each target file descriptor in the
     * poll_fd array with its host_fd.
     */
    int temp_tgt_fds[nfds];
    for (int index = 0; index < nfds; index++) {
        temp_tgt_fds[index] = ((struct pollfd *)fdsBuf.bufferPtr())[index].fd;
        auto tgt_fd = temp_tgt_fds[index];
        auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
        if (!hbfdp)
            return -EBADF;
        auto host_fd = hbfdp->getSimFD();
        ((struct pollfd *)fdsBuf.bufferPtr())[index].fd = host_fd;
    }

    /**
     * We cannot allow an infinite poll to occur or it will inevitably cause
     * a deadlock in the gem5 simulator with clone. We must pass in tmout with
     * a non-negative value, however it also makes no sense to poll on the
     * underlying host for any other time than tmout a zero timeout.
     */
    int status;
    if (tmout < 0) {
        status = poll((struct pollfd *)fdsBuf.bufferPtr(), nfds, 0);
        if (status == 0) {
            /**
             * If blocking indefinitely, check the signal list to see if a
             * signal would break the poll out of the retry cycle and try
             * to return the signal interrupt instead.
             */
            System *sysh = tc->getSystemPtr();
            std::list<BasicSignal>::iterator it;
            for (it=sysh->signalList.begin(); it!=sysh->signalList.end(); it++)
                if (it->receiver == p)
                    return -EINTR;
            return SyscallReturn::retry();
        }
    } else
        status = poll((struct pollfd *)fdsBuf.bufferPtr(), nfds, 0);

    if (status == -1)
        return -errno;

    /**
     * Replace each host_fd in the returned poll_fd array with its original
     * target file descriptor.
     */
    for (int index = 0; index < nfds; index++) {
        auto tgt_fd = temp_tgt_fds[index];
        ((struct pollfd *)fdsBuf.bufferPtr())[index].fd = tgt_fd;
    }

    /**
     * Copy out the pollfd struct because the host may have updated fields
     * in the structure.
     */
    fdsBuf.copyOut(SETranslatingPortProxy(tc));

    return status;
}

/// Target fchmod() handler.
template <class OS>
SyscallReturn
fchmodFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd, uint32_t mode)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    mode_t hostMode = mode;

    int result = fchmod(sim_fd, hostMode);

    return (result < 0) ? -errno : 0;
}

/// Target mremap() handler.
template <class OS>
SyscallReturn
mremapFunc(SyscallDesc *desc, ThreadContext *tc,
        VPtr<> start, uint64_t old_length, uint64_t new_length, uint64_t flags,
        guest_abi::VarArgs<uint64_t> varargs)
{
    auto p = tc->getProcessPtr();
    Addr page_bytes = p->pTable->pageSize();
    uint64_t provided_address = 0;
    bool use_provided_address = flags & OS::TGT_MREMAP_FIXED;

    if (use_provided_address)
        provided_address = varargs.get<uint64_t>();

    if ((start % page_bytes != 0) ||
        (provided_address % page_bytes != 0)) {
        warn("mremap failing: arguments not page aligned");
        return -EINVAL;
    }

    new_length = roundUp(new_length, page_bytes);

    if (new_length > old_length) {
        Addr mmap_end = p->memState->getMmapEnd();

        if ((start + old_length) == mmap_end &&
            (!use_provided_address || provided_address == start)) {
            // This case cannot occur when growing downward, as
            // start is greater than or equal to mmap_end.
            uint64_t diff = new_length - old_length;
            p->memState->mapRegion(mmap_end, diff, "remapped");
            p->memState->setMmapEnd(mmap_end + diff);
            return (Addr)start;
        } else {
            if (!use_provided_address && !(flags & OS::TGT_MREMAP_MAYMOVE)) {
                warn("can't remap here and MREMAP_MAYMOVE flag not set\n");
                return -ENOMEM;
            } else {
                uint64_t new_start = provided_address;
                if (!use_provided_address) {
                    new_start = p->mmapGrowsDown() ?
                                mmap_end - new_length : mmap_end;
                    mmap_end = p->mmapGrowsDown() ?
                               new_start : mmap_end + new_length;
                    p->memState->setMmapEnd(mmap_end);
                }

                warn("mremapping to new vaddr %08p-%08p, adding %d\n",
                     new_start, new_start + new_length,
                     new_length - old_length);

                // add on the remaining unallocated pages
                p->allocateMem(new_start + old_length,
                               new_length - old_length,
                               use_provided_address /* clobber */);

                if (use_provided_address &&
                    ((new_start + new_length > p->memState->getMmapEnd() &&
                      !p->mmapGrowsDown()) ||
                    (new_start < p->memState->getMmapEnd() &&
                      p->mmapGrowsDown()))) {
                    // something fishy going on here, at least notify the user
                    // @todo: increase mmap_end?
                    warn("mmap region limit exceeded with MREMAP_FIXED\n");
                }

                warn("returning %08p as start\n", new_start);
                p->memState->remapRegion(start, new_start, old_length);
                return new_start;
            }
        }
    } else {
        // Shrink a region
        if (use_provided_address && provided_address != start)
            p->memState->remapRegion(start, provided_address, new_length);
        if (new_length != old_length)
            p->memState->unmapRegion(start + new_length,
                                     old_length - new_length);
        return use_provided_address ? provided_address : (Addr)start;
    }
}

/// Target stat() handler.
template <class OS>
SyscallReturn
statFunc(SyscallDesc *desc, ThreadContext *tc,
         VPtr<> pathname, VPtr<typename OS::tgt_stat> tgt_stat)
{
    std::string path;
    auto process = tc->getProcessPtr();

    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = process->checkPathRedirect(path);

    struct stat hostBuf;
    int result = stat(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatBuf<OS>(tgt_stat, &hostBuf);

    return 0;
}

/// Target newfstatat() handler.
template <class OS>
SyscallReturn
newfstatatFunc(SyscallDesc *desc, ThreadContext *tc, int dirfd,
               VPtr<> pathname, VPtr<typename OS::tgt_stat64> tgt_stat,
               int flags)
{
    std::string path;

    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    if (path.empty() && !(flags & OS::TGT_AT_EMPTY_PATH))
        return -ENOENT;
    flags = flags & ~OS::TGT_AT_EMPTY_PATH;

    warn_if(flags != 0, "newfstatat: Flag bits %#x not supported.", flags);

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    auto p = tc->getProcessPtr();

    // Adjust path for cwd and redirection
    path = p->checkPathRedirect(path);

    struct stat host_buf;
    int result = stat(path.c_str(), &host_buf);

    if (result < 0)
        return -errno;

    copyOutStat64Buf<OS>(tgt_stat, &host_buf);

    return 0;
}

/// Target fstatat64() handler.
template <class OS>
SyscallReturn
fstatat64Func(SyscallDesc *desc, ThreadContext *tc,
              int dirfd, VPtr<> pathname,
              VPtr<typename OS::tgt_stat64> tgt_stat)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    auto p = tc->getProcessPtr();

    // Adjust path for cwd and redirection
    path = p->checkPathRedirect(path);

#if NO_STAT64
    struct stat  hostBuf;
    int result = stat(path.c_str(), &hostBuf);
#else
    struct stat64 hostBuf;
    int result = stat64(path.c_str(), &hostBuf);
#endif

    if (result < 0)
        return -errno;

    copyOutStat64Buf<OS>(tgt_stat, &hostBuf);

    return 0;
}

/// Target stat64() handler.
template <class OS>
SyscallReturn
stat64Func(SyscallDesc *desc, ThreadContext *tc,
           VPtr<> pathname, VPtr<typename OS::tgt_stat64> tgt_stat)
{
    return fstatat64Func<OS>(desc, tc, OS::TGT_AT_FDCWD, pathname, tgt_stat);
}

/// Target fstat64() handler.
template <class OS>
SyscallReturn
fstat64Func(SyscallDesc *desc, ThreadContext *tc,
            int tgt_fd, VPtr<typename OS::tgt_stat64> tgt_stat)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

#if NO_STAT64
    struct stat  hostBuf;
    int result = fstat(sim_fd, &hostBuf);
#else
    struct stat64  hostBuf;
    int result = fstat64(sim_fd, &hostBuf);
#endif

    if (result < 0)
        return -errno;

    copyOutStat64Buf<OS>(tgt_stat, &hostBuf, (sim_fd == 1));

    return 0;
}


/// Target lstat() handler.
template <class OS>
SyscallReturn
lstatFunc(SyscallDesc *desc, ThreadContext *tc,
          VPtr<> pathname, VPtr<typename OS::tgt_stat> tgt_stat)
{
    std::string path;
    auto process = tc->getProcessPtr();

    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = process->checkPathRedirect(path);

    struct stat hostBuf;
    int result = lstat(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatBuf<OS>(tgt_stat, &hostBuf);

    return 0;
}

/// Target lstat64() handler.
template <class OS>
SyscallReturn
lstat64Func(SyscallDesc *desc, ThreadContext *tc,
            VPtr<> pathname, VPtr<typename OS::tgt_stat64> tgt_stat)
{
    std::string path;
    auto process = tc->getProcessPtr();

    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = process->checkPathRedirect(path);

#if NO_STAT64
    struct stat hostBuf;
    int result = lstat(path.c_str(), &hostBuf);
#else
    struct stat64 hostBuf;
    int result = lstat64(path.c_str(), &hostBuf);
#endif

    if (result < 0)
        return -errno;

    copyOutStat64Buf<OS>(tgt_stat, &hostBuf);

    return 0;
}

/// Target fstat() handler.
template <class OS>
SyscallReturn
fstatFunc(SyscallDesc *desc, ThreadContext *tc,
          int tgt_fd, VPtr<typename OS::tgt_stat> tgt_stat)
{
    auto p = tc->getProcessPtr();

    DPRINTF_SYSCALL(Verbose, "fstat(%d, ...)\n", tgt_fd);

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    struct stat hostBuf;
    int result = fstat(sim_fd, &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatBuf<OS>(tgt_stat, &hostBuf, (sim_fd == 1));

    return 0;
}

/// Target statfs() handler.
template <class OS>
SyscallReturn
statfsFunc(SyscallDesc *desc, ThreadContext *tc,
           VPtr<> pathname, VPtr<typename OS::tgt_statfs> tgt_stat)
{
#if defined(__linux__)
    std::string path;
    auto process = tc->getProcessPtr();

    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = process->checkPathRedirect(path);

    struct statfs hostBuf;
    int result = statfs(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatfsBuf<OS>(tgt_stat, &hostBuf);
    return 0;
#else
    warnUnsupportedOS("statfs");
    return -1;
#endif
}

template <class OS>
SyscallReturn
doClone(SyscallDesc *desc, ThreadContext *tc, RegVal flags, RegVal newStack,
          VPtr<> ptidPtr, VPtr<> ctidPtr, VPtr<> tlsPtr)
{
    DPRINTF(SyscallVerbose, "Doing clone. pid: %#llx, ctid: %#llx, tls: %#llx"
                            " flags: %#llx, stack: %#llx\n",
            ptidPtr.addr(), ctidPtr.addr(), tlsPtr.addr(), flags, newStack);
    auto p = tc->getProcessPtr();

    if (((flags & OS::TGT_CLONE_SIGHAND)&& !(flags & OS::TGT_CLONE_VM)) ||
        ((flags & OS::TGT_CLONE_THREAD) && !(flags & OS::TGT_CLONE_SIGHAND)) ||
        ((flags & OS::TGT_CLONE_FS)     &&  (flags & OS::TGT_CLONE_NEWNS)) ||
        ((flags & OS::TGT_CLONE_NEWIPC) &&  (flags & OS::TGT_CLONE_SYSVSEM)) ||
        ((flags & OS::TGT_CLONE_NEWPID) &&  (flags & OS::TGT_CLONE_THREAD)) ||
        ((flags & OS::TGT_CLONE_VM)     && !(newStack)))
        return -EINVAL;

    ThreadContext *ctc;
    if (!(ctc = tc->getSystemPtr()->threads.findFree())) {
        DPRINTF_SYSCALL(Verbose, "clone: no spare thread context in system"
                        "[cpu %d, thread %d]", tc->cpuId(), tc->threadId());
        return -EAGAIN;
    }

    /**
     * Note that ProcessParams is generated by swig and there are no other
     * examples of how to create anything but this default constructor. The
     * fields are manually initialized instead of passing parameters to the
     * constructor.
     */
    ProcessParams *pp = new ProcessParams();
    pp->executable.assign(*(new std::string(p->progName())));
    pp->cmd.push_back(*(new std::string(p->progName())));
    pp->system = p->system;
    pp->cwd.assign(p->tgtCwd);
    pp->input.assign("stdin");
    pp->output.assign("stdout");
    pp->errout.assign("stderr");
    pp->uid = p->uid();
    pp->euid = p->euid();
    pp->gid = p->gid();
    pp->egid = p->egid();
    pp->release = p->release;

    /* Find the first free PID that's less than the maximum */
    std::set<int> const& pids = p->system->PIDs;
    int temp_pid = *pids.begin();
    do {
        temp_pid++;
    } while (pids.find(temp_pid) != pids.end());
    if (temp_pid >= System::maxPID)
        fatal("temp_pid is too large: %d", temp_pid);

    pp->pid = temp_pid;
    pp->ppid = (flags & OS::TGT_CLONE_THREAD) ? p->ppid() : p->pid();
    pp->useArchPT = p->useArchPT;
    pp->kvmInSE = p->kvmInSE;
    Process *cp = pp->create();
    // TODO: there is no way to know when the Process SimObject is done with
    // the params pointer. Both the params pointer (pp) and the process
    // pointer (cp) are normally managed in python and are never cleaned up.

    Process *owner = ctc->getProcessPtr();
    ctc->setProcessPtr(cp);
    cp->assignThreadContext(ctc->contextId());
    owner->revokeThreadContext(ctc->contextId());

    if (flags & OS::TGT_CLONE_PARENT_SETTID) {
        BufferArg ptidBuf(ptidPtr, sizeof(long));
        long *ptid = (long *)ptidBuf.bufferPtr();
        *ptid = cp->pid();
        ptidBuf.copyOut(SETranslatingPortProxy(tc));
    }

    if (flags & OS::TGT_CLONE_THREAD) {
        cp->pTable->initState();
        cp->pTable->shared = true;
        cp->useForClone = true;
    }

    ctc->setUseForClone(true);
    cp->initState();
    p->clone(tc, ctc, cp, flags);

    if (flags & OS::TGT_CLONE_THREAD) {
        delete cp->sigchld;
        cp->sigchld = p->sigchld;
    } else if (flags & OS::TGT_SIGCHLD) {
        *cp->sigchld = true;
    }

    if (flags & OS::TGT_CLONE_CHILD_SETTID) {
        BufferArg ctidBuf(ctidPtr, sizeof(long));
        long *ctid = (long *)ctidBuf.bufferPtr();
        *ctid = cp->pid();
        ctidBuf.copyOut(SETranslatingPortProxy(ctc));
    }

    if (flags & OS::TGT_CLONE_CHILD_CLEARTID)
        cp->childClearTID = (uint64_t)ctidPtr;

    ctc->clearArchRegs();

    OS::archClone(flags, p, cp, tc, ctc, newStack, tlsPtr);

    desc->returnInto(ctc, 0);

    ctc->activate();

    if (flags & OS::TGT_CLONE_VFORK) {
        tc->suspend();
    }

    return cp->pid();
}

template <class OS>
SyscallReturn
clone3Func(SyscallDesc *desc, ThreadContext *tc,
           VPtr<typename OS::tgt_clone_args> cl_args, RegVal size)
{
    VPtr<uint64_t> ptidPtr((Addr)cl_args->parent_tid, tc);
    VPtr<uint64_t> ctidPtr((Addr)cl_args->child_tid, tc);
    VPtr<uint64_t> tlsPtr((Addr)cl_args->tls, tc);
    // Clone3 gives the stack as the *lowest* address, but clone/__clone2
    // expects the stack parameter to be the actual stack pointer
    uint64_t new_stack = cl_args->stack + cl_args->stack_size;
    uint64_t flags = cl_args->flags;

    return doClone<OS>(desc, tc, flags, new_stack, ptidPtr, ctidPtr, tlsPtr);
}

template <class OS>
SyscallReturn
cloneFunc(SyscallDesc *desc, ThreadContext *tc, RegVal flags, RegVal newStack,
          VPtr<> ptidPtr, VPtr<> ctidPtr, VPtr<> tlsPtr)
{
    return doClone<OS>(desc, tc, flags, newStack, ptidPtr, ctidPtr, tlsPtr);
}

template <class OS>
SyscallReturn
cloneBackwardsFunc(SyscallDesc *desc, ThreadContext *tc, RegVal flags,
                   RegVal newStack, VPtr<> ptidPtr, VPtr<> tlsPtr,
                   VPtr<> ctidPtr)
{
    return cloneFunc<OS>(desc, tc, flags, newStack, ptidPtr, ctidPtr, tlsPtr);
}

/// Target fstatfs() handler.
template <class OS>
SyscallReturn
fstatfsFunc(SyscallDesc *desc, ThreadContext *tc,
            int tgt_fd, VPtr<typename OS::tgt_statfs> tgt_stat)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    struct statfs hostBuf;
    int result = fstatfs(sim_fd, &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatfsBuf<OS>(tgt_stat, &hostBuf);

    return 0;
}

/// Target readv() handler.
template <class OS>
SyscallReturn
readvFunc(SyscallDesc *desc, ThreadContext *tc,
          int tgt_fd, uint64_t tiov_base,
          typename OS::size_t count)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    SETranslatingPortProxy prox(tc);
    typename OS::tgt_iovec tiov[count];
    struct iovec hiov[count];
    for (typename OS::size_t i = 0; i < count; ++i) {
        prox.readBlob(tiov_base + (i * sizeof(typename OS::tgt_iovec)),
                      &tiov[i], sizeof(typename OS::tgt_iovec));
        hiov[i].iov_len = gtoh(tiov[i].iov_len, OS::byteOrder);
        hiov[i].iov_base = new char [hiov[i].iov_len];
    }

    int result = readv(sim_fd, hiov, count);
    int local_errno = errno;

    for (typename OS::size_t i = 0; i < count; ++i) {
        if (result != -1) {
            prox.writeBlob(htog(tiov[i].iov_base, OS::byteOrder),
                           hiov[i].iov_base, hiov[i].iov_len);
        }
        delete [] (char *)hiov[i].iov_base;
    }

    return (result == -1) ? -local_errno : result;
}

/// Target writev() handler.
template <class OS>
SyscallReturn
writevFunc(SyscallDesc *desc, ThreadContext *tc,
           int tgt_fd, uint64_t tiov_base,
           typename OS::size_t count)
{
    auto p = tc->getProcessPtr();

    auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!hbfdp)
        return -EBADF;
    int sim_fd = hbfdp->getSimFD();

    SETranslatingPortProxy prox(tc);
    struct iovec hiov[count];
    for (typename OS::size_t i = 0; i < count; ++i) {
        typename OS::tgt_iovec tiov;

        prox.readBlob(tiov_base + i*sizeof(typename OS::tgt_iovec),
                      &tiov, sizeof(typename OS::tgt_iovec));
        hiov[i].iov_len = gtoh(tiov.iov_len, OS::byteOrder);
        hiov[i].iov_base = new char [hiov[i].iov_len];
        prox.readBlob(gtoh(tiov.iov_base, OS::byteOrder), hiov[i].iov_base,
                      hiov[i].iov_len);
    }

    int result = writev(sim_fd, hiov, count);

    for (typename OS::size_t i = 0; i < count; ++i)
        delete [] (char *)hiov[i].iov_base;

    return (result == -1) ? -errno : result;
}

/// Target mmap() handler.
template <class OS>
SyscallReturn
mmapFunc(SyscallDesc *desc, ThreadContext *tc,
         VPtr<> start, typename OS::size_t length, int prot,
         int tgt_flags, int tgt_fd, typename OS::off_t offset)
{
    auto p = tc->getProcessPtr();
    Addr page_bytes = p->pTable->pageSize();

    if (start & (page_bytes - 1) ||
        offset & (page_bytes - 1) ||
        (tgt_flags & OS::TGT_MAP_PRIVATE &&
         tgt_flags & OS::TGT_MAP_SHARED) ||
        (!(tgt_flags & OS::TGT_MAP_PRIVATE) &&
         !(tgt_flags & OS::TGT_MAP_SHARED)) ||
        !length) {
        return -EINVAL;
    }

    if ((prot & PROT_WRITE) && (tgt_flags & OS::TGT_MAP_SHARED)) {
        // With shared mmaps, there are two cases to consider:
        // 1) anonymous: writes should modify the mapping and this should be
        // visible to observers who share the mapping. Currently, it's
        // difficult to update the shared mapping because there's no
        // structure which maintains information about the which virtual
        // memory areas are shared. If that structure existed, it would be
        // possible to make the translations point to the same frames.
        // 2) file-backed: writes should modify the mapping and the file
        // which is backed by the mapping. The shared mapping problem is the
        // same as what was mentioned about the anonymous mappings. For
        // file-backed mappings, the writes to the file are difficult
        // because it requires syncing what the mapping holds with the file
        // that resides on the host system. So, any write on a real system
        // would cause the change to be propagated to the file mapping at
        // some point in the future (the inode is tracked along with the
        // mapping). This isn't guaranteed to always happen, but it usually
        // works well enough. The guarantee is provided by the msync system
        // call. We could force the change through with shared mappings with
        // a call to msync, but that again would require more information
        // than we currently maintain.
        warn_once("mmap: writing to shared mmap region is currently "
                  "unsupported. The write succeeds on the target, but it "
                  "will not be propagated to the host or shared mappings");
    }

    length = roundUp(length, page_bytes);

    int sim_fd = -1;
    if (!(tgt_flags & OS::TGT_MAP_ANONYMOUS)) {
        std::shared_ptr<FDEntry> fdep = (*p->fds)[tgt_fd];

        auto dfdp = std::dynamic_pointer_cast<DeviceFDEntry>(fdep);
        if (dfdp) {
            EmulatedDriver *emul_driver = dfdp->getDriver();
            return emul_driver->mmap(tc, start, length, prot, tgt_flags,
                                     tgt_fd, offset);
        }

        auto ffdp = std::dynamic_pointer_cast<FileFDEntry>(fdep);
        if (!ffdp)
            return -EBADF;
        sim_fd = ffdp->getSimFD();

        /**
         * Maintain the symbol table for dynamic executables.
         * The loader will call mmap to map the images into its address
         * space and we intercept that here. We can verify that we are
         * executing inside the loader by checking the program counter value.
         * XXX: with multiprogrammed workloads or multi-node configurations,
         * this will not work since there is a single global symbol table.
         */
        if (p->interpImage.contains(tc->pcState().instAddr())) {
            std::shared_ptr<FDEntry> fdep = (*p->fds)[tgt_fd];
            auto ffdp = std::dynamic_pointer_cast<FileFDEntry>(fdep);
            auto *lib = loader::createObjectFile(p->checkPathRedirect(
                    ffdp->getFileName()));
            DPRINTF_SYSCALL(Verbose, "Loading symbols from %s\n",
                ffdp->getFileName());

            if (lib) {
                Addr offset = lib->buildImage().minAddr() + start;
                loader::debugSymbolTable.insert(*lib->symtab().offset(offset));
            }
        }
    }

    /**
     * Not TGT_MAP_FIXED means we can start wherever we want.
     */
    if (!(tgt_flags & OS::TGT_MAP_FIXED)) {
        /**
         * If the application provides us with a hint, we should make some
         * small amount of effort to accomodate it.  Basically, we check if
         * every single VA within the requested range is unused.  If it is,
         * we give the application the range.  If not, we fall back to
         * extending the global mmap region.
         */
        if (!(start && p->memState->isUnmapped(start, length))) {
            /**
            * Extend global mmap region to give us some room for the app.
            */
            start = p->memState->extendMmap(length);
        }
    }

    DPRINTF_SYSCALL(Verbose, " mmap range is 0x%x - 0x%x\n",
                    start, start + length - 1);

    /**
     * We only allow mappings to overwrite existing mappings if
     * TGT_MAP_FIXED is set. Otherwise it shouldn't be a problem
     * because we ignore the start hint if TGT_MAP_FIXED is not set.
     */
    if (tgt_flags & OS::TGT_MAP_FIXED) {
        /**
         * We might already have some old VMAs mapped to this region, so
         * make sure to clear em out!
         */
        p->memState->unmapRegion(start, length);
    }

    /**
     * Figure out a human-readable name for the mapping.
     */
    std::string region_name;
    if (tgt_flags & OS::TGT_MAP_ANONYMOUS) {
        region_name = "anon";
    } else {
        std::shared_ptr<FDEntry> fdep = (*p->fds)[tgt_fd];
        auto ffdp = std::dynamic_pointer_cast<FileFDEntry>(fdep);
        region_name = ffdp->getFileName();
    }

    /**
     * Setup the correct VMA for this region.  The physical pages will be
     * mapped lazily.
     */
    p->memState->mapRegion(start, length, region_name, sim_fd, offset);

    return (Addr)start;
}

template <class OS>
SyscallReturn
pread64Func(SyscallDesc *desc, ThreadContext *tc,
            int tgt_fd, VPtr<> bufPtr, int nbytes, int offset)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    BufferArg bufArg(bufPtr, nbytes);

    int bytes_read = pread(sim_fd, bufArg.bufferPtr(), nbytes, offset);

    bufArg.copyOut(SETranslatingPortProxy(tc));

    return (bytes_read == -1) ? -errno : bytes_read;
}

template <class OS>
SyscallReturn
pwrite64Func(SyscallDesc *desc, ThreadContext *tc,
             int tgt_fd, VPtr<> bufPtr, int nbytes, int offset)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    BufferArg bufArg(bufPtr, nbytes);
    bufArg.copyIn(SETranslatingPortProxy(tc));

    int bytes_written = pwrite(sim_fd, bufArg.bufferPtr(), nbytes, offset);

    return (bytes_written == -1) ? -errno : bytes_written;
}

/// Target mmap2() handler.
template <class OS>
SyscallReturn
mmap2Func(SyscallDesc *desc, ThreadContext *tc,
          VPtr<> start, typename OS::size_t length, int prot,
          int tgt_flags, int tgt_fd, typename OS::off_t offset)
{
    auto page_size = tc->getProcessPtr()->pTable->pageSize();
    return mmapFunc<OS>(desc, tc, start, length, prot, tgt_flags,
                        tgt_fd, offset * page_size);
}

/// Target getrlimit() handler.
template <class OS>
SyscallReturn
getrlimitFunc(SyscallDesc *desc, ThreadContext *tc,
              unsigned resource, VPtr<typename OS::rlimit> rlp)
{
    const ByteOrder bo = OS::byteOrder;
    switch (resource) {
      case OS::TGT_RLIMIT_STACK:
        // max stack size in bytes: make up a number (8MiB for now)
        rlp->rlim_cur = rlp->rlim_max = 8 * 1024 * 1024;
        rlp->rlim_cur = htog(rlp->rlim_cur, bo);
        rlp->rlim_max = htog(rlp->rlim_max, bo);
        break;

      case OS::TGT_RLIMIT_DATA:
        // max data segment size in bytes: make up a number
        rlp->rlim_cur = rlp->rlim_max = 256 * 1024 * 1024;
        rlp->rlim_cur = htog(rlp->rlim_cur, bo);
        rlp->rlim_max = htog(rlp->rlim_max, bo);
        break;

      case OS::TGT_RLIMIT_NPROC:
        rlp->rlim_cur = rlp->rlim_max = tc->getSystemPtr()->threads.size();
        rlp->rlim_cur = htog(rlp->rlim_cur, bo);
        rlp->rlim_max = htog(rlp->rlim_max, bo);
        break;

      default:
        warn("getrlimit: unimplemented resource %d", resource);
        return -EINVAL;
        break;
    }

    return 0;
}

template <class OS>
SyscallReturn
prlimitFunc(SyscallDesc *desc, ThreadContext *tc,
            int pid, int resource, VPtr<> n, VPtr<typename OS::rlimit> rlp)
{
    if (pid != 0) {
        warn("prlimit: ignoring rlimits for nonzero pid");
        return -EPERM;
    }
    if (n)
        warn("prlimit: ignoring new rlimit");
    if (rlp) {
        const ByteOrder bo = OS::byteOrder;
        switch (resource) {
          case OS::TGT_RLIMIT_STACK:
            // max stack size in bytes: make up a number (8MiB for now)
            rlp->rlim_cur = rlp->rlim_max = 8 * 1024 * 1024;
            rlp->rlim_cur = htog(rlp->rlim_cur, bo);
            rlp->rlim_max = htog(rlp->rlim_max, bo);
            break;
          case OS::TGT_RLIMIT_DATA:
            // max data segment size in bytes: make up a number
            rlp->rlim_cur = rlp->rlim_max = 256*1024*1024;
            rlp->rlim_cur = htog(rlp->rlim_cur, bo);
            rlp->rlim_max = htog(rlp->rlim_max, bo);
            break;
          default:
            warn("prlimit: unimplemented resource %d", resource);
            return -EINVAL;
            break;
        }
    }
    return 0;
}

/// Target clock_gettime() function.
template <class OS>
SyscallReturn
clock_gettimeFunc(SyscallDesc *desc, ThreadContext *tc,
                  int clk_id, VPtr<typename OS::timespec> tp)
{
    getElapsedTimeNano(tp->tv_sec, tp->tv_nsec);
    tp->tv_sec += seconds_since_epoch;
    tp->tv_sec = htog(tp->tv_sec, OS::byteOrder);
    tp->tv_nsec = htog(tp->tv_nsec, OS::byteOrder);

    return 0;
}

/// Target clock_getres() function.
template <class OS>
SyscallReturn
clock_getresFunc(SyscallDesc *desc, ThreadContext *tc, int clk_id,
                 VPtr<typename OS::timespec> tp)
{
    // Set resolution at ns, which is what clock_gettime() returns
    tp->tv_sec = 0;
    tp->tv_nsec = 1;

    return 0;
}

/// Target gettimeofday() handler.
template <class OS>
SyscallReturn
gettimeofdayFunc(SyscallDesc *desc, ThreadContext *tc,
                 VPtr<typename OS::timeval> tp, VPtr<> tz_ptr)
{
    getElapsedTimeMicro(tp->tv_sec, tp->tv_usec);
    tp->tv_sec += seconds_since_epoch;
    tp->tv_sec = htog(tp->tv_sec, OS::byteOrder);
    tp->tv_usec = htog(tp->tv_usec, OS::byteOrder);

    return 0;
}

/// Target futimesat() handler.
template <class OS>
SyscallReturn
futimesatFunc(SyscallDesc *desc, ThreadContext *tc,
              int dirfd, VPtr<> pathname, VPtr<typename OS::timeval [2]> tp)
{
    std::string path;
    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Modifying path from the directory descriptor
    if (auto res = atSyscallPath<OS>(tc, dirfd, path); !res.successful()) {
        return res;
    }

    struct timeval hostTimeval[2];
    for (int i = 0; i < 2; ++i) {
        hostTimeval[i].tv_sec = gtoh((*tp)[i].tv_sec, OS::byteOrder);
        hostTimeval[i].tv_usec = gtoh((*tp)[i].tv_usec, OS::byteOrder);
    }

    // Adjust path for cwd and redirection
    auto process = tc->getProcessPtr();
    path = process->checkPathRedirect(path);

    int result = utimes(path.c_str(), hostTimeval);

    if (result < 0)
        return -errno;

    return 0;
}

/// Target utimes() handler.
template <class OS>
SyscallReturn
utimesFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> pathname,
           VPtr<typename OS::timeval [2]> tp)
{
    return futimesatFunc<OS>(desc, tc, OS::TGT_AT_FDCWD, pathname, tp);
}

template <class OS>
SyscallReturn
execveFunc(SyscallDesc *desc, ThreadContext *tc,
           VPtr<> pathname, VPtr<> argv_mem_loc, VPtr<> envp_mem_loc)
{
    auto p = tc->getProcessPtr();

    std::string path;
    SETranslatingPortProxy mem_proxy(tc);
    if (!mem_proxy.tryReadString(path, pathname))
        return -EFAULT;

    if (access(path.c_str(), F_OK) == -1)
        return -EACCES;

    auto read_in = [](std::vector<std::string> &vect,
                      PortProxy &mem_proxy, VPtr<> mem_loc)
    {
        for (int inc = 0; ; inc++) {
            BufferArg b((mem_loc + sizeof(Addr) * inc), sizeof(Addr));
            b.copyIn(mem_proxy);

            if (!*(Addr*)b.bufferPtr())
                break;

            vect.push_back(std::string());
            mem_proxy.tryReadString(vect[inc], *(Addr*)b.bufferPtr());
        }
    };

    /**
     * If we were a thread created by a clone with vfork set, wake up
     * the thread that created us
     */
    if (!p->vforkContexts.empty()) {
        ThreadContext *vtc = p->system->threads[p->vforkContexts.front()];
        assert(vtc->status() == ThreadContext::Suspended);
        vtc->activate();
    }

    /**
     * Note that ProcessParams is generated by swig and there are no other
     * examples of how to create anything but this default constructor. The
     * fields are manually initialized instead of passing parameters to the
     * constructor.
     */
    ProcessParams *pp = new ProcessParams();
    pp->executable = path;
    read_in(pp->cmd, mem_proxy, argv_mem_loc);
    read_in(pp->env, mem_proxy, envp_mem_loc);
    pp->uid = p->uid();
    pp->egid = p->egid();
    pp->euid = p->euid();
    pp->gid = p->gid();
    pp->ppid = p->ppid();
    pp->pid = p->pid();
    pp->input.assign("cin");
    pp->output.assign("cout");
    pp->errout.assign("cerr");
    pp->cwd.assign(p->tgtCwd);
    pp->system = p->system;
    pp->release = p->release;
    /**
     * Prevent process object creation with identical PIDs (which will trip
     * a fatal check in Process constructor). The execve call is supposed to
     * take over the currently executing process' identity but replace
     * whatever it is doing with a new process image. Instead of hijacking
     * the process object in the simulator, we create a new process object
     * and bind to the previous process' thread below (hijacking the thread).
     */
    p->system->PIDs.erase(p->pid());
    Process *new_p = pp->create();
    // TODO: there is no way to know when the Process SimObject is done with
    // the params pointer. Both the params pointer (pp) and the process
    // pointer (p) are normally managed in python and are never cleaned up.

    /**
     * Work through the file descriptor array and close any files marked
     * close-on-exec.
     */
    new_p->fds = p->fds;
    for (int i = 0; i < new_p->fds->getSize(); i++) {
        std::shared_ptr<FDEntry> fdep = (*new_p->fds)[i];
        if (fdep && fdep->getCOE())
            new_p->fds->closeFDEntry(i);
    }

    *new_p->sigchld = true;

    tc->clearArchRegs();
    tc->setProcessPtr(new_p);
    new_p->assignThreadContext(tc->contextId());
    new_p->init();
    new_p->initState();
    tc->activate();

    return SyscallReturn();
}

/// Target getrusage() function.
template <class OS>
SyscallReturn
getrusageFunc(SyscallDesc *desc, ThreadContext *tc,
              int who /* THREAD, SELF, or CHILDREN */,
              VPtr<typename OS::rusage> rup)
{
    rup->ru_utime.tv_sec = 0;
    rup->ru_utime.tv_usec = 0;
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

    switch (who) {
      case OS::TGT_RUSAGE_SELF:
        getElapsedTimeMicro(rup->ru_utime.tv_sec, rup->ru_utime.tv_usec);
        rup->ru_utime.tv_sec = htog(rup->ru_utime.tv_sec, OS::byteOrder);
        rup->ru_utime.tv_usec = htog(rup->ru_utime.tv_usec, OS::byteOrder);
        break;

      case OS::TGT_RUSAGE_CHILDREN:
        // do nothing.  We have no child processes, so they take no time.
        break;

      default:
        // don't really handle THREAD or CHILDREN, but just warn and
        // plow ahead
        warn("getrusage() only supports RUSAGE_SELF.  Parameter %d ignored.",
             who);
    }

    return 0;
}

/// Target times() function.
template <class OS>
SyscallReturn
timesFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<typename OS::tms> bufp)
{
    // Fill in the time structure (in clocks)
    int64_t clocks = curTick() * OS::M5_SC_CLK_TCK / sim_clock::as_int::s;
    bufp->tms_utime = clocks;
    bufp->tms_stime = 0;
    bufp->tms_cutime = 0;
    bufp->tms_cstime = 0;

    // Convert to host endianness
    bufp->tms_utime = htog(bufp->tms_utime, OS::byteOrder);

    // Return clock ticks since system boot
    return clocks;
}

/// Target time() function.
template <class OS>
SyscallReturn
timeFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> taddr)
{
    typename OS::time_t sec, usec;
    getElapsedTimeMicro(sec, usec);
    sec += seconds_since_epoch;

    SETranslatingPortProxy p(tc);
    if (taddr != 0) {
        typename OS::time_t t = sec;
        t = htog(t, OS::byteOrder);
        p.writeBlob(taddr, &t, (int)sizeof(typename OS::time_t));
    }
    return sec;
}

template <class OS>
SyscallReturn
tgkillFunc(SyscallDesc *desc, ThreadContext *tc, int tgid, int tid, int sig)
{
    /**
     * This system call is intended to allow killing a specific thread
     * within an arbitrary thread group if sanctioned with permission checks.
     * It's usually true that threads share the termination signal as pointed
     * out by the pthread_kill man page and this seems to be the intended
     * usage. Due to this being an emulated environment, assume the following:
     * Threads are allowed to call tgkill because the EUID for all threads
     * should be the same. There is no signal handling mechanism for kernel
     * registration of signal handlers since signals are poorly supported in
     * emulation mode. Since signal handlers cannot be registered, all
     * threads within in a thread group must share the termination signal.
     * We never exhaust PIDs so there's no chance of finding the wrong one
     * due to PID rollover.
     */

    System *sys = tc->getSystemPtr();
    Process *tgt_proc = nullptr;
    for (auto *tc: sys->threads) {
        Process *temp = tc->getProcessPtr();
        if (temp->pid() == tid) {
            tgt_proc = temp;
            break;
        }
    }

    if (sig != 0 || sig != OS::TGT_SIGABRT)
        return -EINVAL;

    if (tgt_proc == nullptr)
        return -ESRCH;

    if (tgid != -1 && tgt_proc->tgid() != tgid)
        return -ESRCH;

    if (sig == OS::TGT_SIGABRT)
        exitGroupFunc(desc, tc, 0);

    return 0;
}

template <class OS>
SyscallReturn
socketFunc(SyscallDesc *desc, ThreadContext *tc,
           int domain, int type, int prot)
{
    auto p = tc->getProcessPtr();

    int sim_fd = socket(domain, type, prot);
    if (sim_fd == -1)
        return -errno;

    auto sfdp = std::make_shared<SocketFDEntry>(sim_fd, domain, type, prot);
    int tgt_fd = p->fds->allocFD(sfdp);

    return tgt_fd;
}

template <class OS>
SyscallReturn
socketpairFunc(SyscallDesc *desc, ThreadContext *tc,
               int domain, int type, int prot, VPtr<> svPtr)
{
    auto p = tc->getProcessPtr();

    BufferArg svBuf((Addr)svPtr, 2 * sizeof(int));
    int status = socketpair(domain, type, prot, (int *)svBuf.bufferPtr());
    if (status == -1)
        return -errno;

    int *fds = (int *)svBuf.bufferPtr();

    auto sfdp1 = std::make_shared<SocketFDEntry>(fds[0], domain, type, prot);
    fds[0] = p->fds->allocFD(sfdp1);
    auto sfdp2 = std::make_shared<SocketFDEntry>(fds[1], domain, type, prot);
    fds[1] = p->fds->allocFD(sfdp2);
    svBuf.copyOut(SETranslatingPortProxy(tc));

    return status;
}

template <class OS>
SyscallReturn
selectFunc(SyscallDesc *desc, ThreadContext *tc, int nfds,
           VPtr<typename OS::fd_set> readfds,
           VPtr<typename OS::fd_set> writefds,
           VPtr<typename OS::fd_set> errorfds,
           VPtr<typename OS::timeval> timeout)
{
    int retval;

    auto p = tc->getProcessPtr();

    /**
     * Host fields. Notice that these use the definitions from the system
     * headers instead of the gem5 headers and libraries. If the host and
     * target have different header file definitions, this will not work.
     */
    fd_set readfds_h;
    FD_ZERO(&readfds_h);
    fd_set writefds_h;
    FD_ZERO(&writefds_h);
    fd_set errorfds_h;
    FD_ZERO(&errorfds_h);

    /**
     * We need to translate the target file descriptor set into a host file
     * descriptor set. This involves both our internal process fd array
     * and the fd_set defined in Linux header files. The nfds field also
     * needs to be updated as it will be only target specific after
     * retrieving it from the target; the nfds value is expected to be the
     * highest file descriptor that needs to be checked, so we need to extend
     * it out for nfds_h when we do the update.
     */
    int nfds_h = 0;
    std::map<int, int> trans_map;
    auto try_add_host_set = [&](typename OS::fd_set *tgt_set_entry,
                                fd_set *hst_set_entry,
                                int iter) -> bool
    {
        /**
         * By this point, we know that we are looking at a valid file
         * descriptor set on the target. We need to check if the target file
         * descriptor value passed in as iter is part of the set.
         */
        if (FD_ISSET(iter, (fd_set *)tgt_set_entry)) {
            /**
             * We know that the target file descriptor belongs to the set,
             * but we do not yet know if the file descriptor is valid or
             * that we have a host mapping. Check that now.
             */
            auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[iter]);
            if (!hbfdp)
                return true;
            auto sim_fd = hbfdp->getSimFD();

            /**
             * Add the sim_fd to tgt_fd translation into trans_map for use
             * later when we need to zero the target fd_set structures and
             * then update them with hits returned from the host select call.
             */
            trans_map[sim_fd] = iter;

            /**
             * We know that the host file descriptor exists so now we check
             * if we need to update the max count for nfds_h before passing
             * the duplicated structure into the host.
             */
            nfds_h = std::max(nfds_h - 1, sim_fd + 1);

            /**
             * Add the host file descriptor to the set that we are going to
             * pass into the host.
             */
            FD_SET(sim_fd, hst_set_entry);
        }
        return false;
    };

    for (int i = 0; i < nfds; i++) {
        if (readfds) {
            bool ebadf = try_add_host_set(readfds, &readfds_h, i);
            if (ebadf)
                return -EBADF;
        }
        if (writefds) {
            bool ebadf = try_add_host_set(writefds, &writefds_h, i);
            if (ebadf)
                return -EBADF;
        }
        if (errorfds) {
            bool ebadf = try_add_host_set(errorfds, &errorfds_h, i);
            if (ebadf)
                return -EBADF;
        }
    }

    if (timeout) {
        /**
         * It might be possible to decrement the timeval based on some
         * derivation of wall clock determined from elapsed simulator ticks
         * but that seems like overkill. Rather, we just set the timeval with
         * zero timeout. (There is no reason to block during the simulation
         * as it only decreases simulator performance.)
         */
        timeout->tv_sec = 0;
        timeout->tv_usec = 0;

        retval = select(nfds_h,
                        readfds ? &readfds_h : nullptr,
                        writefds ? &writefds_h : nullptr,
                        errorfds ? &errorfds_h : nullptr,
                        (timeval *)(typename OS::timeval *)timeout);
    } else {
        /**
         * If the timeval pointer is null, setup a new timeval structure to
         * pass into the host select call. Unfortunately, we will need to
         * manually check the return value and throw a retry fault if the
         * return value is zero. Allowing the system call to block will
         * likely deadlock the event queue.
         */
        struct timeval tv = { 0, 0 };

        retval = select(nfds_h,
                        readfds ? &readfds_h : nullptr,
                        readfds ? &writefds_h : nullptr,
                        readfds ? &errorfds_h : nullptr,
                        &tv);

        if (retval == 0) {
            /**
             * If blocking indefinitely, check the signal list to see if a
             * signal would break the poll out of the retry cycle and try to
             * return the signal interrupt instead.
             */
            for (auto sig : tc->getSystemPtr()->signalList)
                if (sig.receiver == p)
                    return -EINTR;
            return SyscallReturn::retry();
        }
    }

    if (retval == -1)
        return -errno;

    if (readfds) {
        FD_ZERO(reinterpret_cast<fd_set *>((typename OS::fd_set *)readfds));
    }
    if (writefds) {
        FD_ZERO(reinterpret_cast<fd_set *>((typename OS::fd_set *)writefds));
    }
    if (errorfds) {
        FD_ZERO(reinterpret_cast<fd_set *>((typename OS::fd_set *)errorfds));
    }

    /**
     * We need to translate the host file descriptor set into a target file
     * descriptor set. This involves both our internal process fd array
     * and the fd_set defined in header files.
     */
    for (int i = 0; i < nfds_h; i++) {
        if (readfds && FD_ISSET(i, &readfds_h))
            FD_SET(trans_map[i],
                   reinterpret_cast<fd_set *>(
                       (typename OS::fd_set *)readfds));

        if (writefds && FD_ISSET(i, &writefds_h))
            FD_SET(trans_map[i],
                   reinterpret_cast<fd_set *>(
                       (typename OS::fd_set *)writefds));

        if (errorfds && FD_ISSET(i, &errorfds_h))
            FD_SET(trans_map[i],
                   reinterpret_cast<fd_set *>(
                       (typename OS::fd_set *)errorfds));
    }

    return retval;
}

template <class OS>
SyscallReturn
readFunc(SyscallDesc *desc, ThreadContext *tc,
        int tgt_fd, VPtr<> buf_ptr, int nbytes)
{
    auto p = tc->getProcessPtr();

    auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!hbfdp)
        return -EBADF;
    int sim_fd = hbfdp->getSimFD();

    struct pollfd pfd;
    pfd.fd = sim_fd;
    pfd.events = POLLIN | POLLPRI;
    if ((poll(&pfd, 1, 0) == 0)
        && !(hbfdp->getFlags() & OS::TGT_O_NONBLOCK))
        return SyscallReturn::retry();

    BufferArg buf_arg(buf_ptr, nbytes);
    int bytes_read = read(sim_fd, buf_arg.bufferPtr(), nbytes);

    if (bytes_read > 0)
        buf_arg.copyOut(SETranslatingPortProxy(tc));

    return (bytes_read == -1) ? -errno : bytes_read;
}

template <class OS>
SyscallReturn
writeFunc(SyscallDesc *desc, ThreadContext *tc,
        int tgt_fd, VPtr<> buf_ptr, int nbytes)
{
    auto p = tc->getProcessPtr();

    auto hbfdp = std::dynamic_pointer_cast<HBFDEntry>((*p->fds)[tgt_fd]);
    if (!hbfdp)
        return -EBADF;
    int sim_fd = hbfdp->getSimFD();

    BufferArg buf_arg(buf_ptr, nbytes);
    buf_arg.copyIn(SETranslatingPortProxy(tc));

    struct pollfd pfd;
    pfd.fd = sim_fd;
    pfd.events = POLLOUT;

    /**
     * We don't want to poll on /dev/random. The kernel will not enable the
     * file descriptor for writing unless the entropy in the system falls
     * below write_wakeup_threshold. This is not guaranteed to happen
     * depending on host settings.
     */
    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>(hbfdp);
    if (ffdp && (ffdp->getFileName() != "/dev/random")) {
        if (!poll(&pfd, 1, 0) && !(ffdp->getFlags() & OS::TGT_O_NONBLOCK))
            return SyscallReturn::retry();
    }

    int bytes_written = write(sim_fd, buf_arg.bufferPtr(), nbytes);

    if (bytes_written != -1)
        fsync(sim_fd);

    return (bytes_written == -1) ? -errno : bytes_written;
}

template <class OS>
SyscallReturn
wait4Func(SyscallDesc *desc, ThreadContext *tc,
          pid_t pid, VPtr<> statPtr, int options, VPtr<> rusagePtr)
{
    auto p = tc->getProcessPtr();

    if (rusagePtr)
        DPRINTF_SYSCALL(Verbose, "wait4: rusage pointer provided %lx, however "
                 "functionality not supported. Ignoring rusage pointer.\n",
                 rusagePtr);

    /**
     * Currently, wait4 is only implemented so that it will wait for children
     * exit conditions which are denoted by a SIGCHLD signals posted into the
     * system signal list. We return no additional information via any of the
     * parameters supplied to wait4. If nothing is found in the system signal
     * list, we will wait indefinitely for SIGCHLD to post by retrying the
     * call.
     */
    System *sysh = tc->getSystemPtr();
    std::list<BasicSignal>::iterator iter;
    for (iter=sysh->signalList.begin(); iter!=sysh->signalList.end(); iter++) {
        if (iter->receiver == p) {
            if (pid < -1) {
                if ((iter->sender->pgid() == -pid)
                    && (iter->signalValue == OS::TGT_SIGCHLD))
                    goto success;
            } else if (pid == -1) {
                if (iter->signalValue == OS::TGT_SIGCHLD)
                    goto success;
            } else if (pid == 0) {
                if ((iter->sender->pgid() == p->pgid())
                    && (iter->signalValue == OS::TGT_SIGCHLD))
                    goto success;
            } else {
                if ((iter->sender->pid() == pid)
                    && (iter->signalValue == OS::TGT_SIGCHLD))
                    goto success;
            }
        }
    }

    return (options & OS::TGT_WNOHANG) ? 0 : SyscallReturn::retry();

success:
    // Set status to EXITED for WIFEXITED evaluations.
    const int EXITED = 0;
    BufferArg statusBuf(statPtr, sizeof(int));
    *(int *)statusBuf.bufferPtr() = EXITED;
    statusBuf.copyOut(SETranslatingPortProxy(tc));

    // Return the child PID.
    pid_t retval = iter->sender->pid();
    sysh->signalList.erase(iter);
    return retval;
}

template <class OS>
SyscallReturn
acceptFunc(SyscallDesc *desc, ThreadContext *tc,
           int tgt_fd, VPtr<> addrPtr, VPtr<> lenPtr)
{
    struct sockaddr sa;
    socklen_t addrLen;
    int host_fd;
    auto p = tc->getProcessPtr();

    BufferArg *lenBufPtr = nullptr;
    BufferArg *addrBufPtr = nullptr;

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    /**
     * We poll the socket file descriptor first to guarantee that we do not
     * block on our accept call. The socket can be opened without the
     * non-blocking flag (it blocks). This will cause deadlocks between
     * communicating processes.
     */
    struct pollfd pfd;
    pfd.fd = sim_fd;
    pfd.events = POLLIN | POLLPRI;
    if ((poll(&pfd, 1, 0) == 0) && !(sfdp->getFlags() & OS::TGT_O_NONBLOCK))
        return SyscallReturn::retry();

    if (lenPtr) {
        lenBufPtr = new BufferArg(lenPtr, sizeof(socklen_t));
        lenBufPtr->copyIn(SETranslatingPortProxy(tc));
        memcpy(&addrLen, (socklen_t *)lenBufPtr->bufferPtr(),
               sizeof(socklen_t));
    }

    if (addrPtr) {
        addrBufPtr = new BufferArg(addrPtr, sizeof(struct sockaddr));
        addrBufPtr->copyIn(SETranslatingPortProxy(tc));
        memcpy(&sa, (struct sockaddr *)addrBufPtr->bufferPtr(),
               sizeof(struct sockaddr));
    }

    host_fd = accept(sim_fd, &sa, &addrLen);

    if (host_fd == -1)
        return -errno;

    if (addrPtr) {
        memcpy(addrBufPtr->bufferPtr(), &sa, sizeof(sa));
        addrBufPtr->copyOut(SETranslatingPortProxy(tc));
        delete(addrBufPtr);
    }

    if (lenPtr) {
        *(socklen_t *)lenBufPtr->bufferPtr() = addrLen;
        lenBufPtr->copyOut(SETranslatingPortProxy(tc));
        delete(lenBufPtr);
    }

    auto afdp = std::make_shared<SocketFDEntry>(host_fd, sfdp->_domain,
                                                sfdp->_type, sfdp->_protocol);
    return p->fds->allocFD(afdp);
}

/// Target eventfd() function.
template <class OS>
SyscallReturn
eventfdFunc(SyscallDesc *desc, ThreadContext *tc,
            unsigned initval, int in_flags)
{
#if defined(__linux__)
    auto p = tc->getProcessPtr();

    int sim_fd = eventfd(initval, in_flags);
    if (sim_fd == -1)
        return -errno;

    bool cloexec = in_flags & OS::TGT_O_CLOEXEC;

    int flags = cloexec ? OS::TGT_O_CLOEXEC : 0;
    flags |= (in_flags & OS::TGT_O_NONBLOCK) ? OS::TGT_O_NONBLOCK : 0;

    auto hbfdp = std::make_shared<HBFDEntry>(flags, sim_fd, cloexec);
    int tgt_fd = p->fds->allocFD(hbfdp);
    return tgt_fd;
#else
    warnUnsupportedOS("eventfd");
    return -1;
#endif
}

/// Target sched_getaffinity
template <class OS>
SyscallReturn
schedGetaffinityFunc(SyscallDesc *desc, ThreadContext *tc,
                     pid_t pid, typename OS::size_t cpusetsize,
                     VPtr<> cpu_set_mask)
{
#if defined(__linux__)
    if (cpusetsize < CPU_ALLOC_SIZE(tc->getSystemPtr()->threads.size()))
        return -EINVAL;

    SETranslatingPortProxy proxy(tc);
    BufferArg maskBuf(cpu_set_mask, cpusetsize);
    maskBuf.copyIn(proxy);
    for (int i = 0; i < tc->getSystemPtr()->threads.size(); i++) {
        CPU_SET(i, (cpu_set_t *)maskBuf.bufferPtr());
    }
    maskBuf.copyOut(proxy);
    return CPU_ALLOC_SIZE(tc->getSystemPtr()->threads.size());
#else
    warnUnsupportedOS("sched_getaffinity");
    return -1;
#endif
}

// Target recvfrom() handler.
template <class OS>
SyscallReturn
recvfromFunc(SyscallDesc *desc, ThreadContext *tc,
             int tgt_fd, VPtr<> buf_ptr, typename OS::size_t buf_len,
             int flags, VPtr<> addr_ptr, VPtr<> addrlen_ptr)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    // Reserve buffer space.
    BufferArg buf(buf_ptr, buf_len);

    SETranslatingPortProxy proxy(tc);

    // Get address length.
    socklen_t addr_len = 0;
    if (addrlen_ptr != 0) {
        // Read address length parameter.
        BufferArg addrlen_buf(addrlen_ptr, sizeof(socklen_t));
        addrlen_buf.copyIn(proxy);
        addr_len = *((socklen_t *)addrlen_buf.bufferPtr());
    }

    struct sockaddr sa, *sap = NULL;
    if (addr_len != 0) {
        BufferArg addr_buf(addr_ptr, addr_len);
        addr_buf.copyIn(proxy);
        memcpy(&sa, (struct sockaddr *)addr_buf.bufferPtr(),
               sizeof(struct sockaddr));
        sap = &sa;
    }

    ssize_t recvd_size = recvfrom(sim_fd,
                                  (void *)buf.bufferPtr(),
                                  buf_len, flags, sap, (socklen_t *)&addr_len);

    if (recvd_size == -1)
        return -errno;

    // Pass the received data out.
    buf.copyOut(proxy);

    // Copy address to addr_ptr and pass it on.
    if (sap != NULL) {
        BufferArg addr_buf(addr_ptr, addr_len);
        memcpy(addr_buf.bufferPtr(), sap, sizeof(sa));
        addr_buf.copyOut(proxy);
    }

    // Copy len to addrlen_ptr and pass it on.
    if (addr_len != 0) {
        BufferArg addrlen_buf(addrlen_ptr, sizeof(socklen_t));
        *(socklen_t *)addrlen_buf.bufferPtr() = addr_len;
        addrlen_buf.copyOut(proxy);
    }

    return recvd_size;
}

// Target sendto() handler.
template <typename OS>
SyscallReturn
sendtoFunc(SyscallDesc *desc, ThreadContext *tc,
           int tgt_fd, VPtr<> buf_ptr, typename OS::size_t buf_len, int flags,
           VPtr<> addr_ptr, socklen_t addr_len)
{
    auto p = tc->getProcessPtr();

    auto sfdp = std::dynamic_pointer_cast<SocketFDEntry>((*p->fds)[tgt_fd]);
    if (!sfdp)
        return -EBADF;
    int sim_fd = sfdp->getSimFD();

    // Reserve buffer space.
    BufferArg buf(buf_ptr, buf_len);
    buf.copyIn(SETranslatingPortProxy(tc));

    struct sockaddr sa, *sap = nullptr;
    memset(&sa, 0, sizeof(sockaddr));
    if (addr_len != 0) {
        BufferArg addr_buf(addr_ptr, addr_len);
        addr_buf.copyIn(SETranslatingPortProxy(tc));
        memcpy(&sa, (sockaddr*)addr_buf.bufferPtr(), addr_len);
        sap = &sa;
    }

    ssize_t sent_size = sendto(sim_fd,
                               (void *)buf.bufferPtr(),
                               buf_len, flags, sap, (socklen_t)addr_len);

    return (sent_size == -1) ? -errno : sent_size;
}

/// Target munmap() handler.
template <typename OS>
SyscallReturn
munmapFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> start,
           typename OS::size_t length)
{
    // Even if the system is currently not capable of recycling physical
    // pages, there is no reason we can't unmap them so that we trigger
    // appropriate seg faults when the application mistakenly tries to
    // access them again.
    auto p = tc->getProcessPtr();

    if (p->pTable->pageOffset(start))
        return -EINVAL;

    length = roundUp(length, p->pTable->pageSize());

    p->memState->unmapRegion(start, length);

    return 0;
}

// Target fallocate() handler.
template <typename OS>
SyscallReturn
fallocateFunc(SyscallDesc *desc, ThreadContext *tc,
              int tgt_fd, int mode, typename OS::off_t offset,
              typename OS::off_t len)
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

/// Target truncate() handler.
template <typename OS>
SyscallReturn
truncateFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> pathname,
             typename OS::off_t length)
{
    std::string path;
    auto p = tc->getProcessPtr();

    if (!SETranslatingPortProxy(tc).tryReadString(path, pathname))
        return -EFAULT;

    // Adjust path for cwd and redirection
    path = p->checkPathRedirect(path);

    int result = truncate(path.c_str(), length);
    return (result == -1) ? -errno : result;
}

/// Target ftruncate() handler.
template <typename OS>
SyscallReturn
ftruncateFunc(SyscallDesc *desc, ThreadContext *tc, int tgt_fd,
              typename OS::off_t length)
{
    auto p = tc->getProcessPtr();

    auto ffdp = std::dynamic_pointer_cast<FileFDEntry>((*p->fds)[tgt_fd]);
    if (!ffdp)
        return -EBADF;
    int sim_fd = ffdp->getSimFD();

    int result = ftruncate(sim_fd, length);
    return (result == -1) ? -errno : result;
}

template <typename OS>
SyscallReturn
getrandomFunc(SyscallDesc *desc, ThreadContext *tc,
              VPtr<> buf_ptr, typename OS::size_t count,
              unsigned int flags)
{
    SETranslatingPortProxy proxy(tc);

    TypedBufferArg<uint8_t> buf(buf_ptr, count);
    for (int i = 0; i < count; ++i) {
        buf[i] = gem5::random_mt.random<uint8_t>();
    }
    buf.copyOut(proxy);

    return count;
}

} // namespace gem5

#endif // __SIM_SYSCALL_EMUL_HH__
