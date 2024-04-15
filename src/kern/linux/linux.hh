/*
 * Copyright (c) 2021 Arm Limited
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
 * Copyright (c) 2004-2009 The Regents of The University of Michigan
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

#ifndef __LINUX_HH__
#define __LINUX_HH__

#include <string>

#include "base/random.hh"
#include "base/types.hh"
#include "kern/operatingsystem.hh"
#include "sim/process.hh"

namespace gem5
{

class ThreadContext;

///
/// This class encapsulates the types, structures, constants,
/// functions, and syscall-number mappings specific to the Alpha Linux
/// syscall interface.
///
class Linux : public OperatingSystem
{
  public:
    //@{
    /// Basic Linux types.
    typedef uint64_t size_t;
    typedef uint64_t off_t;
    typedef int64_t time_t;
    typedef int64_t clock_t;
    typedef uint32_t uid_t;
    typedef uint32_t gid_t;

    //@}

    /// Stat buffer.  Note that we can't call it 'stat' since that
    /// gets #defined to something else on some systems. This type
    /// can be specialized by architecture specific "Linux" classes
    struct tgt_stat
    {
        uint32_t st_dev;     //!< device
        uint32_t st_ino;     //!< inode
        uint32_t st_mode;    //!< mode
        uint32_t st_nlink;   //!< link count
        uint32_t st_uid;     //!< owner's user ID
        uint32_t st_gid;     //!< owner's group ID
        uint32_t st_rdev;    //!< device number
        int32_t _pad1;       //!< for alignment
        int64_t st_size;     //!< file size in bytes
        uint64_t st_atimeX;  //!< time of last access
        uint64_t st_mtimeX;  //!< time of last modification
        uint64_t st_ctimeX;  //!< time of last status change
        uint32_t st_blksize; //!< optimal I/O block size
        int32_t st_blocks;   //!< number of blocks allocated
        uint32_t st_flags;   //!< flags
        uint32_t st_gen;     //!< unknown
    };

    // same for stat64
    struct tgt_stat64
    {
        uint64_t st_dev;
        uint64_t st_ino;
        uint64_t st_rdev;
        int64_t st_size;
        uint64_t st_blocks;

        uint32_t st_mode;
        uint32_t st_uid;
        uint32_t st_gid;
        uint32_t st_blksize;
        uint32_t st_nlink;
        uint32_t __pad0;

        uint64_t st_atimeX;
        uint64_t st_atime_nsec;
        uint64_t st_mtimeX;
        uint64_t st_mtime_nsec;
        uint64_t st_ctimeX;
        uint64_t st_ctime_nsec;
        int64_t ___unused[3];
    };

    /// Length of strings in struct utsname (plus 1 for null char).
    static const int _SYS_NMLN = 65;

    /// Interface struct for uname().
    struct utsname
    {
        char sysname[_SYS_NMLN];  //!< System name.
        char nodename[_SYS_NMLN]; //!< Node name.
        char release[_SYS_NMLN];  //!< OS release.
        char version[_SYS_NMLN];  //!< OS version.
        char machine[_SYS_NMLN];  //!< Machine type.
    };

    /// Limit struct for getrlimit/setrlimit.
    struct rlimit
    {
        uint64_t rlim_cur; //!< soft limit
        uint64_t rlim_max; //!< hard limit
    };

    /// For gettimeofday().
    struct timeval
    {
        int64_t tv_sec;  //!< seconds
        int64_t tv_usec; //!< microseconds
    };

    /// For clock_gettime().
    struct timespec
    {
        time_t tv_sec;   //!< seconds
        int64_t tv_nsec; //!< nanoseconds
    };

    /// Clock ticks per second, for times().
    static const int M5_SC_CLK_TCK = 100;

    /// For times().
    struct tms
    {
        int64_t tms_utime;  //!< user time
        int64_t tms_stime;  //!< system time
        int64_t tms_cutime; //!< user time of children
        int64_t tms_cstime; //!< system time of children
    };

    // For writev/readv
    struct tgt_iovec
    {
        uint64_t iov_base; // void *
        uint64_t iov_len;
    };

    // For select().
    // linux-3.14-src/include/uapi/linux/posix_types.h
    struct fd_set
    {
#ifndef LINUX__FD_SETSIZE
#define LINUX__FD_SETSIZE 1024
        unsigned long fds_bits[LINUX__FD_SETSIZE / (8 * sizeof(long))];
#endif
    };

    //@{
    /// ioctl() command codes.
    static const unsigned TGT_TCGETS = 0x5401;
    static const unsigned TGT_TCGETA = 0x5405;
    static const unsigned TGT_TCSETAW = 0x5407;
    static const unsigned TGT_FIONREAD = 0x541B;

    //@}

    /// Return true for the ioctl codes for which we return ENOTTY
    /// *without* printing a warning, since we know that ENOTTY is the
    /// correct thing to return (and not just a sign that we don't
    /// recognize the ioctl code.
    static bool
    isTtyReq(unsigned req)
    {
        switch (req) {
        case TGT_FIONREAD:
        case TGT_TCSETAW:
        case TGT_TCGETS:
        case TGT_TCGETA:
            return true;
        default:
            return false;
        }
    }

    /// Resource constants for getrlimit().
    static const unsigned TGT_RLIMIT_CPU = 0;
    static const unsigned TGT_RLIMIT_FSIZE = 1;
    static const unsigned TGT_RLIMIT_DATA = 2;
    static const unsigned TGT_RLIMIT_STACK = 3;
    static const unsigned TGT_RLIMIT_CORE = 4;
    static const unsigned TGT_RLIMIT_RSS = 5;
    static const unsigned TGT_RLIMIT_NPROC = 6;
    static const unsigned TGT_RLIMIT_NOFILE = 7;
    static const unsigned TGT_RLIMIT_MEMLOCK = 8;
    static const unsigned TGT_RLIMIT_AS = 9;
    static const unsigned TGT_RLIMIT_LOCKS = 10;
    static const unsigned TGT_RLIMIT_SIGPENDING = 11;
    static const unsigned TGT_RLIMIT_MSGQUEUE = 12;
    static const unsigned TGT_RLIMIT_NICE = 13;
    static const unsigned TGT_RLIMIT_RTPRIO = 14;
    static const unsigned TGT_RLIMIT_RTTIME = 15;
    static const unsigned TGT_RLIM_NLIMITS = 16;

    /// For getrusage().
    static const int TGT_RUSAGE_SELF = 0;
    static const int TGT_RUSAGE_CHILDREN = -1;
    static const int TGT_RUSAGE_BOTH = -2;

    struct rusage
    {
        struct timeval ru_utime; //!< user time used
        struct timeval ru_stime; //!< system time used
        int64_t ru_maxrss;       //!< max rss
        int64_t ru_ixrss;        //!< integral shared memory size
        int64_t ru_idrss;        //!< integral unshared data "
        int64_t ru_isrss;        //!< integral unshared stack "
        int64_t ru_minflt;       //!< page reclaims - total vmfaults
        int64_t ru_majflt;       //!< page faults
        int64_t ru_nswap;        //!< swaps
        int64_t ru_inblock;      //!< block input operations
        int64_t ru_oublock;      //!< block output operations
        int64_t ru_msgsnd;       //!< messages sent
        int64_t ru_msgrcv;       //!< messages received
        int64_t ru_nsignals;     //!< signals received
        int64_t ru_nvcsw;        //!< voluntary context switches
        int64_t ru_nivcsw;       //!< involuntary "
    };

    // For /dev/urandom accesses
    static Random random;

    static int openSpecialFile(std::string path, Process *process,
                               ThreadContext *tc);
    static std::string procMeminfo(Process *process, ThreadContext *tc);
    static std::string etcPasswd(Process *process, ThreadContext *tc);
    static std::string procSelfMaps(Process *process, ThreadContext *tc);
    static std::string cpuOnline(Process *process, ThreadContext *tc);
    static std::string devRandom(Process *process, ThreadContext *tc);

    // For futex system call
    static const unsigned TGT_FUTEX_WAIT = 0;
    static const unsigned TGT_FUTEX_WAKE = 1;
    static const unsigned TGT_FUTEX_REQUEUE = 3;
    static const unsigned TGT_FUTEX_CMP_REQUEUE = 4;
    static const unsigned TGT_FUTEX_WAKE_OP = 5;
    static const unsigned TGT_FUTEX_WAIT_BITSET = 9;
    static const unsigned TGT_FUTEX_WAKE_BITSET = 10;
    static const unsigned TGT_EAGAIN = 11;
    static const unsigned TGT_EWOULDBLOCK = TGT_EAGAIN;
    static const unsigned TGT_FUTEX_PRIVATE_FLAG = 128;
    static const unsigned TGT_FUTEX_CLOCK_REALTIME_FLAG = 256;
    // op field of futex_wake_op operation
    static const unsigned TGT_FUTEX_OP_SET = 0;  // uaddr2 = oparg;
    static const unsigned TGT_FUTEX_OP_ADD = 1;  // uaddr2 += oparg;
    static const unsigned TGT_FUTEX_OP_OR = 2;   // uaddr2 |= oparg;
    static const unsigned TGT_FUTEX_OP_ANDN = 3; // uaddr2 &= ~oparg;
    static const unsigned TGT_FUTEX_OP_XOR = 4;  // uaddr2 ^= oparg;
    // Use (1 << oparg) as operand
    static const unsigned TGT_FUTEX_OP_ARG_SHIFT = 8;
    // cmp field of futex_wake_op operation
    static const unsigned TGT_FUTEX_OP_CMP_EQ = 0;
    static const unsigned TGT_FUTEX_OP_CMP_NE = 1;
    static const unsigned TGT_FUTEX_OP_CMP_LT = 2;
    static const unsigned TGT_FUTEX_OP_CMP_LE = 3;
    static const unsigned TGT_FUTEX_OP_CMP_GT = 4;
    static const unsigned TGT_FUTEX_OP_CMP_GE = 5;

    // for *at syscalls
    static const int TGT_AT_FDCWD = -100;
    static const int TGT_AT_REMOVEDIR = 0x200;
    static const int TGT_AT_SYMLINK_FOLLOW = 0x400;
    static const int TGT_AT_NO_AUTOMOUNT = 0x800;
    static const int TGT_AT_EMPTY_PATH = 0x1000;

    // for MREMAP
    static const unsigned TGT_MREMAP_MAYMOVE = 0x1;
    static const unsigned TGT_MREMAP_FIXED = 0x2;

    static const unsigned TGT_CLONE_VM = 0x00000100;
    static const unsigned TGT_CLONE_FS = 0x00000200;
    static const unsigned TGT_CLONE_FILES = 0x00000400;
    static const unsigned TGT_CLONE_SIGHAND = 0x00000800;
    static const unsigned TGT_CLONE_PIDFD = 0x00001000;
    static const unsigned TGT_CLONE_PTRACE = 0x00002000;
    static const unsigned TGT_CLONE_VFORK = 0x00004000;
    static const unsigned TGT_CLONE_PARENT = 0x00008000;
    static const unsigned TGT_CLONE_THREAD = 0x00010000;
    static const unsigned TGT_CLONE_NEWNS = 0x00020000;
    static const unsigned TGT_CLONE_SYSVSEM = 0x00040000;
    static const unsigned TGT_CLONE_SETTLS = 0x00080000;
    static const unsigned TGT_CLONE_PARENT_SETTID = 0x00100000;
    static const unsigned TGT_CLONE_CHILD_CLEARTID = 0x00200000;
    static const unsigned TGT_CLONE_DETACHED = 0x00400000;
    static const unsigned TGT_CLONE_UNTRACED = 0x00800000;
    static const unsigned TGT_CLONE_CHILD_SETTID = 0x01000000;
    static const unsigned TGT_CLONE_NEWUTS = 0x04000000;
    static const unsigned TGT_CLONE_NEWIPC = 0x08000000;
    static const unsigned TGT_CLONE_NEWUSER = 0x10000000;
    static const unsigned TGT_CLONE_NEWPID = 0x20000000;
    static const unsigned TGT_CLONE_NEWNET = 0x40000000;
    static const unsigned TGT_CLONE_IO = 0x80000000;

    // linux-3.13-src/include/uapi/linux/wait.h
    static const unsigned TGT_WNOHANG = 0x00000001;
    static const unsigned TGT_WUNTRACED = 0x00000002;
    static const unsigned TGT_WSTOPPED = TGT_WUNTRACED;
    static const unsigned TGT_WEXITED = 0x00000004;
    static const unsigned TGT_WCONTINUED = 0x00000008;
    static const unsigned TGT_WNOWAIT = 0x01000000;
}; // class Linux

} // namespace gem5

#endif // __LINUX_HH__
