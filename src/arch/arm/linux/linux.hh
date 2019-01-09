/*
 * Copyright (c) 2010, 2011-2012, 2015 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Ali Saidi
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_LINUX_LINUX_HH__
#define __ARCH_ARM_LINUX_LINUX_HH__

#include "arch/arm/utility.hh"
#include "kern/linux/linux.hh"

class ArmLinux : public Linux
{
  public:
    static void
    archClone(uint64_t flags,
              Process *pp, Process *cp,
              ThreadContext *ptc, ThreadContext *ctc,
              uint64_t stack, uint64_t tls)
    {
        ArmISA::copyRegs(ptc, ctc);

        if (flags & TGT_CLONE_SETTLS) {
            /* TPIDR_EL0 is architecturally mapped to TPIDRURW, so
             * this works for both aarch32 and aarch64. */
            ctc->setMiscReg(ArmISA::MISCREG_TPIDR_EL0, tls);
        }
    }
};

class ArmLinux32 : public ArmLinux
{
  public:

    static const int TGT_SIGHUP         = 0x000001;
    static const int TGT_SIGINT         = 0x000002;
    static const int TGT_SIGQUIT        = 0x000003;
    static const int TGT_SIGILL         = 0x000004;
    static const int TGT_SIGTRAP        = 0x000005;
    static const int TGT_SIGABRT        = 0x000006;
    static const int TGT_SIGIOT         = 0x000006;
    static const int TGT_SIGBUS         = 0x000007;
    static const int TGT_SIGFPE         = 0x000008;
    static const int TGT_SIGKILL        = 0x000009;
    static const int TGT_SIGUSR1        = 0x00000a;
    static const int TGT_SIGSEGV        = 0x00000b;
    static const int TGT_SIGUSR2        = 0x00000c;
    static const int TGT_SIGPIPE        = 0x00000d;
    static const int TGT_SIGALRM        = 0x00000e;
    static const int TGT_SIGTERM        = 0x00000f;
    static const int TGT_SIGSTKFLT      = 0x000010;
    static const int TGT_SIGCHLD        = 0x000011;
    static const int TGT_SIGCONT        = 0x000012;
    static const int TGT_SIGSTOP        = 0x000013;
    static const int TGT_SIGTSTP        = 0x000014;
    static const int TGT_SIGTTIN        = 0x000015;
    static const int TGT_SIGTTOU        = 0x000016;
    static const int TGT_SIGURG         = 0x000017;
    static const int TGT_SIGXCPU        = 0x000018;
    static const int TGT_SIGXFSZ        = 0x000019;
    static const int TGT_SIGVTALRM      = 0x00001a;
    static const int TGT_SIGPROF        = 0x00001b;
    static const int TGT_SIGWINCH       = 0x00001c;
    static const int TGT_SIGIO          = 0x00001d;
    static const int TGT_SIGPOLL        = 0x00001d;
    static const int TGT_SIGPWR         = 0x00001e;
    static const int TGT_SIGSYS         = 0x00001f;
    static const int TGT_SIGUNUSED      = 0x00001f;

    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static SyscallFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    //@{
    /// Basic ARM Linux types
    typedef uint32_t size_t;
    typedef uint32_t off_t;
    typedef int32_t time_t;
    typedef int32_t clock_t;
    //@}

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY    = 000000000; //!< O_RDONLY
    static const int TGT_O_WRONLY    = 000000001; //!< O_WRONLY
    static const int TGT_O_RDWR      = 000000002; //!< O_RDWR
    static const int TGT_O_CREAT     = 000000100; //!< O_CREAT
    static const int TGT_O_EXCL      = 000000200; //!< O_EXCL
    static const int TGT_O_NOCTTY    = 000000400; //!< O_NOCTTY
    static const int TGT_O_TRUNC     = 000001000; //!< O_TRUNC
    static const int TGT_O_APPEND    = 000002000; //!< O_APPEND
    static const int TGT_O_NONBLOCK  = 000004000; //!< O_NONBLOCK
    static const int TGT_O_DSYNC     = 000010000; //!< O_DSYNC
    static const int TGT_FASYNC      = 000020000; //!< FASYNC
    static const int TGT_O_DIRECT    = 000200000; //!< O_DIRECT
    static const int TGT_O_LARGEFILE = 000400000; //!< O_LARGEFILE
    static const int TGT_O_DIRECTORY = 000040000; //!< O_DIRECTORY
    static const int TGT_O_NOFOLLOW  = 000100000; //!< O_NOFOLLOW
    static const int TGT_O_NOATIME   = 001000000; //!< O_NOATIME
    static const int TGT_O_CLOEXEC   = 002000000; //!< O_NOATIME
    static const int TGT_O_SYNC      = 004010000; //!< O_SYNC
    static const int TGT_O_PATH      = 010000000; //!< O_PATH
    //@}

    static const unsigned TGT_MAP_SHARED        = 0x00001;
    static const unsigned TGT_MAP_PRIVATE       = 0x00002;
    static const unsigned TGT_MAP_ANON          = 0x00020;
    static const unsigned TGT_MAP_DENYWRITE     = 0x00800;
    static const unsigned TGT_MAP_EXECUTABLE    = 0x01000;
    static const unsigned TGT_MAP_FILE          = 0x00000;
    static const unsigned TGT_MAP_GROWSDOWN     = 0x00100;
    static const unsigned TGT_MAP_HUGETLB       = 0x40000;
    static const unsigned TGT_MAP_LOCKED        = 0x02000;
    static const unsigned TGT_MAP_NONBLOCK      = 0x10000;
    static const unsigned TGT_MAP_NORESERVE     = 0x04000;
    static const unsigned TGT_MAP_POPULATE      = 0x08000;
    static const unsigned TGT_MAP_STACK         = 0x20000;
    static const unsigned TGT_MAP_ANONYMOUS     = 0x00020;
    static const unsigned TGT_MAP_FIXED         = 0x00010;

    static const unsigned NUM_MMAP_FLAGS;

    /// For table().
    static const int TBL_SYSINFO = 12;

    /// Limit struct for getrlimit/setrlimit.
    struct rlimit {
        uint32_t  rlim_cur;     //!< soft limit
        uint32_t  rlim_max;     //!< hard limit
    };

    /// For gettimeofday().
    struct timeval {
        int32_t tv_sec;         //!< seconds
        int32_t tv_usec;        //!< microseconds
    };

    struct timespec {
        int32_t tv_sec;   //!< seconds
        int32_t tv_nsec;  //!< nanoseconds
    };

    // For writev/readv
    struct tgt_iovec {
        uint32_t iov_base; // void *
        uint32_t iov_len;
    };


    typedef struct {
        uint32_t st_dev;
        uint32_t st_ino;
        uint16_t st_mode;
        uint16_t st_nlink;
        uint16_t st_uid;
        uint16_t st_gid;
        uint32_t st_rdev;
        uint32_t __pad1;
        uint32_t st_size;
        uint32_t st_blksize;
        uint32_t __pad2;
        uint32_t st_blocks;
        uint32_t st_atimeX;
        uint32_t st_atime_nsec;
        uint32_t st_mtimeX;
        uint32_t st_mtime_nsec;
        uint32_t st_ctimeX;
        uint32_t st_ctime_nsec;
    } tgt_stat;

    typedef struct {
        uint64_t  st_dev;
        uint8_t   __pad0[4];
        uint32_t  __st_ino;
        uint32_t  st_mode;
        uint32_t  st_nlink;
        uint32_t  st_uid;
        uint32_t  st_gid;
        uint64_t  st_rdev;
        uint8_t   __pad3[4];
        int64_t   __attribute__ ((aligned (8))) st_size;
        uint32_t  st_blksize;
        uint64_t  __attribute__ ((aligned (8))) st_blocks;
        uint32_t  st_atimeX;
        uint32_t  st_atime_nsec;
        uint32_t  st_mtimeX;
        uint32_t  st_mtime_nsec;
        uint32_t  st_ctimeX;
        uint32_t  st_ctime_nsec;
        uint64_t  st_ino;
    } tgt_stat64;

    typedef struct {
        int32_t  uptime;    /* Seconds since boot */
        uint32_t loads[3];  /* 1, 5, and 15 minute load averages */
        uint32_t totalram;  /* Total usable main memory size */
        uint32_t freeram;   /* Available memory size */
        uint32_t sharedram; /* Amount of shared memory */
        uint32_t bufferram; /* Memory used by buffers */
        uint32_t totalswap; /* Total swap space size */
        uint32_t freeswap;  /* swap space still available */
        uint16_t procs;     /* Number of current processes */
        uint32_t totalhigh; /* Total high memory size */
        uint32_t freehigh;  /* Available high memory size */
        uint32_t mem_unit;  /* Memory unit size in bytes */
    } tgt_sysinfo;

    /// For getrusage().
    struct rusage {
        struct timeval ru_utime;        //!< user time used
        struct timeval ru_stime;        //!< system time used
        int32_t ru_maxrss;              //!< max rss
        int32_t ru_ixrss;               //!< integral shared memory size
        int32_t ru_idrss;               //!< integral unshared data "
        int32_t ru_isrss;               //!< integral unshared stack "
        int32_t ru_minflt;              //!< page reclaims - total vmfaults
        int32_t ru_majflt;              //!< page faults
        int32_t ru_nswap;               //!< swaps
        int32_t ru_inblock;             //!< block input operations
        int32_t ru_oublock;             //!< block output operations
        int32_t ru_msgsnd;              //!< messages sent
        int32_t ru_msgrcv;              //!< messages received
        int32_t ru_nsignals;            //!< signals received
        int32_t ru_nvcsw;               //!< voluntary context switches
        int32_t ru_nivcsw;              //!< involuntary "
    };

    /// For times().
    struct tms {
        int32_t tms_utime;      //!< user time
        int32_t tms_stime;      //!< system time
        int32_t tms_cutime;     //!< user time of children
        int32_t tms_cstime;     //!< system time of children
    };

    static void
    archClone(uint64_t flags,
              Process *pp, Process *cp,
              ThreadContext *ptc, ThreadContext *ctc,
              uint64_t stack, uint64_t tls)
    {
        ArmLinux::archClone(flags, pp, cp, ptc, ctc, stack, tls);

        if (stack)
            ctc->setIntReg(ArmISA::INTREG_SP, stack);
    }
};

class ArmLinux64 : public ArmLinux
{
  public:

    static const int TGT_SIGHUP         = 0x000001;
    static const int TGT_SIGINT         = 0x000002;
    static const int TGT_SIGQUIT        = 0x000003;
    static const int TGT_SIGILL         = 0x000004;
    static const int TGT_SIGTRAP        = 0x000005;
    static const int TGT_SIGABRT        = 0x000006;
    static const int TGT_SIGIOT         = 0x000006;
    static const int TGT_SIGBUS         = 0x000007;
    static const int TGT_SIGFPE         = 0x000008;
    static const int TGT_SIGKILL        = 0x000009;
    static const int TGT_SIGUSR1        = 0x00000a;
    static const int TGT_SIGSEGV        = 0x00000b;
    static const int TGT_SIGUSR2        = 0x00000c;
    static const int TGT_SIGPIPE        = 0x00000d;
    static const int TGT_SIGALRM        = 0x00000e;
    static const int TGT_SIGTERM        = 0x00000f;
    static const int TGT_SIGSTKFLT      = 0x000010;
    static const int TGT_SIGCHLD        = 0x000011;
    static const int TGT_SIGCONT        = 0x000012;
    static const int TGT_SIGSTOP        = 0x000013;
    static const int TGT_SIGTSTP        = 0x000014;
    static const int TGT_SIGTTIN        = 0x000015;
    static const int TGT_SIGTTOU        = 0x000016;
    static const int TGT_SIGURG         = 0x000017;
    static const int TGT_SIGXCPU        = 0x000018;
    static const int TGT_SIGXFSZ        = 0x000019;
    static const int TGT_SIGVTALRM      = 0x00001a;
    static const int TGT_SIGPROF        = 0x00001b;
    static const int TGT_SIGWINCH       = 0x00001c;
    static const int TGT_SIGIO          = 0x00001d;
    static const int TGT_SIGPOLL        = 0x00001d;
    static const int TGT_SIGPWR         = 0x00001e;
    static const int TGT_SIGSYS         = 0x00001f;
    static const int TGT_SIGUNUSED      = 0x00001f;

    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static SyscallFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    //@{
    /// Basic ARM Linux types
    typedef uint64_t size_t;
    typedef uint64_t off_t;
    typedef int64_t time_t;
    typedef int64_t clock_t;
    //@}

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY    = 000000000; //!< O_RDONLY
    static const int TGT_O_WRONLY    = 000000001; //!< O_WRONLY
    static const int TGT_O_RDWR      = 000000002; //!< O_RDWR
    static const int TGT_O_CREAT     = 000000100; //!< O_CREAT
    static const int TGT_O_EXCL      = 000000200; //!< O_EXCL
    static const int TGT_O_NOCTTY    = 000000400; //!< O_NOCTTY
    static const int TGT_O_TRUNC     = 000001000; //!< O_TRUNC
    static const int TGT_O_APPEND    = 000002000; //!< O_APPEND
    static const int TGT_O_NONBLOCK  = 000004000; //!< O_NONBLOCK
    static const int TGT_O_DSYNC     = 000010000; //!< O_DSYNC
    static const int TGT_FASYNC      = 000020000; //!< FASYNC
    static const int TGT_O_DIRECT    = 000200000; //!< O_DIRECT
    static const int TGT_O_LARGEFILE = 000400000; //!< O_LARGEFILE
    static const int TGT_O_DIRECTORY = 000040000; //!< O_DIRECTORY
    static const int TGT_O_NOFOLLOW  = 000100000; //!< O_NOFOLLOW
    static const int TGT_O_NOATIME   = 001000000; //!< O_NOATIME
    static const int TGT_O_CLOEXEC   = 002000000; //!< O_NOATIME
    static const int TGT_O_SYNC      = 004010000; //!< O_SYNC
    static const int TGT_O_PATH      = 010000000; //!< O_PATH
    //@}

    /// For mmap().
    static SyscallFlagTransTable mmapFlagTable[];

    static const unsigned TGT_MAP_SHARED        = 0x00001;
    static const unsigned TGT_MAP_PRIVATE       = 0x00002;
    static const unsigned TGT_MAP_ANON          = 0x00020;
    static const unsigned TGT_MAP_DENYWRITE     = 0x00800;
    static const unsigned TGT_MAP_EXECUTABLE    = 0x01000;
    static const unsigned TGT_MAP_FILE          = 0x00000;
    static const unsigned TGT_MAP_GROWSDOWN     = 0x00100;
    static const unsigned TGT_MAP_HUGETLB       = 0x40000;
    static const unsigned TGT_MAP_LOCKED        = 0x02000;
    static const unsigned TGT_MAP_NONBLOCK      = 0x10000;
    static const unsigned TGT_MAP_NORESERVE     = 0x04000;
    static const unsigned TGT_MAP_POPULATE      = 0x08000;
    static const unsigned TGT_MAP_STACK         = 0x20000;
    static const unsigned TGT_MAP_ANONYMOUS     = 0x00020;
    static const unsigned TGT_MAP_FIXED         = 0x00010;

    static const unsigned NUM_MMAP_FLAGS;

    //@{
    /// For getrusage().
    static const int TGT_RUSAGE_SELF = 0;
    static const int TGT_RUSAGE_CHILDREN = -1;
    static const int TGT_RUSAGE_BOTH = -2;
    //@}

    //@{
    /// ioctl() command codes.
    static const unsigned TIOCGETP_   = 0x5401;
    static const unsigned TIOCSETP_   = 0x80067409;
    static const unsigned TIOCSETN_   = 0x8006740a;
    static const unsigned TIOCSETC_   = 0x80067411;
    static const unsigned TIOCGETC_   = 0x40067412;
    static const unsigned FIONREAD_   = 0x4004667f;
    static const unsigned TIOCISATTY_ = 0x2000745e;
    static const unsigned TIOCGETS_   = 0x402c7413;
    static const unsigned TIOCGETA_   = 0x5405;
    static const unsigned TCSETAW_    = 0x5407;     // 2.6.15 kernel
    //@}

    /// For table().
    static const int TBL_SYSINFO = 12;

    /// Resource enumeration for getrlimit().
    enum rlimit_resources {
        TGT_RLIMIT_CPU = 0,
        TGT_RLIMIT_FSIZE = 1,
        TGT_RLIMIT_DATA = 2,
        TGT_RLIMIT_STACK = 3,
        TGT_RLIMIT_CORE = 4,
        TGT_RLIMIT_RSS = 5,
        TGT_RLIMIT_NPROC = 6,
        TGT_RLIMIT_NOFILE = 7,
        TGT_RLIMIT_MEMLOCK = 8,
        TGT_RLIMIT_AS = 9,
        TGT_RLIMIT_LOCKS = 10
    };

    /// Limit struct for getrlimit/setrlimit.
    struct rlimit {
        uint64_t  rlim_cur;     //!< soft limit
        uint64_t  rlim_max;     //!< hard limit
    };

    /// For gettimeofday().
    struct timeval {
        int64_t tv_sec;         //!< seconds
        int64_t tv_usec;        //!< microseconds
    };

    struct timespec {
        int64_t tv_sec;   //!< seconds
        int64_t tv_nsec;  //!< nanoseconds
    };

    // For writev/readv
    struct tgt_iovec {
        uint64_t iov_base; // void *
        uint64_t iov_len;
    };

    typedef struct {
        uint64_t st_dev;
        uint64_t st_ino;
        uint64_t st_nlink;
        uint32_t st_mode;
        uint32_t st_uid;
        uint32_t st_gid;
        uint32_t __pad0;
        uint64_t st_rdev;
        uint64_t st_size;
        uint64_t st_blksize;
        uint64_t st_blocks;
        uint64_t st_atimeX;
        uint64_t st_atime_nsec;
        uint64_t st_mtimeX;
        uint64_t st_mtime_nsec;
        uint64_t st_ctimeX;
        uint64_t st_ctime_nsec;
    } tgt_stat;

    typedef struct {
        uint64_t st_dev;
        uint64_t st_ino;
        uint32_t st_mode;
        uint32_t st_nlink;
        uint32_t st_uid;
        uint32_t st_gid;
        uint32_t __pad0;
        uint64_t st_rdev;
        uint64_t st_size;
        uint64_t st_blksize;
        uint64_t st_blocks;
        uint64_t st_atimeX;
        uint64_t st_atime_nsec;
        uint64_t st_mtimeX;
        uint64_t st_mtime_nsec;
        uint64_t st_ctimeX;
        uint64_t st_ctime_nsec;
    } tgt_stat64;

    typedef struct {
        int64_t  uptime;    /* Seconds since boot */
        uint64_t loads[3];  /* 1, 5, and 15 minute load averages */
        uint64_t totalram;  /* Total usable main memory size */
        uint64_t freeram;   /* Available memory size */
        uint64_t sharedram; /* Amount of shared memory */
        uint64_t bufferram; /* Memory used by buffers */
        uint64_t totalswap; /* Total swap space size */
        uint64_t freeswap;  /* swap space still available */
        uint16_t procs;     /* Number of current processes */
        uint16_t pad;
        uint64_t totalhigh; /* Total high memory size */
        uint64_t freehigh;  /* Available high memory size */
        uint32_t mem_unit;  /* Memory unit size in bytes */
    } tgt_sysinfo;

    /// For getrusage().
    struct rusage {
        struct timeval ru_utime;        //!< user time used
        struct timeval ru_stime;        //!< system time used
        int64_t ru_maxrss;              //!< max rss
        int64_t ru_ixrss;               //!< integral shared memory size
        int64_t ru_idrss;               //!< integral unshared data "
        int64_t ru_isrss;               //!< integral unshared stack "
        int64_t ru_minflt;              //!< page reclaims - total vmfaults
        int64_t ru_majflt;              //!< page faults
        int64_t ru_nswap;               //!< swaps
        int64_t ru_inblock;             //!< block input operations
        int64_t ru_oublock;             //!< block output operations
        int64_t ru_msgsnd;              //!< messages sent
        int64_t ru_msgrcv;              //!< messages received
        int64_t ru_nsignals;            //!< signals received
        int64_t ru_nvcsw;               //!< voluntary context switches
        int64_t ru_nivcsw;              //!< involuntary "
    };

    /// For times().
    struct tms {
        int64_t tms_utime;      //!< user time
        int64_t tms_stime;      //!< system time
        int64_t tms_cutime;     //!< user time of children
        int64_t tms_cstime;     //!< system time of children
    };

    static void archClone(uint64_t flags,
                          Process *pp, Process *cp,
                          ThreadContext *ptc, ThreadContext *ctc,
                          uint64_t stack, uint64_t tls)
    {
        ArmLinux::archClone(flags, pp, cp, ptc, ctc, stack, tls);

        if (stack)
            ctc->setIntReg(ArmISA::INTREG_SP0, stack);
    }
};

#endif
