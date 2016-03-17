/*
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#ifndef __ARCH_ARM_FREEBSD_FREEBSD_HH__
#define __ARCH_ARM_FREEBSD_FREEBSD_HH__

#include "kern/freebsd/freebsd.hh"

class ArmFreebsd32 : public FreeBSD
{
  public:

    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static SyscallFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    //@{
    /// Basic ARM FreeBSD types
    typedef uint32_t size_t;
    typedef uint32_t off_t;
    typedef int32_t time_t;
    typedef int32_t clock_t;
    //@}

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY    = 0x00000000; //!< O_RDONLY
    static const int TGT_O_WRONLY    = 0x00000001; //!< O_WRONLY
    static const int TGT_O_RDWR      = 0x00000002; //!< O_RDWR
    static const int TGT_O_CREAT     = 0x00000200; //!< O_CREAT
    static const int TGT_O_EXCL      = 0x00000800; //!< O_EXCL
    static const int TGT_O_NOCTTY    = 0x00008000; //!< O_NOCTTY
    static const int TGT_O_TRUNC     = 0x00000400; //!< O_TRUNC
    static const int TGT_O_APPEND    = 0x00000008; //!< O_APPEND
    static const int TGT_O_NONBLOCK  = 0x00000004; //!< O_NONBLOCK
    static const int TGT_O_SYNC      = 0x00000080; //!< O_SYNC
    static const int TGT_FASYNC      = 0x00000040; //!< FASYNC
    static const int TGT_O_DIRECT    = 0x00010000; //!< O_DIRECT
    static const int TGT_O_DIRECTORY = 0x00020000; //!< O_DIRECTORY
    static const int TGT_O_NOFOLLOW  = 0x00000100; //!< O_NOFOLLOW
    static const int TGT_O_CLOEXEC   = 0x00100000; //!< O_CLOEXEC
    //@}

    /// For mmap().
    static const unsigned TGT_MAP_SHARED    = 0x0001;
    static const unsigned TGT_MAP_PRIVATE   = 0x0002;
    static const unsigned TGT_MAP_ANONYMOUS = 0x1000;
    static const unsigned TGT_MAP_FIXED     = 0x0010;

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

    // For writev/readv
    struct tgt_iovec {
        uint32_t iov_base; // void *
        uint32_t iov_len;
    };

    /*
     * sizeof st 120
     * sizeof st_dev 4
     * sizeof st_ino 4
     * sizeof st_mode 2
     * sizeof st_nlink 2
     * sizeof st_uid 4
     * sizeof st_gid 4
     * sizeof st_rdev 4
     * sizeof st_atim 16
     * sizeof st_size 8
     * sizeof st_blocks 8
     * sizeof st_blksize 4
     * sizeof st_flags 4
     * sizeof st_gen 4
     * sizeof st_lspare 4
     */

    typedef struct {
        uint32_t   st_dev;
        uint32_t   st_ino;
        uint16_t   st_mode;
        uint16_t   st_nlink;
        uint32_t   st_uid;
        uint32_t   st_gid;
        uint32_t   st_rdev;
        uint64_t   st_atimeX;
        uint64_t   st_atime_nsec;
        uint64_t   st_mtimeX;
        uint64_t   st_mtime_nsec;
        uint64_t   st_ctimeX;
        uint64_t   st_ctime_nsec;
        uint64_t   st_size;
        uint64_t   st_blocks;
        uint32_t   st_blksize;
        uint32_t   st_flags;
        uint32_t   st_gen;
        uint32_t   st_lspare;
        uint64_t   st_birthtimX;
        uint64_t   st_birthtim;
    } tgt_stat;

    typedef struct {
        uint32_t   st_dev;
        uint32_t   st_ino;
        uint16_t   st_mode;
        uint16_t   st_nlink;
        uint32_t   st_uid;
        uint32_t   st_gid;
        uint32_t   st_rdev;
        uint64_t   st_atimeX;
        uint64_t   st_atime_nsec;
        uint64_t   st_mtimeX;
        uint64_t   st_mtime_nsec;
        uint64_t   st_ctimeX;
        uint64_t   st_ctime_nsec;
        uint64_t   st_size;
        uint64_t   st_blocks;
        uint32_t   st_blksize;
        uint32_t   st_flags;
        uint32_t   st_gen;
        uint32_t   st_lspare;
        uint64_t   st_birthtimX;
        uint64_t   st_birthtim;
    } tgt_stat64;

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
};

class ArmFreebsd64 : public FreeBSD
{
  public:

    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static SyscallFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    //@{
    /// Basic ARM FreeBSD types
    typedef uint64_t size_t;
    typedef uint64_t off_t;
    typedef int64_t time_t;
    typedef int64_t clock_t;
    //@}

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY    = 0x00000000; //!< O_RDONLY
    static const int TGT_O_WRONLY    = 0x00000001; //!< O_WRONLY
    static const int TGT_O_RDWR      = 0x00000002; //!< O_RDWR
    static const int TGT_O_CREAT     = 0x00000200; //!< O_CREAT
    static const int TGT_O_EXCL      = 0x00000800; //!< O_EXCL
    static const int TGT_O_NOCTTY    = 0x00008000; //!< O_NOCTTY
    static const int TGT_O_TRUNC     = 0x00000400; //!< O_TRUNC
    static const int TGT_O_APPEND    = 0x00000008; //!< O_APPEND
    static const int TGT_O_NONBLOCK  = 0x00000004; //!< O_NONBLOCK
    static const int TGT_O_SYNC      = 0x00000080; //!< O_SYNC
    static const int TGT_FASYNC      = 0x00000040; //!< FASYNC
    static const int TGT_O_DIRECT    = 0x00010000; //!< O_DIRECT
    static const int TGT_O_DIRECTORY = 0x00020000; //!< O_DIRECTORY
    static const int TGT_O_NOFOLLOW  = 0x00000100; //!< O_NOFOLLOW
    static const int TGT_O_CLOEXEC   = 0x00100000; //!< O_CLOEXEC
    //@}

    /// For mmap().
    static const unsigned TGT_MAP_SHARED    = 0x0001;
    static const unsigned TGT_MAP_PRIVATE   = 0x0002;
    static const unsigned TGT_MAP_ANONYMOUS = 0x1000;
    static const unsigned TGT_MAP_FIXED     = 0x0010;

    //@{
    /// For getrusage().
    static const int TGT_RUSAGE_SELF = 0;
    static const int TGT_RUSAGE_CHILDREN = -1;
    static const int TGT_RUSAGE_THREAD = 1;
    //@}

    /// Resource enumeration for getrlimit().
    enum rlimit_resources {
        TGT_RLIMIT_CPU = 0,
        TGT_RLIMIT_FSIZE = 1,
        TGT_RLIMIT_DATA = 2,
        TGT_RLIMIT_STACK = 3,
        TGT_RLIMIT_CORE = 4,
        TGT_RLIMIT_RSS = 5,
        TGT_RLIMIT_MEMLOCK = 6,
        TGT_RLIMIT_NPROC = 7,
        TGT_RLIMIT_NOFILE = 8,
        TGT_RLIMIT_SBSIZE = 9,
        TGT_RLIMIT_VMEM = 10,
        TGT_RLIMIT_AS = TGT_RLIMIT_VMEM,
        TGT_RLIMIT_NPTS = 11,
        TGT_RLIMIT_SWAP = 12,
        TGT_RLIMIT_KQUEUES = 13
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

    // For writev/readv
    struct tgt_iovec {
        uint64_t iov_base; // void *
        uint64_t iov_len;
    };

    typedef struct {
        uint32_t   st_dev;
        uint32_t   st_ino;
        uint16_t   st_mode;
        uint16_t   st_nlink;
        uint32_t   st_uid;
        uint32_t   st_gid;
        uint32_t   st_rdev;
        uint64_t   st_atimeX;
        uint64_t   st_atime_nsec;
        uint64_t   st_mtimeX;
        uint64_t   st_mtime_nsec;
        uint64_t   st_ctimeX;
        uint64_t   st_ctime_nsec;
        uint64_t   st_size;
        uint64_t   st_blocks;
        uint32_t   st_blksize;
        uint32_t   st_flags;
        uint32_t   st_gen;
        uint32_t   st_lspare;
        uint64_t   st_birthtimX;
        uint64_t   st_birthtim;
    } tgt_stat;

    typedef struct {
        uint32_t   st_dev;
        uint32_t   st_ino;
        uint16_t   st_mode;
        uint16_t   st_nlink;
        uint32_t   st_uid;
        uint32_t   st_gid;
        uint32_t   st_rdev;
        uint64_t   st_atimeX;
        uint64_t   st_atime_nsec;
        uint64_t   st_mtimeX;
        uint64_t   st_mtime_nsec;
        uint64_t   st_ctimeX;
        uint64_t   st_ctime_nsec;
        uint64_t   st_size;
        uint64_t   st_blocks;
        uint32_t   st_blksize;
        uint32_t   st_flags;
        uint32_t   st_gen;
        uint32_t   st_lspare;
        uint64_t   st_birthtimX;
        uint64_t   st_birthtim;
    } tgt_stat64;

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
};

#endif
