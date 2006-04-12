/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
#include "config/full_system.hh"

#if FULL_SYSTEM

class Linux {};

#else //!FULL_SYSTEM

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>	// for host open() flags
#include <string.h>	// for memset()
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "sim/syscall_emul.hh"
#include "arch/isa_traits.hh"

class TranslatingPort;

///
/// This class encapsulates the types, structures, constants,
/// functions, and syscall-number mappings specific to the Alpha Linux
/// syscall interface.
///
class Linux {

  public:

    //@{
    /// Basic Linux types.
    typedef uint64_t size_t;
    typedef uint64_t off_t;
    typedef int64_t time_t;
    typedef uint32_t uid_t;
    typedef uint32_t gid_t;
    //@}

    typedef TheISA::OSFlags OSFlags;

#if BSD_HOST
    typedef struct stat hst_stat;
    typedef struct stat hst_stat64;
#else
    typedef struct stat hst_stat ;
    typedef struct stat64 hst_stat64;
#endif


    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static OpenFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    /// Stat buffer.  Note that we can't call it 'stat' since that
    /// gets #defined to something else on some systems.
    struct tgt_stat {
        uint32_t	st_dev;		//!< device
        uint32_t	st_ino;		//!< inode
        uint32_t	st_mode;	//!< mode
        uint32_t	st_nlink;	//!< link count
        uint32_t	st_uid;		//!< owner's user ID
        uint32_t	st_gid;		//!< owner's group ID
        uint32_t	st_rdev;	//!< device number
        int32_t		_pad1;		//!< for alignment
        int64_t		st_size;	//!< file size in bytes
        uint64_t	st_atimeX;	//!< time of last access
        uint64_t	st_mtimeX;	//!< time of last modification
        uint64_t	st_ctimeX;	//!< time of last status change
        uint32_t	st_blksize;	//!< optimal I/O block size
        int32_t		st_blocks;	//!< number of blocks allocated
        uint32_t	st_flags;	//!< flags
        uint32_t	st_gen;		//!< unknown
    };

    // same for stat64
    struct tgt_stat64 {
        uint64_t	st_dev;
        uint64_t	st_ino;
        uint64_t	st_rdev;
        int64_t		st_size;
        uint64_t	st_blocks;

        uint32_t	st_mode;
        uint32_t	st_uid;
        uint32_t	st_gid;
        uint32_t	st_blksize;
        uint32_t	st_nlink;
        uint32_t	__pad0;

        uint64_t	tgt_st_atime;
        uint64_t 	st_atime_nsec;
        uint64_t	tgt_st_mtime;
        uint64_t	st_mtime_nsec;
        uint64_t	tgt_st_ctime;
        uint64_t	st_ctime_nsec;
        int64_t		___unused[3];
    };

    /// Length of strings in struct utsname (plus 1 for null char).
    static const int _SYS_NMLN = 65;

    /// Interface struct for uname().
    struct utsname {
        char sysname[_SYS_NMLN];	//!< System name.
        char nodename[_SYS_NMLN];	//!< Node name.
        char release[_SYS_NMLN];	//!< OS release.
        char version[_SYS_NMLN];	//!< OS version.
        char machine[_SYS_NMLN];	//!< Machine type.
    };


    /// Resource enumeration for getrlimit().
    enum rlimit_resources {
        TGT_RLIMIT_CPU = 0,
        TGT_RLIMIT_FSIZE = 1,
        TGT_RLIMIT_DATA = 2,
        TGT_RLIMIT_STACK = 3,
        TGT_RLIMIT_CORE = 4,
        TGT_RLIMIT_RSS = 5,
        TGT_RLIMIT_NOFILE = 6,
        TGT_RLIMIT_AS = 7,
        TGT_RLIMIT_VMEM = 7,
        TGT_RLIMIT_NPROC = 8,
        TGT_RLIMIT_MEMLOCK = 9,
        TGT_RLIMIT_LOCKS = 10
    };

    /// Limit struct for getrlimit/setrlimit.
    struct rlimit {
        uint64_t  rlim_cur;	//!< soft limit
        uint64_t  rlim_max;	//!< hard limit
    };

    /// For gettimeofday().
    struct timeval {
        int64_t tv_sec;		//!< seconds
        int64_t tv_usec;	//!< microseconds
    };

    // For writev/readv
    struct tgt_iovec {
        uint64_t iov_base; // void *
        uint64_t iov_len;
    };


    /// For getrusage().
    struct rusage {
        struct timeval ru_utime;	//!< user time used
        struct timeval ru_stime;	//!< system time used
        int64_t ru_maxrss;		//!< max rss
        int64_t ru_ixrss;		//!< integral shared memory size
        int64_t ru_idrss;		//!< integral unshared data "
        int64_t ru_isrss;		//!< integral unshared stack "
        int64_t ru_minflt;		//!< page reclaims - total vmfaults
        int64_t ru_majflt;		//!< page faults
        int64_t ru_nswap;		//!< swaps
        int64_t ru_inblock;		//!< block input operations
        int64_t ru_oublock;		//!< block output operations
        int64_t ru_msgsnd;		//!< messages sent
        int64_t ru_msgrcv;		//!< messages received
        int64_t ru_nsignals;		//!< signals received
        int64_t ru_nvcsw;		//!< voluntary context switches
        int64_t ru_nivcsw;		//!< involuntary "
    };

    /// Helper function to convert a host stat buffer to a target stat
    /// buffer.  Also copies the target buffer out to the simulated
    /// memory space.  Used by stat(), fstat(), and lstat().
#if !BSD_HOST
    static void
    copyOutStatBuf(TranslatingPort *mem, Addr addr, hst_stat *host)
    {
        using namespace TheISA;

        TypedBufferArg<Linux::tgt_stat> tgt(addr);

        tgt->st_dev = htog(host->st_dev);
        tgt->st_ino = htog(host->st_ino);
        tgt->st_mode = htog(host->st_mode);
        tgt->st_nlink = htog(host->st_nlink);
        tgt->st_uid = htog(host->st_uid);
        tgt->st_gid = htog(host->st_gid);
        tgt->st_rdev = htog(host->st_rdev);
        tgt->st_size = htog(host->st_size);
        tgt->st_atimeX = htog(host->st_atime);
        tgt->st_mtimeX = htog(host->st_mtime);
        tgt->st_ctimeX = htog(host->st_ctime);
        tgt->st_blksize = htog(host->st_blksize);
        tgt->st_blocks = htog(host->st_blocks);

        tgt.copyOut(mem);
    }
#else
    // Third version for bsd systems which no longer have any support for
    // the old stat() call and stat() is actually a stat64()
    static void
    copyOutStatBuf(TranslatingPort *mem, Addr addr, hst_stat64 *host)
    {
        using namespace TheISA;

        TypedBufferArg<Linux::tgt_stat> tgt(addr);

        tgt->st_dev = htog(host->st_dev);
        tgt->st_ino = htog(host->st_ino);
        tgt->st_mode = htog(host->st_mode);
        tgt->st_nlink = htog(host->st_nlink);
        tgt->st_uid = htog(host->st_uid);
        tgt->st_gid = htog(host->st_gid);
        tgt->st_rdev = htog(host->st_rdev);
        tgt->st_size = htog(host->st_size);
        tgt->st_atimeX = htog(host->st_atime);
        tgt->st_mtimeX = htog(host->st_mtime);
        tgt->st_ctimeX = htog(host->st_ctime);
        tgt->st_blksize = htog(host->st_blksize);
        tgt->st_blocks = htog(host->st_blocks);

        tgt.copyOut(mem);
    }
#endif


    // Same for stat64
    static void
    copyOutStat64Buf(TranslatingPort *mem, int fd, Addr addr, hst_stat64 *host)
    {
        using namespace TheISA;

        TypedBufferArg<Linux::tgt_stat64> tgt(addr);

        // fd == 1 checks are because libc does some checks
        // that the stdout is interactive vs. a file
        // this makes it work on non-linux systems
        if (fd == 1)
            tgt->st_dev = htog((uint64_t)0xA);
        else
            tgt->st_dev = htog((uint64_t)host->st_dev);
        // XXX What about STAT64_HAS_BROKEN_ST_INO ???
        tgt->st_ino = htog((uint64_t)host->st_ino);
        if (fd == 1)
            tgt->st_rdev = htog((uint64_t)0x880d);
        else
            tgt->st_rdev = htog((uint64_t)host->st_rdev);
        tgt->st_size = htog((int64_t)host->st_size);
        tgt->st_blocks = htog((uint64_t)host->st_blocks);

        if (fd == 1)
            tgt->st_mode = htog((uint32_t)0x2190);
        else
            tgt->st_mode = htog((uint32_t)host->st_mode);
        tgt->st_uid = htog((uint32_t)host->st_uid);
        tgt->st_gid = htog((uint32_t)host->st_gid);
        tgt->st_blksize = htog((uint32_t)host->st_blksize);
        tgt->st_nlink = htog((uint32_t)host->st_nlink);
        tgt->tgt_st_atime = htog((uint64_t)host->st_atime);
        tgt->tgt_st_mtime = htog((uint64_t)host->st_mtime);
        tgt->tgt_st_ctime = htog((uint64_t)host->st_ctime);
#if defined(STAT_HAVE_NSEC)
        tgt->st_atime_nsec = htog(host->st_atime_nsec);
        tgt->st_mtime_nsec = htog(host->st_mtime_nsec);
        tgt->st_ctime_nsec = htog(host->st_ctime_nsec);
#else
        tgt->st_atime_nsec = 0;
        tgt->st_mtime_nsec = 0;
        tgt->st_ctime_nsec = 0;
#endif

        tgt.copyOut(mem);
    }

};  // class Linux


#endif // FULL_SYSTEM

#endif // __LINUX_HH__
