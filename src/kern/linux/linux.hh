/*
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
 *
 * Authors: Ali Saidi
 */

#ifndef __LINUX_HH__
#define __LINUX_HH__

#include "base/types.hh"

#include <string>

#include "kern/operatingsystem.hh"

class ThreadContext;
class LiveProcess;

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
    typedef struct {
        uint32_t        st_dev;         //!< device
        uint32_t        st_ino;         //!< inode
        uint32_t        st_mode;        //!< mode
        uint32_t        st_nlink;       //!< link count
        uint32_t        st_uid;         //!< owner's user ID
        uint32_t        st_gid;         //!< owner's group ID
        uint32_t        st_rdev;        //!< device number
        int32_t         _pad1;          //!< for alignment
        int64_t         st_size;        //!< file size in bytes
        uint64_t        st_atimeX;      //!< time of last access
        uint64_t        st_mtimeX;      //!< time of last modification
        uint64_t        st_ctimeX;      //!< time of last status change
        uint32_t        st_blksize;     //!< optimal I/O block size
        int32_t         st_blocks;      //!< number of blocks allocated
        uint32_t        st_flags;       //!< flags
        uint32_t        st_gen;         //!< unknown
    } tgt_stat;

    // same for stat64
    typedef struct {
        uint64_t        st_dev;
        uint64_t        st_ino;
        uint64_t        st_rdev;
        int64_t         st_size;
        uint64_t        st_blocks;

        uint32_t        st_mode;
        uint32_t        st_uid;
        uint32_t        st_gid;
        uint32_t        st_blksize;
        uint32_t        st_nlink;
        uint32_t        __pad0;

        uint64_t        st_atimeX;
        uint64_t        st_atime_nsec;
        uint64_t        st_mtimeX;
        uint64_t        st_mtime_nsec;
        uint64_t        st_ctimeX;
        uint64_t        st_ctime_nsec;
        int64_t         ___unused[3];
    } tgt_stat64;

    /// Length of strings in struct utsname (plus 1 for null char).
    static const int _SYS_NMLN = 65;

    /// Interface struct for uname().
    struct utsname {
        char sysname[_SYS_NMLN];        //!< System name.
        char nodename[_SYS_NMLN];       //!< Node name.
        char release[_SYS_NMLN];        //!< OS release.
        char version[_SYS_NMLN];        //!< OS version.
        char machine[_SYS_NMLN];        //!< Machine type.
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

    /// Clock ticks per second, for times().
    static const int M5_SC_CLK_TCK = 100;

    /// For times().
    struct tms {
        int64_t tms_utime;      //!< user time
        int64_t tms_stime;      //!< system time
        int64_t tms_cutime;     //!< user time of children
        int64_t tms_cstime;     //!< system time of children
    };

    // For writev/readv
    struct tgt_iovec {
        uint64_t iov_base; // void *
        uint64_t iov_len;
    };


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

    static int openSpecialFile(std::string path, LiveProcess *process, ThreadContext *tc);
    static std::string procMeminfo(LiveProcess *process, ThreadContext *tc);

};  // class Linux

#endif // __LINUX_HH__
