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

#ifndef __KERN_OPERATINGSYSTEM_HH__
#define __KERN_OPERATINGSYSTEM_HH__

#include "base/types.hh"

#include <string>

namespace gem5
{

class Process;
class ThreadContext;

///
/// This class encapsulates the types, structures, constants,
/// functions, and syscall-number mappings specific to an operating system
/// syscall interface.
///
class OperatingSystem
{
  public:
    /// Stat buffer.  Note that we can't call it 'stat' since that
    /// gets #defined to something else on some systems. This type
    /// can be specialized by architecture specific "Linux" classes
    typedef void tgt_stat;

    // same for stat64
    typedef void tgt_stat64;

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

    // For writev/readv
    struct tgt_iovec
    {
        uint64_t iov_base; // void *
        uint64_t iov_len;
    };

    /// For getrusage().
    struct rusage
    {
        timeval ru_utime;    //!< user time used
        timeval ru_stime;    //!< system time used
        int64_t ru_maxrss;   //!< max rss
        int64_t ru_ixrss;    //!< integral shared memory size
        int64_t ru_idrss;    //!< integral unshared data "
        int64_t ru_isrss;    //!< integral unshared stack "
        int64_t ru_minflt;   //!< page reclaims - total vmfaults
        int64_t ru_majflt;   //!< page faults
        int64_t ru_nswap;    //!< swaps
        int64_t ru_inblock;  //!< block input operations
        int64_t ru_oublock;  //!< block output operations
        int64_t ru_msgsnd;   //!< messages sent
        int64_t ru_msgrcv;   //!< messages received
        int64_t ru_nsignals; //!< signals received
        int64_t ru_nvcsw;    //!< voluntary context switches
        int64_t ru_nivcsw;   //!< involuntary "
    };

    static int openSpecialFile(std::string path, Process *process,
                               ThreadContext *tc);

}; // class OperatingSystem

} // namespace gem5

#endif // __OPERATINGSYSTEM_HH__
