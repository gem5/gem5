/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __ARCH_MIPS_LINUX_LINUX_HH__
#define __ARCH_MIPS_LINUX_LINUX_HH__

#include <map>

#include "kern/linux/flag_tables.hh"
#include "kern/linux/linux.hh"

namespace gem5
{

class MipsLinux : public Linux, public OpenFlagTable<MipsLinux>
{
  public:
    static const ByteOrder byteOrder = ByteOrder::little;

    static const int TGT_SIGHUP = 0x000001;
    static const int TGT_SIGINT = 0x000002;
    static const int TGT_SIGQUIT = 0x000003;
    static const int TGT_SIGILL = 0x000004;
    static const int TGT_SIGTRAP = 0x000005;
    static const int TGT_SIGIOT = 0x000006;
    static const int TGT_SIGABRT = 0x000006;
    static const int TGT_SIGEMT = 0x000007;
    static const int TGT_SIGFPE = 0x000008;
    static const int TGT_SIGKILL = 0x000009;
    static const int TGT_SIGBUS = 0x00000a;
    static const int TGT_SIGSEGV = 0x00000b;
    static const int TGT_SIGSYS = 0x00000c;
    static const int TGT_SIGPIPE = 0x00000d;
    static const int TGT_SIGALRM = 0x00000e;
    static const int TGT_SIGTERM = 0x00000f;
    static const int TGT_SIGUSR1 = 0x000010;
    static const int TGT_SIGUSR2 = 0x000011;
    static const int TGT_SIGCHLD = 0x000012;
    static const int TGT_SIGCLD = 0x000012;
    static const int TGT_SIGPWR = 0x000013;
    static const int TGT_SIGWINCH = 0x000014;
    static const int TGT_SIGURG = 0x000015;
    static const int TGT_SIGIO = 0x000016;
    static const int TGT_SIGPOLL = 0x000016;
    static const int TGT_SIGSTOP = 0x000017;
    static const int TGT_SIGTSTP = 0x000018;
    static const int TGT_SIGCONT = 0x000019;
    static const int TGT_SIGTTIN = 0x00001a;
    static const int TGT_SIGTTOU = 0x00001b;
    static const int TGT_SIGVTALRM = 0x00001c;
    static const int TGT_SIGPROF = 0x00001d;
    static const int TGT_SIGXCPU = 0x00001e;
    static const int TGT_SIGXFSZ = 0x00001f;

    //@{
    /// open(2) flag values.
    static constexpr int TGT_O_RDONLY = 0x00000000;    //!< O_RDONLY
    static constexpr int TGT_O_WRONLY = 0x00000001;    //!< O_WRONLY
    static constexpr int TGT_O_RDWR = 0x00000002;      //!< O_RDWR
    static constexpr int TGT_O_CREAT = 0x00000100;     //!< O_CREAT
    static constexpr int TGT_O_EXCL = 0x00000400;      //!< O_EXCL
    static constexpr int TGT_O_NOCTTY = 0x00000800;    //!< O_NOCTTY
    static constexpr int TGT_O_TRUNC = 0x00000200;     //!< O_TRUNC
    static constexpr int TGT_O_APPEND = 0x00000008;    //!< O_APPEND
    static constexpr int TGT_O_NONBLOCK = 0x00000080;  //!< O_NONBLOCK
    static constexpr int TGT_O_DSYNC = 0x00000010;     //!< O_DSYNC
    static constexpr int TGT_FASYNC = 0x00001000;      //!< O_FASYNC
    static constexpr int TGT_O_DIRECT = 0x00008000;    //!< O_DIRECT
    static constexpr int TGT_O_LARGEFILE = 0x00002000; //!< O_LARGEFILE
    static constexpr int TGT_O_DIRECTORY = 0x00010000; //!< O_DIRECTORY
    static constexpr int TGT_O_NOFOLLOW = 0x00020000;  //!< O_NOFOLLOW
    static constexpr int TGT_O_NOATIME = 0x00040000;   //!< O_NOATIME
    static constexpr int TGT_O_CLOEXEC = 0x00080000;   //!< O_CLOEXEC
    static constexpr int TGT_O_SYNC = 0x00004010;      //!< O_SYNC
    static constexpr int TGT_O_PATH = 0x00200000;      //!< O_PATH
    //@}

    static constexpr unsigned TGT_MAP_SHARED = 0x00001;
    static constexpr unsigned TGT_MAP_PRIVATE = 0x00002;
    static constexpr unsigned TGT_MAP_ANON = 0x00800;
    static constexpr unsigned TGT_MAP_DENYWRITE = 0x02000;
    static constexpr unsigned TGT_MAP_EXECUTABLE = 0x04000;
    static constexpr unsigned TGT_MAP_FILE = 0x00000;
    static constexpr unsigned TGT_MAP_GROWSDOWN = 0x01000;
    static constexpr unsigned TGT_MAP_HUGETLB = 0x80000;
    static constexpr unsigned TGT_MAP_LOCKED = 0x08000;
    static constexpr unsigned TGT_MAP_NONBLOCK = 0x20000;
    static constexpr unsigned TGT_MAP_NORESERVE = 0x00400;
    static constexpr unsigned TGT_MAP_POPULATE = 0x10000;
    static constexpr unsigned TGT_MAP_STACK = 0x40000;
    static constexpr unsigned TGT_MAP_ANONYMOUS = 0x00800;
    static constexpr unsigned TGT_MAP_FIXED = 0x00010;

    //@{
    /// For getsysinfo().
    static const unsigned GSI_PLATFORM_NAME = 103; //!< platform name as string
    static const unsigned GSI_CPU_INFO = 59;       //!< CPU information
    static const unsigned GSI_PROC_TYPE = 60;      //!< get proc_type
    static const unsigned GSI_MAX_CPU = 30; //!< max # cpu's on this machine
    static const unsigned GSI_CPUS_IN_BOX = 55; //!< number of CPUs in system
    static const unsigned GSI_PHYSMEM = 19;     //!< Physical memory in KB
    static const unsigned GSI_CLK_TCK = 42;     //!< clock freq in Hz
    //@}

    //@{
    /// For setsysinfo().
    static const unsigned SSI_IEEE_FP_CONTROL = 14; //!< ieee_set_fp_control()
    //@}

    //@{
    /// ioctl() command codes.
    static const unsigned TGT_TCGETA = 0x5401;
    static const unsigned TGT_TCSETAW = 0x5403;
    static const unsigned TGT_TCGETS = 0x540d;
    static const unsigned TGT_FIONREAD = 0x467f;
    static const unsigned TGT_TIOCGETP = 0x7408;
    static const unsigned TGT_TIOCSETP = 0x7409;
    static const unsigned TGT_TIOCSETN = 0x740a;

    //@}

    static bool
    isTtyReq(unsigned req)
    {
        switch (req) {
        case TGT_TIOCGETP:
        case TGT_TIOCSETP:
        case TGT_TIOCSETN:
        case TGT_FIONREAD:
        case TGT_TCGETS:
        case TGT_TCGETA:
        case TGT_TCSETAW:
            return true;
        default:
            return false;
        }
    }

    /// For table().
    static const int TBL_SYSINFO = 12;

    /// Resource constants for getrlimit() (overide some generics).
    static const unsigned TGT_RLIMIT_NPROC = 8;
    static const unsigned TGT_RLIMIT_AS = 6;
    static const unsigned TGT_RLIMIT_RSS = 7;
    static const unsigned TGT_RLIMIT_NOFILE = 5;
    static const unsigned TGT_RLIMIT_MEMLOCK = 9;

    /// Offset used to make sure that processes don't
    /// assign themselves to process IDs reserved for
    /// the root users.
    static const int NUM_ROOT_PROCS = 2;

    struct tgt_sysinfo
    {
        int32_t uptime;     /* Seconds since boot */
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
    };
};

} // namespace gem5

#endif
