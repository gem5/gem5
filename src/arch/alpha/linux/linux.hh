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
 * Authors: Korey Sewell
 */

#ifndef __ALPHA_ALPHA_LINUX_LINUX_HH__
#define __ALPHA_ALPHA_LINUX_LINUX_HH__

#include "kern/linux/linux.hh"

/* AlphaLinux class contains static constants/definitions/misc.
 * structures which are specific to the Linux OS AND the Alpha
 * architecture
 */
class AlphaLinux : public Linux
{
  public:

    static const int TGT_SIGHUP         = 0x000001;
    static const int TGT_SIGINT         = 0x000002;
    static const int TGT_SIGQUIT        = 0x000003;
    static const int TGT_SIGILL         = 0x000004;
    static const int TGT_SIGTRAP        = 0x000005;
    static const int TGT_SIGABRT        = 0x000006;
    static const int TGT_SIGEMT         = 0x000007;
    static const int TGT_SIGFPE         = 0x000008;
    static const int TGT_SIGKILL        = 0x000009;
    static const int TGT_SIGBUS         = 0x00000a;
    static const int TGT_SIGSEGV        = 0x00000b;
    static const int TGT_SIGSYS         = 0x00000c;
    static const int TGT_SIGPIPE        = 0x00000d;
    static const int TGT_SIGALRM        = 0x00000e;
    static const int TGT_SIGTERM        = 0x00000f;
    static const int TGT_SIGURG         = 0x000010;
    static const int TGT_SIGSTOP        = 0x000011;
    static const int TGT_SIGTSTP        = 0x000012;
    static const int TGT_SIGCONT        = 0x000013;
    static const int TGT_SIGCHLD        = 0x000014;
    static const int TGT_SIGTTIN        = 0x000015;
    static const int TGT_SIGTTOU        = 0x000016;
    static const int TGT_SIGIO          = 0x000017;
    static const int TGT_SIGXCPU        = 0x000018;
    static const int TGT_SIGXFSZ        = 0x000019;
    static const int TGT_SIGVTALRM      = 0x00001a;
    static const int TGT_SIGPROF        = 0x00001b;
    static const int TGT_SIGWINCH       = 0x00001c;
    static const int TGT_SIGINFO        = 0x00001d;
    static const int TGT_SIGUSR1        = 0x00001e;
    static const int TGT_SIGUSR2        = 0x00001f;

    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static SyscallFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY       = 000000000;    //!< O_RDONLY
    static const int TGT_O_WRONLY       = 000000001;    //!< O_WRONLY
    static const int TGT_O_RDWR         = 000000002;    //!< O_RDWR
    static const int TGT_O_CREAT        = 000001000;    //!< O_CREAT
    static const int TGT_O_EXCL         = 000004000;    //!< O_EXCL
    static const int TGT_O_NOCTTY       = 000010000;    //!< O_NOCTTY
    static const int TGT_O_TRUNC        = 000002000;    //!< O_TRUNC
    static const int TGT_O_APPEND       = 000000010;    //!< O_APPEND
    static const int TGT_O_NONBLOCK     = 000000004;    //!< O_NONBLOCK
    static const int TGT_O_DSYNC        = 000040000;    //!< O_DSYNC
    static const int TGT_FASYNC         = 000020000;    //!< FASYNC
    static const int TGT_O_DIRECT       = 002000000;    //!< O_DIRECT
    static const int TGT_O_LARGEFILE    = 000400000;    //!< O_LARGEFILE
    static const int TGT_O_DIRECTORY    = 000100000;    //!< O_DIRECTORY
    static const int TGT_O_NOFOLLOW     = 000200000;    //!< O_NOFOLLOW
    static const int TGT_O_NOATIME      = 004000000;    //!< O_NOATIME
    static const int TGT_O_CLOEXEC      = 010000000;    //!< O_CLOEXEC
    static const int TGT_O_SYNC         = 020040000;    //!< O_SYNC
    static const int TGT_O_PATH         = 040000000;    //!< O_PATH
    //@}

    static const unsigned TGT_MAP_SHARED        = 0x000001;
    static const unsigned TGT_MAP_PRIVATE       = 0x000002;
    static const unsigned TGT_MAP_ANON          = 0x000010;
    static const unsigned TGT_MAP_DENYWRITE     = 0x002000;
    static const unsigned TGT_MAP_EXECUTABLE    = 0x004000;
    static const unsigned TGT_MAP_FILE          = 0x000000;
    static const unsigned TGT_MAP_GROWSDOWN     = 0x001000;
    static const unsigned TGT_MAP_HUGETLB       = 0x100000;
    static const unsigned TGT_MAP_LOCKED        = 0x008000;
    static const unsigned TGT_MAP_NONBLOCK      = 0x040000;
    static const unsigned TGT_MAP_NORESERVE     = 0x010000;
    static const unsigned TGT_MAP_POPULATE      = 0x020000;
    static const unsigned TGT_MAP_STACK         = 0x080000;
    static const unsigned TGT_MAP_ANONYMOUS     = 0x000010;
    static const unsigned TGT_MAP_FIXED         = 0x000100;

    static const unsigned NUM_MMAP_FLAGS;

    //@{
    /// For getsysinfo().
    static const unsigned GSI_PLATFORM_NAME = 103; //!< platform name as string
    static const unsigned GSI_CPU_INFO      = 59; //!< CPU information
    static const unsigned GSI_PROC_TYPE     = 60; //!< get proc_type
    static const unsigned GSI_MAX_CPU       = 30; //!< max # CPUs on machine
    static const unsigned GSI_CPUS_IN_BOX   = 55; //!< number of CPUs in system
    static const unsigned GSI_PHYSMEM       = 19; //!< Physical memory in KB
    static const unsigned GSI_CLK_TCK       = 42; //!< clock freq in Hz
    static const unsigned GSI_IEEE_FP_CONTROL = 45;
    //@}

    //@{
    /// For setsysinfo().
    static const unsigned SSI_IEEE_FP_CONTROL = 14; //!< ieee_set_fp_control()
    //@}

    //@{
    /// ioctl() command codes.
    static const unsigned TGT_TIOCGETP   = 0x40067408;
    static const unsigned TGT_TIOCSETP   = 0x80067409;
    static const unsigned TGT_TIOCSETN   = 0x8006740a;
    static const unsigned TGT_TIOCSETC   = 0x80067411;
    static const unsigned TGT_TIOCGETC   = 0x40067412;
    static const unsigned TGT_FIONREAD   = 0x4004667f;
    static const unsigned TGT_TCGETS     = 0x402c7413;
    static const unsigned TGT_TCGETA     = 0x40127417;
    static const unsigned TGT_TCSETAW    = 0x80147419; // 2.6.15 kernel
    //@}

    static bool
    isTtyReq(unsigned req)
    {
        switch (req) {
          case TGT_TIOCGETP:
          case TGT_TIOCSETP:
          case TGT_TIOCSETN:
          case TGT_TIOCSETC:
          case TGT_TIOCGETC:
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
    static const unsigned TGT_RLIMIT_AS = 7;
    static const unsigned TGT_RLIMIT_NOFILE = 6;
    static const unsigned TGT_RLIMIT_MEMLOCK = 9;

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
       uint64_t totalhigh; /* Total high memory size */
       uint64_t freehigh;  /* Available high memory size */
       uint64_t mem_unit;  /* Memory unit size in bytes */
    } tgt_sysinfo;

    // For futex system call
    static const unsigned TGT_EAGAIN      = 35;
    static const unsigned TGT_EWOULDBLOCK = TGT_EAGAIN;
};

#endif // __ALPHA_ALPHA_LINUX_LINUX_HH__
