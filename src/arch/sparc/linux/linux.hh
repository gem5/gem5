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
 */

#ifndef __ARCH_SPARC_LINUX_LINUX_HH__
#define __ARCH_SPARC_LINUX_LINUX_HH__

#include "arch/sparc/utility.hh"
#include "kern/linux/linux.hh"

class SparcLinux : public Linux
{
  public:

    static const ByteOrder byteOrder = BigEndianByteOrder;

    struct tgt_stat
    {
        uint32_t st_dev;
        char __pad1[4];
        uint64_t st_ino;
        uint32_t st_mode;
        uint16_t st_nlink;
        uint32_t st_uid;
        uint32_t st_gid;
        uint32_t st_rdev;
        char __pad2[4];
        int64_t st_size;
        int64_t st_atimeX;
        int64_t st_mtimeX;
        int64_t st_ctimeX;
        int64_t st_blksize;
        int64_t st_blocks;
        uint64_t __unused4[2];
    };

    struct tgt_stat64
    {
        uint64_t st_dev;
        uint64_t st_ino;
        uint64_t st_nlink;

        uint32_t st_mode;
        uint32_t st_uid;
        uint32_t st_gid;
        uint32_t __pad0;

        uint64_t st_rdev;
        int64_t st_size;
        int64_t st_blksize;
        int64_t st_blocks;

        uint64_t st_atimeX;
        uint64_t st_atime_nsec;
        uint64_t st_mtimeX;
        uint64_t st_mtime_nsec;
        uint64_t st_ctimeX;
        uint64_t st_ctime_nsec;
        int64_t __unused[3];
    };

    // SPARC receives weird subsignals for several of its signals. If you
    // find yourself needing to implement these in detail, look at the
    // Linux source.
    static const int TGT_SIGHUP         = 0x000001;
    static const int TGT_SIGINT         = 0x000002;
    static const int TGT_SIGQUIT        = 0x000003;
    static const int TGT_SIGILL         = 0x000004;
    static const int TGT_SIGTRAP        = 0x000005;
    static const int TGT_SIGABRT        = 0x000006;
    static const int TGT_SIGIOT         = 0x000006;
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
    static const int TGT_SIGPOLL        = 0x000017;
    static const int TGT_SIGXCPU        = 0x000018;
    static const int TGT_SIGXFSZ        = 0x000019;
    static const int TGT_SIGVTALRM      = 0x00001a;
    static const int TGT_SIGPROF        = 0x00001b;
    static const int TGT_SIGWINCH       = 0x00001c;
    static const int TGT_SIGLOST        = 0x00001d;
    static const int TGT_SIGPWR         = 0x00001d;
    static const int TGT_SIGUSR1        = 0x00001e;
    static const int TGT_SIGUSR2        = 0x00001f;

    static SyscallFlagTransTable openFlagTable[];

    static const int TGT_O_RDONLY       = 0x00000000;   //!< O_RDONLY
    static const int TGT_O_WRONLY       = 0x00000001;   //!< O_WRONLY
    static const int TGT_O_RDWR         = 0x00000002;   //!< O_RDWR
    static const int TGT_O_NONBLOCK     = 0x00004000;   //!< O_NONBLOCK
    static const int TGT_O_APPEND       = 0x00000008;   //!< O_APPEND
    static const int TGT_FASYNC         = 0x00000040;   //!< FASYNC
    static const int TGT_O_CREAT        = 0x00000200;   //!< O_CREAT
    static const int TGT_O_TRUNC        = 0x00000400;   //!< O_TRUNC
    static const int TGT_O_EXCL         = 0x00000800;   //!< O_EXCL
    static const int TGT_O_NOCTTY       = 0x00008000;   //!< O_NOCTTY
    static const int TGT_O_DSYNC        = 0x00002000;   //!< O_DSYNC
    static const int TGT_O_LARGEFILE    = 0x00040000;   //!< O_LARGEFILE
    static const int TGT_O_DIRECT       = 0x00100000;   //!< O_DIRECT
    static const int TGT_O_NOATIME      = 0x00200000;   //!< O_NOATIME
    static const int TGT_O_CLOEXEC      = 0x00400000;   //!< O_CLOEXEC
    static const int TGT_O_SYNC         = 0x00802000;   //!< O_SYNC
    static const int TGT_O_PATH         = 0x01000000;   //!< O_PATH

    static const int TGT_O_DIRECTORY    = 000200000;   //!< O_DIRECTORY
    static const int TGT_O_NOFOLLOW     = 000400000;   //!< O_NOFOLLOW

    static const int NUM_OPEN_FLAGS;

    static const unsigned TGT_MAP_SHARED        = 0x00001;
    static const unsigned TGT_MAP_PRIVATE       = 0x00002;
    static const unsigned TGT_MAP_ANON          = 0x00020;
    static const unsigned TGT_MAP_DENYWRITE     = 0x00800;
    static const unsigned TGT_MAP_EXECUTABLE    = 0x01000;
    static const unsigned TGT_MAP_FILE          = 0x00000;
    static const unsigned TGT_MAP_GROWSDOWN     = 0x00200;
    static const unsigned TGT_MAP_HUGETLB       = 0x40000;
    static const unsigned TGT_MAP_LOCKED        = 0x00100;
    static const unsigned TGT_MAP_NONBLOCK      = 0x10000;
    static const unsigned TGT_MAP_NORESERVE     = 0x00040;
    static const unsigned TGT_MAP_POPULATE      = 0x08000;
    static const unsigned TGT_MAP_STACK         = 0x20000;
    static const unsigned TGT_MAP_ANONYMOUS     = 0x00020;
    static const unsigned TGT_MAP_FIXED         = 0x00010;
    static const unsigned TGT_MAP_INHERIT       = 0x00080;

    static const unsigned NUM_MMAP_FLAGS;

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

    //@{
    /// ioctl() command codes.
    /// These were calculated using the SPARC Linux headers on an x86
    /// machine and thus may not be correct.  It would be good to
    /// verify/update these values on an actual SPARC Linux machine.
    static const unsigned TGT_TCGETA    = 0x40125401;
    static const unsigned TGT_TCSETAW   = 0x80125403;
    static const unsigned TGT_TCGETS    = 0x40385408;
    static const unsigned TGT_FIONREAD  = 0x4004667f;
    static const unsigned TGT_TIOCGETP  = 0x40067408;
    static const unsigned TGT_TIOCSETP  = 0x80067409;
    static const unsigned TGT_TIOCSETN  = 0x8006740a;
    //@}

    static bool
    isTtyReq(unsigned req)
    {
        switch (req) {
          case TGT_TIOCGETP:
          case TGT_TIOCSETP:
          case TGT_TIOCSETN:
          case TGT_TCGETS:
          case TGT_TCGETA:
          case TGT_TCSETAW:
            return true;
          default:
            return false;
        }
    }

    static void
    archClone(uint64_t flags,
              Process *pp, Process *cp,
              ThreadContext *ptc, ThreadContext *ctc,
              uint64_t stack, uint64_t tls)
    {
        SparcISA::copyRegs(ptc, ctc);
        ctc->setIntReg(SparcISA::INTREG_OTHERWIN, 0);
        ctc->setIntReg(SparcISA::INTREG_CANRESTORE, 0);
        ctc->setIntReg(SparcISA::INTREG_CANSAVE, SparcISA::NWindows - 2);
        ctc->setIntReg(SparcISA::INTREG_CLEANWIN, SparcISA::NWindows);
        ctc->setMiscReg(SparcISA::MISCREG_CWP, 0);
        ctc->setIntReg(SparcISA::INTREG_WSTATE, 0);
        ctc->setMiscRegNoEffect(SparcISA::MISCREG_TL, 0);
        ctc->setMiscReg(SparcISA::MISCREG_ASI, SparcISA::ASI_PRIMARY);
        for (int y = 8; y < 32; y++)
            ctc->setIntReg(y, ptc->readIntReg(y));

        if (stack)
            ctc->setIntReg(SparcISA::StackPointerReg, stack);
    }
};

class Sparc32Linux : public SparcLinux
{
  public:

    typedef struct {
        uint64_t st_dev;
        uint64_t st_ino;
        uint32_t st_mode;
        uint32_t st_nlink;
        uint32_t st_uid;
        uint32_t st_gid;
        uint64_t st_rdev;
        uint8_t __pad3[8];
        int64_t st_size;
        int32_t st_blksize;
        uint8_t __pad4[8];
        int64_t st_blocks;
        uint64_t st_atimeX;
        uint64_t st_atime_nsec;
        uint64_t st_mtimeX;
        uint64_t st_mtime_nsec;
        uint64_t st_ctimeX;
        uint64_t st_ctime_nsec;
        uint32_t __unused4;
        uint32_t __unused5;
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

    /// Resource constants for getrlimit() (overide some generics).
    static const unsigned TGT_RLIMIT_NPROC = 7;
    static const unsigned TGT_RLIMIT_NOFILE = 6;
};

#endif
