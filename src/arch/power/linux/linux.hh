/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
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

#ifndef __ARCH_POWER_LINUX_LINUX_HH__
#define __ARCH_POWER_LINUX_LINUX_HH__

#include "kern/linux/linux.hh"

/*
 * This works for a 2.6.15 kernel.
 */

class PowerLinux : public Linux
{
  public:

    static const ByteOrder byteOrder = BigEndianByteOrder;

    typedef int32_t time_t;

    typedef struct {
        uint64_t st_dev;
        uint32_t __pad1;
        uint32_t st_ino;
        uint32_t st_mode;
        uint32_t st_nlink;
        uint32_t st_uid;
        uint32_t st_gid;
        uint64_t st_rdev;
        uint32_t __pad2;
        uint32_t st_size;
        uint32_t st_blksize;
        uint32_t st_blocks;
        uint32_t st_atimeX;
        uint32_t st_atime_nsec;
        uint32_t st_mtimeX;
        uint32_t st_mtime_nsec;
        uint32_t st_ctimeX;
        uint32_t st_ctime_nsec;
        uint32_t __unused4;
        uint32_t __unused5;
    } tgt_stat;

    typedef struct {
        uint64_t st_dev;
        uint64_t st_ino;
        uint32_t st_mode;
        uint32_t st_nlink;
        uint32_t st_uid;
        uint32_t st_gid;
        uint64_t st_rdev;
        uint64_t __pad2;
        uint64_t st_size;
        uint32_t st_blksize;
        uint32_t __blksize_pad;
        uint64_t st_blocks;
        uint32_t st_atimeX;
        uint32_t st_atime_nsec;
        uint32_t st_mtimeX;
        uint32_t st_mtime_nsec;
        uint32_t st_ctimeX;
        uint32_t st_ctime_nsec;
        uint32_t __unused4;
        uint32_t __unused5;
    } tgt_stat64;

    /// For times().
    struct tms {
        int32_t tms_utime;      //!< user time
        int32_t tms_stime;      //!< system time
        int32_t tms_cutime;     //!< user time of children
        int32_t tms_cstime;     //!< system time of children
    };

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
    static const int TGT_O_DIRECT    = 000400000; //!< O_DIRECT
    static const int TGT_O_LARGEFILE = 000200000; //!< O_LARGEFILE
    static const int TGT_O_DIRECTORY = 000040000; //!< O_DIRECTORY
    static const int TGT_O_NOFOLLOW  = 000100000; //!< O_NOFOLLOW
    static const int TGT_O_NOATIME   = 001000000; //!< O_NOATIME
    static const int TGT_O_CLOEXEC   = 002000000; //!< O_CLOEXEC
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
    static const unsigned TGT_MAP_LOCKED        = 0x00080;
    static const unsigned TGT_MAP_NONBLOCK      = 0x10000;
    static const unsigned TGT_MAP_NORESERVE     = 0x00040;
    static const unsigned TGT_MAP_POPULATE      = 0x08000;
    static const unsigned TGT_MAP_STACK         = 0x20000;
    static const unsigned TGT_MAP_ANONYMOUS     = 0x00020;
    static const unsigned TGT_MAP_FIXED         = 0x00010;

    static const unsigned NUM_MMAP_FLAGS;

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
};

#endif // __ARCH_POWER_LINUX_LINUX_HH__
