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
 *
 * Authors: Korey Sewell
 */

#ifndef __ARCH_MIPS_LINUX_LINUX_HH__
#define __ARCH_MIPS_LINUX_LINUX_HH__

#include "kern/linux/linux.hh"

class MipsLinux : public Linux
{
  public:

    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static OpenFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY       = 0x00000000;   //!< O_RDONLY
    static const int TGT_O_WRONLY       = 0x00000001;   //!< O_WRONLY
    static const int TGT_O_RDWR         = 0x00000002;   //!< O_RDWR
    static const int TGT_O_NONBLOCK     = 0x00000080;   //!< O_NONBLOCK
    static const int TGT_O_APPEND       = 0x00000008;   //!< O_APPEND
    static const int TGT_O_CREAT        = 0x00000100;   //!< O_CREAT
    static const int TGT_O_TRUNC        = 0x00000200;   //!< O_TRUNC
    static const int TGT_O_EXCL         = 0x00000400;   //!< O_EXCL
    static const int TGT_O_NOCTTY       = 0x00000800;   //!< O_NOCTTY
    static const int TGT_O_SYNC         = 0x00000010;   //!< O_SYNC
    static const int TGT_O_DRD          = 0x00010000;   //!< O_DRD
    static const int TGT_O_DIRECTIO     = 0x00020000;   //!< O_DIRECTIO
    static const int TGT_O_CACHE        = 0x00002000;   //!< O_CACHE
    static const int TGT_O_DSYNC        = 0x00008000;   //!< O_DSYNC
    static const int TGT_O_RSYNC        = 0x00040000;   //!< O_RSYNC
    //@}

    /// For mmap().
    static const unsigned TGT_MAP_ANONYMOUS = 0x800;
    static const unsigned TGT_MAP_FIXED     = 0x10;

    //@{
    /// For getsysinfo().
    static const unsigned GSI_PLATFORM_NAME = 103; //!< platform name as string
    static const unsigned GSI_CPU_INFO = 59;       //!< CPU information
    static const unsigned GSI_PROC_TYPE = 60;      //!< get proc_type
    static const unsigned GSI_MAX_CPU = 30;        //!< max # cpu's on this machine
    static const unsigned GSI_CPUS_IN_BOX = 55;    //!< number of CPUs in system
    static const unsigned GSI_PHYSMEM = 19;        //!< Physical memory in KB
    static const unsigned GSI_CLK_TCK = 42;        //!< clock freq in Hz
    //@}

    //@{
    /// For setsysinfo().
    static const unsigned SSI_IEEE_FP_CONTROL = 14; //!< ieee_set_fp_control()
    //@}

    //@{
    /// ioctl() command codes.
    static const unsigned TGT_TCGETA     = 0x5401;
    static const unsigned TGT_TCSETAW    = 0x5403;
    static const unsigned TGT_TCGETS     = 0x540d;
    static const unsigned TGT_FIONREAD   = 0x467f;
    static const unsigned TGT_TIOCGETP   = 0x7408;
    static const unsigned TGT_TIOCSETP   = 0x7409;
    static const unsigned TGT_TIOCSETN   = 0x740a;
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
   
};

#endif
