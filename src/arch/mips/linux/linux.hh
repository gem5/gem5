/*
 * Copyright © 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright © <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. (“MIPS”) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED “AS IS.”  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Korey L. Sewell
 */

#ifndef __ARCH_MIPS_LINUX_LINUX_HH__
#define __ARCH_MIPS_LINUX_LINUX_HH__

#include "kern/linux/linux.hh"
#include <string>

using std::string;

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
    static const int TGT_O_RDONLY	= 0x00000000;	//!< O_RDONLY
    static const int TGT_O_WRONLY	= 0x00000001;	//!< O_WRONLY
    static const int TGT_O_RDWR	        = 0x00000002;	//!< O_RDWR
    static const int TGT_O_NONBLOCK     = 0x00000080;	//!< O_NONBLOCK
    static const int TGT_O_APPEND	= 0x00000008;	//!< O_APPEND
    static const int TGT_O_CREAT	= 0x00000100;	//!< O_CREAT
    static const int TGT_O_TRUNC	= 0x00000200;	//!< O_TRUNC
    static const int TGT_O_EXCL	        = 0x00000400;	//!< O_EXCL
    static const int TGT_O_NOCTTY	= 0x00000800;	//!< O_NOCTTY
    static const int TGT_O_SYNC	        = 0x00000010;	//!< O_SYNC
    static const int TGT_O_DRD	        = 0x00010000;	//!< O_DRD
    static const int TGT_O_DIRECTIO     = 0x00020000;	//!< O_DIRECTIO
    static const int TGT_O_CACHE	= 0x00002000;	//!< O_CACHE
    static const int TGT_O_DSYNC	= 0x00008000;	//!< O_DSYNC
    static const int TGT_O_RSYNC	= 0x00040000;	//!< O_RSYNC
    //@}

    /// For mmap().
    static const unsigned TGT_MAP_ANONYMOUS = 0x800;

    //@{
    /// For getsysinfo().
    static const unsigned GSI_PLATFORM_NAME = 103;  //!< platform name as string
    static const unsigned GSI_CPU_INFO = 59;	//!< CPU information
    static const unsigned GSI_PROC_TYPE = 60;	//!< get proc_type
    static const unsigned GSI_MAX_CPU = 30;         //!< max # cpu's on this machine
    static const unsigned GSI_CPUS_IN_BOX = 55;	//!< number of CPUs in system
    static const unsigned GSI_PHYSMEM = 19;	        //!< Physical memory in KB
    static const unsigned GSI_CLK_TCK = 42;	        //!< clock freq in Hz
    //@}

    //@{
    /// For getrusage().
    static const int TGT_RUSAGE_SELF = 0;
    static const int TGT_RUSAGE_CHILDREN = -1;
    static const int TGT_RUSAGE_BOTH = -2;
    //@}

    //@{
    /// For setsysinfo().
    static const unsigned SSI_IEEE_FP_CONTROL = 14; //!< ieee_set_fp_control()
    //@}

    //@{
    /// ioctl() command codes.
    static const unsigned TIOCGETP   = 0x7408;
    static const unsigned TIOCSETP   = 0x7409;
    static const unsigned TIOCSETN   = 0x740a;
    static const unsigned TIOCSETC   = 0x7411;
    static const unsigned TIOCGETC   = 0x7412;
    static const unsigned FIONREAD   = 0x467f;
    static const unsigned TIOCISATTY = 0x5480;
    static const unsigned TIOCGETS   = 0x540d;
    static const unsigned TIOCGETA   = 0x7417;
    //@}

    /// For table().
    static const int TBL_SYSINFO = 12;

    /// Resource enumeration for getrlimit()/setrlimit().
    enum rlimit_resources {
        TGT_RLIMIT_CPU = 0,
        TGT_RLIMIT_FSIZE = 1,
        TGT_RLIMIT_DATA = 2,
        TGT_RLIMIT_STACK = 3,
        TGT_RLIMIT_CORE = 4,
        TGT_RLIMIT_NOFILE = 5,
        TGT_RLIMIT_AS = 6,
        TGT_RLIMIT_RSS = 7,
        TGT_RLIMIT_VMEM = 7,
        TGT_RLIMIT_NPROC = 8,
        TGT_RLIMIT_MEMLOCK = 9,
        TGT_RLIMIT_LOCKS = 10,
        NUM_RLIMIT_RESOURCES
    };

    /// Offset used to make sure that processes don't
    /// assign themselves to process IDs reserved for
    /// the root users.
    static const int NUM_ROOT_PROCS = 2;
};

#endif
