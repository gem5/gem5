/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>	// for host open() flags
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>	// for memset()
#include <dirent.h>

#include "sim/host.hh"
#include "cpu/base_cpu.hh"
#include "mem/functional_mem/functional_memory.hh"
#include "sim/process.hh"
#include "cpu/exec_context.hh"
#include "sim/fake_syscall.hh"
#include "sim/sim_events.hh"

#include "sim/syscall_emul.hh"
#include "arch/alpha/alpha_common_syscall_emul.hh"
#include "sim/universe.hh"	// for curTick & ticksPerSecond

#include "arch/alpha/alpha_linux_process.hh"

#include "base/trace.hh"

using namespace std;

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

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY	= 00000000;	//!< O_RDONLY
    static const int TGT_O_WRONLY	= 00000001;	//!< O_WRONLY
    static const int TGT_O_RDWR	 	= 00000002;	//!< O_RDWR
    static const int TGT_O_NONBLOCK  	= 00000004;	//!< O_NONBLOCK
    static const int TGT_O_APPEND	= 00000010;	//!< O_APPEND
    static const int TGT_O_CREAT	= 00001000;	//!< O_CREAT
    static const int TGT_O_TRUNC	= 00002000;	//!< O_TRUNC
    static const int TGT_O_EXCL	 	= 00004000;	//!< O_EXCL
    static const int TGT_O_NOCTTY	= 00010000;	//!< O_NOCTTY
    static const int TGT_O_SYNC	 	= 00040000;	//!< O_SYNC
    static const int TGT_O_DRD	 	= 00100000;	//!< O_DRD
    static const int TGT_O_DIRECTIO  	= 00200000;	//!< O_DIRECTIO
    static const int TGT_O_CACHE	= 00400000;	//!< O_CACHE
    static const int TGT_O_DSYNC	= 02000000;	//!< O_DSYNC
    static const int TGT_O_RSYNC	= 04000000;	//!< O_RSYNC
    //@}

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
        int64_t		st_size;	//!< file size in bytes
        uint64_t	st_atimeX;	//!< time of last access
        uint64_t	st_mtimeX;	//!< time of last modification
        uint64_t	st_ctimeX;	//!< time of last status change
        uint32_t	st_blksize;	//!< optimal I/O block size
        int32_t		st_blocks;	//!< number of blocks allocated
        uint32_t	st_flags;	//!< flags
        uint32_t	st_gen;		//!< unknown
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


    //@{
    /// ioctl() command codes.
    static const unsigned TIOCGETP   = 0x40067408;
    static const unsigned TIOCSETP   = 0x80067409;
    static const unsigned TIOCSETN   = 0x8006740a;
    static const unsigned TIOCSETC   = 0x80067411;
    static const unsigned TIOCGETC   = 0x40067412;
    static const unsigned FIONREAD   = 0x4004667f;
    static const unsigned TIOCISATTY = 0x2000745e;
    //@}

    /// Resource enumeration for getrlimit().
    enum rlimit_resources {
        RLIMIT_CPU = 0,
        RLIMIT_FSIZE = 1,
        RLIMIT_DATA = 2,
        RLIMIT_STACK = 3,
        RLIMIT_CORE = 4,
        RLIMIT_RSS = 5,
        RLIMIT_NOFILE = 6,
        RLIMIT_AS = 7,
        RLIMIT_VMEM = 7,
        RLIMIT_NPROC = 8,
        RLIMIT_MEMLOCK = 9,
        RLIMIT_LOCKS = 10
    };

    /// Limit struct for getrlimit/setrlimit.
    struct rlimit {
        uint64_t  rlim_cur;	//!< soft limit
        uint64_t  rlim_max;	//!< hard limit
    };


    /// For mmap().
    static const unsigned TGT_MAP_ANONYMOUS = 0x10;

    /// For gettimeofday().
    struct timeval {
        int64_t tv_sec;		//!< seconds
        int64_t tv_usec;	//!< microseconds
    };

    //@{
    /// For getrusage().
    static const int RUSAGE_SELF = 0;
    static const int RUSAGE_CHILDREN = -1;
    static const int RUSAGE_BOTH = -2;
    //@}

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
    /// memorty space.  Used by stat(), fstat(), and lstat().
    static void
    copyOutStatBuf(FunctionalMemory *mem, Addr addr, struct stat *host)
    {
        TypedBufferArg<Linux::tgt_stat> tgt(addr);

        tgt->st_dev = host->st_dev;
        tgt->st_ino = host->st_ino;
        tgt->st_mode = host->st_mode;
        tgt->st_nlink = host->st_nlink;
        tgt->st_uid = host->st_uid;
        tgt->st_gid = host->st_gid;
        tgt->st_rdev = host->st_rdev;
        tgt->st_size = host->st_size;
        tgt->st_atimeX = host->st_atime;
        tgt->st_mtimeX = host->st_mtime;
        tgt->st_ctimeX = host->st_ctime;
        tgt->st_blksize = host->st_blksize;
        tgt->st_blocks = host->st_blocks;

        tgt.copyOut(mem);
    }

    /// The target system's hostname.
    static const char *hostname;

    /// Target uname() handler.
    static int
    unameFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
    {
        TypedBufferArg<Linux::utsname> name(xc->getSyscallArg(0));

        strcpy(name->sysname, "Linux");
        strcpy(name->nodename, hostname);
        strcpy(name->release, "2.4.20");
        strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
        strcpy(name->machine, "alpha");

        name.copyOut(xc->mem);
        return 0;
    }

    /// Target osf_getsysyinfo() handler.  Even though this call is
    /// borrowed from Tru64, the subcases that get used appear to be
    /// different in practice from those used by Tru64 processes.
    static int
    osf_getsysinfoFunc(SyscallDesc *desc, int callnum, Process *process,
                       ExecContext *xc)
    {
        unsigned op = xc->getSyscallArg(0);
        // unsigned nbytes = xc->getSyscallArg(2);

        switch (op) {

          case 45: { // GSI_IEEE_FP_CONTROL
              TypedBufferArg<uint64_t> fpcr(xc->getSyscallArg(1));
              // I don't think this exactly matches the HW FPCR
              *fpcr = 0;
              fpcr.copyOut(xc->mem);
              return 1;
          }

          default:
            cerr << "osf_getsysinfo: unknown op " << op << endl;
            abort();
            break;
        }

        return 0;
    }

    /// Target osf_setsysinfo() handler.
    static int
    osf_setsysinfoFunc(SyscallDesc *desc, int callnum, Process *process,
                       ExecContext *xc)
    {
        unsigned op = xc->getSyscallArg(0);
        // unsigned nbytes = xc->getSyscallArg(2);

        switch (op) {

          case 14: { // SSI_IEEE_FP_CONTROL
              TypedBufferArg<uint64_t> fpcr(xc->getSyscallArg(1));
              // I don't think this exactly matches the HW FPCR
              fpcr.copyIn(xc->mem);
              DPRINTFR(SyscallVerbose, "osf_setsysinfo(SSI_IEEE_FP_CONTROL): "
                       " setting FPCR to 0x%x\n", *(uint64_t*)fpcr);
              return 1;
          }

          default:
            cerr << "osf_getsysinfo: unknown op " << op << endl;
            abort();
            break;
        }

        return 0;
    }

    /// Target fnctl() handler.
    static int
    fcntlFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
    {
        int fd = xc->getSyscallArg(0);

        if (fd < 0 || process->sim_fd(fd) < 0)
            return -EBADF;

        int cmd = xc->getSyscallArg(1);
        switch (cmd) {
          case 0: // F_DUPFD
            // if we really wanted to support this, we'd need to do it
            // in the target fd space.
            warn("fcntl(%d, F_DUPFD) not supported, error returned\n", fd);
            return -EMFILE;

          case 1: // F_GETFD (get close-on-exec flag)
          case 2: // F_SETFD (set close-on-exec flag)
            return 0;

          case 3: // F_GETFL (get file flags)
          case 4: // F_SETFL (set file flags)
            // not sure if this is totally valid, but we'll pass it through
            // to the underlying OS
            warn("fcntl(%d, %d) passed through to host\n", fd, cmd);
            return fcntl(process->sim_fd(fd), cmd);
            // return 0;

          case 7: // F_GETLK  (get lock)
          case 8: // F_SETLK  (set lock)
          case 9: // F_SETLKW (set lock and wait)
            // don't mess with file locking... just act like it's OK
            warn("File lock call (fcntl(%d, %d)) ignored.\n", fd, cmd);
            return 0;

          default:
            warn("Unknown fcntl command %d\n", cmd);
            return 0;
        }
    }

    /// Array of syscall descriptors, indexed by call number.
    static SyscallDesc syscallDescs[];

    /// Number of syscalls in syscallDescs[].
    static const int Num_Syscall_Descs;

    /// Max supported syscall number.
    static const int Max_Syscall_Desc;

    /// Do the specified syscall.  Just looks the call number up in
    /// the table and invokes the appropriate handler.
    static void
    doSyscall(int callnum, Process *process, ExecContext *xc)
    {
        if (callnum < 0 || callnum > Max_Syscall_Desc) {
            fatal("Syscall %d out of range", callnum);
        }

        SyscallDesc *desc = &syscallDescs[callnum];

        desc->doSyscall(callnum, process, xc);
    }
};  // class Linux


// open(2) flags translation table
OpenFlagTransTable Linux::openFlagTable[] = {
#ifdef _MSC_VER
  { Linux::TGT_O_RDONLY,	_O_RDONLY },
  { Linux::TGT_O_WRONLY,	_O_WRONLY },
  { Linux::TGT_O_RDWR,		_O_RDWR },
  { Linux::TGT_O_APPEND,	_O_APPEND },
  { Linux::TGT_O_CREAT,		_O_CREAT },
  { Linux::TGT_O_TRUNC,		_O_TRUNC },
  { Linux::TGT_O_EXCL,		_O_EXCL },
#ifdef _O_NONBLOCK
  { Linux::TGT_O_NONBLOCK,	_O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { Linux::TGT_O_NOCTTY,	_O_NOCTTY },
#endif
#ifdef _O_SYNC
  { Linux::TGT_O_SYNC,		_O_SYNC },
#endif
#else /* !_MSC_VER */
  { Linux::TGT_O_RDONLY,	O_RDONLY },
  { Linux::TGT_O_WRONLY,	O_WRONLY },
  { Linux::TGT_O_RDWR,		O_RDWR },
  { Linux::TGT_O_APPEND,	O_APPEND },
  { Linux::TGT_O_CREAT,		O_CREAT },
  { Linux::TGT_O_TRUNC,		O_TRUNC },
  { Linux::TGT_O_EXCL,		O_EXCL },
  { Linux::TGT_O_NONBLOCK,	O_NONBLOCK },
  { Linux::TGT_O_NOCTTY,	O_NOCTTY },
#ifdef O_SYNC
  { Linux::TGT_O_SYNC,		O_SYNC },
#endif
#endif /* _MSC_VER */
};

const int Linux::NUM_OPEN_FLAGS =
        (sizeof(Linux::openFlagTable)/sizeof(Linux::openFlagTable[0]));

const char *Linux::hostname = "m5.eecs.umich.edu";

SyscallDesc Linux::syscallDescs[] = {
    /*  0 */ SyscallDesc("osf_syscall", unimplementedFunc),
    /*  1 */ SyscallDesc("exit", exitFunc),
    /*  2 */ SyscallDesc("fork", unimplementedFunc),
    /*  3 */ SyscallDesc("read", readFunc),
    /*  4 */ SyscallDesc("write", writeFunc),
    /*  5 */ SyscallDesc("osf_old_open", unimplementedFunc),
    /*  6 */ SyscallDesc("close", closeFunc),
    /*  7 */ SyscallDesc("osf_wait4", unimplementedFunc),
    /*  8 */ SyscallDesc("osf_old_creat", unimplementedFunc),
    /*  9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unimplementedFunc),
    /* 11 */ SyscallDesc("osf_execve", unimplementedFunc),
    /* 12 */ SyscallDesc("chdir", unimplementedFunc),
    /* 13 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 14 */ SyscallDesc("mknod", unimplementedFunc),
    /* 15 */ SyscallDesc("chmod", unimplementedFunc),
    /* 16 */ SyscallDesc("chown", unimplementedFunc),
    /* 17 */ SyscallDesc("brk", obreakFunc),
    /* 18 */ SyscallDesc("osf_getfsstat", unimplementedFunc),
    /* 19 */ SyscallDesc("lseek", lseekFunc),
    /* 20 */ SyscallDesc("getxpid", getpidFunc),
    /* 21 */ SyscallDesc("osf_mount", unimplementedFunc),
    /* 22 */ SyscallDesc("umount", unimplementedFunc),
    /* 23 */ SyscallDesc("setuid", setuidFunc),
    /* 24 */ SyscallDesc("getxuid", getuidFunc),
    /* 25 */ SyscallDesc("exec_with_loader", unimplementedFunc),
    /* 26 */ SyscallDesc("osf_ptrace", unimplementedFunc),
    /* 27 */ SyscallDesc("osf_nrecvmsg", unimplementedFunc),
    /* 28 */ SyscallDesc("osf_nsendmsg", unimplementedFunc),
    /* 29 */ SyscallDesc("osf_nrecvfrom", unimplementedFunc),
    /* 30 */ SyscallDesc("osf_naccept", unimplementedFunc),
    /* 31 */ SyscallDesc("osf_ngetpeername", unimplementedFunc),
    /* 32 */ SyscallDesc("osf_ngetsockname", unimplementedFunc),
    /* 33 */ SyscallDesc("access", unimplementedFunc),
    /* 34 */ SyscallDesc("osf_chflags", unimplementedFunc),
    /* 35 */ SyscallDesc("osf_fchflags", unimplementedFunc),
    /* 36 */ SyscallDesc("sync", unimplementedFunc),
    /* 37 */ SyscallDesc("kill", unimplementedFunc),
    /* 38 */ SyscallDesc("osf_old_stat", unimplementedFunc),
    /* 39 */ SyscallDesc("setpgid", unimplementedFunc),
    /* 40 */ SyscallDesc("osf_old_lstat", unimplementedFunc),
    /* 41 */ SyscallDesc("dup", unimplementedFunc),
    /* 42 */ SyscallDesc("pipe", unimplementedFunc),
    /* 43 */ SyscallDesc("osf_set_program_attributes", unimplementedFunc),
    /* 44 */ SyscallDesc("osf_profil", unimplementedFunc),
    /* 45 */ SyscallDesc("open", openFunc<Linux>),
    /* 46 */ SyscallDesc("osf_old_sigaction", unimplementedFunc),
    /* 47 */ SyscallDesc("getxgid", getgidFunc),
    /* 48 */ SyscallDesc("osf_sigprocmask", ignoreFunc),
    /* 49 */ SyscallDesc("osf_getlogin", unimplementedFunc),
    /* 50 */ SyscallDesc("osf_setlogin", unimplementedFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 53 */ SyscallDesc("osf_classcntl", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", ioctlFunc<Linux>),
    /* 55 */ SyscallDesc("osf_reboot", unimplementedFunc),
    /* 56 */ SyscallDesc("osf_revoke", unimplementedFunc),
    /* 57 */ SyscallDesc("symlink", unimplementedFunc),
    /* 58 */ SyscallDesc("readlink", unimplementedFunc),
    /* 59 */ SyscallDesc("execve", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", unimplementedFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("osf_old_fstat", unimplementedFunc),
    /* 63 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 64 */ SyscallDesc("getpagesize", getpagesizeFunc),
    /* 65 */ SyscallDesc("osf_mremap", unimplementedFunc),
    /* 66 */ SyscallDesc("vfork", unimplementedFunc),
    /* 67 */ SyscallDesc("stat", statFunc<Linux>),
    /* 68 */ SyscallDesc("lstat", lstatFunc<Linux>),
    /* 69 */ SyscallDesc("osf_sbrk", unimplementedFunc),
    /* 70 */ SyscallDesc("osf_sstk", unimplementedFunc),
    /* 71 */ SyscallDesc("mmap", mmapFunc<Linux>),
    /* 72 */ SyscallDesc("osf_old_vadvise", unimplementedFunc),
    /* 73 */ SyscallDesc("munmap", munmapFunc),
    /* 74 */ SyscallDesc("mprotect", ignoreFunc),
    /* 75 */ SyscallDesc("madvise", unimplementedFunc),
    /* 76 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 77 */ SyscallDesc("osf_kmodcall", unimplementedFunc),
    /* 78 */ SyscallDesc("osf_mincore", unimplementedFunc),
    /* 79 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 80 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 81 */ SyscallDesc("osf_old_getpgrp", unimplementedFunc),
    /* 82 */ SyscallDesc("setpgrp", unimplementedFunc),
    /* 83 */ SyscallDesc("osf_setitimer", unimplementedFunc),
    /* 84 */ SyscallDesc("osf_old_wait", unimplementedFunc),
    /* 85 */ SyscallDesc("osf_table", unimplementedFunc),
    /* 86 */ SyscallDesc("osf_getitimer", unimplementedFunc),
    /* 87 */ SyscallDesc("gethostname", gethostnameFunc),
    /* 88 */ SyscallDesc("sethostname", unimplementedFunc),
    /* 89 */ SyscallDesc("getdtablesize", unimplementedFunc),
    /* 90 */ SyscallDesc("dup2", unimplementedFunc),
    /* 91 */ SyscallDesc("fstat", fstatFunc<Linux>),
    /* 92 */ SyscallDesc("fcntl", fcntlFunc),
    /* 93 */ SyscallDesc("osf_select", unimplementedFunc),
    /* 94 */ SyscallDesc("poll", unimplementedFunc),
    /* 95 */ SyscallDesc("fsync", unimplementedFunc),
    /* 96 */ SyscallDesc("setpriority", unimplementedFunc),
    /* 97 */ SyscallDesc("socket", unimplementedFunc),
    /* 98 */ SyscallDesc("connect", unimplementedFunc),
    /* 99 */ SyscallDesc("accept", unimplementedFunc),
    /* 100 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 101 */ SyscallDesc("send", unimplementedFunc),
    /* 102 */ SyscallDesc("recv", unimplementedFunc),
    /* 103 */ SyscallDesc("sigreturn", unimplementedFunc),
    /* 104 */ SyscallDesc("bind", unimplementedFunc),
    /* 105 */ SyscallDesc("setsockopt", unimplementedFunc),
    /* 106 */ SyscallDesc("listen", unimplementedFunc),
    /* 107 */ SyscallDesc("osf_plock", unimplementedFunc),
    /* 108 */ SyscallDesc("osf_old_sigvec", unimplementedFunc),
    /* 109 */ SyscallDesc("osf_old_sigblock", unimplementedFunc),
    /* 110 */ SyscallDesc("osf_old_sigsetmask", unimplementedFunc),
    /* 111 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 112 */ SyscallDesc("osf_sigstack", ignoreFunc),
    /* 113 */ SyscallDesc("recvmsg", unimplementedFunc),
    /* 114 */ SyscallDesc("sendmsg", unimplementedFunc),
    /* 115 */ SyscallDesc("osf_old_vtrace", unimplementedFunc),
    /* 116 */ SyscallDesc("osf_gettimeofday", unimplementedFunc),
    /* 117 */ SyscallDesc("osf_getrusage", unimplementedFunc),
    /* 118 */ SyscallDesc("getsockopt", unimplementedFunc),
    /* 119 */ SyscallDesc("numa_syscalls", unimplementedFunc),
    /* 120 */ SyscallDesc("readv", unimplementedFunc),
    /* 121 */ SyscallDesc("writev", unimplementedFunc),
    /* 122 */ SyscallDesc("osf_settimeofday", unimplementedFunc),
    /* 123 */ SyscallDesc("fchown", unimplementedFunc),
    /* 124 */ SyscallDesc("fchmod", unimplementedFunc),
    /* 125 */ SyscallDesc("recvfrom", unimplementedFunc),
    /* 126 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 127 */ SyscallDesc("setregid", unimplementedFunc),
    /* 128 */ SyscallDesc("rename", unimplementedFunc),
    /* 129 */ SyscallDesc("truncate", unimplementedFunc),
    /* 130 */ SyscallDesc("ftruncate", unimplementedFunc),
    /* 131 */ SyscallDesc("flock", unimplementedFunc),
    /* 132 */ SyscallDesc("setgid", unimplementedFunc),
    /* 133 */ SyscallDesc("sendto", unimplementedFunc),
    /* 134 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 135 */ SyscallDesc("socketpair", unimplementedFunc),
    /* 136 */ SyscallDesc("mkdir", unimplementedFunc),
    /* 137 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 138 */ SyscallDesc("osf_utimes", unimplementedFunc),
    /* 139 */ SyscallDesc("osf_old_sigreturn", unimplementedFunc),
    /* 140 */ SyscallDesc("osf_adjtime", unimplementedFunc),
    /* 141 */ SyscallDesc("getpeername", unimplementedFunc),
    /* 142 */ SyscallDesc("osf_gethostid", unimplementedFunc),
    /* 143 */ SyscallDesc("osf_sethostid", unimplementedFunc),
    /* 144 */ SyscallDesc("getrlimit", getrlimitFunc<Linux>),
    /* 145 */ SyscallDesc("setrlimit", unimplementedFunc),
    /* 146 */ SyscallDesc("osf_old_killpg", unimplementedFunc),
    /* 147 */ SyscallDesc("setsid", unimplementedFunc),
    /* 148 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 149 */ SyscallDesc("osf_oldquota", unimplementedFunc),
    /* 150 */ SyscallDesc("getsockname", unimplementedFunc),
    /* 151 */ SyscallDesc("osf_pread", unimplementedFunc),
    /* 152 */ SyscallDesc("osf_pwrite", unimplementedFunc),
    /* 153 */ SyscallDesc("osf_pid_block", unimplementedFunc),
    /* 154 */ SyscallDesc("osf_pid_unblock", unimplementedFunc),
    /* 155 */ SyscallDesc("osf_signal_urti", unimplementedFunc),
    /* 156 */ SyscallDesc("sigaction", ignoreFunc),
    /* 157 */ SyscallDesc("osf_sigwaitprim", unimplementedFunc),
    /* 158 */ SyscallDesc("osf_nfssvc", unimplementedFunc),
    /* 159 */ SyscallDesc("osf_getdirentries", unimplementedFunc),
    /* 160 */ SyscallDesc("osf_statfs", unimplementedFunc),
    /* 161 */ SyscallDesc("osf_fstatfs", unimplementedFunc),
    /* 162 */ SyscallDesc("unknown #162", unimplementedFunc),
    /* 163 */ SyscallDesc("osf_async_daemon", unimplementedFunc),
    /* 164 */ SyscallDesc("osf_getfh", unimplementedFunc),
    /* 165 */ SyscallDesc("osf_getdomainname", unimplementedFunc),
    /* 166 */ SyscallDesc("setdomainname", unimplementedFunc),
    /* 167 */ SyscallDesc("unknown #167", unimplementedFunc),
    /* 168 */ SyscallDesc("unknown #168", unimplementedFunc),
    /* 169 */ SyscallDesc("osf_exportfs", unimplementedFunc),
    /* 170 */ SyscallDesc("unknown #170", unimplementedFunc),
    /* 171 */ SyscallDesc("unknown #171", unimplementedFunc),
    /* 172 */ SyscallDesc("unknown #172", unimplementedFunc),
    /* 173 */ SyscallDesc("unknown #173", unimplementedFunc),
    /* 174 */ SyscallDesc("unknown #174", unimplementedFunc),
    /* 175 */ SyscallDesc("unknown #175", unimplementedFunc),
    /* 176 */ SyscallDesc("unknown #176", unimplementedFunc),
    /* 177 */ SyscallDesc("unknown #177", unimplementedFunc),
    /* 178 */ SyscallDesc("unknown #178", unimplementedFunc),
    /* 179 */ SyscallDesc("unknown #179", unimplementedFunc),
    /* 180 */ SyscallDesc("unknown #180", unimplementedFunc),
    /* 181 */ SyscallDesc("osf_alt_plock", unimplementedFunc),
    /* 182 */ SyscallDesc("unknown #182", unimplementedFunc),
    /* 183 */ SyscallDesc("unknown #183", unimplementedFunc),
    /* 184 */ SyscallDesc("osf_getmnt", unimplementedFunc),
    /* 185 */ SyscallDesc("unknown #185", unimplementedFunc),
    /* 186 */ SyscallDesc("unknown #186", unimplementedFunc),
    /* 187 */ SyscallDesc("osf_alt_sigpending", unimplementedFunc),
    /* 188 */ SyscallDesc("osf_alt_setsid", unimplementedFunc),
    /* 189 */ SyscallDesc("unknown #189", unimplementedFunc),
    /* 190 */ SyscallDesc("unknown #190", unimplementedFunc),
    /* 191 */ SyscallDesc("unknown #191", unimplementedFunc),
    /* 192 */ SyscallDesc("unknown #192", unimplementedFunc),
    /* 193 */ SyscallDesc("unknown #193", unimplementedFunc),
    /* 194 */ SyscallDesc("unknown #194", unimplementedFunc),
    /* 195 */ SyscallDesc("unknown #195", unimplementedFunc),
    /* 196 */ SyscallDesc("unknown #196", unimplementedFunc),
    /* 197 */ SyscallDesc("unknown #197", unimplementedFunc),
    /* 198 */ SyscallDesc("unknown #198", unimplementedFunc),
    /* 199 */ SyscallDesc("osf_swapon", unimplementedFunc),
    /* 200 */ SyscallDesc("msgctl", unimplementedFunc),
    /* 201 */ SyscallDesc("msgget", unimplementedFunc),
    /* 202 */ SyscallDesc("msgrcv", unimplementedFunc),
    /* 203 */ SyscallDesc("msgsnd", unimplementedFunc),
    /* 204 */ SyscallDesc("semctl", unimplementedFunc),
    /* 205 */ SyscallDesc("semget", unimplementedFunc),
    /* 206 */ SyscallDesc("semop", unimplementedFunc),
    /* 207 */ SyscallDesc("osf_utsname", unimplementedFunc),
    /* 208 */ SyscallDesc("lchown", unimplementedFunc),
    /* 209 */ SyscallDesc("osf_shmat", unimplementedFunc),
    /* 210 */ SyscallDesc("shmctl", unimplementedFunc),
    /* 211 */ SyscallDesc("shmdt", unimplementedFunc),
    /* 212 */ SyscallDesc("shmget", unimplementedFunc),
    /* 213 */ SyscallDesc("osf_mvalid", unimplementedFunc),
    /* 214 */ SyscallDesc("osf_getaddressconf", unimplementedFunc),
    /* 215 */ SyscallDesc("osf_msleep", unimplementedFunc),
    /* 216 */ SyscallDesc("osf_mwakeup", unimplementedFunc),
    /* 217 */ SyscallDesc("msync", unimplementedFunc),
    /* 218 */ SyscallDesc("osf_signal", unimplementedFunc),
    /* 219 */ SyscallDesc("osf_utc_gettime", unimplementedFunc),
    /* 220 */ SyscallDesc("osf_utc_adjtime", unimplementedFunc),
    /* 221 */ SyscallDesc("unknown #221", unimplementedFunc),
    /* 222 */ SyscallDesc("osf_security", unimplementedFunc),
    /* 223 */ SyscallDesc("osf_kloadcall", unimplementedFunc),
    /* 224 */ SyscallDesc("unknown #224", unimplementedFunc),
    /* 225 */ SyscallDesc("unknown #225", unimplementedFunc),
    /* 226 */ SyscallDesc("unknown #226", unimplementedFunc),
    /* 227 */ SyscallDesc("unknown #227", unimplementedFunc),
    /* 228 */ SyscallDesc("unknown #228", unimplementedFunc),
    /* 229 */ SyscallDesc("unknown #229", unimplementedFunc),
    /* 230 */ SyscallDesc("unknown #230", unimplementedFunc),
    /* 231 */ SyscallDesc("unknown #231", unimplementedFunc),
    /* 232 */ SyscallDesc("unknown #232", unimplementedFunc),
    /* 233 */ SyscallDesc("getpgid", unimplementedFunc),
    /* 234 */ SyscallDesc("getsid", unimplementedFunc),
    /* 235 */ SyscallDesc("sigaltstack", ignoreFunc),
    /* 236 */ SyscallDesc("osf_waitid", unimplementedFunc),
    /* 237 */ SyscallDesc("osf_priocntlset", unimplementedFunc),
    /* 238 */ SyscallDesc("osf_sigsendset", unimplementedFunc),
    /* 239 */ SyscallDesc("osf_set_speculative", unimplementedFunc),
    /* 240 */ SyscallDesc("osf_msfs_syscall", unimplementedFunc),
    /* 241 */ SyscallDesc("osf_sysinfo", unimplementedFunc),
    /* 242 */ SyscallDesc("osf_uadmin", unimplementedFunc),
    /* 243 */ SyscallDesc("osf_fuser", unimplementedFunc),
    /* 244 */ SyscallDesc("osf_proplist_syscall", unimplementedFunc),
    /* 245 */ SyscallDesc("osf_ntp_adjtime", unimplementedFunc),
    /* 246 */ SyscallDesc("osf_ntp_gettime", unimplementedFunc),
    /* 247 */ SyscallDesc("osf_pathconf", unimplementedFunc),
    /* 248 */ SyscallDesc("osf_fpathconf", unimplementedFunc),
    /* 249 */ SyscallDesc("unknown #249", unimplementedFunc),
    /* 250 */ SyscallDesc("osf_uswitch", unimplementedFunc),
    /* 251 */ SyscallDesc("osf_usleep_thread", unimplementedFunc),
    /* 252 */ SyscallDesc("osf_audcntl", unimplementedFunc),
    /* 253 */ SyscallDesc("osf_audgen", unimplementedFunc),
    /* 254 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 255 */ SyscallDesc("osf_subsys_info", unimplementedFunc),
    /* 256 */ SyscallDesc("osf_getsysinfo", osf_getsysinfoFunc),
    /* 257 */ SyscallDesc("osf_setsysinfo", osf_setsysinfoFunc),
    /* 258 */ SyscallDesc("osf_afs_syscall", unimplementedFunc),
    /* 259 */ SyscallDesc("osf_swapctl", unimplementedFunc),
    /* 260 */ SyscallDesc("osf_memcntl", unimplementedFunc),
    /* 261 */ SyscallDesc("osf_fdatasync", unimplementedFunc),
    /* 262 */ SyscallDesc("unknown #262", unimplementedFunc),
    /* 263 */ SyscallDesc("unknown #263", unimplementedFunc),
    /* 264 */ SyscallDesc("unknown #264", unimplementedFunc),
    /* 265 */ SyscallDesc("unknown #265", unimplementedFunc),
    /* 266 */ SyscallDesc("unknown #266", unimplementedFunc),
    /* 267 */ SyscallDesc("unknown #267", unimplementedFunc),
    /* 268 */ SyscallDesc("unknown #268", unimplementedFunc),
    /* 269 */ SyscallDesc("unknown #269", unimplementedFunc),
    /* 270 */ SyscallDesc("unknown #270", unimplementedFunc),
    /* 271 */ SyscallDesc("unknown #271", unimplementedFunc),
    /* 272 */ SyscallDesc("unknown #272", unimplementedFunc),
    /* 273 */ SyscallDesc("unknown #273", unimplementedFunc),
    /* 274 */ SyscallDesc("unknown #274", unimplementedFunc),
    /* 275 */ SyscallDesc("unknown #275", unimplementedFunc),
    /* 276 */ SyscallDesc("unknown #276", unimplementedFunc),
    /* 277 */ SyscallDesc("unknown #277", unimplementedFunc),
    /* 278 */ SyscallDesc("unknown #278", unimplementedFunc),
    /* 279 */ SyscallDesc("unknown #279", unimplementedFunc),
    /* 280 */ SyscallDesc("unknown #280", unimplementedFunc),
    /* 281 */ SyscallDesc("unknown #281", unimplementedFunc),
    /* 282 */ SyscallDesc("unknown #282", unimplementedFunc),
    /* 283 */ SyscallDesc("unknown #283", unimplementedFunc),
    /* 284 */ SyscallDesc("unknown #284", unimplementedFunc),
    /* 285 */ SyscallDesc("unknown #285", unimplementedFunc),
    /* 286 */ SyscallDesc("unknown #286", unimplementedFunc),
    /* 287 */ SyscallDesc("unknown #287", unimplementedFunc),
    /* 288 */ SyscallDesc("unknown #288", unimplementedFunc),
    /* 289 */ SyscallDesc("unknown #289", unimplementedFunc),
    /* 290 */ SyscallDesc("unknown #290", unimplementedFunc),
    /* 291 */ SyscallDesc("unknown #291", unimplementedFunc),
    /* 292 */ SyscallDesc("unknown #292", unimplementedFunc),
    /* 293 */ SyscallDesc("unknown #293", unimplementedFunc),
    /* 294 */ SyscallDesc("unknown #294", unimplementedFunc),
    /* 295 */ SyscallDesc("unknown #295", unimplementedFunc),
    /* 296 */ SyscallDesc("unknown #296", unimplementedFunc),
    /* 297 */ SyscallDesc("unknown #297", unimplementedFunc),
    /* 298 */ SyscallDesc("unknown #298", unimplementedFunc),
    /* 299 */ SyscallDesc("unknown #299", unimplementedFunc),
/*
 * Linux-specific system calls begin at 300
 */
    /* 300 */ SyscallDesc("bdflush", unimplementedFunc),
    /* 301 */ SyscallDesc("sethae", unimplementedFunc),
    /* 302 */ SyscallDesc("mount", unimplementedFunc),
    /* 303 */ SyscallDesc("old_adjtimex", unimplementedFunc),
    /* 304 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 305 */ SyscallDesc("getdents", unimplementedFunc),
    /* 306 */ SyscallDesc("create_module", unimplementedFunc),
    /* 307 */ SyscallDesc("init_module", unimplementedFunc),
    /* 308 */ SyscallDesc("delete_module", unimplementedFunc),
    /* 309 */ SyscallDesc("get_kernel_syms", unimplementedFunc),
    /* 310 */ SyscallDesc("syslog", unimplementedFunc),
    /* 311 */ SyscallDesc("reboot", unimplementedFunc),
    /* 312 */ SyscallDesc("clone", unimplementedFunc),
    /* 313 */ SyscallDesc("uselib", unimplementedFunc),
    /* 314 */ SyscallDesc("mlock", unimplementedFunc),
    /* 315 */ SyscallDesc("munlock", unimplementedFunc),
    /* 316 */ SyscallDesc("mlockall", unimplementedFunc),
    /* 317 */ SyscallDesc("munlockall", unimplementedFunc),
    /* 318 */ SyscallDesc("sysinfo", unimplementedFunc),
    /* 319 */ SyscallDesc("_sysctl", unimplementedFunc),
    /* 320 */ SyscallDesc("was sys_idle", unimplementedFunc),
    /* 321 */ SyscallDesc("oldumount", unimplementedFunc),
    /* 322 */ SyscallDesc("swapon", unimplementedFunc),
    /* 323 */ SyscallDesc("times", unimplementedFunc),
    /* 324 */ SyscallDesc("personality", unimplementedFunc),
    /* 325 */ SyscallDesc("setfsuid", unimplementedFunc),
    /* 326 */ SyscallDesc("setfsgid", unimplementedFunc),
    /* 327 */ SyscallDesc("ustat", unimplementedFunc),
    /* 328 */ SyscallDesc("statfs", unimplementedFunc),
    /* 329 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 330 */ SyscallDesc("sched_setparam", unimplementedFunc),
    /* 331 */ SyscallDesc("sched_getparam", unimplementedFunc),
    /* 332 */ SyscallDesc("sched_setscheduler", unimplementedFunc),
    /* 333 */ SyscallDesc("sched_getscheduler", unimplementedFunc),
    /* 334 */ SyscallDesc("sched_yield", unimplementedFunc),
    /* 335 */ SyscallDesc("sched_get_priority_max", unimplementedFunc),
    /* 336 */ SyscallDesc("sched_get_priority_min", unimplementedFunc),
    /* 337 */ SyscallDesc("sched_rr_get_interval", unimplementedFunc),
    /* 338 */ SyscallDesc("afs_syscall", unimplementedFunc),
    /* 339 */ SyscallDesc("uname", unameFunc),
    /* 340 */ SyscallDesc("nanosleep", unimplementedFunc),
    /* 341 */ SyscallDesc("mremap", unimplementedFunc),
    /* 342 */ SyscallDesc("nfsservctl", unimplementedFunc),
    /* 343 */ SyscallDesc("setresuid", unimplementedFunc),
    /* 344 */ SyscallDesc("getresuid", unimplementedFunc),
    /* 345 */ SyscallDesc("pciconfig_read", unimplementedFunc),
    /* 346 */ SyscallDesc("pciconfig_write", unimplementedFunc),
    /* 347 */ SyscallDesc("query_module", unimplementedFunc),
    /* 348 */ SyscallDesc("prctl", unimplementedFunc),
    /* 349 */ SyscallDesc("pread", unimplementedFunc),
    /* 350 */ SyscallDesc("pwrite", unimplementedFunc),
    /* 351 */ SyscallDesc("rt_sigreturn", unimplementedFunc),
    /* 352 */ SyscallDesc("rt_sigaction", unimplementedFunc),
    /* 353 */ SyscallDesc("rt_sigprocmask", unimplementedFunc),
    /* 354 */ SyscallDesc("rt_sigpending", unimplementedFunc),
    /* 355 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 356 */ SyscallDesc("rt_sigqueueinfo", unimplementedFunc),
    /* 357 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 358 */ SyscallDesc("select", unimplementedFunc),
    /* 359 */ SyscallDesc("gettimeofday", gettimeofdayFunc<Linux>),
    /* 360 */ SyscallDesc("settimeofday", unimplementedFunc),
    /* 361 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 362 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 363 */ SyscallDesc("utimes", unimplementedFunc),
    /* 364 */ SyscallDesc("getrusage", getrusageFunc<Linux>),
    /* 365 */ SyscallDesc("wait4", unimplementedFunc),
    /* 366 */ SyscallDesc("adjtimex", unimplementedFunc),
    /* 367 */ SyscallDesc("getcwd", unimplementedFunc),
    /* 368 */ SyscallDesc("capget", unimplementedFunc),
    /* 369 */ SyscallDesc("capset", unimplementedFunc),
    /* 370 */ SyscallDesc("sendfile", unimplementedFunc),
    /* 371 */ SyscallDesc("setresgid", unimplementedFunc),
    /* 372 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 373 */ SyscallDesc("dipc", unimplementedFunc),
    /* 374 */ SyscallDesc("pivot_root", unimplementedFunc),
    /* 375 */ SyscallDesc("mincore", unimplementedFunc),
    /* 376 */ SyscallDesc("pciconfig_iobase", unimplementedFunc),
    /* 377 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 378 */ SyscallDesc("gettid", unimplementedFunc),
    /* 379 */ SyscallDesc("readahead", unimplementedFunc),
    /* 380 */ SyscallDesc("security", unimplementedFunc),
    /* 381 */ SyscallDesc("tkill", unimplementedFunc),
    /* 382 */ SyscallDesc("setxattr", unimplementedFunc),
    /* 383 */ SyscallDesc("lsetxattr", unimplementedFunc),
    /* 384 */ SyscallDesc("fsetxattr", unimplementedFunc),
    /* 385 */ SyscallDesc("getxattr", unimplementedFunc),
    /* 386 */ SyscallDesc("lgetxattr", unimplementedFunc),
    /* 387 */ SyscallDesc("fgetxattr", unimplementedFunc),
    /* 388 */ SyscallDesc("listxattr", unimplementedFunc),
    /* 389 */ SyscallDesc("llistxattr", unimplementedFunc),
    /* 390 */ SyscallDesc("flistxattr", unimplementedFunc),
    /* 391 */ SyscallDesc("removexattr", unimplementedFunc),
    /* 392 */ SyscallDesc("lremovexattr", unimplementedFunc),
    /* 393 */ SyscallDesc("fremovexattr", unimplementedFunc),
};

const int Linux::Num_Syscall_Descs =
        sizeof(Linux::syscallDescs) / sizeof(SyscallDesc);

const int Linux::Max_Syscall_Desc = Linux::Num_Syscall_Descs - 1;


void
AlphaLinuxProcess::syscall(ExecContext *xc)
{
    num_syscalls++;

    int64_t callnum = xc->regs.intRegFile[ReturnValueReg];

    Linux::doSyscall(callnum, this, xc);
}


AlphaLinuxProcess::AlphaLinuxProcess(const std::string &name,
                                     ObjectFile *objFile,
                                     int stdin_fd,
                                     int stdout_fd,
                                     int stderr_fd,
                                     std::vector<std::string> &argv,
                                     std::vector<std::string> &envp)
    : LiveProcess(name, objFile, stdin_fd, stdout_fd, stderr_fd, argv, envp)
{
}
