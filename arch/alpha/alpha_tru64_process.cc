/*
 * Copyright (c) 2001-2004 The Regents of The University of Michigan
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

#include "arch/alpha/alpha_common_syscall_emul.hh"
#include "arch/alpha/alpha_tru64_process.hh"

#include "sim/syscall_emul.hh"
#include "sim/universe.hh"	// for curTick & ticksPerSecond

#include "base/trace.hh"

using namespace std;

///
/// This class encapsulates the types, structures, constants,
/// functions, and syscall-number mappings specific to the Alpha Tru64
/// syscall interface.
///
class Tru64 {

  public:

    //@{
    /// Basic Tru64 types.
    typedef uint64_t size_t;
    typedef uint64_t off_t;
    typedef uint16_t nlink_t;
    typedef int32_t  dev_t;
    typedef uint32_t uid_t;
    typedef uint32_t gid_t;
    typedef uint32_t time_t;
    typedef uint32_t mode_t;
    typedef uint32_t ino_t;
    //@}

    //@{
    /// open(2) flag values.
    static const int TGT_O_RDONLY	= 00000000;
    static const int TGT_O_WRONLY	= 00000001;
    static const int TGT_O_RDWR	 	= 00000002;
    static const int TGT_O_NONBLOCK 	= 00000004;
    static const int TGT_O_APPEND	= 00000010;
    static const int TGT_O_CREAT	= 00001000;
    static const int TGT_O_TRUNC	= 00002000;
    static const int TGT_O_EXCL	 	= 00004000;
    static const int TGT_O_NOCTTY	= 00010000;
    static const int TGT_O_SYNC	 	= 00040000;
    static const int TGT_O_DRD	 	= 00100000;
    static const int TGT_O_DIRECTIO  	= 00200000;
    static const int TGT_O_CACHE	= 00400000;
    static const int TGT_O_DSYNC	= 02000000;
    static const int TGT_O_RSYNC	= 04000000;
    //@}

    /// This table maps the target open() flags to the corresponding
    /// host open() flags.
    static OpenFlagTransTable openFlagTable[];

    /// Number of entries in openFlagTable[].
    static const int NUM_OPEN_FLAGS;

    /// Stat buffer.  Note that Tru64 v5.0+ use a new "F64" stat structure,
    /// and a new set of syscall numbers for stat calls.  Backwards
    /// compatibility with v4.x should be feasible by implementing
    /// another set of stat functions using the old structure
    /// definition and binding them to the old syscall numbers, but we
    /// haven't done that yet.
    struct F64_stat {
        dev_t	st_dev;			//!< st_dev
        int32_t	st_retired1;		//!< st_retired1
        mode_t	st_mode;		//!< st_mode
        nlink_t	st_nlink;		//!< st_nlink
        uint16_t st_nlink_reserved;	//!< st_nlink_reserved
        uid_t	st_uid;			//!< st_uid
        gid_t	st_gid;			//!< st_gid
        dev_t	st_rdev;		//!< st_rdev
        dev_t	st_ldev;		//!< st_ldev
        off_t	st_size;		//!< st_size
        time_t	st_retired2;		//!< st_retired2
        int32_t	st_uatime;		//!< st_uatime
        time_t	st_retired3;		//!< st_retired3
        int32_t	st_umtime;		//!< st_umtime
        time_t	st_retired4;		//!< st_retired4
        int32_t	st_uctime;		//!< st_uctime
        int32_t	st_retired5;		//!< st_retired5
        int32_t	st_retired6;		//!< st_retired6
        uint32_t	st_flags;	//!< st_flags
        uint32_t	st_gen;		//!< st_gen
        uint64_t	st_spare[4];	//!< st_spare[4]
        ino_t	st_ino;			//!< st_ino
        int32_t	st_ino_reserved;	//!< st_ino_reserved
        time_t	st_atimeX;		//!< st_atime
        int32_t	st_atime_reserved;	//!< st_atime_reserved
        time_t	st_mtimeX;		//!< st_mtime
        int32_t	st_mtime_reserved;	//!< st_mtime_reserved
        time_t	st_ctimeX;		//!< st_ctime
        int32_t	st_ctime_reserved;	//!< st_ctime_reserved
        uint64_t	st_blksize;	//!< st_blksize
        uint64_t	st_blocks;	//!< st_blocks
    };


    /// For getdirentries().
    struct dirent
    {
        ino_t d_ino;		//!< file number of entry
        uint16_t d_reclen;	//!< length of this record
        uint16_t d_namlen;	//!< length of string in d_name
        char d_name[256];	//!< dummy name length
    };


    /// Length of strings in struct utsname (plus 1 for null char).
    static const int _SYS_NMLN = 32;

    /// Interface struct for uname().
    struct utsname {
        char sysname[_SYS_NMLN];        //!< System name.
        char nodename[_SYS_NMLN];       //!< Node name.
        char release[_SYS_NMLN];        //!< OS release.
        char version[_SYS_NMLN];        //!< OS version.
        char machine[_SYS_NMLN];        //!< Machine type.
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
    // TIOCGETS not defined in tru64, so I made up a number
    static const unsigned TIOCGETS   = 0x40000000;
    static const unsigned TIOCGETA   = 0x402c7413;
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
        RLIMIT_VMEM = 7
    };

    /// Limit struct for getrlimit/setrlimit.
    struct rlimit {
        uint64_t  rlim_cur;	//!< soft limit
        uint64_t  rlim_max;	//!< hard limit
    };


    /// For mmap().
    static const unsigned TGT_MAP_ANONYMOUS = 0x10;


    //@{
    /// For getsysinfo().
    static const unsigned GSI_PLATFORM_NAME = 103; //!< platform name as string
    static const unsigned GSI_CPU_INFO = 59;	//!< CPU information
    static const unsigned GSI_PROC_TYPE = 60;	//!< get proc_type
    static const unsigned GSI_MAX_CPU = 30;   //!< max # cpu's on this machine
    static const unsigned GSI_CPUS_IN_BOX = 55;	//!< number of CPUs in system
    static const unsigned GSI_PHYSMEM = 19;	//!< Physical memory in KB
    static const unsigned GSI_CLK_TCK = 42;	//!< clock freq in Hz
    //@}

    /// For getsysinfo() GSI_CPU_INFO option.
    struct cpu_info {
        uint32_t     current_cpu;	//!< current_cpu
        uint32_t     cpus_in_box;	//!< cpus_in_box
        uint32_t     cpu_type;		//!< cpu_type
        uint32_t     ncpus;		//!< ncpus
        uint64_t     cpus_present;	//!< cpus_present
        uint64_t     cpus_running;	//!< cpus_running
        uint64_t     cpu_binding;	//!< cpu_binding
        uint64_t     cpu_ex_binding;	//!< cpu_ex_binding
        uint32_t     mhz;		//!< mhz
        uint32_t     unused[3];		//!< future expansion
    };

    /// For gettimeofday.
    struct timeval {
        uint32_t tv_sec;	//!< seconds
        uint32_t tv_usec;	//!< microseconds
    };

    //@{
    /// For getrusage().
    static const int RUSAGE_THREAD = 1;
    static const int RUSAGE_SELF = 0;
    static const int RUSAGE_CHILDREN = -1;
    //@}

    /// For getrusage().
    struct rusage {
        struct timeval ru_utime;	//!< user time used
        struct timeval ru_stime;	//!< system time used
        uint64_t ru_maxrss;		//!< ru_maxrss
        uint64_t ru_ixrss;		//!< integral shared memory size
        uint64_t ru_idrss;		//!< integral unshared data "
        uint64_t ru_isrss;		//!< integral unshared stack "
        uint64_t ru_minflt;		//!< page reclaims - total vmfaults
        uint64_t ru_majflt;		//!< page faults
        uint64_t ru_nswap;		//!< swaps
        uint64_t ru_inblock;		//!< block input operations
        uint64_t ru_oublock;		//!< block output operations
        uint64_t ru_msgsnd;		//!< messages sent
        uint64_t ru_msgrcv;		//!< messages received
        uint64_t ru_nsignals;		//!< signals received
        uint64_t ru_nvcsw;		//!< voluntary context switches
        uint64_t ru_nivcsw;		//!< involuntary "
    };

    /// For sigreturn().
    struct sigcontext {
        int64_t sc_onstack;		//!< sigstack state to restore
        int64_t sc_mask;		//!< signal mask to restore
        int64_t sc_pc;			//!< pc at time of signal
        int64_t sc_ps;			//!< psl to retore
        int64_t sc_regs[32];		//!< processor regs 0 to 31
        int64_t sc_ownedfp;		//!< fp has been used
        int64_t sc_fpregs[32];		//!< fp regs 0 to 31
        uint64_t sc_fpcr;		//!< floating point control reg
        uint64_t sc_fp_control;		//!< software fpcr
        int64_t sc_reserved1;		//!< reserved for kernel
        uint32_t sc_kreserved1;		//!< reserved for kernel
        uint32_t sc_kreserved2;		//!< reserved for kernel
        size_t  sc_ssize;		//!< stack size
        caddr_t sc_sbase;		//!< stack start
        uint64_t sc_traparg_a0;		//!< a0 argument to trap on exc
        uint64_t sc_traparg_a1;		//!< a1 argument to trap on exc
        uint64_t sc_traparg_a2;		//!< a2 argument to trap on exc
        uint64_t sc_fp_trap_pc;		//!< imprecise pc
        uint64_t sc_fp_trigger_sum;	//!< Exception summary at trigg
        uint64_t sc_fp_trigger_inst; 	//!< Instruction at trigger pc
    };


    /// For table().
    static const int TBL_SYSINFO = 12;

    /// For table().
    struct tbl_sysinfo {
        uint64_t si_user;	//!< User time
        uint64_t si_nice;	//!< Nice time
        uint64_t si_sys;	//!< System time
        uint64_t si_idle;	//!< Idle time
        uint64_t si_hz;		//!< hz
        uint64_t si_phz;	//!< phz
        uint64_t si_boottime;	//!< Boot time in seconds
        uint64_t wait;		//!< Wait time
        uint32_t  si_max_procs;	//!< rpb->rpb_numprocs
        uint32_t  pad;		//!< padding
    };


    /// For stack_create.
    struct vm_stack {
        // was void *
        Addr	address;	//!< address hint
        size_t	rsize;		//!< red zone size
        size_t	ysize;		//!< yellow zone size
        size_t	gsize;		//!< green zone size
        size_t	swap;		//!< amount of swap to reserve
        size_t	incr;		//!< growth increment
        uint64_t	align;		//!< address alignment
        uint64_t	flags;		//!< MAP_FIXED etc.
        // was struct memalloc_attr *
        Addr	attr;		//!< allocation policy
        uint64_t reserved;	//!< reserved
    };

    /// Return values for nxm calls.
    enum {
        KERN_NOT_RECEIVER = 7,
        KERN_NOT_IN_SET = 12
    };

    /// For nxm_task_init.
    static const int NXM_TASK_INIT_VP = 2;	//!< initial thread is VP

    /// Task attribute structure.
    struct nxm_task_attr {
        int64_t nxm_callback;	//!< nxm_callback
        unsigned int nxm_version;	//!< nxm_version
        unsigned short nxm_uniq_offset;	//!< nxm_uniq_offset
        unsigned short flags;	//!< flags
        int nxm_quantum;	//!< nxm_quantum
        int pad1;		//!< pad1
        int64_t pad2;		//!< pad2
    };

    /// Signal set.
    typedef uint64_t   sigset_t;

    /// Thread state shared between user & kernel.
    struct ushared_state {
        sigset_t        sigmask;        //!< thread signal mask
        sigset_t        sig;            //!< thread pending mask
        // struct nxm_pth_state *
        Addr pth_id; //!< out-of-line state
        int             flags;          //!< shared flags
#define US_SIGSTACK     0x1             // thread called sigaltstack
#define US_ONSTACK      0x2             // thread is running on altstack
#define US_PROFILE      0x4             // thread called profil
#define US_SYSCALL      0x8             // thread in syscall
#define US_TRAP         0x10            // thread has trapped
#define US_YELLOW       0x20            // thread has mellowed yellow
#define US_YZONE        0x40            // thread has zoned out
#define US_FP_OWNED     0x80            // thread used floating point

        int             cancel_state;   //!< thread's cancelation state
#define US_CANCEL         0x1           // cancel pending
#define US_NOCANCEL       0X2           // synch cancel disabled
#define US_SYS_NOCANCEL   0x4           // syscall cancel disabled
#define US_ASYNC_NOCANCEL 0x8           // asynch cancel disabled
#define US_CANCEL_BITS  (US_NOCANCEL|US_SYS_NOCANCEL|US_ASYNC_NOCANCEL)
#define US_CANCEL_MASK  (US_CANCEL|US_NOCANCEL|US_SYS_NOCANCEL| \
                         US_ASYNC_NOCANCEL)

        // These are semi-shared. They are always visible to
        // the kernel but are never context-switched by the library.

        int             nxm_ssig;       //!< scheduler's synchronous signals
        int             reserved1;	//!< reserved1
        int64_t            nxm_active;     //!< scheduler active
        int64_t            reserved2;	//!< reserved2
    };

    struct nxm_sched_state {
        struct          ushared_state nxm_u;    //!< state own by user thread
        unsigned int    nxm_bits;               //!< scheduler state / slot
        int             nxm_quantum;            //!< quantum count-down value
        int             nxm_set_quantum;        //!< quantum reset value
        int             nxm_sysevent;           //!< syscall state
        // struct nxm_upcall *
        Addr	    nxm_uc_ret; //!< stack ptr of null thread
        // void *
        Addr nxm_tid;               //!< scheduler's thread id
        int64_t            nxm_va;                 //!< page fault address
        // struct nxm_pth_state *
        Addr nxm_pthid; //!< id of null thread
        uint64_t   nxm_bound_pcs_count;    //!< bound PCS thread count
        int64_t            pad[2];	   //!< pad
    };

    /// nxm_shared.
    struct nxm_shared {
        int64_t nxm_callback;              //!< address of upcall routine
        unsigned int nxm_version;       //!< version number
        unsigned short nxm_uniq_offset; //!< correction factor for TEB
        unsigned short pad1;		//!< pad1
        int64_t space[2];                  //!< future growth
        struct nxm_sched_state nxm_ss[1]; //!< array of shared areas
    };

    /// nxm_slot_state_t.
    enum nxm_slot_state_t {
        NXM_SLOT_AVAIL,
        NXM_SLOT_BOUND,
        NXM_SLOT_UNBOUND,
        NXM_SLOT_EMPTY
    };

    /// nxm_config_info
    struct nxm_config_info {
        int nxm_nslots_per_rad;         //!< max number of VP slots per RAD
        int nxm_nrads;                  //!< max number of RADs
        // nxm_slot_state_t *
        Addr nxm_slot_state; //!< per-VP slot state
        // struct nxm_shared *
        Addr nxm_rad[1];  //!< per-RAD shared areas
    };

    /// For nxm_thread_create.
    enum nxm_thread_type {
        NXM_TYPE_SCS	= 0,
        NXM_TYPE_VP		= 1,
        NXM_TYPE_MANAGER	= 2
    };

    /// Thread attributes.
    struct nxm_thread_attr {
        int version;	//!< version
        int type;	//!< type
        int cancel_flags;	//!< cancel_flags
        int priority;	//!< priority
        int policy;	//!< policy
        int signal_type;	//!< signal_type
        // void *
        Addr pthid;	//!< pthid
        sigset_t sigmask;	//!< sigmask
        /// Initial register values.
        struct {
            uint64_t pc;	//!< pc
            uint64_t sp;	//!< sp
            uint64_t a0;	//!< a0
        } registers;
        uint64_t pad2[2];	//!< pad2
    };

    /// Helper function to convert a host stat buffer to a target stat
    /// buffer.  Also copies the target buffer out to the simulated
    /// memorty space.  Used by stat(), fstat(), and lstat().
    static void
    copyOutStatBuf(FunctionalMemory *mem, Addr addr, struct stat *host)
    {
        TypedBufferArg<Tru64::F64_stat> tgt(addr);

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
    static SyscallReturn
    unameFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
    {
        TypedBufferArg<Tru64::utsname> name(xc->getSyscallArg(0));

        strcpy(name->sysname, "OSF1");
        strcpy(name->nodename, hostname);
        strcpy(name->release, "V5.1");
        strcpy(name->version, "732");
        strcpy(name->machine, "alpha");

        name.copyOut(xc->mem);
        return SyscallReturn(0);
    }


    /// Target getsysyinfo() handler.
    static SyscallReturn
    getsysinfoFunc(SyscallDesc *desc, int callnum, Process *process,
                   ExecContext *xc)
    {
        unsigned op = xc->getSyscallArg(0);
        unsigned nbytes = xc->getSyscallArg(2);

        switch (op) {

          case Tru64::GSI_MAX_CPU: {
              TypedBufferArg<uint32_t> max_cpu(xc->getSyscallArg(1));
              *max_cpu = process->numCpus();
              max_cpu.copyOut(xc->mem);
              return SyscallReturn(1);
          }

          case Tru64::GSI_CPUS_IN_BOX: {
              TypedBufferArg<uint32_t> cpus_in_box(xc->getSyscallArg(1));
              *cpus_in_box = process->numCpus();
              cpus_in_box.copyOut(xc->mem);
              return SyscallReturn(1);
          }

          case Tru64::GSI_PHYSMEM: {
              TypedBufferArg<uint64_t> physmem(xc->getSyscallArg(1));
              *physmem = 1024 * 1024;	// physical memory in KB
              physmem.copyOut(xc->mem);
              return SyscallReturn(1);
          }

          case Tru64::GSI_CPU_INFO: {
              TypedBufferArg<Tru64::cpu_info> infop(xc->getSyscallArg(1));

              infop->current_cpu = 0;
              infop->cpus_in_box = process->numCpus();
              infop->cpu_type = 57;
              infop->ncpus = process->numCpus();
              int cpumask = (1 << process->numCpus()) - 1;
              infop->cpus_present = infop->cpus_running = cpumask;
              infop->cpu_binding = 0;
              infop->cpu_ex_binding = 0;
              infop->mhz = 667;

              infop.copyOut(xc->mem);
              return SyscallReturn(1);
          }

          case Tru64::GSI_PROC_TYPE: {
              TypedBufferArg<uint64_t> proc_type(xc->getSyscallArg(1));
              *proc_type = 11;
              proc_type.copyOut(xc->mem);
              return SyscallReturn(1);
          }

          case Tru64::GSI_PLATFORM_NAME: {
              BufferArg bufArg(xc->getSyscallArg(1), nbytes);
              strncpy((char *)bufArg.bufferPtr(),
                      "COMPAQ Professional Workstation XP1000",
                      nbytes);
              bufArg.copyOut(xc->mem);
              return SyscallReturn(1);
          }

          case Tru64::GSI_CLK_TCK: {
              TypedBufferArg<uint64_t> clk_hz(xc->getSyscallArg(1));
              *clk_hz = 1024;
              clk_hz.copyOut(xc->mem);
              return SyscallReturn(1);
          }

          default:
            cerr << "getsysinfo: unknown op " << op << endl;
            abort();
            break;
        }

        return SyscallReturn(0);
    }

    /// Target fnctl() handler.
    static SyscallReturn
    fcntlFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
    {
        int fd = xc->getSyscallArg(0);

        if (fd < 0 || process->sim_fd(fd) < 0)
            return SyscallReturn(-EBADF);

        int cmd = xc->getSyscallArg(1);
        switch (cmd) {
          case 0: // F_DUPFD
            // if we really wanted to support this, we'd need to do it
            // in the target fd space.
            warn("fcntl(%d, F_DUPFD) not supported, error returned\n", fd);
            return SyscallReturn(-EMFILE);

          case 1: // F_GETFD (get close-on-exec flag)
          case 2: // F_SETFD (set close-on-exec flag)
            return SyscallReturn(0);

          case 3: // F_GETFL (get file flags)
          case 4: // F_SETFL (set file flags)
            // not sure if this is totally valid, but we'll pass it through
            // to the underlying OS
            warn("fcntl(%d, %d) passed through to host\n", fd, cmd);
            return SyscallReturn(fcntl(process->sim_fd(fd), cmd));
            // return 0;

          case 7: // F_GETLK  (get lock)
          case 8: // F_SETLK  (set lock)
          case 9: // F_SETLKW (set lock and wait)
            // don't mess with file locking... just act like it's OK
            warn("File lock call (fcntl(%d, %d)) ignored.\n", fd, cmd);
            return SyscallReturn(0);

          default:
            warn("Unknown fcntl command %d\n", cmd);
            return SyscallReturn(0);
        }
    }


    /// Target getdirentries() handler.
    static SyscallReturn
    getdirentriesFunc(SyscallDesc *desc, int callnum, Process *process,
                      ExecContext *xc)
    {
        int fd = process->sim_fd(xc->getSyscallArg(0));
        Addr tgt_buf = xc->getSyscallArg(1);
        int tgt_nbytes = xc->getSyscallArg(2);
        Addr tgt_basep = xc->getSyscallArg(3);

        char * const host_buf = new char[tgt_nbytes];

        // just pass basep through uninterpreted.
        TypedBufferArg<int64_t> basep(tgt_basep);
        basep.copyIn(xc->mem);
        long host_basep = (off_t)*basep;
        int host_result = getdirentries(fd, host_buf, tgt_nbytes, &host_basep);

        // check for error
        if (host_result < 0) {
            delete [] host_buf;
            return SyscallReturn(-errno);
        }

        // no error: copy results back to target space
        Addr tgt_buf_ptr = tgt_buf;
        char *host_buf_ptr = host_buf;
        char *host_buf_end = host_buf + host_result;
        while (host_buf_ptr < host_buf_end) {
            struct dirent *host_dp = (struct dirent *)host_buf_ptr;
            int namelen = strlen(host_dp->d_name);

            // Actual size includes padded string rounded up for alignment.
            // Subtract 256 for dummy char array in Tru64::dirent definition.
            // Add 1 to namelen for terminating null char.
            int tgt_bufsize = sizeof(Tru64::dirent) - 256 + RoundUp(namelen+1, 8);
            TypedBufferArg<Tru64::dirent> tgt_dp(tgt_buf_ptr, tgt_bufsize);
            tgt_dp->d_ino = host_dp->d_ino;
            tgt_dp->d_reclen = tgt_bufsize;
            tgt_dp->d_namlen = namelen;
            strcpy(tgt_dp->d_name, host_dp->d_name);
            tgt_dp.copyOut(xc->mem);

            tgt_buf_ptr += tgt_bufsize;
            host_buf_ptr += host_dp->d_reclen;
        }

        delete [] host_buf;

        *basep = host_basep;
        basep.copyOut(xc->mem);

        return SyscallReturn(tgt_buf_ptr - tgt_buf);
    }

    /// Target sigreturn() handler.
    static SyscallReturn
    sigreturnFunc(SyscallDesc *desc, int callnum, Process *process,
                  ExecContext *xc)
    {
        RegFile *regs = &xc->regs;
        TypedBufferArg<Tru64::sigcontext> sc(xc->getSyscallArg(0));

        sc.copyIn(xc->mem);

        // Restore state from sigcontext structure.
        // Note that we'll advance PC <- NPC before the end of the cycle,
        // so we need to restore the desired PC into NPC.
        // The current regs->pc will get clobbered.
        regs->npc = sc->sc_pc;

        for (int i = 0; i < 31; ++i) {
            regs->intRegFile[i] = sc->sc_regs[i];
            regs->floatRegFile.q[i] = sc->sc_fpregs[i];
        }

        regs->miscRegs.fpcr = sc->sc_fpcr;

        return SyscallReturn(0);
    }

    /// Target table() handler.
    static SyscallReturn
    tableFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
    {
        int id = xc->getSyscallArg(0);		// table ID
        int index = xc->getSyscallArg(1);	// index into table
        // arg 2 is buffer pointer; type depends on table ID
        int nel = xc->getSyscallArg(3);		// number of elements
        int lel = xc->getSyscallArg(4);		// expected element size

        switch (id) {
          case Tru64::TBL_SYSINFO: {
              if (index != 0 || nel != 1 || lel != sizeof(Tru64::tbl_sysinfo))
                  return SyscallReturn(-EINVAL);
              TypedBufferArg<Tru64::tbl_sysinfo> elp(xc->getSyscallArg(2));

              const int clk_hz = one_million;
              elp->si_user = curTick / (ticksPerSecond / clk_hz);
              elp->si_nice = 0;
              elp->si_sys = 0;
              elp->si_idle = 0;
              elp->wait = 0;
              elp->si_hz = clk_hz;
              elp->si_phz = clk_hz;
              elp->si_boottime = seconds_since_epoch; // seconds since epoch?
              elp->si_max_procs = process->numCpus();
              elp.copyOut(xc->mem);
              return SyscallReturn(0);
          }

          default:
            cerr << "table(): id " << id << " unknown." << endl;
            return SyscallReturn(-EINVAL);
        }
    }

    /// Array of syscall descriptors, indexed by call number.
    static SyscallDesc syscallDescs[];

    /// Number of syscalls in syscallDescs[].
    static const int Num_Syscall_Descs;

    /// Max supported syscall number.
    static const int Max_Syscall_Desc;

    //
    // Mach syscalls -- identified by negated syscall numbers
    //

    /// Create a stack region for a thread.
    static SyscallReturn
    stack_createFunc(SyscallDesc *desc, int callnum, Process *process,
                     ExecContext *xc)
    {
        TypedBufferArg<Tru64::vm_stack> argp(xc->getSyscallArg(0));

        argp.copyIn(xc->mem);

        // if the user chose an address, just let them have it.  Otherwise
        // pick one for them.
        if (argp->address == 0) {
            argp->address = process->next_thread_stack_base;
            int stack_size = (argp->rsize + argp->ysize + argp->gsize);
            process->next_thread_stack_base -= stack_size;
            argp.copyOut(xc->mem);
        }

        return SyscallReturn(0);
    }

    /// NXM library version stamp.
    static
    const int NXM_LIB_VERSION = 301003;

    /// This call sets up the interface between the user and kernel
    /// schedulers by creating a shared-memory region.  The shared memory
    /// region has several structs, some global, some per-RAD, some per-VP.
    static SyscallReturn
    nxm_task_initFunc(SyscallDesc *desc, int callnum, Process *process,
                      ExecContext *xc)
    {
        TypedBufferArg<Tru64::nxm_task_attr> attrp(xc->getSyscallArg(0));
        TypedBufferArg<Addr> configptr_ptr(xc->getSyscallArg(1));

        attrp.copyIn(xc->mem);

        if (attrp->nxm_version != NXM_LIB_VERSION) {
            cerr << "nxm_task_init: thread library version mismatch! "
                 << "got " << attrp->nxm_version
                 << ", expected " << NXM_LIB_VERSION << endl;
            abort();
        }

        if (attrp->flags != Tru64::NXM_TASK_INIT_VP) {
            cerr << "nxm_task_init: bad flag value " << attrp->flags
                 << " (expected " << Tru64::NXM_TASK_INIT_VP << ")" << endl;
            abort();
        }

        const Addr base_addr = 0x12000; // was 0x3f0000000LL;
        Addr cur_addr = base_addr; // next addresses to use
        // first comes the config_info struct
        Addr config_addr = cur_addr;
        cur_addr += sizeof(Tru64::nxm_config_info);
        // next comes the per-cpu state vector
        Addr slot_state_addr = cur_addr;
        int slot_state_size =
            process->numCpus() * sizeof(Tru64::nxm_slot_state_t);
        cur_addr += slot_state_size;
        // now the per-RAD state struct (we only support one RAD)
        cur_addr = 0x14000;	// bump up addr for alignment
        Addr rad_state_addr = cur_addr;
        int rad_state_size =
            (sizeof(Tru64::nxm_shared)
             + (process->numCpus()-1) * sizeof(Tru64::nxm_sched_state));
        cur_addr += rad_state_size;

        // now initialize a config_info struct and copy it out to user space
        TypedBufferArg<Tru64::nxm_config_info> config(config_addr);

        config->nxm_nslots_per_rad = process->numCpus();
        config->nxm_nrads = 1;	// only one RAD in our system!
        config->nxm_slot_state = slot_state_addr;
        config->nxm_rad[0] = rad_state_addr;

        config.copyOut(xc->mem);

        // initialize the slot_state array and copy it out
        TypedBufferArg<Tru64::nxm_slot_state_t> slot_state(slot_state_addr,
                                                           slot_state_size);
        for (int i = 0; i < process->numCpus(); ++i) {
            // CPU 0 is bound to the calling process; all others are available
            slot_state[i] =
                (i == 0) ? Tru64::NXM_SLOT_BOUND : Tru64::NXM_SLOT_AVAIL;
        }

        slot_state.copyOut(xc->mem);

        // same for the per-RAD "shared" struct.  Note that we need to
        // allocate extra bytes for the per-VP array which is embedded at
        // the end.
        TypedBufferArg<Tru64::nxm_shared> rad_state(rad_state_addr,
                                                    rad_state_size);

        rad_state->nxm_callback = attrp->nxm_callback;
        rad_state->nxm_version = attrp->nxm_version;
        rad_state->nxm_uniq_offset = attrp->nxm_uniq_offset;
        for (int i = 0; i < process->numCpus(); ++i) {
            Tru64::nxm_sched_state *ssp = &rad_state->nxm_ss[i];
            ssp->nxm_u.sigmask = 0;
            ssp->nxm_u.sig = 0;
            ssp->nxm_u.flags = 0;
            ssp->nxm_u.cancel_state = 0;
            ssp->nxm_u.nxm_ssig = 0;
            ssp->nxm_bits = 0;
            ssp->nxm_quantum = attrp->nxm_quantum;
            ssp->nxm_set_quantum = attrp->nxm_quantum;
            ssp->nxm_sysevent = 0;

            if (i == 0) {
                uint64_t uniq = xc->regs.miscRegs.uniq;
                ssp->nxm_u.pth_id = uniq + attrp->nxm_uniq_offset;
                ssp->nxm_u.nxm_active = uniq | 1;
            }
            else {
                ssp->nxm_u.pth_id = 0;
                ssp->nxm_u.nxm_active = 0;
            }
        }

        rad_state.copyOut(xc->mem);

        //
        // copy pointer to shared config area out to user
        //
        *configptr_ptr = config_addr;
        configptr_ptr.copyOut(xc->mem);

        return SyscallReturn(0);
    }

    /// Initialize execution context.
    static void
    init_exec_context(ExecContext *ec,
                      Tru64::nxm_thread_attr *attrp, uint64_t uniq_val)
    {
        memset(&ec->regs, 0, sizeof(ec->regs));

        ec->regs.intRegFile[ArgumentReg0] = attrp->registers.a0;
        ec->regs.intRegFile[27/*t12*/] = attrp->registers.pc;
        ec->regs.intRegFile[StackPointerReg] = attrp->registers.sp;
        ec->regs.miscRegs.uniq = uniq_val;

        ec->regs.pc = attrp->registers.pc;
        ec->regs.npc = attrp->registers.pc + sizeof(MachInst);

        ec->activate();
    }

    /// Create thread.
    static SyscallReturn
    nxm_thread_createFunc(SyscallDesc *desc, int callnum, Process *process,
                          ExecContext *xc)
    {
        TypedBufferArg<Tru64::nxm_thread_attr> attrp(xc->getSyscallArg(0));
        TypedBufferArg<uint64_t> kidp(xc->getSyscallArg(1));
        int thread_index = xc->getSyscallArg(2);

        // get attribute args
        attrp.copyIn(xc->mem);

        if (attrp->version != NXM_LIB_VERSION) {
            cerr << "nxm_thread_create: thread library version mismatch! "
                 << "got " << attrp->version
                 << ", expected " << NXM_LIB_VERSION << endl;
            abort();
        }

        if (thread_index < 0 | thread_index > process->numCpus()) {
            cerr << "nxm_thread_create: bad thread index " << thread_index
                 << endl;
            abort();
        }

        // On a real machine, the per-RAD shared structure is in
        // shared memory, so both the user and kernel can get at it.
        // We don't have that luxury, so we just copy it in and then
        // back out again.
        int rad_state_size =
            (sizeof(Tru64::nxm_shared) +
             (process->numCpus()-1) * sizeof(Tru64::nxm_sched_state));

        TypedBufferArg<Tru64::nxm_shared> rad_state(0x14000,
                                                    rad_state_size);
        rad_state.copyIn(xc->mem);

        uint64_t uniq_val = attrp->pthid - rad_state->nxm_uniq_offset;

        if (attrp->type == Tru64::NXM_TYPE_MANAGER) {
            // DEC pthreads seems to always create one of these (in
            // addition to N application threads), but we don't use it,
            // so don't bother creating it.

            // This is supposed to be a port number.  Make something up.
            *kidp = 99;
            kidp.copyOut(xc->mem);

            return SyscallReturn(0);
        } else if (attrp->type == Tru64::NXM_TYPE_VP) {
            // A real "virtual processor" kernel thread.  Need to fork
            // this thread on another CPU.
            Tru64::nxm_sched_state *ssp = &rad_state->nxm_ss[thread_index];

            if (ssp->nxm_u.nxm_active != 0)
                return Tru64::KERN_NOT_RECEIVER;

            ssp->nxm_u.pth_id = attrp->pthid;
            ssp->nxm_u.nxm_active = uniq_val | 1;

            rad_state.copyOut(xc->mem);

            Addr slot_state_addr = 0x12000 + sizeof(Tru64::nxm_config_info);
            int slot_state_size =
                process->numCpus() * sizeof(Tru64::nxm_slot_state_t);

            TypedBufferArg<Tru64::nxm_slot_state_t>
                slot_state(slot_state_addr,
                           slot_state_size);

            slot_state.copyIn(xc->mem);

            if (slot_state[thread_index] != Tru64::NXM_SLOT_AVAIL) {
                cerr << "nxm_thread_createFunc: requested VP slot "
                     << thread_index << " not available!" << endl;
                fatal("");
            }

            slot_state[thread_index] = Tru64::NXM_SLOT_BOUND;

            slot_state.copyOut(xc->mem);

            // Find a free simulator execution context.
            for (int i = 0; i < process->numCpus(); ++i) {
                ExecContext *xc = process->execContexts[i];

                if (xc->status() == ExecContext::Unallocated) {
                    // inactive context... grab it
                    init_exec_context(xc, attrp, uniq_val);

                    // This is supposed to be a port number, but we'll try
                    // and get away with just sticking the thread index
                    // here.
                    *kidp = thread_index;
                    kidp.copyOut(xc->mem);

                    return SyscallReturn(0);
                }
            }

            // fell out of loop... no available inactive context
            cerr << "nxm_thread_create: no idle contexts available." << endl;
            abort();
        } else {
            cerr << "nxm_thread_create: can't handle thread type "
                 << attrp->type << endl;
            abort();
        }

        return SyscallReturn(0);
    }

    /// Thread idle call (like yield()).
    static SyscallReturn
    nxm_idleFunc(SyscallDesc *desc, int callnum, Process *process,
                 ExecContext *xc)
    {
        return SyscallReturn(0);
    }

    /// Block thread.
    static SyscallReturn
    nxm_thread_blockFunc(SyscallDesc *desc, int callnum, Process *process,
                         ExecContext *xc)
    {
        uint64_t tid = xc->getSyscallArg(0);
        uint64_t secs = xc->getSyscallArg(1);
        uint64_t flags = xc->getSyscallArg(2);
        uint64_t action = xc->getSyscallArg(3);
        uint64_t usecs = xc->getSyscallArg(4);

        cout << xc->cpu->name() << ": nxm_thread_block " << tid << " " << secs
             << " " << flags << " " << action << " " << usecs << endl;

        return SyscallReturn(0);
    }

    /// block.
    static SyscallReturn
    nxm_blockFunc(SyscallDesc *desc, int callnum, Process *process,
                  ExecContext *xc)
    {
        Addr uaddr = xc->getSyscallArg(0);
        uint64_t val = xc->getSyscallArg(1);
        uint64_t secs = xc->getSyscallArg(2);
        uint64_t usecs = xc->getSyscallArg(3);
        uint64_t flags = xc->getSyscallArg(4);

        BaseCPU *cpu = xc->cpu;

        cout << cpu->name() << ": nxm_block "
             << hex << uaddr << dec << " " << val
             << " " << secs << " " << usecs
             << " " << flags << endl;

        return SyscallReturn(0);
    }

    /// Unblock thread.
    static SyscallReturn
    nxm_unblockFunc(SyscallDesc *desc, int callnum, Process *process,
                    ExecContext *xc)
    {
        Addr uaddr = xc->getSyscallArg(0);

        cout << xc->cpu->name() << ": nxm_unblock "
             << hex << uaddr << dec << endl;

        return SyscallReturn(0);
    }

    /// Switch thread priority.
    static SyscallReturn
    swtch_priFunc(SyscallDesc *desc, int callnum, Process *process,
                  ExecContext *xc)
    {
        // Attempts to switch to another runnable thread (if there is
        // one).  Returns false if there are no other threads to run
        // (i.e., the thread can reasonably spin-wait) or true if there
        // are other threads.
        //
        // Since we assume at most one "kernel" thread per CPU, it's
        // always safe to return false here.
        return SyscallReturn(0); //false;
    }


    /// Activate exec context waiting on a channel.  Just activate one
    /// by default.
    static int
    activate_waiting_context(Addr uaddr, Process *process,
                             bool activate_all = false)
    {
        int num_activated = 0;

        list<Process::WaitRec>::iterator i = process->waitList.begin();
        list<Process::WaitRec>::iterator end = process->waitList.end();

        while (i != end && (num_activated == 0 || activate_all)) {
            if (i->waitChan == uaddr) {
                // found waiting process: make it active
                ExecContext *newCtx = i->waitingContext;
                assert(newCtx->status() == ExecContext::Suspended);
                newCtx->activate();

                // get rid of this record
                i = process->waitList.erase(i);

                ++num_activated;
            } else {
                ++i;
            }
        }

        return num_activated;
    }

    /// M5 hacked-up lock acquire.
    static void
    m5_lock_mutex(Addr uaddr, Process *process, ExecContext *xc)
    {
        TypedBufferArg<uint64_t> lockp(uaddr);

        lockp.copyIn(xc->mem);

        if (*lockp == 0) {
            // lock is free: grab it
            *lockp = 1;
            lockp.copyOut(xc->mem);
        } else {
            // lock is busy: disable until free
            process->waitList.push_back(Process::WaitRec(uaddr, xc));
            xc->suspend();
        }
    }

    /// M5 unlock call.
    static void
    m5_unlock_mutex(Addr uaddr, Process *process, ExecContext *xc)
    {
        TypedBufferArg<uint64_t> lockp(uaddr);

        lockp.copyIn(xc->mem);
        assert(*lockp != 0);

        // Check for a process waiting on the lock.
        int num_waiting = activate_waiting_context(uaddr, process);

        // clear lock field if no waiting context is taking over the lock
        if (num_waiting == 0) {
            *lockp = 0;
            lockp.copyOut(xc->mem);
        }
    }

    /// Lock acquire syscall handler.
    static SyscallReturn
    m5_mutex_lockFunc(SyscallDesc *desc, int callnum, Process *process,
                      ExecContext *xc)
    {
        Addr uaddr = xc->getSyscallArg(0);

        m5_lock_mutex(uaddr, process, xc);

        // Return 0 since we will always return to the user with the lock
        // acquired.  We will just keep the context inactive until that is
        // true.
        return SyscallReturn(0);
    }

    /// Try lock (non-blocking).
    static SyscallReturn
    m5_mutex_trylockFunc(SyscallDesc *desc, int callnum, Process *process,
                         ExecContext *xc)
    {
        Addr uaddr = xc->getSyscallArg(0);
        TypedBufferArg<uint64_t> lockp(uaddr);

        lockp.copyIn(xc->mem);

        if (*lockp == 0) {
            // lock is free: grab it
            *lockp = 1;
            lockp.copyOut(xc->mem);
            return SyscallReturn(0);
        } else {
            return SyscallReturn(1);
        }
    }

    /// Unlock syscall handler.
    static SyscallReturn
    m5_mutex_unlockFunc(SyscallDesc *desc, int callnum, Process *process,
                        ExecContext *xc)
    {
        Addr uaddr = xc->getSyscallArg(0);

        m5_unlock_mutex(uaddr, process, xc);

        return SyscallReturn(0);
    }

    /// Signal ocndition.
    static SyscallReturn
    m5_cond_signalFunc(SyscallDesc *desc, int callnum, Process *process,
                       ExecContext *xc)
    {
        Addr cond_addr = xc->getSyscallArg(0);

        // Wake up one process waiting on the condition variable.
        activate_waiting_context(cond_addr, process);

        return SyscallReturn(0);
    }

    /// Wake up all processes waiting on the condition variable.
    static SyscallReturn
    m5_cond_broadcastFunc(SyscallDesc *desc, int callnum, Process *process,
                          ExecContext *xc)
    {
        Addr cond_addr = xc->getSyscallArg(0);

        activate_waiting_context(cond_addr, process, true);

        return SyscallReturn(0);
    }

    /// Wait on a condition.
    static SyscallReturn
    m5_cond_waitFunc(SyscallDesc *desc, int callnum, Process *process,
                     ExecContext *xc)
    {
        Addr cond_addr = xc->getSyscallArg(0);
        Addr lock_addr = xc->getSyscallArg(1);
        TypedBufferArg<uint64_t> condp(cond_addr);
        TypedBufferArg<uint64_t> lockp(lock_addr);

        // user is supposed to acquire lock before entering
        lockp.copyIn(xc->mem);
        assert(*lockp != 0);

        m5_unlock_mutex(lock_addr, process, xc);

        process->waitList.push_back(Process::WaitRec(cond_addr, xc));
        xc->suspend();

        return SyscallReturn(0);
    }

    /// Thread exit.
    static SyscallReturn
    m5_thread_exitFunc(SyscallDesc *desc, int callnum, Process *process,
                       ExecContext *xc)
    {
        assert(xc->status() == ExecContext::Active);
        xc->deallocate();

        return SyscallReturn(0);
    }

    /// Array of syscall descriptors for Mach syscalls, indexed by
    /// (negated) call number.
    static SyscallDesc machSyscallDescs[];

    /// Number of syscalls in machSyscallDescs[].
    static const int Num_Mach_Syscall_Descs;

    /// Max supported Mach syscall number.
    static const int Max_Mach_Syscall_Desc;

    /// Since negated values are used to identify Mach syscalls, the
    /// minimum (signed) valid syscall number is the negated max Mach
    /// syscall number.
    static const int Min_Syscall_Desc;

    /// Do the specified syscall.  Just looks the call number up in
    /// the table and invokes the appropriate handler.
    static void
    doSyscall(int callnum, Process *process, ExecContext *xc)
    {
        if (callnum < Min_Syscall_Desc || callnum > Max_Syscall_Desc) {
            cerr << "Syscall " << callnum << " out of range" << endl;
            abort();
        }

        SyscallDesc *desc =
            (callnum < 0) ?
            &machSyscallDescs[-callnum] : &syscallDescs[callnum];

        desc->doSyscall(callnum, process, xc);
    }

    /// Indirect syscall invocation (call #0).
    static SyscallReturn
    indirectSyscallFunc(SyscallDesc *desc, int callnum, Process *process,
                        ExecContext *xc)
    {
        int new_callnum = xc->getSyscallArg(0);

        for (int i = 0; i < 5; ++i)
            xc->setSyscallArg(i, xc->getSyscallArg(i+1));

        doSyscall(new_callnum, process, xc);

        return SyscallReturn(0);
    }

};  // class Tru64


// open(2) flags translation table
OpenFlagTransTable Tru64::openFlagTable[] = {
#ifdef _MSC_VER
  { Tru64::TGT_O_RDONLY,	_O_RDONLY },
  { Tru64::TGT_O_WRONLY,	_O_WRONLY },
  { Tru64::TGT_O_RDWR,		_O_RDWR },
  { Tru64::TGT_O_APPEND,	_O_APPEND },
  { Tru64::TGT_O_CREAT,		_O_CREAT },
  { Tru64::TGT_O_TRUNC,			_O_TRUNC },
  { Tru64::TGT_O_EXCL,			_O_EXCL },
#ifdef _O_NONBLOCK
  { Tru64::TGT_O_NONBLOCK,	_O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { Tru64::TGT_O_NOCTTY,	_O_NOCTTY },
#endif
#ifdef _O_SYNC
  { Tru64::TGT_O_SYNC,	_O_SYNC },
#endif
#else /* !_MSC_VER */
  { Tru64::TGT_O_RDONLY,	O_RDONLY },
  { Tru64::TGT_O_WRONLY,	O_WRONLY },
  { Tru64::TGT_O_RDWR,		O_RDWR },
  { Tru64::TGT_O_APPEND,	O_APPEND },
  { Tru64::TGT_O_CREAT,		O_CREAT },
  { Tru64::TGT_O_TRUNC,		O_TRUNC },
  { Tru64::TGT_O_EXCL,		O_EXCL },
  { Tru64::TGT_O_NONBLOCK,	O_NONBLOCK },
  { Tru64::TGT_O_NOCTTY,	O_NOCTTY },
#ifdef O_SYNC
  { Tru64::TGT_O_SYNC,		O_SYNC },
#endif
#endif /* _MSC_VER */
};

const int Tru64::NUM_OPEN_FLAGS = (sizeof(Tru64::openFlagTable)/sizeof(Tru64::openFlagTable[0]));

const char *Tru64::hostname = "m5.eecs.umich.edu";

SyscallDesc Tru64::syscallDescs[] = {
    /* 0 */ SyscallDesc("syscall (#0)", indirectSyscallFunc,
                        SyscallDesc::SuppressReturnValue),
    /* 1 */ SyscallDesc("exit", exitFunc),
    /* 2 */ SyscallDesc("fork", unimplementedFunc),
    /* 3 */ SyscallDesc("read", readFunc),
    /* 4 */ SyscallDesc("write", writeFunc),
    /* 5 */ SyscallDesc("old_open", unimplementedFunc),
    /* 6 */ SyscallDesc("close", closeFunc),
    /* 7 */ SyscallDesc("wait4", unimplementedFunc),
    /* 8 */ SyscallDesc("old_creat", unimplementedFunc),
    /* 9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unlinkFunc),
    /* 11 */ SyscallDesc("execv", unimplementedFunc),
    /* 12 */ SyscallDesc("chdir", unimplementedFunc),
    /* 13 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 14 */ SyscallDesc("mknod", unimplementedFunc),
    /* 15 */ SyscallDesc("chmod", unimplementedFunc),
    /* 16 */ SyscallDesc("chown", unimplementedFunc),
    /* 17 */ SyscallDesc("obreak", obreakFunc),
    /* 18 */ SyscallDesc("pre_F64_getfsstat", unimplementedFunc),
    /* 19 */ SyscallDesc("lseek", lseekFunc),
    /* 20 */ SyscallDesc("getpid", getpidFunc),
    /* 21 */ SyscallDesc("mount", unimplementedFunc),
    /* 22 */ SyscallDesc("unmount", unimplementedFunc),
    /* 23 */ SyscallDesc("setuid", setuidFunc),
    /* 24 */ SyscallDesc("getuid", getuidFunc),
    /* 25 */ SyscallDesc("exec_with_loader", unimplementedFunc),
    /* 26 */ SyscallDesc("ptrace", unimplementedFunc),
    /* 27 */ SyscallDesc("recvmsg", unimplementedFunc),
    /* 28 */ SyscallDesc("sendmsg", unimplementedFunc),
    /* 29 */ SyscallDesc("recvfrom", unimplementedFunc),
    /* 30 */ SyscallDesc("accept", unimplementedFunc),
    /* 31 */ SyscallDesc("getpeername", unimplementedFunc),
    /* 32 */ SyscallDesc("getsockname", unimplementedFunc),
    /* 33 */ SyscallDesc("access", unimplementedFunc),
    /* 34 */ SyscallDesc("chflags", unimplementedFunc),
    /* 35 */ SyscallDesc("fchflags", unimplementedFunc),
    /* 36 */ SyscallDesc("sync", unimplementedFunc),
    /* 37 */ SyscallDesc("kill", unimplementedFunc),
    /* 38 */ SyscallDesc("old_stat", unimplementedFunc),
    /* 39 */ SyscallDesc("setpgid", unimplementedFunc),
    /* 40 */ SyscallDesc("old_lstat", unimplementedFunc),
    /* 41 */ SyscallDesc("dup", unimplementedFunc),
    /* 42 */ SyscallDesc("pipe", unimplementedFunc),
    /* 43 */ SyscallDesc("set_program_attributes", unimplementedFunc),
    /* 44 */ SyscallDesc("profil", unimplementedFunc),
    /* 45 */ SyscallDesc("open", openFunc<Tru64>),
    /* 46 */ SyscallDesc("obsolete osigaction", unimplementedFunc),
    /* 47 */ SyscallDesc("getgid", getgidFunc),
    /* 48 */ SyscallDesc("sigprocmask", ignoreFunc),
    /* 49 */ SyscallDesc("getlogin", unimplementedFunc),
    /* 50 */ SyscallDesc("setlogin", unimplementedFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 53 */ SyscallDesc("classcntl", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", ioctlFunc<Tru64>),
    /* 55 */ SyscallDesc("reboot", unimplementedFunc),
    /* 56 */ SyscallDesc("revoke", unimplementedFunc),
    /* 57 */ SyscallDesc("symlink", unimplementedFunc),
    /* 58 */ SyscallDesc("readlink", unimplementedFunc),
    /* 59 */ SyscallDesc("execve", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", unimplementedFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("old_fstat", unimplementedFunc),
    /* 63 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 64 */ SyscallDesc("getpagesize", getpagesizeFunc),
    /* 65 */ SyscallDesc("mremap", unimplementedFunc),
    /* 66 */ SyscallDesc("vfork", unimplementedFunc),
    /* 67 */ SyscallDesc("pre_F64_stat", unimplementedFunc),
    /* 68 */ SyscallDesc("pre_F64_lstat", unimplementedFunc),
    /* 69 */ SyscallDesc("sbrk", unimplementedFunc),
    /* 70 */ SyscallDesc("sstk", unimplementedFunc),
    /* 71 */ SyscallDesc("mmap", mmapFunc<Tru64>),
    /* 72 */ SyscallDesc("ovadvise", unimplementedFunc),
    /* 73 */ SyscallDesc("munmap", munmapFunc),
    /* 74 */ SyscallDesc("mprotect", ignoreFunc),
    /* 75 */ SyscallDesc("madvise", unimplementedFunc),
    /* 76 */ SyscallDesc("old_vhangup", unimplementedFunc),
    /* 77 */ SyscallDesc("kmodcall", unimplementedFunc),
    /* 78 */ SyscallDesc("mincore", unimplementedFunc),
    /* 79 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 80 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 81 */ SyscallDesc("old_getpgrp", unimplementedFunc),
    /* 82 */ SyscallDesc("setpgrp", unimplementedFunc),
    /* 83 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 84 */ SyscallDesc("old_wait", unimplementedFunc),
    /* 85 */ SyscallDesc("table", tableFunc),
    /* 86 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 87 */ SyscallDesc("gethostname", gethostnameFunc),
    /* 88 */ SyscallDesc("sethostname", unimplementedFunc),
    /* 89 */ SyscallDesc("getdtablesize", unimplementedFunc),
    /* 90 */ SyscallDesc("dup2", unimplementedFunc),
    /* 91 */ SyscallDesc("pre_F64_fstat", unimplementedFunc),
    /* 92 */ SyscallDesc("fcntl", fcntlFunc),
    /* 93 */ SyscallDesc("select", unimplementedFunc),
    /* 94 */ SyscallDesc("poll", unimplementedFunc),
    /* 95 */ SyscallDesc("fsync", unimplementedFunc),
    /* 96 */ SyscallDesc("setpriority", unimplementedFunc),
    /* 97 */ SyscallDesc("socket", unimplementedFunc),
    /* 98 */ SyscallDesc("connect", unimplementedFunc),
    /* 99 */ SyscallDesc("old_accept", unimplementedFunc),
    /* 100 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 101 */ SyscallDesc("old_send", unimplementedFunc),
    /* 102 */ SyscallDesc("old_recv", unimplementedFunc),
    /* 103 */ SyscallDesc("sigreturn", sigreturnFunc,
                          SyscallDesc::SuppressReturnValue),
    /* 104 */ SyscallDesc("bind", unimplementedFunc),
    /* 105 */ SyscallDesc("setsockopt", unimplementedFunc),
    /* 106 */ SyscallDesc("listen", unimplementedFunc),
    /* 107 */ SyscallDesc("plock", unimplementedFunc),
    /* 108 */ SyscallDesc("old_sigvec", unimplementedFunc),
    /* 109 */ SyscallDesc("old_sigblock", unimplementedFunc),
    /* 110 */ SyscallDesc("old_sigsetmask", unimplementedFunc),
    /* 111 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 112 */ SyscallDesc("sigstack", ignoreFunc),
    /* 113 */ SyscallDesc("old_recvmsg", unimplementedFunc),
    /* 114 */ SyscallDesc("old_sendmsg", unimplementedFunc),
    /* 115 */ SyscallDesc("obsolete vtrace", unimplementedFunc),
    /* 116 */ SyscallDesc("gettimeofday", gettimeofdayFunc<Tru64>),
    /* 117 */ SyscallDesc("getrusage", getrusageFunc<Tru64>),
    /* 118 */ SyscallDesc("getsockopt", unimplementedFunc),
    /* 119 */ SyscallDesc("numa_syscalls", unimplementedFunc),
    /* 120 */ SyscallDesc("readv", unimplementedFunc),
    /* 121 */ SyscallDesc("writev", unimplementedFunc),
    /* 122 */ SyscallDesc("settimeofday", unimplementedFunc),
    /* 123 */ SyscallDesc("fchown", unimplementedFunc),
    /* 124 */ SyscallDesc("fchmod", unimplementedFunc),
    /* 125 */ SyscallDesc("old_recvfrom", unimplementedFunc),
    /* 126 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 127 */ SyscallDesc("setregid", unimplementedFunc),
    /* 128 */ SyscallDesc("rename", renameFunc),
    /* 129 */ SyscallDesc("truncate", unimplementedFunc),
    /* 130 */ SyscallDesc("ftruncate", unimplementedFunc),
    /* 131 */ SyscallDesc("flock", unimplementedFunc),
    /* 132 */ SyscallDesc("setgid", unimplementedFunc),
    /* 133 */ SyscallDesc("sendto", unimplementedFunc),
    /* 134 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 135 */ SyscallDesc("socketpair", unimplementedFunc),
    /* 136 */ SyscallDesc("mkdir", unimplementedFunc),
    /* 137 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 138 */ SyscallDesc("utimes", unimplementedFunc),
    /* 139 */ SyscallDesc("obsolete 4.2 sigreturn", unimplementedFunc),
    /* 140 */ SyscallDesc("adjtime", unimplementedFunc),
    /* 141 */ SyscallDesc("old_getpeername", unimplementedFunc),
    /* 142 */ SyscallDesc("gethostid", unimplementedFunc),
    /* 143 */ SyscallDesc("sethostid", unimplementedFunc),
    /* 144 */ SyscallDesc("getrlimit", getrlimitFunc<Tru64>),
    /* 145 */ SyscallDesc("setrlimit", ignoreFunc),
    /* 146 */ SyscallDesc("old_killpg", unimplementedFunc),
    /* 147 */ SyscallDesc("setsid", unimplementedFunc),
    /* 148 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 149 */ SyscallDesc("oldquota", unimplementedFunc),
    /* 150 */ SyscallDesc("old_getsockname", unimplementedFunc),
    /* 151 */ SyscallDesc("pread", unimplementedFunc),
    /* 152 */ SyscallDesc("pwrite", unimplementedFunc),
    /* 153 */ SyscallDesc("pid_block", unimplementedFunc),
    /* 154 */ SyscallDesc("pid_unblock", unimplementedFunc),
    /* 155 */ SyscallDesc("signal_urti", unimplementedFunc),
    /* 156 */ SyscallDesc("sigaction", ignoreFunc),
    /* 157 */ SyscallDesc("sigwaitprim", unimplementedFunc),
    /* 158 */ SyscallDesc("nfssvc", unimplementedFunc),
    /* 159 */ SyscallDesc("getdirentries", getdirentriesFunc),
    /* 160 */ SyscallDesc("pre_F64_statfs", unimplementedFunc),
    /* 161 */ SyscallDesc("pre_F64_fstatfs", unimplementedFunc),
    /* 162 */ SyscallDesc("unknown #162", unimplementedFunc),
    /* 163 */ SyscallDesc("async_daemon", unimplementedFunc),
    /* 164 */ SyscallDesc("getfh", unimplementedFunc),
    /* 165 */ SyscallDesc("getdomainname", unimplementedFunc),
    /* 166 */ SyscallDesc("setdomainname", unimplementedFunc),
    /* 167 */ SyscallDesc("unknown #167", unimplementedFunc),
    /* 168 */ SyscallDesc("unknown #168", unimplementedFunc),
    /* 169 */ SyscallDesc("exportfs", unimplementedFunc),
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
    /* 181 */ SyscallDesc("alt_plock", unimplementedFunc),
    /* 182 */ SyscallDesc("unknown #182", unimplementedFunc),
    /* 183 */ SyscallDesc("unknown #183", unimplementedFunc),
    /* 184 */ SyscallDesc("getmnt", unimplementedFunc),
    /* 185 */ SyscallDesc("unknown #185", unimplementedFunc),
    /* 186 */ SyscallDesc("unknown #186", unimplementedFunc),
    /* 187 */ SyscallDesc("alt_sigpending", unimplementedFunc),
    /* 188 */ SyscallDesc("alt_setsid", unimplementedFunc),
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
    /* 199 */ SyscallDesc("swapon", unimplementedFunc),
    /* 200 */ SyscallDesc("msgctl", unimplementedFunc),
    /* 201 */ SyscallDesc("msgget", unimplementedFunc),
    /* 202 */ SyscallDesc("msgrcv", unimplementedFunc),
    /* 203 */ SyscallDesc("msgsnd", unimplementedFunc),
    /* 204 */ SyscallDesc("semctl", unimplementedFunc),
    /* 205 */ SyscallDesc("semget", unimplementedFunc),
    /* 206 */ SyscallDesc("semop", unimplementedFunc),
    /* 207 */ SyscallDesc("uname", unameFunc),
    /* 208 */ SyscallDesc("lchown", unimplementedFunc),
    /* 209 */ SyscallDesc("shmat", unimplementedFunc),
    /* 210 */ SyscallDesc("shmctl", unimplementedFunc),
    /* 211 */ SyscallDesc("shmdt", unimplementedFunc),
    /* 212 */ SyscallDesc("shmget", unimplementedFunc),
    /* 213 */ SyscallDesc("mvalid", unimplementedFunc),
    /* 214 */ SyscallDesc("getaddressconf", unimplementedFunc),
    /* 215 */ SyscallDesc("msleep", unimplementedFunc),
    /* 216 */ SyscallDesc("mwakeup", unimplementedFunc),
    /* 217 */ SyscallDesc("msync", unimplementedFunc),
    /* 218 */ SyscallDesc("signal", unimplementedFunc),
    /* 219 */ SyscallDesc("utc_gettime", unimplementedFunc),
    /* 220 */ SyscallDesc("utc_adjtime", unimplementedFunc),
    /* 221 */ SyscallDesc("unknown #221", unimplementedFunc),
    /* 222 */ SyscallDesc("security", unimplementedFunc),
    /* 223 */ SyscallDesc("kloadcall", unimplementedFunc),
    /* 224 */ SyscallDesc("stat", statFunc<Tru64>),
    /* 225 */ SyscallDesc("lstat", lstatFunc<Tru64>),
    /* 226 */ SyscallDesc("fstat", fstatFunc<Tru64>),
    /* 227 */ SyscallDesc("statfs", unimplementedFunc),
    /* 228 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 229 */ SyscallDesc("getfsstat", unimplementedFunc),
    /* 230 */ SyscallDesc("gettimeofday64", unimplementedFunc),
    /* 231 */ SyscallDesc("settimeofday64", unimplementedFunc),
    /* 232 */ SyscallDesc("unknown #232", unimplementedFunc),
    /* 233 */ SyscallDesc("getpgid", unimplementedFunc),
    /* 234 */ SyscallDesc("getsid", unimplementedFunc),
    /* 235 */ SyscallDesc("sigaltstack", ignoreFunc),
    /* 236 */ SyscallDesc("waitid", unimplementedFunc),
    /* 237 */ SyscallDesc("priocntlset", unimplementedFunc),
    /* 238 */ SyscallDesc("sigsendset", unimplementedFunc),
    /* 239 */ SyscallDesc("set_speculative", unimplementedFunc),
    /* 240 */ SyscallDesc("msfs_syscall", unimplementedFunc),
    /* 241 */ SyscallDesc("sysinfo", unimplementedFunc),
    /* 242 */ SyscallDesc("uadmin", unimplementedFunc),
    /* 243 */ SyscallDesc("fuser", unimplementedFunc),
    /* 244 */ SyscallDesc("proplist_syscall", unimplementedFunc),
    /* 245 */ SyscallDesc("ntp_adjtime", unimplementedFunc),
    /* 246 */ SyscallDesc("ntp_gettime", unimplementedFunc),
    /* 247 */ SyscallDesc("pathconf", unimplementedFunc),
    /* 248 */ SyscallDesc("fpathconf", unimplementedFunc),
    /* 249 */ SyscallDesc("sync2", unimplementedFunc),
    /* 250 */ SyscallDesc("uswitch", unimplementedFunc),
    /* 251 */ SyscallDesc("usleep_thread", unimplementedFunc),
    /* 252 */ SyscallDesc("audcntl", unimplementedFunc),
    /* 253 */ SyscallDesc("audgen", unimplementedFunc),
    /* 254 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 255 */ SyscallDesc("subsys_info", unimplementedFunc),
    /* 256 */ SyscallDesc("getsysinfo", getsysinfoFunc),
    /* 257 */ SyscallDesc("setsysinfo", unimplementedFunc),
    /* 258 */ SyscallDesc("afs_syscall", unimplementedFunc),
    /* 259 */ SyscallDesc("swapctl", unimplementedFunc),
    /* 260 */ SyscallDesc("memcntl", unimplementedFunc),
    /* 261 */ SyscallDesc("fdatasync", unimplementedFunc),
    /* 262 */ SyscallDesc("oflock", unimplementedFunc),
    /* 263 */ SyscallDesc("F64_readv", unimplementedFunc),
    /* 264 */ SyscallDesc("F64_writev", unimplementedFunc),
    /* 265 */ SyscallDesc("cdslxlate", unimplementedFunc),
    /* 266 */ SyscallDesc("sendfile", unimplementedFunc),
};

const int Tru64::Num_Syscall_Descs =
        sizeof(Tru64::syscallDescs) / sizeof(SyscallDesc);

const int Tru64::Max_Syscall_Desc = Tru64::Num_Syscall_Descs - 1;

SyscallDesc Tru64::machSyscallDescs[] = {
    /* 0 */  SyscallDesc("kern_invalid", unimplementedFunc),
    /* 1 */  SyscallDesc("m5_mutex_lock", m5_mutex_lockFunc),
    /* 2 */  SyscallDesc("m5_mutex_trylock", m5_mutex_trylockFunc),
    /* 3 */  SyscallDesc("m5_mutex_unlock", m5_mutex_unlockFunc),
    /* 4 */  SyscallDesc("m5_cond_signal", m5_cond_signalFunc),
    /* 5 */  SyscallDesc("m5_cond_broadcast", m5_cond_broadcastFunc),
    /* 6 */  SyscallDesc("m5_cond_wait", m5_cond_waitFunc),
    /* 7 */  SyscallDesc("m5_thread_exit", m5_thread_exitFunc),
    /* 8 */  SyscallDesc("kern_invalid", unimplementedFunc),
    /* 9 */  SyscallDesc("kern_invalid", unimplementedFunc),
    /* 10 */ SyscallDesc("task_self", unimplementedFunc),
    /* 11 */ SyscallDesc("thread_reply", unimplementedFunc),
    /* 12 */ SyscallDesc("task_notify", unimplementedFunc),
    /* 13 */ SyscallDesc("thread_self", unimplementedFunc),
    /* 14 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 15 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 16 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 17 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 18 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 19 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 20 */ SyscallDesc("msg_send_trap", unimplementedFunc),
    /* 21 */ SyscallDesc("msg_receive_trap", unimplementedFunc),
    /* 22 */ SyscallDesc("msg_rpc_trap", unimplementedFunc),
    /* 23 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 24 */ SyscallDesc("nxm_block", nxm_blockFunc),
    /* 25 */ SyscallDesc("nxm_unblock", nxm_unblockFunc),
    /* 26 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 27 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 28 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 29 */ SyscallDesc("nxm_thread_destroy", unimplementedFunc),
    /* 30 */ SyscallDesc("lw_wire", unimplementedFunc),
    /* 31 */ SyscallDesc("lw_unwire", unimplementedFunc),
    /* 32 */ SyscallDesc("nxm_thread_create", nxm_thread_createFunc),
    /* 33 */ SyscallDesc("nxm_task_init", nxm_task_initFunc),
    /* 34 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 35 */ SyscallDesc("nxm_idle", nxm_idleFunc),
    /* 36 */ SyscallDesc("nxm_wakeup_idle", unimplementedFunc),
    /* 37 */ SyscallDesc("nxm_set_pthid", unimplementedFunc),
    /* 38 */ SyscallDesc("nxm_thread_kill", unimplementedFunc),
    /* 39 */ SyscallDesc("nxm_thread_block", nxm_thread_blockFunc),
    /* 40 */ SyscallDesc("nxm_thread_wakeup", unimplementedFunc),
    /* 41 */ SyscallDesc("init_process", unimplementedFunc),
    /* 42 */ SyscallDesc("nxm_get_binding", unimplementedFunc),
    /* 43 */ SyscallDesc("map_fd", unimplementedFunc),
    /* 44 */ SyscallDesc("nxm_resched", unimplementedFunc),
    /* 45 */ SyscallDesc("nxm_set_cancel", unimplementedFunc),
    /* 46 */ SyscallDesc("nxm_set_binding", unimplementedFunc),
    /* 47 */ SyscallDesc("stack_create", stack_createFunc),
    /* 48 */ SyscallDesc("nxm_get_state", unimplementedFunc),
    /* 49 */ SyscallDesc("nxm_thread_suspend", unimplementedFunc),
    /* 50 */ SyscallDesc("nxm_thread_resume", unimplementedFunc),
    /* 51 */ SyscallDesc("nxm_signal_check", unimplementedFunc),
    /* 52 */ SyscallDesc("htg_unix_syscall", unimplementedFunc),
    /* 53 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 54 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 55 */ SyscallDesc("host_self", unimplementedFunc),
    /* 56 */ SyscallDesc("host_priv_self", unimplementedFunc),
    /* 57 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 58 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 59 */ SyscallDesc("swtch_pri", swtch_priFunc),
    /* 60 */ SyscallDesc("swtch", unimplementedFunc),
    /* 61 */ SyscallDesc("thread_switch", unimplementedFunc),
    /* 62 */ SyscallDesc("semop_fast", unimplementedFunc),
    /* 63 */ SyscallDesc("nxm_pshared_init", unimplementedFunc),
    /* 64 */ SyscallDesc("nxm_pshared_block", unimplementedFunc),
    /* 65 */ SyscallDesc("nxm_pshared_unblock", unimplementedFunc),
    /* 66 */ SyscallDesc("nxm_pshared_destroy", unimplementedFunc),
    /* 67 */ SyscallDesc("nxm_swtch_pri", swtch_priFunc),
    /* 68 */ SyscallDesc("lw_syscall", unimplementedFunc),
    /* 69 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 70 */ SyscallDesc("mach_sctimes_0", unimplementedFunc),
    /* 71 */ SyscallDesc("mach_sctimes_1", unimplementedFunc),
    /* 72 */ SyscallDesc("mach_sctimes_2", unimplementedFunc),
    /* 73 */ SyscallDesc("mach_sctimes_3", unimplementedFunc),
    /* 74 */ SyscallDesc("mach_sctimes_4", unimplementedFunc),
    /* 75 */ SyscallDesc("mach_sctimes_5", unimplementedFunc),
    /* 76 */ SyscallDesc("mach_sctimes_6", unimplementedFunc),
    /* 77 */ SyscallDesc("mach_sctimes_7", unimplementedFunc),
    /* 78 */ SyscallDesc("mach_sctimes_8", unimplementedFunc),
    /* 79 */ SyscallDesc("mach_sctimes_9", unimplementedFunc),
    /* 80 */ SyscallDesc("mach_sctimes_10", unimplementedFunc),
    /* 81 */ SyscallDesc("mach_sctimes_11", unimplementedFunc),
    /* 82 */ SyscallDesc("mach_sctimes_port_alloc_dealloc", unimplementedFunc)
};

const int Tru64::Num_Mach_Syscall_Descs =
                sizeof(Tru64::machSyscallDescs) / sizeof(SyscallDesc);
const int Tru64::Max_Mach_Syscall_Desc = Tru64::Num_Mach_Syscall_Descs - 1;
const int Tru64::Min_Syscall_Desc = -Tru64::Max_Mach_Syscall_Desc;


void
AlphaTru64Process::syscall(ExecContext *xc)
{
    num_syscalls++;

    int64_t callnum = xc->regs.intRegFile[ReturnValueReg];

    Tru64::doSyscall(callnum, this, xc);
}


AlphaTru64Process::AlphaTru64Process(const std::string &name,
                                     ObjectFile *objFile,
                                     int stdin_fd,
                                     int stdout_fd,
                                     int stderr_fd,
                                     std::vector<std::string> &argv,
                                     std::vector<std::string> &envp)
    : LiveProcess(name, objFile, stdin_fd, stdout_fd, stderr_fd, argv, envp)
{
}
