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
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>	// for memset()

#include "sim/host.hh"
#include "cpu/base_cpu.hh"
#include "mem/functional_mem/functional_memory.hh"
#include "sim/prog.hh"
#include "cpu/exec_context.hh"
#include "sim/fake_syscall.hh"
#include "sim/sim_events.hh"

#include "targetarch/osf_syscalls.h"
#include "sim/universe.hh"	// for curTick & ticksPerSecond

#include "base/trace.hh"

using namespace std;

//
// System call descriptor
//
class SyscallDesc {

  public:

    typedef int (*FuncPtr)(SyscallDesc *, int num,
                           Process *, ExecContext *);

    const char *name;
    FuncPtr funcPtr;
    int flags;

    SyscallDesc(const char *_name, FuncPtr _funcPtr, int _flags = 0)
        : name(_name), funcPtr(_funcPtr), flags(_flags)
        {}

    int doFunc(int num, Process *proc, ExecContext *xc) {
        return (*funcPtr)(this, num, proc, xc);
    }
};


class BaseBufferArg {

  public:

    BaseBufferArg(Addr _addr, int _size) : addr(_addr), size(_size) {
        bufPtr = new uint8_t[size];
        // clear out buffer: in case we only partially populate this,
        // and then do a copyOut(), we want to make sure we don't
        // introduce any random junk into the simulated address space
        memset(bufPtr, 0, size);
    }

    virtual ~BaseBufferArg() { delete [] bufPtr; }

    //
    // copy data into simulator space (read from target memory)
    //
    virtual bool copyIn(FunctionalMemory *mem) {
        mem->access(Read, addr, bufPtr, size);
        return true;	// no EFAULT detection for now
    }

    //
    // copy data out of simulator space (write to target memory)
    //
    virtual bool copyOut(FunctionalMemory *mem) {
        mem->access(Write, addr, bufPtr, size);
        return true;	// no EFAULT detection for now
    }

  protected:
    Addr addr;
    int size;
    uint8_t *bufPtr;
};


class BufferArg : public BaseBufferArg
{
  public:
    BufferArg(Addr _addr, int _size) : BaseBufferArg(_addr, _size) { }
    void *bufferPtr()	{ return bufPtr; }
};

template <class T>
class TypedBufferArg : public BaseBufferArg
{
  public:
    // user can optionally specify a specific number of bytes to
    // allocate to deal with those structs that have variable-size
    // arrays at the end
    TypedBufferArg(Addr _addr, int _size = sizeof(T))
        : BaseBufferArg(_addr, _size)
    { }

    // type case
    operator T*() { return (T *)bufPtr; }

    // dereference operators
    T& operator*()	 { return *((T *)bufPtr); }
    T* operator->()	 { return (T *)bufPtr; }
    T& operator[](int i) { return ((T *)bufPtr)[i]; }
};


static IntReg
getArg(ExecContext *xc, int i)
{
    return xc->regs.intRegFile[ArgumentReg0 + i];
}


//
// used to shift args for indirect syscall
//
static void
setArg(ExecContext *xc, int i, IntReg val)
{
    xc->regs.intRegFile[ArgumentReg0 + i] = val;
}


static void
set_return_value(ExecContext *xc, IntReg return_value)
{
    // check for error condition.  Alpha syscall convention is to
    // indicate success/failure in reg a3 (r19) and put the
    // return value itself in the standard return value reg (v0).
    const int RegA3 = 19;	// only place this is used
    if (return_value >= 0) {
        // no error
        xc->regs.intRegFile[RegA3] = 0;
        xc->regs.intRegFile[ReturnValueReg] = return_value;
    } else {
        // got an error, return details
        xc->regs.intRegFile[RegA3] = (IntReg) -1;
        xc->regs.intRegFile[ReturnValueReg] = -return_value;
    }
}


int
getpagesizeFunc(SyscallDesc *desc, int callnum, Process *process,
                ExecContext *xc)
{
    return VMPageSize;
}


int
obreakFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // change brk addr to first arg
    process->brk_point = getArg(xc, 0);
    return process->brk_point;
}


int
ioctlFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    int fd = process->sim_fd(getArg(xc, 0));
    unsigned req = getArg(xc, 1);

    switch (req) {
      case OSF::TIOCGETP: {
          // get tty parameters: the main use of this call is by
          // isatty(), which really just wants to see whether it
          // succeeds or returns ENOTTY to determine whether this is
          // a terminal or not.  This call is in turn used by the
          // stdio library to determine whether to do line buffering
          // or block buffering on a specific file descriptor.
          TypedBufferArg<OSF::sgttyb> buf(getArg(xc, 2));

          if (fd < 0) {
              // bad file descriptor
              return -EBADF;
          } else if (0 <= fd < 3) {
              // stdin/stdout/stderr: make it look like a terminal
              // so we get line buffering & not block buffering
              buf->sg_ispeed = 0xf;
              buf->sg_ospeed = 0xf;
              buf->sg_erase = 0x7f;
              buf->sg_kill = 0x15;
              buf->sg_flags = 0x18;
              buf.copyOut(xc->mem);
              return 0;
          } else {
              // any other file descriptor: assume it's a file or
              // pipe and not a terminal
              return -ENOTTY;
          }
          break;
      }

      case OSF::TIOCISATTY:
        if (fd < 0) {
            // bad file descriptor
            return -EBADF;
        } else if (0 <= fd < 3) {
            // stdin/stdout/stderr: make it look like a terminal
            // so we get line buffering & not block buffering
            return 0;
        } else {
            // any other file descriptor: assume it's a file or
            // pipe and not a terminal
            return -ENOTTY;
        }
        break;

      default:
        cerr << "Unsupported ioctl call: ioctl("
             << fd << ", " << req << ", ...)" << endl;
        abort();
        break;
    }
}


int
openFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    string path;

    if (xc->mem->readString(path, getArg(xc, 0)) != No_Fault)
        return -EFAULT;

    if (path == "/dev/sysdev0") {
        // This is a memory-mapped high-resolution timer device on Alpha.
        // We don't support it, so just punt.
        DCOUT(SyscallWarnings) << "Ignoring open(" << path << ", ...)" << endl;
        return -ENOENT;
    }

    int osfFlags = getArg(xc, 1);
    int mode = getArg(xc, 2);
    int hostFlags = 0;

    // translate open flags
    for (int i = 0; i < OSF::NUM_OPEN_FLAGS; i++) {
        if (osfFlags & OSF::openFlagTable[i].osfFlag) {
            osfFlags &= ~OSF::openFlagTable[i].osfFlag;
            hostFlags |= OSF::openFlagTable[i].hostFlag;
        }
    }

    // any target flags left?
    if (osfFlags != 0)
        cerr << "Syscall: open: cannot decode flags: " <<  osfFlags << endl;

#ifdef __CYGWIN32__
    hostFlags |= O_BINARY;
#endif

    // open the file
    int fd = open(path.c_str(), hostFlags, mode);

    return (fd == -1) ? -errno : process->open_fd(fd);
}


int
closeFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    int fd = process->sim_fd(getArg(xc, 0));
    return close(fd);
}


int
readFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    int fd = process->sim_fd(getArg(xc, 0));
    int nbytes = getArg(xc, 2);
    BufferArg bufArg(getArg(xc, 1), nbytes);

    int bytes_read = read(fd, bufArg.bufferPtr(), nbytes);

    if (bytes_read != -1)
        bufArg.copyOut(xc->mem);

    return bytes_read;
}

int
writeFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    int fd = process->sim_fd(getArg(xc, 0));
    int nbytes = getArg(xc, 2);
    BufferArg bufArg(getArg(xc, 1), nbytes);

    bufArg.copyIn(xc->mem);

    int bytes_written = write(fd, bufArg.bufferPtr(), nbytes);

    fsync(fd);

    return bytes_written;
}


int
lseekFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    int fd = process->sim_fd(getArg(xc, 0));
    uint64_t offs = getArg(xc, 1);
    int whence = getArg(xc, 2);

    off_t result = lseek(fd, offs, whence);

    return (result == (off_t)-1) ? -errno : result;
}

static
void
copyOutStatBuf(FunctionalMemory *mem, Addr addr, struct stat *host)
{
    TypedBufferArg<OSF::F64_stat> tgt(addr);

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

int
statFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    string path;

    if (xc->mem->readString(path, getArg(xc, 0)) != No_Fault)
        return -EFAULT;

    struct stat hostBuf;
    int result = stat(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatBuf(xc->mem, getArg(xc, 1), &hostBuf);

    return 0;
}


int
lstatFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    string path;

    if (xc->mem->readString(path, getArg(xc, 0)) != No_Fault)
        return -EFAULT;

    struct stat hostBuf;
    int result = lstat(path.c_str(), &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatBuf(xc->mem, getArg(xc, 1), &hostBuf);

    return 0;
}

int
fstatFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    int fd = process->sim_fd(getArg(xc, 0));

    if (fd < 0)
        return -EBADF;

    struct stat hostBuf;
    int result = fstat(fd, &hostBuf);

    if (result < 0)
        return -errno;

    copyOutStatBuf(xc->mem, getArg(xc, 1), &hostBuf);

    return 0;
}


//
// We don't handle mmap().  If the target is really mmaping /dev/zero,
// we can get away with doing nothing (since the simulator doesn't
// really check addresses anyway).  Always print a warning, since this
// could be seriously broken if we're not mapping /dev/zero.
//
// Someday we should explicitly check for /dev/zero in open, flag the
// file descriptor, and fail an mmap to anything else.
//
int
mmapFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    Addr start = getArg(xc, 0);
    uint64_t length = getArg(xc, 1);
    int prot = getArg(xc, 2);
    int flags = getArg(xc, 3);
    int fd = process->sim_fd(getArg(xc, 4));
    int offset = getArg(xc, 5);

    cerr << "Warning: ignoring syscall mmap("
         << start << ", " << length << ", "
         << prot << ", " << flags << ", "
         << fd << " " << getArg(xc, 4) << ", "
         << offset << ")" << endl;

    return start;
}


const char *hostname = "m5.eecs.umich.edu";

int
unameFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    TypedBufferArg<OSF::utsname> name(getArg(xc, 0));

    strcpy(name->sysname, "OSF1");
    strcpy(name->nodename, hostname);
    strcpy(name->release, "V5.1");
    strcpy(name->version, "732");
    strcpy(name->machine, "alpha");

    name.copyOut(xc->mem);
    return 0;
}


int
gethostnameFunc(SyscallDesc *desc, int callnum, Process *process,
                ExecContext *xc)
{
    int name_len = getArg(xc, 1);
    BufferArg name(getArg(xc, 0), name_len);

    strncpy((char *)name.bufferPtr(), hostname, name_len);

    name.copyOut(xc->mem);

    return 0;
}


int
getsysinfoFunc(SyscallDesc *desc, int callnum, Process *process,
               ExecContext *xc)
{
    unsigned op = getArg(xc, 0);
    unsigned nbytes = getArg(xc, 2);

    switch (op) {

      case OSF::GSI_MAX_CPU: {
          TypedBufferArg<uint32_t> max_cpu(getArg(xc, 1));
          *max_cpu = process->numCpus;
          max_cpu.copyOut(xc->mem);
          return 1;
      }

      case OSF::GSI_CPUS_IN_BOX: {
          TypedBufferArg<uint32_t> cpus_in_box(getArg(xc, 1));
          *cpus_in_box = process->numCpus;
          cpus_in_box.copyOut(xc->mem);
          return 1;
      }

      case OSF::GSI_PHYSMEM: {
          TypedBufferArg<uint64_t> physmem(getArg(xc, 1));
          *physmem = 1024 * 1024;	// physical memory in KB
          physmem.copyOut(xc->mem);
          return 1;
      }

      case OSF::GSI_CPU_INFO: {
          TypedBufferArg<OSF::cpu_info> infop(getArg(xc, 1));

          infop->current_cpu = 0;
          infop->cpus_in_box = process->numCpus;
          infop->cpu_type = 57;
          infop->ncpus = process->numCpus;
          int cpumask = (1 << process->numCpus) - 1;
          infop->cpus_present = infop->cpus_running = cpumask;
          infop->cpu_binding = 0;
          infop->cpu_ex_binding = 0;
          infop->mhz = 667;

          infop.copyOut(xc->mem);
          return 1;
        }

      case OSF::GSI_PROC_TYPE: {
          TypedBufferArg<uint64_t> proc_type(getArg(xc, 1));
          *proc_type = 11;
          proc_type.copyOut(xc->mem);
          return 1;
      }

      case OSF::GSI_PLATFORM_NAME: {
          BufferArg bufArg(getArg(xc, 1), nbytes);
          strncpy((char *)bufArg.bufferPtr(),
                  "COMPAQ Professional Workstation XP1000",
                  nbytes);
          bufArg.copyOut(xc->mem);
          return 1;
      }

      case OSF::GSI_CLK_TCK: {
          TypedBufferArg<uint64_t> clk_hz(getArg(xc, 1));
          *clk_hz = 1024;
          clk_hz.copyOut(xc->mem);
          return 1;
      }

      default:
        cerr << "getsysinfo: unknown op " << op << endl;
        abort();
        break;
    }

    return 0;
}

int
getpidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // Make up a PID.  There's no interprocess communication in
    // fake_syscall mode, so there's no way for a process to know it's
    // not getting a unique value.
    return 100;
}

int
getuidFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    // Make up a UID.
    return 100;
}

int
getrlimitFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
{
    unsigned resource = getArg(xc, 0);
    TypedBufferArg<OSF::rlimit> rlp(getArg(xc, 1));

    switch (resource) {
      case OSF::RLIMIT_STACK:
        // max stack size in bytes: make up a number (2MB for now)
        rlp->rlim_cur = rlp->rlim_max = 8 * 1024 * 1024;
        break;

      default:
        cerr << "getrlimitFunc: unimplemented resource " << resource << endl;
        abort();
        break;
    }

    rlp.copyOut(xc->mem);
    return 0;
}

// 1M usecs in 1 sec, for readability
static const int one_million = 1000000;

// seconds since the epoch (1/1/1970)... about a billion, by my reckoning
static const unsigned seconds_since_epoch = 1000000000;

//
// helper function: populate struct timeval with approximation of
// current elapsed time
//
static void
getElapsedTime(OSF::timeval *tp)
{
    int cycles_per_usec = ticksPerSecond / one_million;

    int elapsed_usecs = curTick / cycles_per_usec;
    tp->tv_sec = elapsed_usecs / one_million;
    tp->tv_usec = elapsed_usecs % one_million;
}


int
gettimeofdayFunc(SyscallDesc *desc, int callnum, Process *process,
                 ExecContext *xc)
{
    TypedBufferArg<OSF::timeval> tp(getArg(xc, 0));

    getElapsedTime(tp);
    tp->tv_sec += seconds_since_epoch;

    tp.copyOut(xc->mem);

    return 0;
}


int
getrusageFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
{
    int who = getArg(xc, 0);	// THREAD, SELF, or CHILDREN
    TypedBufferArg<OSF::rusage> rup(getArg(xc, 1));

    if (who != OSF::RUSAGE_SELF) {
        // don't really handle THREAD or CHILDREN, but just warn and
        // plow ahead
        DCOUT(SyscallWarnings)
            << "Warning: getrusage() only supports RUSAGE_SELF."
            << "  Parameter " << who << " ignored." << endl;
    }

    getElapsedTime(&rup->ru_utime);
    rup->ru_stime.tv_sec = 0;
    rup->ru_stime.tv_usec = 0;
    rup->ru_maxrss = 0;
    rup->ru_ixrss = 0;
    rup->ru_idrss = 0;
    rup->ru_isrss = 0;
    rup->ru_minflt = 0;
    rup->ru_majflt = 0;
    rup->ru_nswap = 0;
    rup->ru_inblock = 0;
    rup->ru_oublock = 0;
    rup->ru_msgsnd = 0;
    rup->ru_msgrcv = 0;
    rup->ru_nsignals = 0;
    rup->ru_nvcsw = 0;
    rup->ru_nivcsw = 0;

    rup.copyOut(xc->mem);

    return 0;
}


int
sigreturnFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
{
    RegFile *regs = &xc->regs;
    TypedBufferArg<OSF::sigcontext> sc(getArg(xc, 0));

    sc.copyIn(xc->mem);

    // restore state from sigcontext structure
    regs->pc = sc->sc_pc;
    regs->npc = regs->pc + sizeof(MachInst);

    for (int i = 0; i < 31; ++i) {
        regs->intRegFile[i] = sc->sc_regs[i];
        regs->floatRegFile.q[i] = sc->sc_fpregs[i];
    }

    regs->miscRegs.fpcr = sc->sc_fpcr;

    return 0;
}

int
tableFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    int id = getArg(xc, 0);		// table ID
    int index = getArg(xc, 1);	// index into table
    // arg 2 is buffer pointer; type depends on table ID
    int nel = getArg(xc, 3);		// number of elements
    int lel = getArg(xc, 4);		// expected element size

    switch (id) {
      case OSF::TBL_SYSINFO: {
          if (index != 0 || nel != 1 || lel != sizeof(OSF::tbl_sysinfo))
              return -EINVAL;
          TypedBufferArg<OSF::tbl_sysinfo> elp(getArg(xc, 2));

          const int clk_hz = one_million;
          elp->si_user = curTick / (ticksPerSecond / clk_hz);
          elp->si_nice = 0;
          elp->si_sys = 0;
          elp->si_idle = 0;
          elp->wait = 0;
          elp->si_hz = clk_hz;
          elp->si_phz = clk_hz;
          elp->si_boottime = seconds_since_epoch; // seconds since epoch?
          elp->si_max_procs = process->numCpus;
          elp.copyOut(xc->mem);
          return 0;
      }

      default:
        cerr << "table(): id " << id << " unknown." << endl;
        return -EINVAL;
    }
}

//
// forward declaration... defined below table
//
int
indirectSyscallFunc(SyscallDesc *desc, int callnum, Process *process,
                    ExecContext *xc);

//
// Handler for unimplemented syscalls that we haven't thought about.
//
int
unimplementedFunc(SyscallDesc *desc, int callnum, Process *process,
                  ExecContext *xc)
{
    cerr << "Error: syscall " << desc->name
         << " (#" << callnum << ") unimplemented.";
    cerr << "  Args: " << getArg(xc, 0) << ", " << getArg(xc, 1)
         << ", ..." << endl;

    abort();
}


//
// Handler for unimplemented syscalls that we never intend to
// implement (signal handling, etc.) and should not affect the correct
// behavior of the program.  Print a warning only if the appropriate
// trace flag is enabled.  Return success to the target program.
//
int
ignoreFunc(SyscallDesc *desc, int callnum, Process *process,
           ExecContext *xc)
{
    DCOUT(SyscallWarnings) << "Warning: ignoring syscall " << desc->name
                           << "(" << getArg(xc, 0)
                           << ", " << getArg(xc, 1)
                           << ", ...)" << endl;

    return 0;
}


int
exitFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    new SimExitEvent("syscall caused exit", getArg(xc, 0) & 0xff);

    return 1;
}


SyscallDesc syscallDescs[] = {
    /* 0 */ SyscallDesc("syscall (#0)", indirectSyscallFunc),
    /* 1 */ SyscallDesc("exit", exitFunc),
    /* 2 */ SyscallDesc("fork", unimplementedFunc),
    /* 3 */ SyscallDesc("read", readFunc),
    /* 4 */ SyscallDesc("write", writeFunc),
    /* 5 */ SyscallDesc("old_open", unimplementedFunc),
    /* 6 */ SyscallDesc("close", closeFunc),
    /* 7 */ SyscallDesc("wait4", unimplementedFunc),
    /* 8 */ SyscallDesc("old_creat", unimplementedFunc),
    /* 9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unimplementedFunc),
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
    /* 23 */ SyscallDesc("setuid", unimplementedFunc),
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
    /* 45 */ SyscallDesc("open", openFunc),
    /* 46 */ SyscallDesc("obsolete osigaction", unimplementedFunc),
    /* 47 */ SyscallDesc("getgid", unimplementedFunc),
    /* 48 */ SyscallDesc("sigprocmask", ignoreFunc),
    /* 49 */ SyscallDesc("getlogin", unimplementedFunc),
    /* 50 */ SyscallDesc("setlogin", unimplementedFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 53 */ SyscallDesc("classcntl", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", ioctlFunc),
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
    /* 71 */ SyscallDesc("mmap", mmapFunc),
    /* 72 */ SyscallDesc("ovadvise", unimplementedFunc),
    /* 73 */ SyscallDesc("munmap", unimplementedFunc),
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
    /* 92 */ SyscallDesc("fcntl", unimplementedFunc),
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
    /* 103 */ SyscallDesc("sigreturn", sigreturnFunc),
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
    /* 116 */ SyscallDesc("gettimeofday", gettimeofdayFunc),
    /* 117 */ SyscallDesc("getrusage", getrusageFunc),
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
    /* 138 */ SyscallDesc("utimes", unimplementedFunc),
    /* 139 */ SyscallDesc("obsolete 4.2 sigreturn", unimplementedFunc),
    /* 140 */ SyscallDesc("adjtime", unimplementedFunc),
    /* 141 */ SyscallDesc("old_getpeername", unimplementedFunc),
    /* 142 */ SyscallDesc("gethostid", unimplementedFunc),
    /* 143 */ SyscallDesc("sethostid", unimplementedFunc),
    /* 144 */ SyscallDesc("getrlimit", getrlimitFunc),
    /* 145 */ SyscallDesc("setrlimit", unimplementedFunc),
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
    /* 159 */ SyscallDesc("getdirentries", unimplementedFunc),
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
    /* 224 */ SyscallDesc("stat", statFunc),
    /* 225 */ SyscallDesc("lstat", lstatFunc),
    /* 226 */ SyscallDesc("fstat", fstatFunc),
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

const int Num_Syscall_Descs = sizeof(syscallDescs) / sizeof(SyscallDesc);

const int Max_Syscall_Desc = Num_Syscall_Descs - 1;

//
// Mach syscalls -- identified by negated syscall numbers
//

// Create a stack region for a thread.
int
stack_createFunc(SyscallDesc *desc, int callnum, Process *process,
                 ExecContext *xc)
{
    TypedBufferArg<OSF::vm_stack> argp(getArg(xc, 0));

    argp.copyIn(xc->mem);

    // if the user chose an address, just let them have it.  Otherwise
    // pick one for them.
    if (argp->address == 0) {
        argp->address = process->next_thread_stack_base;
        int stack_size = (argp->rsize + argp->ysize + argp->gsize);
        process->next_thread_stack_base -= stack_size;
        argp.copyOut(xc->mem);
    }

    return 0;
}

const int NXM_LIB_VERSION = 301003;

//
// This call sets up the interface between the user and kernel
// schedulers by creating a shared-memory region.  The shared memory
// region has several structs, some global, some per-RAD, some per-VP.
//
int
nxm_task_initFunc(SyscallDesc *desc, int callnum, Process *process,
                 ExecContext *xc)
{
    TypedBufferArg<OSF::nxm_task_attr> attrp(getArg(xc, 0));
    TypedBufferArg<Addr> configptr_ptr(getArg(xc, 1));

    attrp.copyIn(xc->mem);

    if (attrp->nxm_version != NXM_LIB_VERSION) {
        cerr << "nxm_task_init: thread library version mismatch! "
             << "got " << attrp->nxm_version
             << ", expected " << NXM_LIB_VERSION << endl;
        abort();
    }

    if (attrp->flags != OSF::NXM_TASK_INIT_VP) {
        cerr << "nxm_task_init: bad flag value " << attrp->flags
             << " (expected " << OSF::NXM_TASK_INIT_VP << ")" << endl;
        abort();
    }

    const Addr base_addr = 0x12000; // was 0x3f0000000LL;
    Addr cur_addr = base_addr; // next addresses to use
    // first comes the config_info struct
    Addr config_addr = cur_addr;
    cur_addr += sizeof(OSF::nxm_config_info);
    // next comes the per-cpu state vector
    Addr slot_state_addr = cur_addr;
    int slot_state_size = process->numCpus * sizeof(OSF::nxm_slot_state_t);
    cur_addr += slot_state_size;
    // now the per-RAD state struct (we only support one RAD)
    cur_addr = 0x14000;	// bump up addr for alignment
    Addr rad_state_addr = cur_addr;
    int rad_state_size =
        (sizeof(OSF::nxm_shared)
         + (process->numCpus-1) * sizeof(OSF::nxm_sched_state));
    cur_addr += rad_state_size;

    // now initialize a config_info struct and copy it out to user space
    TypedBufferArg<OSF::nxm_config_info> config(config_addr);

    config->nxm_nslots_per_rad = process->numCpus;
    config->nxm_nrads = 1;	// only one RAD in our system!
    config->nxm_slot_state = slot_state_addr;
    config->nxm_rad[0] = rad_state_addr;

    config.copyOut(xc->mem);

    // initialize the slot_state array and copy it out
    TypedBufferArg<OSF::nxm_slot_state_t> slot_state(slot_state_addr,
                                                     slot_state_size);
    for (int i = 0; i < process->numCpus; ++i) {
        // CPU 0 is bound to the calling process; all others are available
        slot_state[i] = (i == 0) ? OSF::NXM_SLOT_BOUND : OSF::NXM_SLOT_AVAIL;
    }

    slot_state.copyOut(xc->mem);

    // same for the per-RAD "shared" struct.  Note that we need to
    // allocate extra bytes for the per-VP array which is embedded at
    // the end.
    TypedBufferArg<OSF::nxm_shared> rad_state(rad_state_addr,
                                              rad_state_size);

    rad_state->nxm_callback = attrp->nxm_callback;
    rad_state->nxm_version = attrp->nxm_version;
    rad_state->nxm_uniq_offset = attrp->nxm_uniq_offset;
    for (int i = 0; i < process->numCpus; ++i) {
        OSF::nxm_sched_state *ssp = &rad_state->nxm_ss[i];
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

    return 0;
}


static void
init_exec_context(ExecContext *ec,
                  OSF::nxm_thread_attr *attrp, uint64_t uniq_val)
{
    memset(&ec->regs, 0, sizeof(ec->regs));

    ec->regs.intRegFile[ArgumentReg0] = attrp->registers.a0;
    ec->regs.intRegFile[27/*t12*/] = attrp->registers.pc;
    ec->regs.intRegFile[StackPointerReg] = attrp->registers.sp;
    ec->regs.miscRegs.uniq = uniq_val;

    ec->regs.pc = attrp->registers.pc;
    ec->regs.npc = attrp->registers.pc + sizeof(MachInst);

    ec->setStatus(ExecContext::Active);
}

int
nxm_thread_createFunc(SyscallDesc *desc, int callnum, Process *process,
                      ExecContext *xc)
{
    TypedBufferArg<OSF::nxm_thread_attr> attrp(getArg(xc, 0));
    TypedBufferArg<uint64_t> kidp(getArg(xc, 1));
    int thread_index = getArg(xc, 2);

    // get attribute args
    attrp.copyIn(xc->mem);

    if (attrp->version != NXM_LIB_VERSION) {
        cerr << "nxm_thread_create: thread library version mismatch! "
             << "got " << attrp->version
             << ", expected " << NXM_LIB_VERSION << endl;
        abort();
    }

    if (thread_index < 0 | thread_index > process->numCpus) {
        cerr << "nxm_thread_create: bad thread index " << thread_index
             << endl;
        abort();
    }

    // On a real machine, the per-RAD shared structure is in
    // shared memory, so both the user and kernel can get at it.
    // We don't have that luxury, so we just copy it in and then
    // back out again.
    int rad_state_size =
        (sizeof(OSF::nxm_shared) +
         (process->numCpus-1) * sizeof(OSF::nxm_sched_state));

    TypedBufferArg<OSF::nxm_shared> rad_state(0x14000,
                                              rad_state_size);
    rad_state.copyIn(xc->mem);

    uint64_t uniq_val = attrp->pthid - rad_state->nxm_uniq_offset;

    if (attrp->type == OSF::NXM_TYPE_MANAGER) {
        // DEC pthreads seems to always create one of these (in
        // addition to N application threads), but we don't use it,
        // so don't bother creating it.

        // This is supposed to be a port number.  Make something up.
        *kidp = 99;
        kidp.copyOut(xc->mem);

        return 0;
    } else if (attrp->type == OSF::NXM_TYPE_VP) {
        // A real "virtual processor" kernel thread.  Need to fork
        // this thread on another CPU.
        OSF::nxm_sched_state *ssp = &rad_state->nxm_ss[thread_index];

        if (ssp->nxm_u.nxm_active != 0)
            return OSF::KERN_NOT_RECEIVER;

        ssp->nxm_u.pth_id = attrp->pthid;
        ssp->nxm_u.nxm_active = uniq_val | 1;

        rad_state.copyOut(xc->mem);

        Addr slot_state_addr = 0x12000 + sizeof(OSF::nxm_config_info);
        int slot_state_size = process->numCpus * sizeof(OSF::nxm_slot_state_t);

        TypedBufferArg<OSF::nxm_slot_state_t> slot_state(slot_state_addr,
                                                         slot_state_size);

        slot_state.copyIn(xc->mem);

        if (slot_state[thread_index] != OSF::NXM_SLOT_AVAIL) {
            cerr << "nxm_thread_createFunc: requested VP slot "
                 << thread_index << " not available!" << endl;
            fatal("");
        }

        slot_state[thread_index] = OSF::NXM_SLOT_BOUND;

        slot_state.copyOut(xc->mem);

        // Find a free simulator execution context.
        list<ExecContext *> &ecList = process->execContexts;
        list<ExecContext *>::iterator i = ecList.begin();
        list<ExecContext *>::iterator end = ecList.end();
        for (; i != end; ++i) {
            ExecContext *xc = *i;

            if (xc->status() == ExecContext::Unallocated) {
                // inactive context... grab it
                init_exec_context(xc, attrp, uniq_val);

                // This is supposed to be a port number, but we'll try
                // and get away with just sticking the thread index
                // here.
                *kidp = thread_index;
                kidp.copyOut(xc->mem);

                return 0;
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

    return 0;
}


int
nxm_idleFunc(SyscallDesc *desc, int callnum, Process *process,
             ExecContext *xc)
{
    return 0;
}

int
nxm_thread_blockFunc(SyscallDesc *desc, int callnum, Process *process,
                     ExecContext *xc)
{
    uint64_t tid = getArg(xc, 0);
    uint64_t secs = getArg(xc, 1);
    uint64_t flags = getArg(xc, 2);
    uint64_t action = getArg(xc, 3);
    uint64_t usecs = getArg(xc, 4);

    cout << xc->cpu->name() << ": nxm_thread_block " << tid << " " << secs
         << " " << flags << " " << action << " " << usecs << endl;

    return 0;
}


int
nxm_blockFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
{
    Addr uaddr = getArg(xc, 0);
    uint64_t val = getArg(xc, 1);
    uint64_t secs = getArg(xc, 2);
    uint64_t usecs = getArg(xc, 3);
    uint64_t flags = getArg(xc, 4);

    BaseCPU *cpu = xc->cpu;

    cout << cpu->name() << ": nxm_block " << hex << uaddr << dec << " " << val
         << " " << secs << " " << usecs
         << " " << flags << endl;

    return 0;
}


int
nxm_unblockFunc(SyscallDesc *desc, int callnum, Process *process,
              ExecContext *xc)
{
    Addr uaddr = getArg(xc, 0);

    cout << xc->cpu->name() << ": nxm_unblock "
         << hex << uaddr << dec << endl;

    return 0;
}


int
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
    return false;
}


// just activate one by default
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
            newCtx->setStatus(ExecContext::Active);

            // get rid of this record
            i = process->waitList.erase(i);

            ++num_activated;
        } else {
            ++i;
        }
    }

    return num_activated;
}


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
        xc->setStatus(ExecContext::Suspended);
    }
}

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


int
m5_mutex_lockFunc(SyscallDesc *desc, int callnum, Process *process,
                  ExecContext *xc)
{
    Addr uaddr = getArg(xc, 0);

    m5_lock_mutex(uaddr, process, xc);

    // Return 0 since we will always return to the user with the lock
    // acquired.  We will just keep the context inactive until that is
    // true.
    return 0;
}


int
m5_mutex_trylockFunc(SyscallDesc *desc, int callnum, Process *process,
                     ExecContext *xc)
{
    Addr uaddr = getArg(xc, 0);
    TypedBufferArg<uint64_t> lockp(uaddr);

    lockp.copyIn(xc->mem);

    if (*lockp == 0) {
        // lock is free: grab it
        *lockp = 1;
        lockp.copyOut(xc->mem);
        return 0;
    } else {
        return 1;
    }
}


int
m5_mutex_unlockFunc(SyscallDesc *desc, int callnum, Process *process,
                    ExecContext *xc)
{
    Addr uaddr = getArg(xc, 0);

    m5_unlock_mutex(uaddr, process, xc);

    return 0;
}


int
m5_cond_signalFunc(SyscallDesc *desc, int callnum, Process *process,
                   ExecContext *xc)
{
    Addr cond_addr = getArg(xc, 0);

    // Wqake up one process waiting on the condition variable.
    activate_waiting_context(cond_addr, process);

    return 0;
}


int
m5_cond_broadcastFunc(SyscallDesc *desc, int callnum, Process *process,
                      ExecContext *xc)
{
    Addr cond_addr = getArg(xc, 0);

    // Wake up all processes waiting on the condition variable.
    activate_waiting_context(cond_addr, process, true);

    return 0;
}


int
m5_cond_waitFunc(SyscallDesc *desc, int callnum, Process *process,
                 ExecContext *xc)
{
    Addr cond_addr = getArg(xc, 0);
    Addr lock_addr = getArg(xc, 1);
    TypedBufferArg<uint64_t> condp(cond_addr);
    TypedBufferArg<uint64_t> lockp(lock_addr);

    // user is supposed to acquire lock before entering
    lockp.copyIn(xc->mem);
    assert(*lockp != 0);

    m5_unlock_mutex(lock_addr, process, xc);

    process->waitList.push_back(Process::WaitRec(cond_addr, xc));
    xc->setStatus(ExecContext::Suspended);

    return 0;
}


int
m5_thread_exitFunc(SyscallDesc *desc, int callnum, Process *process,
                   ExecContext *xc)
{
    assert(xc->status() == ExecContext::Active);
    xc->setStatus(ExecContext::Unallocated);

    return 0;
}


SyscallDesc machSyscallDescs[] = {
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

const int Num_Mach_Syscall_Descs =
                sizeof(machSyscallDescs) / sizeof(SyscallDesc);

const int Max_Mach_Syscall_Desc = Num_Mach_Syscall_Descs - 1;

// Since negated values are used to identify Mach syscalls, the
// minimum (signed) valid syscall number is the negated max Mach
// syscall number.
const int Min_Syscall_Desc = -Max_Mach_Syscall_Desc;


//
// helper function for invoking syscalls
//
static
int
doSyscall(int callnum, Process *process,
          ExecContext *xc)
{
    if (callnum < Min_Syscall_Desc || callnum > Max_Syscall_Desc) {
        cerr << "Syscall " << callnum << " out of range" << endl;
        abort();
    }

    SyscallDesc *desc =
        (callnum < 0) ? &machSyscallDescs[-callnum] : &syscallDescs[callnum];

    DCOUT(SyscallVerbose) << xc->cpu->name() << ": syscall " << desc->name
                          << " called @ " << curTick << endl;

    return desc->doFunc(callnum, process, xc);
}

//
// Indirect syscall invocation (call #0)
//
int
indirectSyscallFunc(SyscallDesc *desc, int callnum, Process *process,
                    ExecContext *xc)
{
    int new_callnum = getArg(xc, 0);

    for (int i = 0; i < 5; ++i)
        setArg(xc, i, getArg(xc, i+1));

    return doSyscall(new_callnum, process, xc);
}


void
fake_syscall(Process *process, ExecContext *xc)
{
    int64_t callnum = xc->regs.intRegFile[ReturnValueReg];

    int retval = doSyscall(callnum, process, xc);

    set_return_value(xc, retval);
}
