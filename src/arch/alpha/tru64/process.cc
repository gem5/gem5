/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Ali Saidi
 */

#include "arch/alpha/tru64/process.hh"
#include "arch/alpha/tru64/tru64.hh"
#include "arch/alpha/isa_traits.hh"
#include "cpu/thread_context.hh"
#include "kern/tru64/tru64.hh"
#include "sim/byteswap.hh"
#include "sim/process.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace AlphaISA;

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
          ThreadContext *tc)
{
    int index = 0;
    TypedBufferArg<AlphaTru64::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "OSF1");
    strcpy(name->nodename, "m5.eecs.umich.edu");
    strcpy(name->release, "V5.1");
    strcpy(name->version, "732");
    strcpy(name->machine, "alpha");

    name.copyOut(tc->getMemProxy());
    return 0;
}

/// Target getsysyinfo() handler.
static SyscallReturn
getsysinfoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
               ThreadContext *tc)
{
    int index = 0;
    unsigned op = process->getSyscallArg(tc, index);
    Addr bufPtr = process->getSyscallArg(tc, index);
    unsigned nbytes = process->getSyscallArg(tc, index);

    switch (op) {

      case AlphaTru64::GSI_MAX_CPU: {
          TypedBufferArg<uint32_t> max_cpu(bufPtr);
          *max_cpu = htog((uint32_t)process->numCpus());
          max_cpu.copyOut(tc->getMemProxy());
          return 1;
      }

      case AlphaTru64::GSI_CPUS_IN_BOX: {
          TypedBufferArg<uint32_t> cpus_in_box(bufPtr);
          *cpus_in_box = htog((uint32_t)process->numCpus());
          cpus_in_box.copyOut(tc->getMemProxy());
          return 1;
      }

      case AlphaTru64::GSI_PHYSMEM: {
          TypedBufferArg<uint64_t> physmem(bufPtr);
          *physmem = htog((uint64_t)1024 * 1024);  // physical memory in KB
          physmem.copyOut(tc->getMemProxy());
          return 1;
      }

      case AlphaTru64::GSI_CPU_INFO: {
          TypedBufferArg<AlphaTru64::cpu_info> infop(bufPtr);

          infop->current_cpu = htog(0);
          infop->cpus_in_box = htog(process->numCpus());
          infop->cpu_type = htog(57);
          infop->ncpus = htog(process->numCpus());
          uint64_t cpumask = (1 << process->numCpus()) - 1;
          infop->cpus_present = infop->cpus_running = htog(cpumask);
          infop->cpu_binding = htog(0);
          infop->cpu_ex_binding = htog(0);
          infop->mhz = htog(667);

          infop.copyOut(tc->getMemProxy());
          return 1;
      }

      case AlphaTru64::GSI_PROC_TYPE: {
          TypedBufferArg<uint64_t> proc_type(bufPtr);
          *proc_type = htog((uint64_t)11);
          proc_type.copyOut(tc->getMemProxy());
          return 1;
      }

      case AlphaTru64::GSI_PLATFORM_NAME: {
          BufferArg bufArg(bufPtr, nbytes);
          strncpy((char *)bufArg.bufferPtr(),
                  "COMPAQ Professional Workstation XP1000",
                  nbytes);
          bufArg.copyOut(tc->getMemProxy());
          return 1;
      }

      case AlphaTru64::GSI_CLK_TCK: {
          TypedBufferArg<uint64_t> clk_hz(bufPtr);
          *clk_hz = htog((uint64_t)1024);
          clk_hz.copyOut(tc->getMemProxy());
          return 1;
      }

      default:
        warn("getsysinfo: unknown op %d\n", op);
        break;
    }

    return 0;
}

/// Target setsysyinfo() handler.
static SyscallReturn
setsysinfoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
               ThreadContext *tc)
{
    int index = 0;
    unsigned op = process->getSyscallArg(tc, index);

    switch (op) {
      case AlphaTru64::SSI_IEEE_FP_CONTROL:
        warn("setsysinfo: ignoring ieee_set_fp_control() arg 0x%x\n",
             process->getSyscallArg(tc, index));
        break;

      default:
        warn("setsysinfo: unknown op %d\n", op);
        break;
    }

    return 0;
}

/// Target table() handler.
static SyscallReturn
tableFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
          ThreadContext *tc)
{
    using namespace std;

    int argIndex = 0;
    int id = process->getSyscallArg(tc, argIndex);    // table ID
    int index = process->getSyscallArg(tc, argIndex); // index into table
    Addr bufPtr = process->getSyscallArg(tc, argIndex);
    // arg 2 is buffer pointer; type depends on table ID
    int nel = process->getSyscallArg(tc, argIndex);   // number of elements
    int lel = process->getSyscallArg(tc, argIndex);   // expected element size

    switch (id) {
      case AlphaTru64::TBL_SYSINFO: {
          if (index != 0 || nel != 1 || lel != sizeof(Tru64::tbl_sysinfo))
              return -EINVAL;
          TypedBufferArg<Tru64::tbl_sysinfo> elp(bufPtr);

          const int clk_hz = one_million;
          elp->si_user = htog(curTick() / (SimClock::Frequency / clk_hz));
          elp->si_nice = htog(0);
          elp->si_sys = htog(0);
          elp->si_idle = htog(0);
          elp->wait = htog(0);
          elp->si_hz = htog(clk_hz);
          elp->si_phz = htog(clk_hz);
          elp->si_boottime = htog(seconds_since_epoch); // seconds since epoch?
          elp->si_max_procs = htog(process->numCpus());
          elp.copyOut(tc->getMemProxy());
          return 0;
      }

      default:
        cerr << "table(): id " << id << " unknown." << endl;
        return -EINVAL;
    }
}

SyscallDesc AlphaTru64Process::syscallDescs[] = {
    /* 0 */ SyscallDesc("syscall (#0)", AlphaTru64::indirectSyscallFunc,
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
    /* 17 */ SyscallDesc("obreak", brkFunc),
    /* 18 */ SyscallDesc("pre_F64_getfsstat", unimplementedFunc),
    /* 19 */ SyscallDesc("lseek", lseekFunc),
    /* 20 */ SyscallDesc("getpid", getpidPseudoFunc),
    /* 21 */ SyscallDesc("mount", unimplementedFunc),
    /* 22 */ SyscallDesc("unmount", unimplementedFunc),
    /* 23 */ SyscallDesc("setuid", setuidFunc),
    /* 24 */ SyscallDesc("getuid", getuidPseudoFunc),
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
    /* 45 */ SyscallDesc("open", openFunc<AlphaTru64>),
    /* 46 */ SyscallDesc("obsolete osigaction", unimplementedFunc),
    /* 47 */ SyscallDesc("getgid", getgidPseudoFunc),
    /* 48 */ SyscallDesc("sigprocmask", ignoreFunc),
    /* 49 */ SyscallDesc("getlogin", unimplementedFunc),
    /* 50 */ SyscallDesc("setlogin", unimplementedFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 53 */ SyscallDesc("classcntl", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", ioctlFunc<AlphaTru64>),
    /* 55 */ SyscallDesc("reboot", unimplementedFunc),
    /* 56 */ SyscallDesc("revoke", unimplementedFunc),
    /* 57 */ SyscallDesc("symlink", unimplementedFunc),
    /* 58 */ SyscallDesc("readlink", readlinkFunc),
    /* 59 */ SyscallDesc("execve", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", umaskFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("old_fstat", unimplementedFunc),
    /* 63 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 64 */ SyscallDesc("getpagesize", getpagesizeFunc),
    /* 65 */ SyscallDesc("mremap", unimplementedFunc),
    /* 66 */ SyscallDesc("vfork", unimplementedFunc),
    /* 67 */ SyscallDesc("pre_F64_stat", statFunc<Tru64_PreF64>),
    /* 68 */ SyscallDesc("pre_F64_lstat", lstatFunc<Tru64_PreF64>),
    /* 69 */ SyscallDesc("sbrk", unimplementedFunc),
    /* 70 */ SyscallDesc("sstk", unimplementedFunc),
    /* 71 */ SyscallDesc("mmap", mmapFunc<AlphaTru64>),
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
    /* 91 */ SyscallDesc("pre_F64_fstat", fstatFunc<Tru64_PreF64>),
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
    /* 103 */ SyscallDesc("sigreturn", AlphaTru64::sigreturnFunc,
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
    /* 116 */ SyscallDesc("gettimeofday", gettimeofdayFunc<AlphaTru64>),
    /* 117 */ SyscallDesc("getrusage", getrusageFunc<AlphaTru64>),
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
    /* 129 */ SyscallDesc("truncate", truncateFunc),
    /* 130 */ SyscallDesc("ftruncate", ftruncateFunc),
    /* 131 */ SyscallDesc("flock", unimplementedFunc),
    /* 132 */ SyscallDesc("setgid", unimplementedFunc),
    /* 133 */ SyscallDesc("sendto", unimplementedFunc),
    /* 134 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 135 */ SyscallDesc("socketpair", unimplementedFunc),
    /* 136 */ SyscallDesc("mkdir", mkdirFunc),
    /* 137 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 138 */ SyscallDesc("utimes", unimplementedFunc),
    /* 139 */ SyscallDesc("obsolete 4.2 sigreturn", unimplementedFunc),
    /* 140 */ SyscallDesc("adjtime", unimplementedFunc),
    /* 141 */ SyscallDesc("old_getpeername", unimplementedFunc),
    /* 142 */ SyscallDesc("gethostid", unimplementedFunc),
    /* 143 */ SyscallDesc("sethostid", unimplementedFunc),
    /* 144 */ SyscallDesc("getrlimit", getrlimitFunc<AlphaTru64>),
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
    /* 159 */ SyscallDesc("getdirentries", AlphaTru64::getdirentriesFunc),
    /* 160 */ SyscallDesc("pre_F64_statfs", statfsFunc<Tru64_PreF64>),
    /* 161 */ SyscallDesc("pre_F64_fstatfs", fstatfsFunc<Tru64_PreF64>),
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
    /* 224 */ SyscallDesc("stat", statFunc<Tru64_F64>),
    /* 225 */ SyscallDesc("lstat", lstatFunc<Tru64_F64>),
    /* 226 */ SyscallDesc("fstat", fstatFunc<Tru64_F64>),
    /* 227 */ SyscallDesc("statfs", statfsFunc<Tru64_F64>),
    /* 228 */ SyscallDesc("fstatfs", fstatfsFunc<Tru64_F64>),
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
    /* 257 */ SyscallDesc("setsysinfo", setsysinfoFunc),
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

SyscallDesc AlphaTru64Process::machSyscallDescs[] = {
    /* 0 */  SyscallDesc("kern_invalid", unimplementedFunc),
    /* 1 */  SyscallDesc("m5_mutex_lock", AlphaTru64::m5_mutex_lockFunc),
    /* 2 */  SyscallDesc("m5_mutex_trylock", AlphaTru64::m5_mutex_trylockFunc),
    /* 3 */  SyscallDesc("m5_mutex_unlock", AlphaTru64::m5_mutex_unlockFunc),
    /* 4 */  SyscallDesc("m5_cond_signal", AlphaTru64::m5_cond_signalFunc),
    /* 5 */  SyscallDesc("m5_cond_broadcast",
                         AlphaTru64::m5_cond_broadcastFunc),
    /* 6 */  SyscallDesc("m5_cond_wait", AlphaTru64::m5_cond_waitFunc),
    /* 7 */  SyscallDesc("m5_thread_exit", AlphaTru64::m5_thread_exitFunc),
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
    /* 24 */ SyscallDesc("nxm_block", AlphaTru64::nxm_blockFunc),
    /* 25 */ SyscallDesc("nxm_unblock", AlphaTru64::nxm_unblockFunc),
    /* 26 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 27 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 28 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 29 */ SyscallDesc("nxm_thread_destroy", unimplementedFunc),
    /* 30 */ SyscallDesc("lw_wire", unimplementedFunc),
    /* 31 */ SyscallDesc("lw_unwire", unimplementedFunc),
    /* 32 */ SyscallDesc("nxm_thread_create",
                         AlphaTru64::nxm_thread_createFunc),
    /* 33 */ SyscallDesc("nxm_task_init", AlphaTru64::nxm_task_initFunc),
    /* 34 */ SyscallDesc("kern_invalid", unimplementedFunc),
    /* 35 */ SyscallDesc("nxm_idle", AlphaTru64::nxm_idleFunc),
    /* 36 */ SyscallDesc("nxm_wakeup_idle", unimplementedFunc),
    /* 37 */ SyscallDesc("nxm_set_pthid", unimplementedFunc),
    /* 38 */ SyscallDesc("nxm_thread_kill", unimplementedFunc),
    /* 39 */ SyscallDesc("nxm_thread_block", AlphaTru64::nxm_thread_blockFunc),
    /* 40 */ SyscallDesc("nxm_thread_wakeup", unimplementedFunc),
    /* 41 */ SyscallDesc("init_process", unimplementedFunc),
    /* 42 */ SyscallDesc("nxm_get_binding", unimplementedFunc),
    /* 43 */ SyscallDesc("map_fd", unimplementedFunc),
    /* 44 */ SyscallDesc("nxm_resched", unimplementedFunc),
    /* 45 */ SyscallDesc("nxm_set_cancel", unimplementedFunc),
    /* 46 */ SyscallDesc("nxm_set_binding", unimplementedFunc),
    /* 47 */ SyscallDesc("stack_create", AlphaTru64::stack_createFunc),
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
    /* 59 */ SyscallDesc("swtch_pri", AlphaTru64::swtch_priFunc),
    /* 60 */ SyscallDesc("swtch", unimplementedFunc),
    /* 61 */ SyscallDesc("thread_switch", unimplementedFunc),
    /* 62 */ SyscallDesc("semop_fast", unimplementedFunc),
    /* 63 */ SyscallDesc("nxm_pshared_init", unimplementedFunc),
    /* 64 */ SyscallDesc("nxm_pshared_block", unimplementedFunc),
    /* 65 */ SyscallDesc("nxm_pshared_unblock", unimplementedFunc),
    /* 66 */ SyscallDesc("nxm_pshared_destroy", unimplementedFunc),
    /* 67 */ SyscallDesc("nxm_swtch_pri", AlphaTru64::swtch_priFunc),
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

SyscallDesc*
AlphaTru64Process::getDesc(int callnum)
{
    if (callnum < -Num_Mach_Syscall_Descs || callnum > Num_Syscall_Descs)
        return NULL;

    if (callnum < 0)
        return &machSyscallDescs[-callnum];
    else
        return &syscallDescs[callnum];
}

AlphaTru64Process::AlphaTru64Process(LiveProcessParams *params,
    ObjectFile *objFile)
    : AlphaLiveProcess(params, objFile),
      Num_Syscall_Descs(sizeof(syscallDescs) / sizeof(SyscallDesc)),
      Num_Mach_Syscall_Descs(sizeof(machSyscallDescs) / sizeof(SyscallDesc))
{
}
