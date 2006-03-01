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

#include "arch/sparc/common_syscall_emul.hh"
#include "arch/sparc/linux_process.hh"
#include "arch/sparc/isa_traits.hh"

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "kern/linux/linux.hh"
#include "mem/functional/functional.hh"

#include "sim/process.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace SPARCISA;

/// Target pipe() handler.  Even though this is a generic Posix call,
/// the Alpha return convention is funky, so that makes it
/// Alpha-specific.
SyscallReturn
pipeFunc(SyscallDesc *desc, int callnum, Process *process,
         ExecContext *xc)
{
    int fds[2], sim_fds[2];
    int pipe_retval = pipe(fds);

    if (pipe_retval < 0) {
        // error
        return pipe_retval;
    }

    sim_fds[0] = process->alloc_fd(fds[0]);
    sim_fds[1] = process->alloc_fd(fds[1]);

    // Alpha Linux convention for pipe() is that fd[0] is returned as
    // the return value of the function, and fd[1] is returned in r20.
    xc->regs.intRegFile[20] = sim_fds[1];
    return sim_fds[0];
}


/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, Process *process,
          ExecContext *xc)
{
    TypedBufferArg<Linux::utsname> name(xc->getSyscallArg(0));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "m5.eecs.umich.edu");
    strcpy(name->release, "2.4.20");
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "alpha");

    name.copyOut(xc->mem);
    return 0;
}

/// Target osf_getsysyinfo() handler.  Even though this call is
/// borrowed from Tru64, the subcases that get used appear to be
/// different in practice from those used by Tru64 processes.
static SyscallReturn
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
          return 0;
      }

      default:
        cerr << "osf_getsysinfo: unknown op " << op << endl;
        abort();
        break;
    }

    return 1;
}

/// Target osf_setsysinfo() handler.
static SyscallReturn
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
                   " setting FPCR to 0x%x\n", gtoh(*(uint64_t*)fpcr));
          return 0;
      }

      default:
        cerr << "osf_setsysinfo: unknown op " << op << endl;
        abort();
        break;
    }

    return 1;
}


SyscallDesc SparcLinuxProcess::syscallDescs[] = {
    /*  0 */ SyscallDesc("restart_syscall", unimplimentedFunc);
    /*  1 */ SyscallDesc("exit", unimplimentedFunc);
    /*  2 */ SyscallDesc("fork", unimplimentedFunc);
    /*  3 */ SyscallDesc("read", unimplimentedFunc);
    /*  4 */ SyscallDesc("write", unimplimentedFunc);
    /*  5 */ SyscallDesc("open", unimplimentedFunc);
    /*  6 */ SyscallDesc("close", unimplimentedFunc);
    /*  7 */ SyscallDesc("wait4", unimplimentedFunc);
    /*  8 */ SyscallDesc("creat", unimplimentedFunc);
    /*  9 */ SyscallDesc("link", unimplimentedFunc);
    /* 10 */ SyscallDesc("unlink", unimplimentedFunc);
    /* 11 */ SyscallDesc("execv", unimplimentedFunc);
    /* 12 */ SyscallDesc("chdir", unimplimentedFunc);
    /* 13 */ SyscallDesc("chown", unimplimentedFunc);
    /* 14 */ SyscallDesc("mknod", unimplimentedFunc);
    /* 15 */ SyscallDesc("chmod", unimplimentedFunc);
    /* 16 */ SyscallDesc("lchown", unimplimentedFunc);
    /* 17 */ SyscallDesc("brk", unimplimentedFunc);
    /* 18 */ SyscallDesc("perfctr", unimplimentedFunc);
    /* 19 */ SyscallDesc("lseek", unimplimentedFunc);
    /* 20 */ SyscallDesc("getpid", unimplimentedFunc);
    /* 21 */ SyscallDesc("capget", unimplimentedFunc);
    /* 22 */ SyscallDesc("capset", unimplimentedFunc);
    /* 23 */ SyscallDesc("setuid", unimplimentedFunc);
    /* 24 */ SyscallDesc("getuid", unimplimentedFunc);
    /* 25 */ SyscallDesc("time", unimplimentedFunc);
    /* 26 */ SyscallDesc("ptrace", unimplimentedFunc);
    /* 27 */ SyscallDesc("alarm", unimplimentedFunc);
    /* 28 */ SyscallDesc("sigaltstack", unimplimentedFunc);
    /* 29 */ SyscallDesc("pause", unimplimentedFunc);
    /* 30 */ SyscallDesc("utime", unimplimentedFunc);
    /* 31 */ SyscallDesc("lchown32", unimplimentedFunc);
    /* 32 */ SyscallDesc("fchown32", unimplimentedFunc);
    /* 33 */ SyscallDesc("access", unimplimentedFunc);
    /* 34 */ SyscallDesc("nice", unimplimentedFunc);
    /* 35 */ SyscallDesc("chown32", unimplimentedFunc);
    /* 36 */ SyscallDesc("sync", unimplimentedFunc);
    /* 37 */ SyscallDesc("kill", unimplimentedFunc);
    /* 38 */ SyscallDesc("stat", unimplimentedFunc);
    /* 39 */ SyscallDesc("sendfile", unimplimentedFunc);
    /* 40 */ SyscallDesc("lstat", unimplimentedFunc);
    /* 41 */ SyscallDesc("dup", unimplimentedFunc);
    /* 42 */ SyscallDesc("pipe", unimplimentedFunc);
    /* 43 */ SyscallDesc("times", unimplimentedFunc);
    /* 44 */ SyscallDesc("getuid32", unimplimentedFunc);
    /* 45 */ SyscallDesc("umount2", unimplimentedFunc);
    /* 46 */ SyscallDesc("setgid", unimplimentedFunc);
    /* 47 */ SyscallDesc("getgid", unimplimentedFunc);
    /* 48 */ SyscallDesc("signal", unimplimentedFunc);
    /* 49 */ SyscallDesc("geteuid", unimplimentedFunc);
    /* 50 */ SyscallDesc("getegid", unimplimentedFunc);
    /* 51 */ SyscallDesc("acct", unimplimentedFunc);
    /* 52 */ SyscallDesc("memory_ordering", unimplimentedFunc);
    /* 53 */ SyscallDesc("getgid32", unimplimentedFunc);
    /* 54 */ SyscallDesc("ioctl", unimplimentedFunc);
    /* 55 */ SyscallDesc("reboot", unimplimentedFunc);
    /* 56 */ SyscallDesc("mmap2", unimplimentedFunc);
    /* 57 */ SyscallDesc("symlink", unimplimentedFunc);
    /* 58 */ SyscallDesc("readlink", unimplimentedFunc);
    /* 59 */ SyscallDesc("execve", unimplimentedFunc);
    /* 60 */ SyscallDesc("umask", unimplimentedFunc);
    /* 61 */ SyscallDesc("chroot", unimplimentedFunc);
    /* 62 */ SyscallDesc("fstat", unimplimentedFunc);
    /* 63 */ SyscallDesc("fstat64", unimplimentedFunc);
    /* 64 */ SyscallDesc("getpagesize", unimplimentedFunc);
    /* 65 */ SyscallDesc("msync", unimplimentedFunc);
    /* 66 */ SyscallDesc("vfork", unimplimentedFunc);
    /* 67 */ SyscallDesc("pread64", unimplimentedFunc);
    /* 68 */ SyscallDesc("pwrite64", unimplimentedFunc);
    /* 69 */ SyscallDesc("geteuid32", unimplimentedFunc);
    /* 70 */ SyscallDesc("getdgid32", unimplimentedFunc);
    /* 71 */ SyscallDesc("mmap", unimplimentedFunc);
    /* 72 */ SyscallDesc("setreuid32", unimplimentedFunc);
    /* 73 */ SyscallDesc("munmap", unimplimentedFunc);
    /* 74 */ SyscallDesc("mprotect", unimplimentedFunc);
    /* 75 */ SyscallDesc("madvise", unimplimentedFunc);
    /* 76 */ SyscallDesc("vhangup", unimplimentedFunc);
    /* 77 */ SyscallDesc("truncate64", unimplimentedFunc);
    /* 78 */ SyscallDesc("mincore", unimplimentedFunc);
    /* 79 */ SyscallDesc("getgroups", unimplimentedFunc);
    /* 80 */ SyscallDesc("setgroups", unimplimentedFunc);
    /* 81 */ SyscallDesc("getpgrp", unimplimentedFunc);
    /* 82 */ SyscallDesc("setgroups32", unimplimentedFunc);
    /* 83 */ SyscallDesc("setitimer", unimplimentedFunc);
    /* 84 */ SyscallDesc("ftruncate64", unimplimentedFunc);
    /* 85 */ SyscallDesc("swapon", unimplimentedFunc);
    /* 86 */ SyscallDesc("getitimer", unimplimentedFunc);
    /* 87 */ SyscallDesc("setuid32", unimplimentedFunc);
    /* 88 */ SyscallDesc("sethostname", unimplimentedFunc);
    /* 89 */ SyscallDesc("setgid32", unimplimentedFunc);
    /* 90 */ SyscallDesc("dup2", unimplimentedFunc);
    /* 91 */ SyscallDesc("setfsuid32", unimplimentedFunc);
    /* 92 */ SyscallDesc("fcntl", unimplimentedFunc);
    /* 93 */ SyscallDesc("select", unimplimentedFunc);
    /* 94 */ SyscallDesc("setfsgid32", unimplimentedFunc);
    /* 95 */ SyscallDesc("fsync", unimplimentedFunc);
    /* 96 */ SyscallDesc("setpriority", unimplimentedFunc);
    /* 97 */ SyscallDesc("socket", unimplimentedFunc);
    /* 98 */ SyscallDesc("connect", unimplimentedFunc);
    /* 99 */ SyscallDesc("accept", unimplimentedFunc);
    /* 100 */ SyscallDesc("getpriority", unimplimentedFunc);
    /* 101 */ SyscallDesc("rt_sigreturn", unimplimentedFunc);
    /* 102 */ SyscallDesc("rt_sigaction", unimplimentedFunc);
    /* 103 */ SyscallDesc("rt_sigprocmask", unimplimentedFunc);
    /* 104 */ SyscallDesc("rt_sigpending", unimplimentedFunc);
    /* 105 */ SyscallDesc("rt_sigtimedwait", unimplimentedFunc);
    /* 106 */ SyscallDesc("rt_sigqueueinfo", unimplimentedFunc);
    /* 107 */ SyscallDesc("rt_sigsuspend", unimplimentedFunc);
    /* 108 */ SyscallDesc("setresuid", unimplimentedFunc);
    /* 109 */ SyscallDesc("getresuid", unimplimentedFunc);
    /* 110 */ SyscallDesc("setresgid", unimplimentedFunc);
    /* 111 */ SyscallDesc("getresgid", unimplimentedFunc);
    /* 112 */ SyscallDesc("setregid32", unimplimentedFunc);
    /* 113 */ SyscallDesc("recvmsg", unimplimentedFunc);
    /* 114 */ SyscallDesc("sendmsg", unimplimentedFunc);
    /* 115 */ SyscallDesc("getgroups32", unimplimentedFunc);
    /* 116 */ SyscallDesc("gettimeofday", unimplimentedFunc);
    /* 117 */ SyscallDesc("getrusage", unimplimentedFunc);
    /* 118 */ SyscallDesc("getsockopt", unimplimentedFunc);
    /* 119 */ SyscallDesc("getcwd", unimplimentedFunc);
    /* 120 */ SyscallDesc("readv", unimplimentedFunc);
    /* 121 */ SyscallDesc("writev", unimplimentedFunc);
    /* 122 */ SyscallDesc("settimeofday", unimplimentedFunc);
    /* 123 */ SyscallDesc("fchown", unimplimentedFunc);
    /* 124 */ SyscallDesc("fchmod", unimplimentedFunc);
    /* 125 */ SyscallDesc("recvfrom", unimplimentedFunc);
    /* 126 */ SyscallDesc("setreuid", unimplimentedFunc);
    /* 127 */ SyscallDesc("setregid", unimplimentedFunc);
    /* 128 */ SyscallDesc("rename", unimplimentedFunc);
    /* 129 */ SyscallDesc("truncate", unimplimentedFunc);
    /* 130 */ SyscallDesc("ftruncate", unimplimentedFunc);
    /* 131 */ SyscallDesc("flock", unimplimentedFunc);
    /* 132 */ SyscallDesc("lstat64", unimplimentedFunc);
    /* 133 */ SyscallDesc("sendto", unimplimentedFunc);
    /* 134 */ SyscallDesc("shutdown", unimplimentedFunc);
    /* 135 */ SyscallDesc("socketpair", unimplimentedFunc);
    /* 136 */ SyscallDesc("mkdir", unimplimentedFunc);
    /* 137 */ SyscallDesc("rmdir", unimplimentedFunc);
    /* 138 */ SyscallDesc("utimes", unimplimentedFunc);
    /* 139 */ SyscallDesc("stat64", unimplimentedFunc);
    /* 140 */ SyscallDesc("sendfile64", unimplimentedFunc);
    /* 141 */ SyscallDesc("getpeername", unimplimentedFunc);
    /* 142 */ SyscallDesc("futex", unimplimentedFunc);
    /* 143 */ SyscallDesc("gettid", unimplimentedFunc);
    /* 144 */ SyscallDesc("getrlimit", unimplimentedFunc);
    /* 145 */ SyscallDesc("setrlimit", unimplimentedFunc);
    /* 146 */ SyscallDesc("pivot_root", unimplimentedFunc);
    /* 147 */ SyscallDesc("prctl", unimplimentedFunc);
    /* 148 */ SyscallDesc("pciconfig_read", unimplimentedFunc);
    /* 149 */ SyscallDesc("pciconfig_write", unimplimentedFunc);
    /* 150 */ SyscallDesc("getsockname", unimplimentedFunc);
    /* 151 */ SyscallDesc("inotify_init", unimplimentedFunc);
    /* 152 */ SyscallDesc("inotify_add_watch", unimplimentedFunc);
    /* 153 */ SyscallDesc("poll", unimplimentedFunc);
    /* 154 */ SyscallDesc("getdents64", unimplimentedFunc);
    /* 155 */ SyscallDesc("fcntl64", unimplimentedFunc);
    /* 156 */ SyscallDesc("inotify_rm_watch", unimplimentedFunc);
    /* 157 */ SyscallDesc("statfs", unimplimentedFunc);
    /* 158 */ SyscallDesc("fstatfs", unimplimentedFunc);
    /* 159 */ SyscallDesc("umount", unimplimentedFunc);
    /* 160 */ SyscallDesc("sched_set_affinity", unimplimentedFunc);
    /* 161 */ SyscallDesc("sched_get_affinity", unimplimentedFunc);
    /* 162 */ SyscallDesc("getdomainname", unimplimentedFunc);
    /* 163 */ SyscallDesc("setdomainname", unimplimentedFunc);
    /* 164 */ SyscallDesc("utrap_install", unimplimentedFunc);
    /* 165 */ SyscallDesc("quotactl", unimplimentedFunc);
    /* 166 */ SyscallDesc("set_tid_address", unimplimentedFunc);
    /* 167 */ SyscallDesc("mount", unimplimentedFunc);
    /* 168 */ SyscallDesc("ustat", unimplimentedFunc);
    /* 169 */ SyscallDesc("setxattr", unimplimentedFunc);
    /* 170 */ SyscallDesc("lsetxattr", unimplimentedFunc);
    /* 171 */ SyscallDesc("fsetxattr", unimplimentedFunc);
    /* 172 */ SyscallDesc("getxattr", unimplimentedFunc);
    /* 173 */ SyscallDesc("lgetxattr", unimplimentedFunc);
    /* 174 */ SyscallDesc("getdents", unimplimentedFunc);
    /* 175 */ SyscallDesc("setsid", unimplimentedFunc);
    /* 176 */ SyscallDesc("fchdir", unimplimentedFunc);
    /* 177 */ SyscallDesc("fgetxattr", unimplimentedFunc);
    /* 178 */ SyscallDesc("listxattr", unimplimentedFunc);
    /* 179 */ SyscallDesc("llistxattr", unimplimentedFunc);
    /* 180 */ SyscallDesc("flistxattr", unimplimentedFunc);
    /* 181 */ SyscallDesc("removexattr", unimplimentedFunc);
    /* 182 */ SyscallDesc("lremovexattr", unimplimentedFunc);
    /* 183 */ SyscallDesc("sigpending", unimplimentedFunc);
    /* 184 */ SyscallDesc("query_module", unimplimentedFunc);
    /* 185 */ SyscallDesc("setpgid", unimplimentedFunc);
    /* 186 */ SyscallDesc("fremovexattr", unimplimentedFunc);
    /* 187 */ SyscallDesc("tkill", unimplimentedFunc);
    /* 188 */ SyscallDesc("exit_group", unimplimentedFunc);
    /* 189 */ SyscallDesc("uname", unimplimentedFunc);
    /* 190 */ SyscallDesc("init_module", unimplimentedFunc);
    /* 191 */ SyscallDesc("personality", unimplimentedFunc);
    /* 192 */ SyscallDesc("remap_file_pages", unimplimentedFunc);
    /* 193 */ SyscallDesc("epoll_create", unimplimentedFunc);
    /* 194 */ SyscallDesc("epoll_ctl", unimplimentedFunc);
    /* 195 */ SyscallDesc("epoll_wait", unimplimentedFunc);
    /* 196 */ SyscallDesc("ioprio_set", unimplimentedFunc);
    /* 197 */ SyscallDesc("getppid", unimplimentedFunc);
    /* 198 */ SyscallDesc("sigaction", unimplimentedFunc);
    /* 199 */ SyscallDesc("sgetmask", unimplimentedFunc);
    /* 200 */ SyscallDesc("ssetmask", unimplimentedFunc);
    /* 201 */ SyscallDesc("sigsuspend", unimplimentedFunc);
    /* 202 */ SyscallDesc("oldlstat", unimplimentedFunc);
    /* 203 */ SyscallDesc("uselib", unimplimentedFunc);
    /* 204 */ SyscallDesc("readdir", unimplimentedFunc);
    /* 205 */ SyscallDesc("readahead", unimplimentedFunc);
    /* 206 */ SyscallDesc("socketcall", unimplimentedFunc);
    /* 207 */ SyscallDesc("syslog", unimplimentedFunc);
    /* 208 */ SyscallDesc("lookup_dcookie", unimplimentedFunc);
    /* 209 */ SyscallDesc("fadvise64", unimplimentedFunc);
    /* 210 */ SyscallDesc("fadvise64_64", unimplimentedFunc);
    /* 211 */ SyscallDesc("tgkill", unimplimentedFunc);
    /* 212 */ SyscallDesc("waitpid", unimplimentedFunc);
    /* 213 */ SyscallDesc("swapoff", unimplimentedFunc);
    /* 214 */ SyscallDesc("sysinfo", unimplimentedFunc);
    /* 215 */ SyscallDesc("ipc", unimplimentedFunc);
    /* 216 */ SyscallDesc("sigreturn", unimplimentedFunc);
    /* 217 */ SyscallDesc("clone", unimplimentedFunc);
    /* 218 */ SyscallDesc("ioprio_get", unimplimentedFunc);
    /* 219 */ SyscallDesc("adjtimex", unimplimentedFunc);
    /* 220 */ SyscallDesc("sigprocmask", unimplimentedFunc);
    /* 221 */ SyscallDesc("create_module", unimplimentedFunc);
    /* 222 */ SyscallDesc("delete_module", unimplimentedFunc);
    /* 223 */ SyscallDesc("get_kernel_syms", unimplimentedFunc);
    /* 224 */ SyscallDesc("getpgid", unimplimentedFunc);
    /* 225 */ SyscallDesc("bdflush", unimplimentedFunc);
    /* 226 */ SyscallDesc("sysfs", unimplimentedFunc);
    /* 227 */ SyscallDesc("afs_syscall", unimplimentedFunc);
    /* 228 */ SyscallDesc("setfsuid", unimplimentedFunc);
    /* 229 */ SyscallDesc("setfsgid", unimplimentedFunc);
    /* 230 */ SyscallDesc("_newselect", unimplimentedFunc);
    /* 231 */ SyscallDesc("time", unimplimentedFunc);
    /* 232 */ SyscallDesc("oldstat", unimplimentedFunc);
    /* 233 */ SyscallDesc("stime", unimplimentedFunc);
    /* 234 */ SyscallDesc("statfs64", unimplimentedFunc);
    /* 235 */ SyscallDesc("fstatfs64", unimplimentedFunc);
    /* 236 */ SyscallDesc("_llseek", unimplimentedFunc);
    /* 237 */ SyscallDesc("mlock", unimplimentedFunc);
    /* 238 */ SyscallDesc("munlock", unimplimentedFunc);
    /* 239 */ SyscallDesc("mlockall", unimplimentedFunc);
    /* 240 */ SyscallDesc("munlockall", unimplimentedFunc);
    /* 241 */ SyscallDesc("sched_setparam", unimplimentedFunc);
    /* 242 */ SyscallDesc("sched_getparam", unimplimentedFunc);
    /* 243 */ SyscallDesc("sched_setscheduler", unimplimentedFunc);
    /* 244 */ SyscallDesc("sched_getscheduler", unimplimentedFunc);
    /* 245 */ SyscallDesc("sched_yield", unimplimentedFunc);
    /* 246 */ SyscallDesc("sched_get_priority_max", unimplimented);
    /* 247 */ SyscallDesc("sched_get_priority_min", unimplimented);
    /* 248 */ SyscallDesc("sched_rr_get_interval", unimplimented);
    /* 249 */ SyscallDesc("nanosleep", unimplimentedFunc);
    /* 250 */ SyscallDesc("mremap", unimplimentedFunc);
    /* 251 */ SyscallDesc("_sysctl", unimplimentedFunc);
    /* 252 */ SyscallDesc("getsid", unimplimentedFunc);
    /* 253 */ SyscallDesc("fdatasync", unimplimentedFunc);
    /* 254 */ SyscallDesc("nfsservctl", unimplimentedFunc);
    /* 255 */ SyscallDesc("aplib", unimplimentedFunc);
    /* 256 */ SyscallDesc("clock_settime", unimplimentedFunc);
    /* 257 */ SyscallDesc("clock_gettime", unimplimentedFunc);
    /* 258 */ SyscallDesc("clock_getres", unimplimentedFunc);
    /* 259 */ SyscallDesc("clock_nanosleep", unimplimentedFunc);
    /* 260 */ SyscallDesc("sched_getaffinity", unimplimentedFunc);
    /* 261 */ SyscallDesc("sched_setaffinity", unimplimentedFunc);
    /* 262 */ SyscallDesc("timer_settime", unimplimentedFunc);
    /* 263 */ SyscallDesc("timer_gettime", unimplimentedFunc);
    /* 264 */ SyscallDesc("timer_getoverrun", unimplimentedFunc);
    /* 265 */ SyscallDesc("timer_delete", unimplimentedFunc);
    /* 266 */ SyscallDesc("timer_create", unimplimentedFunc);
    /* 267 */ SyscallDesc("vserver", unimplimentedFunc);
    /* 268 */ SyscallDesc("io_setup", unimplimentedFunc);
    /* 269 */ SyscallDesc("io_destroy", unimplimentedFunc);
    /* 270 */ SyscallDesc("io_submit", unimplimentedFunc);
    /* 271 */ SyscallDesc("io_cancel", unimplimentedFunc);
    /* 272 */ SyscallDesc("io_getevents", unimplimentedFunc);
    /* 273 */ SyscallDesc("mq_open", unimplimentedFunc);
    /* 274 */ SyscallDesc("mq_unlink", unimplimentedFunc);
    /* 275 */ SyscallDesc("mq_timedsend", unimplimentedFunc);
    /* 276 */ SyscallDesc("mq_timedreceive", unimplimentedFunc);
    /* 277 */ SyscallDesc("mq_notify", unimplimentedFunc);
    /* 278 */ SyscallDesc("mq_getsetattr", unimplimentedFunc);
    /* 279 */ SyscallDesc("waitid", unimplimentedFunc);
    /* 280 */ SyscallDesc("sys_setaltroot", unimplimentedFunc);
    /* 281 */ SyscallDesc("add_key", unimplimentedFunc);
    /* 282 */ SyscallDesc("request_key", unimplimentedFunc);
    /* 283 */ SyscallDesc("keyctl", unimplimentedFunc);
};

SparcLinuxProcess::SparcLinuxProcess(const std::string &name,
                                     ObjectFile *objFile,
                                     int stdin_fd,
                                     int stdout_fd,
                                     int stderr_fd,
                                     std::vector<std::string> &argv,
                                     std::vector<std::string> &envp)
    : LiveProcess(name, objFile, stdin_fd, stdout_fd, stderr_fd, argv, envp),
     Num_Syscall_Descs(sizeof(syscallDescs) / sizeof(SyscallDesc))
{
    init_regs->intRegFile[0] = 0;
}



SyscallDesc*
AlphaLinuxProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum > Num_Syscall_Descs)
        return NULL;
    return &syscallDescs[callnum];
}
