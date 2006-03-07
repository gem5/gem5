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
#include "arch/sparc/linux/process.hh"
#include "arch/sparc/isa_traits.hh"

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "kern/linux/linux.hh"
#include "mem/functional/functional.hh"

#include "sim/process.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace SparcISA;

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
    /*  0 */ SyscallDesc("restart_syscall", unimplementedFunc);
    /*  1 */ SyscallDesc("exit", unimplementedFunc);
    /*  2 */ SyscallDesc("fork", unimplementedFunc);
    /*  3 */ SyscallDesc("read", unimplementedFunc);
    /*  4 */ SyscallDesc("write", unimplementedFunc);
    /*  5 */ SyscallDesc("open", unimplementedFunc);
    /*  6 */ SyscallDesc("close", unimplementedFunc);
    /*  7 */ SyscallDesc("wait4", unimplementedFunc);
    /*  8 */ SyscallDesc("creat", unimplementedFunc);
    /*  9 */ SyscallDesc("link", unimplementedFunc);
    /* 10 */ SyscallDesc("unlink", unimplementedFunc);
    /* 11 */ SyscallDesc("execv", unimplementedFunc);
    /* 12 */ SyscallDesc("chdir", unimplementedFunc);
    /* 13 */ SyscallDesc("chown", unimplementedFunc);
    /* 14 */ SyscallDesc("mknod", unimplementedFunc);
    /* 15 */ SyscallDesc("chmod", unimplementedFunc);
    /* 16 */ SyscallDesc("lchown", unimplementedFunc);
    /* 17 */ SyscallDesc("brk", unimplementedFunc);
    /* 18 */ SyscallDesc("perfctr", unimplementedFunc);
    /* 19 */ SyscallDesc("lseek", unimplementedFunc);
    /* 20 */ SyscallDesc("getpid", unimplementedFunc);
    /* 21 */ SyscallDesc("capget", unimplementedFunc);
    /* 22 */ SyscallDesc("capset", unimplementedFunc);
    /* 23 */ SyscallDesc("setuid", unimplementedFunc);
    /* 24 */ SyscallDesc("getuid", unimplementedFunc);
    /* 25 */ SyscallDesc("time", unimplementedFunc);
    /* 26 */ SyscallDesc("ptrace", unimplementedFunc);
    /* 27 */ SyscallDesc("alarm", unimplementedFunc);
    /* 28 */ SyscallDesc("sigaltstack", unimplementedFunc);
    /* 29 */ SyscallDesc("pause", unimplementedFunc);
    /* 30 */ SyscallDesc("utime", unimplementedFunc);
    /* 31 */ SyscallDesc("lchown32", unimplementedFunc);
    /* 32 */ SyscallDesc("fchown32", unimplementedFunc);
    /* 33 */ SyscallDesc("access", unimplementedFunc);
    /* 34 */ SyscallDesc("nice", unimplementedFunc);
    /* 35 */ SyscallDesc("chown32", unimplementedFunc);
    /* 36 */ SyscallDesc("sync", unimplementedFunc);
    /* 37 */ SyscallDesc("kill", unimplementedFunc);
    /* 38 */ SyscallDesc("stat", unimplementedFunc);
    /* 39 */ SyscallDesc("sendfile", unimplementedFunc);
    /* 40 */ SyscallDesc("lstat", unimplementedFunc);
    /* 41 */ SyscallDesc("dup", unimplementedFunc);
    /* 42 */ SyscallDesc("pipe", unimplementedFunc);
    /* 43 */ SyscallDesc("times", unimplementedFunc);
    /* 44 */ SyscallDesc("getuid32", unimplementedFunc);
    /* 45 */ SyscallDesc("umount2", unimplementedFunc);
    /* 46 */ SyscallDesc("setgid", unimplementedFunc);
    /* 47 */ SyscallDesc("getgid", unimplementedFunc);
    /* 48 */ SyscallDesc("signal", unimplementedFunc);
    /* 49 */ SyscallDesc("geteuid", unimplementedFunc);
    /* 50 */ SyscallDesc("getegid", unimplementedFunc);
    /* 51 */ SyscallDesc("acct", unimplementedFunc);
    /* 52 */ SyscallDesc("memory_ordering", unimplementedFunc);
    /* 53 */ SyscallDesc("getgid32", unimplementedFunc);
    /* 54 */ SyscallDesc("ioctl", unimplementedFunc);
    /* 55 */ SyscallDesc("reboot", unimplementedFunc);
    /* 56 */ SyscallDesc("mmap2", unimplementedFunc);
    /* 57 */ SyscallDesc("symlink", unimplementedFunc);
    /* 58 */ SyscallDesc("readlink", unimplementedFunc);
    /* 59 */ SyscallDesc("execve", unimplementedFunc);
    /* 60 */ SyscallDesc("umask", unimplementedFunc);
    /* 61 */ SyscallDesc("chroot", unimplementedFunc);
    /* 62 */ SyscallDesc("fstat", unimplementedFunc);
    /* 63 */ SyscallDesc("fstat64", unimplementedFunc);
    /* 64 */ SyscallDesc("getpagesize", unimplementedFunc);
    /* 65 */ SyscallDesc("msync", unimplementedFunc);
    /* 66 */ SyscallDesc("vfork", unimplementedFunc);
    /* 67 */ SyscallDesc("pread64", unimplementedFunc);
    /* 68 */ SyscallDesc("pwrite64", unimplementedFunc);
    /* 69 */ SyscallDesc("geteuid32", unimplementedFunc);
    /* 70 */ SyscallDesc("getdgid32", unimplementedFunc);
    /* 71 */ SyscallDesc("mmap", unimplementedFunc);
    /* 72 */ SyscallDesc("setreuid32", unimplementedFunc);
    /* 73 */ SyscallDesc("munmap", unimplementedFunc);
    /* 74 */ SyscallDesc("mprotect", unimplementedFunc);
    /* 75 */ SyscallDesc("madvise", unimplementedFunc);
    /* 76 */ SyscallDesc("vhangup", unimplementedFunc);
    /* 77 */ SyscallDesc("truncate64", unimplementedFunc);
    /* 78 */ SyscallDesc("mincore", unimplementedFunc);
    /* 79 */ SyscallDesc("getgroups", unimplementedFunc);
    /* 80 */ SyscallDesc("setgroups", unimplementedFunc);
    /* 81 */ SyscallDesc("getpgrp", unimplementedFunc);
    /* 82 */ SyscallDesc("setgroups32", unimplementedFunc);
    /* 83 */ SyscallDesc("setitimer", unimplementedFunc);
    /* 84 */ SyscallDesc("ftruncate64", unimplementedFunc);
    /* 85 */ SyscallDesc("swapon", unimplementedFunc);
    /* 86 */ SyscallDesc("getitimer", unimplementedFunc);
    /* 87 */ SyscallDesc("setuid32", unimplementedFunc);
    /* 88 */ SyscallDesc("sethostname", unimplementedFunc);
    /* 89 */ SyscallDesc("setgid32", unimplementedFunc);
    /* 90 */ SyscallDesc("dup2", unimplementedFunc);
    /* 91 */ SyscallDesc("setfsuid32", unimplementedFunc);
    /* 92 */ SyscallDesc("fcntl", unimplementedFunc);
    /* 93 */ SyscallDesc("select", unimplementedFunc);
    /* 94 */ SyscallDesc("setfsgid32", unimplementedFunc);
    /* 95 */ SyscallDesc("fsync", unimplementedFunc);
    /* 96 */ SyscallDesc("setpriority", unimplementedFunc);
    /* 97 */ SyscallDesc("socket", unimplementedFunc);
    /* 98 */ SyscallDesc("connect", unimplementedFunc);
    /* 99 */ SyscallDesc("accept", unimplementedFunc);
    /* 100 */ SyscallDesc("getpriority", unimplementedFunc);
    /* 101 */ SyscallDesc("rt_sigreturn", unimplementedFunc);
    /* 102 */ SyscallDesc("rt_sigaction", unimplementedFunc);
    /* 103 */ SyscallDesc("rt_sigprocmask", unimplementedFunc);
    /* 104 */ SyscallDesc("rt_sigpending", unimplementedFunc);
    /* 105 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc);
    /* 106 */ SyscallDesc("rt_sigqueueinfo", unimplementedFunc);
    /* 107 */ SyscallDesc("rt_sigsuspend", unimplementedFunc);
    /* 108 */ SyscallDesc("setresuid", unimplementedFunc);
    /* 109 */ SyscallDesc("getresuid", unimplementedFunc);
    /* 110 */ SyscallDesc("setresgid", unimplementedFunc);
    /* 111 */ SyscallDesc("getresgid", unimplementedFunc);
    /* 112 */ SyscallDesc("setregid32", unimplementedFunc);
    /* 113 */ SyscallDesc("recvmsg", unimplementedFunc);
    /* 114 */ SyscallDesc("sendmsg", unimplementedFunc);
    /* 115 */ SyscallDesc("getgroups32", unimplementedFunc);
    /* 116 */ SyscallDesc("gettimeofday", unimplementedFunc);
    /* 117 */ SyscallDesc("getrusage", unimplementedFunc);
    /* 118 */ SyscallDesc("getsockopt", unimplementedFunc);
    /* 119 */ SyscallDesc("getcwd", unimplementedFunc);
    /* 120 */ SyscallDesc("readv", unimplementedFunc);
    /* 121 */ SyscallDesc("writev", unimplementedFunc);
    /* 122 */ SyscallDesc("settimeofday", unimplementedFunc);
    /* 123 */ SyscallDesc("fchown", unimplementedFunc);
    /* 124 */ SyscallDesc("fchmod", unimplementedFunc);
    /* 125 */ SyscallDesc("recvfrom", unimplementedFunc);
    /* 126 */ SyscallDesc("setreuid", unimplementedFunc);
    /* 127 */ SyscallDesc("setregid", unimplementedFunc);
    /* 128 */ SyscallDesc("rename", unimplementedFunc);
    /* 129 */ SyscallDesc("truncate", unimplementedFunc);
    /* 130 */ SyscallDesc("ftruncate", unimplementedFunc);
    /* 131 */ SyscallDesc("flock", unimplementedFunc);
    /* 132 */ SyscallDesc("lstat64", unimplementedFunc);
    /* 133 */ SyscallDesc("sendto", unimplementedFunc);
    /* 134 */ SyscallDesc("shutdown", unimplementedFunc);
    /* 135 */ SyscallDesc("socketpair", unimplementedFunc);
    /* 136 */ SyscallDesc("mkdir", unimplementedFunc);
    /* 137 */ SyscallDesc("rmdir", unimplementedFunc);
    /* 138 */ SyscallDesc("utimes", unimplementedFunc);
    /* 139 */ SyscallDesc("stat64", unimplementedFunc);
    /* 140 */ SyscallDesc("sendfile64", unimplementedFunc);
    /* 141 */ SyscallDesc("getpeername", unimplementedFunc);
    /* 142 */ SyscallDesc("futex", unimplementedFunc);
    /* 143 */ SyscallDesc("gettid", unimplementedFunc);
    /* 144 */ SyscallDesc("getrlimit", unimplementedFunc);
    /* 145 */ SyscallDesc("setrlimit", unimplementedFunc);
    /* 146 */ SyscallDesc("pivot_root", unimplementedFunc);
    /* 147 */ SyscallDesc("prctl", unimplementedFunc);
    /* 148 */ SyscallDesc("pciconfig_read", unimplementedFunc);
    /* 149 */ SyscallDesc("pciconfig_write", unimplementedFunc);
    /* 150 */ SyscallDesc("getsockname", unimplementedFunc);
    /* 151 */ SyscallDesc("inotify_init", unimplementedFunc);
    /* 152 */ SyscallDesc("inotify_add_watch", unimplementedFunc);
    /* 153 */ SyscallDesc("poll", unimplementedFunc);
    /* 154 */ SyscallDesc("getdents64", unimplementedFunc);
    /* 155 */ SyscallDesc("fcntl64", unimplementedFunc);
    /* 156 */ SyscallDesc("inotify_rm_watch", unimplementedFunc);
    /* 157 */ SyscallDesc("statfs", unimplementedFunc);
    /* 158 */ SyscallDesc("fstatfs", unimplementedFunc);
    /* 159 */ SyscallDesc("umount", unimplementedFunc);
    /* 160 */ SyscallDesc("sched_set_affinity", unimplementedFunc);
    /* 161 */ SyscallDesc("sched_get_affinity", unimplementedFunc);
    /* 162 */ SyscallDesc("getdomainname", unimplementedFunc);
    /* 163 */ SyscallDesc("setdomainname", unimplementedFunc);
    /* 164 */ SyscallDesc("utrap_install", unimplementedFunc);
    /* 165 */ SyscallDesc("quotactl", unimplementedFunc);
    /* 166 */ SyscallDesc("set_tid_address", unimplementedFunc);
    /* 167 */ SyscallDesc("mount", unimplementedFunc);
    /* 168 */ SyscallDesc("ustat", unimplementedFunc);
    /* 169 */ SyscallDesc("setxattr", unimplementedFunc);
    /* 170 */ SyscallDesc("lsetxattr", unimplementedFunc);
    /* 171 */ SyscallDesc("fsetxattr", unimplementedFunc);
    /* 172 */ SyscallDesc("getxattr", unimplementedFunc);
    /* 173 */ SyscallDesc("lgetxattr", unimplementedFunc);
    /* 174 */ SyscallDesc("getdents", unimplementedFunc);
    /* 175 */ SyscallDesc("setsid", unimplementedFunc);
    /* 176 */ SyscallDesc("fchdir", unimplementedFunc);
    /* 177 */ SyscallDesc("fgetxattr", unimplementedFunc);
    /* 178 */ SyscallDesc("listxattr", unimplementedFunc);
    /* 179 */ SyscallDesc("llistxattr", unimplementedFunc);
    /* 180 */ SyscallDesc("flistxattr", unimplementedFunc);
    /* 181 */ SyscallDesc("removexattr", unimplementedFunc);
    /* 182 */ SyscallDesc("lremovexattr", unimplementedFunc);
    /* 183 */ SyscallDesc("sigpending", unimplementedFunc);
    /* 184 */ SyscallDesc("query_module", unimplementedFunc);
    /* 185 */ SyscallDesc("setpgid", unimplementedFunc);
    /* 186 */ SyscallDesc("fremovexattr", unimplementedFunc);
    /* 187 */ SyscallDesc("tkill", unimplementedFunc);
    /* 188 */ SyscallDesc("exit_group", unimplementedFunc);
    /* 189 */ SyscallDesc("uname", unimplementedFunc);
    /* 190 */ SyscallDesc("init_module", unimplementedFunc);
    /* 191 */ SyscallDesc("personality", unimplementedFunc);
    /* 192 */ SyscallDesc("remap_file_pages", unimplementedFunc);
    /* 193 */ SyscallDesc("epoll_create", unimplementedFunc);
    /* 194 */ SyscallDesc("epoll_ctl", unimplementedFunc);
    /* 195 */ SyscallDesc("epoll_wait", unimplementedFunc);
    /* 196 */ SyscallDesc("ioprio_set", unimplementedFunc);
    /* 197 */ SyscallDesc("getppid", unimplementedFunc);
    /* 198 */ SyscallDesc("sigaction", unimplementedFunc);
    /* 199 */ SyscallDesc("sgetmask", unimplementedFunc);
    /* 200 */ SyscallDesc("ssetmask", unimplementedFunc);
    /* 201 */ SyscallDesc("sigsuspend", unimplementedFunc);
    /* 202 */ SyscallDesc("oldlstat", unimplementedFunc);
    /* 203 */ SyscallDesc("uselib", unimplementedFunc);
    /* 204 */ SyscallDesc("readdir", unimplementedFunc);
    /* 205 */ SyscallDesc("readahead", unimplementedFunc);
    /* 206 */ SyscallDesc("socketcall", unimplementedFunc);
    /* 207 */ SyscallDesc("syslog", unimplementedFunc);
    /* 208 */ SyscallDesc("lookup_dcookie", unimplementedFunc);
    /* 209 */ SyscallDesc("fadvise64", unimplementedFunc);
    /* 210 */ SyscallDesc("fadvise64_64", unimplementedFunc);
    /* 211 */ SyscallDesc("tgkill", unimplementedFunc);
    /* 212 */ SyscallDesc("waitpid", unimplementedFunc);
    /* 213 */ SyscallDesc("swapoff", unimplementedFunc);
    /* 214 */ SyscallDesc("sysinfo", unimplementedFunc);
    /* 215 */ SyscallDesc("ipc", unimplementedFunc);
    /* 216 */ SyscallDesc("sigreturn", unimplementedFunc);
    /* 217 */ SyscallDesc("clone", unimplementedFunc);
    /* 218 */ SyscallDesc("ioprio_get", unimplementedFunc);
    /* 219 */ SyscallDesc("adjtimex", unimplementedFunc);
    /* 220 */ SyscallDesc("sigprocmask", unimplementedFunc);
    /* 221 */ SyscallDesc("create_module", unimplementedFunc);
    /* 222 */ SyscallDesc("delete_module", unimplementedFunc);
    /* 223 */ SyscallDesc("get_kernel_syms", unimplementedFunc);
    /* 224 */ SyscallDesc("getpgid", unimplementedFunc);
    /* 225 */ SyscallDesc("bdflush", unimplementedFunc);
    /* 226 */ SyscallDesc("sysfs", unimplementedFunc);
    /* 227 */ SyscallDesc("afs_syscall", unimplementedFunc);
    /* 228 */ SyscallDesc("setfsuid", unimplementedFunc);
    /* 229 */ SyscallDesc("setfsgid", unimplementedFunc);
    /* 230 */ SyscallDesc("_newselect", unimplementedFunc);
    /* 231 */ SyscallDesc("time", unimplementedFunc);
    /* 232 */ SyscallDesc("oldstat", unimplementedFunc);
    /* 233 */ SyscallDesc("stime", unimplementedFunc);
    /* 234 */ SyscallDesc("statfs64", unimplementedFunc);
    /* 235 */ SyscallDesc("fstatfs64", unimplementedFunc);
    /* 236 */ SyscallDesc("_llseek", unimplementedFunc);
    /* 237 */ SyscallDesc("mlock", unimplementedFunc);
    /* 238 */ SyscallDesc("munlock", unimplementedFunc);
    /* 239 */ SyscallDesc("mlockall", unimplementedFunc);
    /* 240 */ SyscallDesc("munlockall", unimplementedFunc);
    /* 241 */ SyscallDesc("sched_setparam", unimplementedFunc);
    /* 242 */ SyscallDesc("sched_getparam", unimplementedFunc);
    /* 243 */ SyscallDesc("sched_setscheduler", unimplementedFunc);
    /* 244 */ SyscallDesc("sched_getscheduler", unimplementedFunc);
    /* 245 */ SyscallDesc("sched_yield", unimplementedFunc);
    /* 246 */ SyscallDesc("sched_get_priority_max", unimplemented);
    /* 247 */ SyscallDesc("sched_get_priority_min", unimplemented);
    /* 248 */ SyscallDesc("sched_rr_get_interval", unimplemented);
    /* 249 */ SyscallDesc("nanosleep", unimplementedFunc);
    /* 250 */ SyscallDesc("mremap", unimplementedFunc);
    /* 251 */ SyscallDesc("_sysctl", unimplementedFunc);
    /* 252 */ SyscallDesc("getsid", unimplementedFunc);
    /* 253 */ SyscallDesc("fdatasync", unimplementedFunc);
    /* 254 */ SyscallDesc("nfsservctl", unimplementedFunc);
    /* 255 */ SyscallDesc("aplib", unimplementedFunc);
    /* 256 */ SyscallDesc("clock_settime", unimplementedFunc);
    /* 257 */ SyscallDesc("clock_gettime", unimplementedFunc);
    /* 258 */ SyscallDesc("clock_getres", unimplementedFunc);
    /* 259 */ SyscallDesc("clock_nanosleep", unimplementedFunc);
    /* 260 */ SyscallDesc("sched_getaffinity", unimplementedFunc);
    /* 261 */ SyscallDesc("sched_setaffinity", unimplementedFunc);
    /* 262 */ SyscallDesc("timer_settime", unimplementedFunc);
    /* 263 */ SyscallDesc("timer_gettime", unimplementedFunc);
    /* 264 */ SyscallDesc("timer_getoverrun", unimplementedFunc);
    /* 265 */ SyscallDesc("timer_delete", unimplementedFunc);
    /* 266 */ SyscallDesc("timer_create", unimplementedFunc);
    /* 267 */ SyscallDesc("vserver", unimplementedFunc);
    /* 268 */ SyscallDesc("io_setup", unimplementedFunc);
    /* 269 */ SyscallDesc("io_destroy", unimplementedFunc);
    /* 270 */ SyscallDesc("io_submit", unimplementedFunc);
    /* 271 */ SyscallDesc("io_cancel", unimplementedFunc);
    /* 272 */ SyscallDesc("io_getevents", unimplementedFunc);
    /* 273 */ SyscallDesc("mq_open", unimplementedFunc);
    /* 274 */ SyscallDesc("mq_unlink", unimplementedFunc);
    /* 275 */ SyscallDesc("mq_timedsend", unimplementedFunc);
    /* 276 */ SyscallDesc("mq_timedreceive", unimplementedFunc);
    /* 277 */ SyscallDesc("mq_notify", unimplementedFunc);
    /* 278 */ SyscallDesc("mq_getsetattr", unimplementedFunc);
    /* 279 */ SyscallDesc("waitid", unimplementedFunc);
    /* 280 */ SyscallDesc("sys_setaltroot", unimplementedFunc);
    /* 281 */ SyscallDesc("add_key", unimplementedFunc);
    /* 282 */ SyscallDesc("request_key", unimplementedFunc);
    /* 283 */ SyscallDesc("keyctl", unimplementedFunc);
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
