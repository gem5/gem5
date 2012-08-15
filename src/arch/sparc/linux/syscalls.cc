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
 *
 * Authors: Gabe Black
 */

#include "arch/sparc/linux/process.hh"
#include "sim/syscall_emul.hh"

class LiveProcess;
class ThreadContext;

namespace SparcISA {

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
        ThreadContext *tc)
{
    int index = 0;
    TypedBufferArg<Linux::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, "3.0.0-sparc64");
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "sparc");

    name.copyOut(tc->getMemProxy());

    return 0;
}


SyscallReturn
getresuidFunc(SyscallDesc *desc, int num, LiveProcess *p, ThreadContext *tc)
{
    const IntReg id = htog(100);
    int index = 0;
    Addr ruid = p->getSyscallArg(tc, index);
    Addr euid = p->getSyscallArg(tc, index);
    Addr suid = p->getSyscallArg(tc, index);
    // Handle the EFAULT case
    // Set the ruid
    if (ruid) {
        BufferArg ruidBuff(ruid, sizeof(IntReg));
        memcpy(ruidBuff.bufferPtr(), &id, sizeof(IntReg));
        ruidBuff.copyOut(tc->getMemProxy());
    }
    // Set the euid
    if (euid) {
        BufferArg euidBuff(euid, sizeof(IntReg));
        memcpy(euidBuff.bufferPtr(), &id, sizeof(IntReg));
        euidBuff.copyOut(tc->getMemProxy());
    }
    // Set the suid
    if (suid) {
        BufferArg suidBuff(suid, sizeof(IntReg));
        memcpy(suidBuff.bufferPtr(), &id, sizeof(IntReg));
        suidBuff.copyOut(tc->getMemProxy());
    }
    return 0;
}

SyscallDesc SparcLinuxProcess::syscall32Descs[] = {
    /*   0 */ SyscallDesc("restart_syscall", unimplementedFunc),
    /*   1 */ SyscallDesc("exit", exitFunc), // 32 bit
    /*   2 */ SyscallDesc("fork", unimplementedFunc),
    /*   3 */ SyscallDesc("read", readFunc),
    /*   4 */ SyscallDesc("write", writeFunc),
    /*   5 */ SyscallDesc("open", openFunc<Sparc32Linux>), // 32 bit
    /*   6 */ SyscallDesc("close", closeFunc),
    /*   7 */ SyscallDesc("wait4", unimplementedFunc), // 32 bit
    /*   8 */ SyscallDesc("creat", unimplementedFunc), // 32 bit
    /*   9 */ SyscallDesc("link", unimplementedFunc),
    /*  10 */ SyscallDesc("unlink", unlinkFunc),
    /*  11 */ SyscallDesc("execv", unimplementedFunc),
    /*  12 */ SyscallDesc("chdir", unimplementedFunc),
    /*  13 */ SyscallDesc("chown", chownFunc), // 32 bit
    /*  14 */ SyscallDesc("mknod", unimplementedFunc),
    /*  15 */ SyscallDesc("chmod", unimplementedFunc),
    /*  16 */ SyscallDesc("lchown", unimplementedFunc), // 32 bit
    /*  17 */ SyscallDesc("brk", brkFunc),
    /*  18 */ SyscallDesc("perfctr", unimplementedFunc), // 32 bit
    /*  19 */ SyscallDesc("lseek", lseekFunc), // 32 bit
    /*  20 */ SyscallDesc("getpid", getpidFunc),
    /*  21 */ SyscallDesc("capget", unimplementedFunc),
    /*  22 */ SyscallDesc("capset", unimplementedFunc),
    /*  23 */ SyscallDesc("setuid", setuidFunc), // 32 bit
    /*  24 */ SyscallDesc("getuid", getuidFunc), // 32 bit
    /*  25 */ SyscallDesc("time", unimplementedFunc),
    /*  26 */ SyscallDesc("ptrace", unimplementedFunc),
    /*  27 */ SyscallDesc("alarm", unimplementedFunc),
    /*  28 */ SyscallDesc("sigaltstack", unimplementedFunc), // 32 bit
    /*  29 */ SyscallDesc("pause", unimplementedFunc), // 32 bit
    /*  30 */ SyscallDesc("utime", unimplementedFunc),
    /*  31 */ SyscallDesc("lchown32", unimplementedFunc),
    /*  32 */ SyscallDesc("fchown32", unimplementedFunc),
    /*  33 */ SyscallDesc("access", unimplementedFunc), // 32 bit
    /*  34 */ SyscallDesc("nice", unimplementedFunc), // 32 bit
    /*  35 */ SyscallDesc("chown32", unimplementedFunc),
    /*  36 */ SyscallDesc("sync", unimplementedFunc),
    /*  37 */ SyscallDesc("kill", unimplementedFunc), // 32 bit
    /*  38 */ SyscallDesc("stat", unimplementedFunc),
    /*  39 */ SyscallDesc("sendfile", unimplementedFunc), // 32 bit
    /*  40 */ SyscallDesc("lstat", unimplementedFunc),
    /*  41 */ SyscallDesc("dup", unimplementedFunc),
    /*  42 */ SyscallDesc("pipe", pipePseudoFunc),
    /*  43 */ SyscallDesc("times", ignoreFunc),
    /*  44 */ SyscallDesc("getuid32", unimplementedFunc),
    /*  45 */ SyscallDesc("umount2", unimplementedFunc), // 32 bit
    /*  46 */ SyscallDesc("setgid", unimplementedFunc), // 32 bit
    /*  47 */ SyscallDesc("getgid", getgidFunc), // 32 bit
    /*  48 */ SyscallDesc("signal", unimplementedFunc), // 32 bit
    /*  49 */ SyscallDesc("geteuid", geteuidFunc), // 32 bit
    /*  50 */ SyscallDesc("getegid", getegidFunc), // 32 bit
    /*  51 */ SyscallDesc("acct", unimplementedFunc),
    /*  52 */ SyscallDesc("memory_ordering", unimplementedFunc),
    /*  53 */ SyscallDesc("getgid32", unimplementedFunc),
    /*  54 */ SyscallDesc("ioctl", unimplementedFunc),
    /*  55 */ SyscallDesc("reboot", unimplementedFunc), // 32 bit
    /*  56 */ SyscallDesc("mmap2", unimplementedFunc), // 32 bit
    /*  57 */ SyscallDesc("symlink", unimplementedFunc),
    /*  58 */ SyscallDesc("readlink", readlinkFunc), // 32 bit
    /*  59 */ SyscallDesc("execve", unimplementedFunc), // 32 bit
    /*  60 */ SyscallDesc("umask", unimplementedFunc), // 32 bit
    /*  61 */ SyscallDesc("chroot", unimplementedFunc),
    /*  62 */ SyscallDesc("fstat", unimplementedFunc),
    /*  63 */ SyscallDesc("fstat64", fstat64Func<Sparc32Linux>),
    /*  64 */ SyscallDesc("getpagesize", unimplementedFunc),
    /*  65 */ SyscallDesc("msync", unimplementedFunc), // 32 bit
    /*  66 */ SyscallDesc("vfork", unimplementedFunc),
    /*  67 */ SyscallDesc("pread64", unimplementedFunc), // 32 bit
    /*  68 */ SyscallDesc("pwrite64", unimplementedFunc), // 32 bit
    /*  69 */ SyscallDesc("geteuid32", unimplementedFunc),
    /*  70 */ SyscallDesc("getegid32", unimplementedFunc),
    /*  71 */ SyscallDesc("mmap", mmapFunc<Sparc32Linux>),
    /*  72 */ SyscallDesc("setreuid32", unimplementedFunc),
    /*  73 */ SyscallDesc("munmap", munmapFunc),
    /*  74 */ SyscallDesc("mprotect", ignoreFunc),
    /*  75 */ SyscallDesc("madvise", unimplementedFunc),
    /*  76 */ SyscallDesc("vhangup", unimplementedFunc),
    /*  77 */ SyscallDesc("truncate64", unimplementedFunc), // 32 bit
    /*  78 */ SyscallDesc("mincore", unimplementedFunc),
    /*  79 */ SyscallDesc("getgroups", unimplementedFunc), // 32 bit
    /*  80 */ SyscallDesc("setgroups", unimplementedFunc), // 32 bit
    /*  81 */ SyscallDesc("getpgrp", unimplementedFunc),
    /*  82 */ SyscallDesc("setgroups32", unimplementedFunc), // 32 bit
    /*  83 */ SyscallDesc("setitimer", unimplementedFunc), // 32 bit
    /*  84 */ SyscallDesc("ftruncate64", unimplementedFunc), // 32 bit
    /*  85 */ SyscallDesc("swapon", unimplementedFunc), // 32 bit
    /*  86 */ SyscallDesc("getitimer", unimplementedFunc), // 32 bit
    /*  87 */ SyscallDesc("setuid32", unimplementedFunc),
    /*  88 */ SyscallDesc("sethostname", unimplementedFunc), // 32 bit
    /*  89 */ SyscallDesc("setgid32", unimplementedFunc),
    /*  90 */ SyscallDesc("dup2", unimplementedFunc),
    /*  91 */ SyscallDesc("setfsuid32", unimplementedFunc),
    /*  92 */ SyscallDesc("fcntl", unimplementedFunc),
    /*  93 */ SyscallDesc("select", unimplementedFunc), // 32 bit
    /*  94 */ SyscallDesc("setfsgid32", unimplementedFunc),
    /*  95 */ SyscallDesc("fsync", unimplementedFunc),
    /*  96 */ SyscallDesc("setpriority", unimplementedFunc), // 32 bit
    /*  97 */ SyscallDesc("socket", unimplementedFunc),
    /*  98 */ SyscallDesc("connect", unimplementedFunc),
    /*  99 */ SyscallDesc("accept", unimplementedFunc),
    /* 100 */ SyscallDesc("getpriority", unimplementedFunc), // 32 bit
    /* 101 */ SyscallDesc("rt_sigreturn", unimplementedFunc), // 32 bit
    /* 102 */ SyscallDesc("rt_sigaction", ignoreFunc), // 32 bit
    /* 103 */ SyscallDesc("rt_sigprocmask", ignoreFunc), // 32 bit
    /* 104 */ SyscallDesc("rt_sigpending", unimplementedFunc), // 32 bit
    /* 105 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 106 */ SyscallDesc("rt_sigqueueinfo", unimplementedFunc), // 32 bit
    /* 107 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 108 */ SyscallDesc("setresuid32", unimplementedFunc),
    /* 109 */ SyscallDesc("getresuid32", getresuidFunc),
    /* 110 */ SyscallDesc("setresgid32", ignoreFunc),
    /* 111 */ SyscallDesc("getresgid32", unimplementedFunc),
    /* 112 */ SyscallDesc("setregid32", unimplementedFunc),
    /* 113 */ SyscallDesc("revcmsg", unimplementedFunc),
    /* 114 */ SyscallDesc("sendmsg", unimplementedFunc),
    /* 115 */ SyscallDesc("getgroups32", unimplementedFunc), // 32 bit
    /* 116 */ SyscallDesc("gettimeofday", gettimeofdayFunc<Sparc32Linux>), // 32 bit
    /* 117 */ SyscallDesc("getrusage", unimplementedFunc), // 32 bit
    /* 118 */ SyscallDesc("getsockopt", unimplementedFunc),
    /* 119 */ SyscallDesc("getcwd", getcwdFunc),
    /* 120 */ SyscallDesc("readv", unimplementedFunc),
    /* 121 */ SyscallDesc("writev", unimplementedFunc),
    /* 122 */ SyscallDesc("settimeofday", unimplementedFunc), // 32 bit
    /* 123 */ SyscallDesc("fchown", unimplementedFunc), // 32 bit
    /* 124 */ SyscallDesc("fchmod", unimplementedFunc),
    /* 125 */ SyscallDesc("recvfrom", unimplementedFunc),
    /* 126 */ SyscallDesc("setreuid", unimplementedFunc), // 32 bit
    /* 127 */ SyscallDesc("setregid", unimplementedFunc), // 32 bit
    /* 128 */ SyscallDesc("rename", renameFunc),
    /* 129 */ SyscallDesc("truncate", unimplementedFunc),
    /* 130 */ SyscallDesc("ftruncate", unimplementedFunc),
    /* 131 */ SyscallDesc("flock", unimplementedFunc),
    /* 132 */ SyscallDesc("lstat64", unimplementedFunc),
    /* 133 */ SyscallDesc("sendto", unimplementedFunc),
    /* 134 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 135 */ SyscallDesc("socketpair", unimplementedFunc),
    /* 136 */ SyscallDesc("mkdir", mkdirFunc), // 32 bit
    /* 137 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 138 */ SyscallDesc("utimes", unimplementedFunc), // 32 bit
    /* 139 */ SyscallDesc("stat64", unimplementedFunc),
    /* 140 */ SyscallDesc("sendfile64", unimplementedFunc), // 32 bit
    /* 141 */ SyscallDesc("getpeername", unimplementedFunc),
    /* 142 */ SyscallDesc("futex", unimplementedFunc), // 32 bit
    /* 143 */ SyscallDesc("gettid", unimplementedFunc),
    /* 144 */ SyscallDesc("getrlimit", unimplementedFunc),
    /* 145 */ SyscallDesc("setrlimit", unimplementedFunc),
    /* 146 */ SyscallDesc("pivot_root", unimplementedFunc),
    /* 147 */ SyscallDesc("prctl", unimplementedFunc), // 32 bit
    /* 148 */ SyscallDesc("pciconfig_read", unimplementedFunc),
    /* 149 */ SyscallDesc("pciconfig_write", unimplementedFunc),
    /* 150 */ SyscallDesc("getsockname", unimplementedFunc),
    /* 151 */ SyscallDesc("inotify_init", unimplementedFunc),
    /* 152 */ SyscallDesc("inotify_add_watch", unimplementedFunc),
    /* 153 */ SyscallDesc("poll", unimplementedFunc),
    /* 154 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 155 */ SyscallDesc("fcntl64", unimplementedFunc),
    /* 156 */ SyscallDesc("inotify_rm_watch", unimplementedFunc),
    /* 157 */ SyscallDesc("statfs", unimplementedFunc),
    /* 158 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 159 */ SyscallDesc("umount", unimplementedFunc),
    /* 160 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 161 */ SyscallDesc("sched_getaffinity", unimplementedFunc),
    /* 162 */ SyscallDesc("getdomainname", unimplementedFunc), // 32 bit
    /* 163 */ SyscallDesc("setdomainname", unimplementedFunc), // 32 bit
    /* 164 */ SyscallDesc("ni_syscall", unimplementedFunc),
    /* 165 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 166 */ SyscallDesc("set_tid_address", unimplementedFunc),
    /* 167 */ SyscallDesc("mount", unimplementedFunc),
    /* 168 */ SyscallDesc("ustat", unimplementedFunc),
    /* 169 */ SyscallDesc("setxattr", unimplementedFunc), // 32 bit
    /* 170 */ SyscallDesc("lsetxattr", unimplementedFunc), // 32 bit
    /* 171 */ SyscallDesc("fsetxattr", unimplementedFunc), // 32 bit
    /* 172 */ SyscallDesc("getxattr", unimplementedFunc),
    /* 173 */ SyscallDesc("lgetxattr", unimplementedFunc),
    /* 174 */ SyscallDesc("getdents", unimplementedFunc),
    /* 175 */ SyscallDesc("setsid", unimplementedFunc),
    /* 176 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 177 */ SyscallDesc("fgetxattr", unimplementedFunc), // 32 bit
    /* 178 */ SyscallDesc("listxattr", unimplementedFunc),
    /* 179 */ SyscallDesc("llistxattr", unimplementedFunc),
    /* 180 */ SyscallDesc("flistxattr", unimplementedFunc), // 32 bit
    /* 181 */ SyscallDesc("removexattr", unimplementedFunc),
    /* 182 */ SyscallDesc("lremovexattr", unimplementedFunc),
    /* 183 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 184 */ SyscallDesc("query_module", unimplementedFunc),
    /* 185 */ SyscallDesc("setpgid", unimplementedFunc), // 32 bit
    /* 186 */ SyscallDesc("fremovexattr", unimplementedFunc), // 32 bit
    /* 187 */ SyscallDesc("tkill", unimplementedFunc), // 32 bit
    /* 188 */ SyscallDesc("exit_group", exitGroupFunc), // 32 bit
    /* 189 */ SyscallDesc("uname", unameFunc),
    /* 190 */ SyscallDesc("init_module", unimplementedFunc), // 32 bit
    /* 191 */ SyscallDesc("personality", unimplementedFunc),
    /* 192 */ SyscallDesc("remap_file_pages", unimplementedFunc),
    /* 193 */ SyscallDesc("epoll_create", unimplementedFunc), // 32 bit
    /* 194 */ SyscallDesc("epoll_ctl", unimplementedFunc), // 32 bit
    /* 195 */ SyscallDesc("epoll_wait", unimplementedFunc), // 32 bit
    /* 196 */ SyscallDesc("ioprio_set", unimplementedFunc), // 32 bit
    /* 197 */ SyscallDesc("getppid", getppidFunc),
    /* 198 */ SyscallDesc("sigaction", unimplementedFunc), // 32 bit
    /* 199 */ SyscallDesc("sgetmask", unimplementedFunc),
    /* 200 */ SyscallDesc("ssetmask", unimplementedFunc),
    /* 201 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 202 */ SyscallDesc("oldlstat", unimplementedFunc),
    /* 203 */ SyscallDesc("uselib", unimplementedFunc),
    /* 204 */ SyscallDesc("readdir", unimplementedFunc),
    /* 205 */ SyscallDesc("readahead", unimplementedFunc), // 32 bit
    /* 206 */ SyscallDesc("socketcall", unimplementedFunc), // 32 bit
    /* 207 */ SyscallDesc("syslog", unimplementedFunc), // 32 bit
    /* 208 */ SyscallDesc("lookup_dcookie", unimplementedFunc), // 32 bit
    /* 209 */ SyscallDesc("fadvise64", unimplementedFunc), // 32 bit
    /* 210 */ SyscallDesc("fadvise64_64", unimplementedFunc), // 32 bit
    /* 211 */ SyscallDesc("tgkill", unimplementedFunc), // 32 bit
    /* 212 */ SyscallDesc("waitpid", unimplementedFunc), // 32 bit
    /* 213 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 214 */ SyscallDesc("sysinfo", sysinfoFunc<Sparc32Linux>), // 32 bit
    /* 215 */ SyscallDesc("ipc", unimplementedFunc), // 32 bit
    /* 216 */ SyscallDesc("sigreturn", unimplementedFunc), // 32 bit
    /* 217 */ SyscallDesc("clone", cloneFunc),
    /* 218 */ SyscallDesc("ioprio_get", unimplementedFunc), // 32 bit
    /* 219 */ SyscallDesc("adjtimex", unimplementedFunc), // 32 bit
    /* 220 */ SyscallDesc("sigprocmask", unimplementedFunc), // 32 bit
    /* 221 */ SyscallDesc("create_module", unimplementedFunc),
    /* 222 */ SyscallDesc("delete_module", unimplementedFunc), // 32 bit
    /* 223 */ SyscallDesc("get_kernel_syms", unimplementedFunc),
    /* 224 */ SyscallDesc("getpgid", unimplementedFunc), // 32 bit
    /* 225 */ SyscallDesc("bdflush", unimplementedFunc), // 32 bit
    /* 226 */ SyscallDesc("sysfs", unimplementedFunc), // 32 bit
    /* 227 */ SyscallDesc("afs_syscall", unimplementedFunc),
    /* 228 */ SyscallDesc("setfsuid", unimplementedFunc), // 32 bit
    /* 229 */ SyscallDesc("setfsgid", unimplementedFunc), // 32 bit
    /* 230 */ SyscallDesc("_newselect", unimplementedFunc), // 32 bit
    /* 231 */ SyscallDesc("time", ignoreFunc),
    /* 232 */ SyscallDesc("oldstat", unimplementedFunc),
    /* 233 */ SyscallDesc("stime", unimplementedFunc),
    /* 234 */ SyscallDesc("statfs64", unimplementedFunc),
    /* 235 */ SyscallDesc("fstatfs64", unimplementedFunc),
    /* 236 */ SyscallDesc("_llseek", _llseekFunc),
    /* 237 */ SyscallDesc("mlock", unimplementedFunc),
    /* 238 */ SyscallDesc("munlock", unimplementedFunc),
    /* 239 */ SyscallDesc("mlockall", unimplementedFunc), // 32 bit
    /* 240 */ SyscallDesc("munlockall", unimplementedFunc),
    /* 241 */ SyscallDesc("sched_setparam", unimplementedFunc), // 32 bit
    /* 242 */ SyscallDesc("sched_getparam", unimplementedFunc), // 32 bit
    /* 243 */ SyscallDesc("sched_setscheduler", unimplementedFunc), // 32 bit
    /* 244 */ SyscallDesc("sched_getscheduler", unimplementedFunc), // 32 bit
    /* 245 */ SyscallDesc("sched_yield", unimplementedFunc),
    /* 246 */ SyscallDesc("sched_get_priority_max", unimplementedFunc), // 32 bit
    /* 247 */ SyscallDesc("sched_get_priority_min", unimplementedFunc), // 32 bit
    /* 248 */ SyscallDesc("sched_rr_get_interval", unimplementedFunc), // 32 bit
    /* 249 */ SyscallDesc("nanosleep", unimplementedFunc),
    /* 250 */ SyscallDesc("mremap", mremapFunc<Sparc32Linux>), // 32 bit
    /* 251 */ SyscallDesc("_sysctl", unimplementedFunc), // 32 bit
    /* 252 */ SyscallDesc("getsid", unimplementedFunc), // 32 bit
    /* 253 */ SyscallDesc("fdatasync", unimplementedFunc),
    /* 254 */ SyscallDesc("nfsservctl", unimplementedFunc), // 32 bit
    /* 255 */ SyscallDesc("aplib", unimplementedFunc),
    /* 256 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 257 */ SyscallDesc("clock_gettime", unimplementedFunc),
    /* 258 */ SyscallDesc("clock_getres", unimplementedFunc),
    /* 259 */ SyscallDesc("clock_nanosleep", unimplementedFunc), // 32 bit
    /* 260 */ SyscallDesc("sched_getaffinity", unimplementedFunc),
    /* 261 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 262 */ SyscallDesc("timer_settime", unimplementedFunc), // 32 bit
    /* 263 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 264 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 265 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 266 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 267 */ SyscallDesc("vserver", unimplementedFunc),
    /* 268 */ SyscallDesc("io_setup", unimplementedFunc),
    /* 269 */ SyscallDesc("io_destroy", unimplementedFunc),
    /* 270 */ SyscallDesc("io_submit", unimplementedFunc), // 32 bit
    /* 271 */ SyscallDesc("io_cancel", unimplementedFunc),
    /* 272 */ SyscallDesc("io_getevents", unimplementedFunc),
    /* 273 */ SyscallDesc("mq_open", unimplementedFunc), // 32 bit
    /* 274 */ SyscallDesc("mq_unlink", unimplementedFunc),
    /* 275 */ SyscallDesc("mq_timedsend", unimplementedFunc),
    /* 276 */ SyscallDesc("mq_timedreceive", unimplementedFunc),
    /* 277 */ SyscallDesc("mq_notify", unimplementedFunc),
    /* 278 */ SyscallDesc("mq_getsetattr", unimplementedFunc),
    /* 279 */ SyscallDesc("waitid", unimplementedFunc),
    /* 280 */ SyscallDesc("sys_setaltroot", unimplementedFunc),
    /* 281 */ SyscallDesc("add_key", unimplementedFunc),
    /* 282 */ SyscallDesc("request_key", unimplementedFunc),
    /* 283 */ SyscallDesc("keyctl", unimplementedFunc),
    /* 284 */ SyscallDesc("openat", unimplementedFunc),
    /* 285 */ SyscallDesc("mkdirat", unimplementedFunc),
    /* 286 */ SyscallDesc("mknodat", unimplementedFunc),
    /* 287 */ SyscallDesc("fchownat", unimplementedFunc),
    /* 288 */ SyscallDesc("futimesat", unimplementedFunc),
    /* 289 */ SyscallDesc("fstatat64", unimplementedFunc),
    /* 290 */ SyscallDesc("unlinkat", unimplementedFunc),
    /* 291 */ SyscallDesc("renameat", unimplementedFunc),
    /* 292 */ SyscallDesc("linkat", unimplementedFunc),
    /* 293 */ SyscallDesc("symlinkat", unimplementedFunc),
    /* 294 */ SyscallDesc("readlinkat", unimplementedFunc),
    /* 295 */ SyscallDesc("fchmodat", unimplementedFunc),
    /* 296 */ SyscallDesc("faccessat", unimplementedFunc),
    /* 297 */ SyscallDesc("pselect6", unimplementedFunc),
    /* 298 */ SyscallDesc("ppoll", unimplementedFunc),
    /* 299 */ SyscallDesc("unshare", unimplementedFunc)
};

const int SparcLinuxProcess::Num_Syscall32_Descs =
    sizeof(SparcLinuxProcess::syscall32Descs) / sizeof(SyscallDesc);

SyscallDesc SparcLinuxProcess::syscallDescs[] = {
    /*  0 */ SyscallDesc("restart_syscall", unimplementedFunc),
    /*  1 */ SyscallDesc("exit", exitFunc),
    /*  2 */ SyscallDesc("fork", unimplementedFunc),
    /*  3 */ SyscallDesc("read", readFunc),
    /*  4 */ SyscallDesc("write", writeFunc),
    /*  5 */ SyscallDesc("open", openFunc<SparcLinux>),
    /*  6 */ SyscallDesc("close", closeFunc),
    /*  7 */ SyscallDesc("wait4", unimplementedFunc),
    /*  8 */ SyscallDesc("creat", unimplementedFunc),
    /*  9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unlinkFunc),
    /* 11 */ SyscallDesc("execv", unimplementedFunc),
    /* 12 */ SyscallDesc("chdir", unimplementedFunc),
    /* 13 */ SyscallDesc("chown", chownFunc),
    /* 14 */ SyscallDesc("mknod", unimplementedFunc),
    /* 15 */ SyscallDesc("chmod", chmodFunc<Linux>),
    /* 16 */ SyscallDesc("lchown", unimplementedFunc),
    /* 17 */ SyscallDesc("brk", brkFunc),
    /* 18 */ SyscallDesc("perfctr", unimplementedFunc),
    /* 19 */ SyscallDesc("lseek", lseekFunc),
    /* 20 */ SyscallDesc("getpid", getpidFunc),
    /* 21 */ SyscallDesc("capget", unimplementedFunc),
    /* 22 */ SyscallDesc("capset", unimplementedFunc),
    /* 23 */ SyscallDesc("setuid", setuidFunc),
    /* 24 */ SyscallDesc("getuid", getuidFunc),
    /* 25 */ SyscallDesc("time", unimplementedFunc),
    /* 26 */ SyscallDesc("ptrace", unimplementedFunc),
    /* 27 */ SyscallDesc("alarm", unimplementedFunc),
    /* 28 */ SyscallDesc("sigaltstack", unimplementedFunc),
    /* 29 */ SyscallDesc("pause", unimplementedFunc),
    /* 30 */ SyscallDesc("utime", unimplementedFunc),
    /* 31 */ SyscallDesc("lchown32", unimplementedFunc),
    /* 32 */ SyscallDesc("fchown32", unimplementedFunc),
    /* 33 */ SyscallDesc("access", unimplementedFunc),
    /* 34 */ SyscallDesc("nice", unimplementedFunc),
    /* 35 */ SyscallDesc("chown32", unimplementedFunc),
    /* 36 */ SyscallDesc("sync", unimplementedFunc),
    /* 37 */ SyscallDesc("kill", unimplementedFunc),
    /* 38 */ SyscallDesc("stat", unimplementedFunc),
    /* 39 */ SyscallDesc("sendfile", unimplementedFunc),
    /* 40 */ SyscallDesc("lstat", unimplementedFunc),
    /* 41 */ SyscallDesc("dup", unimplementedFunc),
    /* 42 */ SyscallDesc("pipe", pipePseudoFunc),
    /* 43 */ SyscallDesc("times", ignoreFunc),
    /* 44 */ SyscallDesc("getuid32", unimplementedFunc),
    /* 45 */ SyscallDesc("umount2", unimplementedFunc),
    /* 46 */ SyscallDesc("setgid", unimplementedFunc),
    /* 47 */ SyscallDesc("getgid", getgidFunc),
    /* 48 */ SyscallDesc("signal", unimplementedFunc),
    /* 49 */ SyscallDesc("geteuid", geteuidFunc),
    /* 50 */ SyscallDesc("getegid", getegidFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("memory_ordering", unimplementedFunc),
    /* 53 */ SyscallDesc("getgid32", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", unimplementedFunc),
    /* 55 */ SyscallDesc("reboot", unimplementedFunc),
    /* 56 */ SyscallDesc("mmap2", unimplementedFunc),
    /* 57 */ SyscallDesc("symlink", unimplementedFunc),
    /* 58 */ SyscallDesc("readlink", readlinkFunc),
    /* 59 */ SyscallDesc("execve", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", unimplementedFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("fstat", fstatFunc<SparcLinux>),
    /* 63 */ SyscallDesc("fstat64", unimplementedFunc),
    /* 64 */ SyscallDesc("getpagesize", unimplementedFunc),
    /* 65 */ SyscallDesc("msync", unimplementedFunc),
    /* 66 */ SyscallDesc("vfork", unimplementedFunc),
    /* 67 */ SyscallDesc("pread64", unimplementedFunc),
    /* 68 */ SyscallDesc("pwrite64", unimplementedFunc),
    /* 69 */ SyscallDesc("geteuid32", unimplementedFunc),
    /* 70 */ SyscallDesc("getegid32", unimplementedFunc),
    /* 71 */ SyscallDesc("mmap", mmapFunc<SparcLinux>),
    /* 72 */ SyscallDesc("setreuid32", unimplementedFunc),
    /* 73 */ SyscallDesc("munmap", munmapFunc),
    /* 74 */ SyscallDesc("mprotect", ignoreFunc),
    /* 75 */ SyscallDesc("madvise", unimplementedFunc),
    /* 76 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 77 */ SyscallDesc("truncate64", unimplementedFunc),
    /* 78 */ SyscallDesc("mincore", unimplementedFunc),
    /* 79 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 80 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 81 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 82 */ SyscallDesc("setgroups32", unimplementedFunc),
    /* 83 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 84 */ SyscallDesc("ftruncate64", unimplementedFunc),
    /* 85 */ SyscallDesc("swapon", unimplementedFunc),
    /* 86 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 87 */ SyscallDesc("setuid32", unimplementedFunc),
    /* 88 */ SyscallDesc("sethostname", unimplementedFunc),
    /* 89 */ SyscallDesc("setgid32", unimplementedFunc),
    /* 90 */ SyscallDesc("dup2", unimplementedFunc),
    /* 91 */ SyscallDesc("setfsuid32", unimplementedFunc),
    /* 92 */ SyscallDesc("fcntl", unimplementedFunc),
    /* 93 */ SyscallDesc("select", unimplementedFunc),
    /* 94 */ SyscallDesc("setfsgid32", unimplementedFunc),
    /* 95 */ SyscallDesc("fsync", unimplementedFunc),
    /* 96 */ SyscallDesc("setpriority", unimplementedFunc),
    /* 97 */ SyscallDesc("socket", unimplementedFunc),
    /* 98 */ SyscallDesc("connect", unimplementedFunc),
    /* 99 */ SyscallDesc("accept", unimplementedFunc),
    /* 100 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 101 */ SyscallDesc("rt_sigreturn", unimplementedFunc),
    /* 102 */ SyscallDesc("rt_sigaction", ignoreFunc),
    /* 103 */ SyscallDesc("rt_sigprocmask", ignoreFunc),
    /* 104 */ SyscallDesc("rt_sigpending", unimplementedFunc),
    /* 105 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 106 */ SyscallDesc("rt_sigqueueinfo", unimplementedFunc),
    /* 107 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 108 */ SyscallDesc("setresuid", unimplementedFunc),
    /* 109 */ SyscallDesc("getresuid", getresuidFunc),
    /* 110 */ SyscallDesc("setresgid", ignoreFunc),
    /* 111 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 112 */ SyscallDesc("setregid32", unimplementedFunc),
    /* 113 */ SyscallDesc("recvmsg", unimplementedFunc),
    /* 114 */ SyscallDesc("sendmsg", unimplementedFunc),
    /* 115 */ SyscallDesc("getgroups32", unimplementedFunc),
    /* 116 */ SyscallDesc("gettimeofday", gettimeofdayFunc<SparcLinux>),
    /* 117 */ SyscallDesc("getrusage", unimplementedFunc),
    /* 118 */ SyscallDesc("getsockopt", unimplementedFunc),
    /* 119 */ SyscallDesc("getcwd", unimplementedFunc),
    /* 120 */ SyscallDesc("readv", unimplementedFunc),
    /* 121 */ SyscallDesc("writev", unimplementedFunc),
    /* 122 */ SyscallDesc("settimeofday", unimplementedFunc),
    /* 123 */ SyscallDesc("fchown", unimplementedFunc),
    /* 124 */ SyscallDesc("fchmod", unimplementedFunc),
    /* 125 */ SyscallDesc("recvfrom", unimplementedFunc),
    /* 126 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 127 */ SyscallDesc("setregid", unimplementedFunc),
    /* 128 */ SyscallDesc("rename", renameFunc),
    /* 129 */ SyscallDesc("truncate", unimplementedFunc),
    /* 130 */ SyscallDesc("ftruncate", unimplementedFunc),
    /* 131 */ SyscallDesc("flock", unimplementedFunc),
    /* 132 */ SyscallDesc("lstat64", unimplementedFunc),
    /* 133 */ SyscallDesc("sendto", unimplementedFunc),
    /* 134 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 135 */ SyscallDesc("socketpair", unimplementedFunc),
    /* 136 */ SyscallDesc("mkdir", mkdirFunc),
    /* 137 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 138 */ SyscallDesc("utimes", unimplementedFunc),
    /* 139 */ SyscallDesc("stat64", unimplementedFunc),
    /* 140 */ SyscallDesc("sendfile64", unimplementedFunc),
    /* 141 */ SyscallDesc("getpeername", unimplementedFunc),
    /* 142 */ SyscallDesc("futex", unimplementedFunc),
    /* 143 */ SyscallDesc("gettid", unimplementedFunc),
    /* 144 */ SyscallDesc("getrlimit", unimplementedFunc),
    /* 145 */ SyscallDesc("setrlimit", unimplementedFunc),
    /* 146 */ SyscallDesc("pivot_root", unimplementedFunc),
    /* 147 */ SyscallDesc("prctl", unimplementedFunc),
    /* 148 */ SyscallDesc("pciconfig_read", unimplementedFunc),
    /* 149 */ SyscallDesc("pciconfig_write", unimplementedFunc),
    /* 150 */ SyscallDesc("getsockname", unimplementedFunc),
    /* 151 */ SyscallDesc("inotify_init", unimplementedFunc),
    /* 152 */ SyscallDesc("inotify_add_watch", unimplementedFunc),
    /* 153 */ SyscallDesc("poll", unimplementedFunc),
    /* 154 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 155 */ SyscallDesc("fcntl64", unimplementedFunc),
    /* 156 */ SyscallDesc("inotify_rm_watch", unimplementedFunc),
    /* 157 */ SyscallDesc("statfs", unimplementedFunc),
    /* 158 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 159 */ SyscallDesc("umount", unimplementedFunc),
    /* 160 */ SyscallDesc("sched_set_affinity", unimplementedFunc),
    /* 161 */ SyscallDesc("sched_get_affinity", unimplementedFunc),
    /* 162 */ SyscallDesc("getdomainname", unimplementedFunc),
    /* 163 */ SyscallDesc("setdomainname", unimplementedFunc),
    /* 164 */ SyscallDesc("utrap_install", unimplementedFunc),
    /* 165 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 166 */ SyscallDesc("set_tid_address", unimplementedFunc),
    /* 167 */ SyscallDesc("mount", unimplementedFunc),
    /* 168 */ SyscallDesc("ustat", unimplementedFunc),
    /* 169 */ SyscallDesc("setxattr", unimplementedFunc),
    /* 170 */ SyscallDesc("lsetxattr", unimplementedFunc),
    /* 171 */ SyscallDesc("fsetxattr", unimplementedFunc),
    /* 172 */ SyscallDesc("getxattr", unimplementedFunc),
    /* 173 */ SyscallDesc("lgetxattr", unimplementedFunc),
    /* 174 */ SyscallDesc("getdents", unimplementedFunc),
    /* 175 */ SyscallDesc("setsid", unimplementedFunc),
    /* 176 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 177 */ SyscallDesc("fgetxattr", unimplementedFunc),
    /* 178 */ SyscallDesc("listxattr", unimplementedFunc),
    /* 179 */ SyscallDesc("llistxattr", unimplementedFunc),
    /* 180 */ SyscallDesc("flistxattr", unimplementedFunc),
    /* 181 */ SyscallDesc("removexattr", unimplementedFunc),
    /* 182 */ SyscallDesc("lremovexattr", unimplementedFunc),
    /* 183 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 184 */ SyscallDesc("query_module", unimplementedFunc),
    /* 185 */ SyscallDesc("setpgid", unimplementedFunc),
    /* 186 */ SyscallDesc("fremovexattr", unimplementedFunc),
    /* 187 */ SyscallDesc("tkill", unimplementedFunc),
    /* 188 */ SyscallDesc("exit_group", exitGroupFunc),
    /* 189 */ SyscallDesc("uname", unameFunc),
    /* 190 */ SyscallDesc("init_module", unimplementedFunc),
    /* 191 */ SyscallDesc("personality", unimplementedFunc),
    /* 192 */ SyscallDesc("remap_file_pages", unimplementedFunc),
    /* 193 */ SyscallDesc("epoll_create", unimplementedFunc),
    /* 194 */ SyscallDesc("epoll_ctl", unimplementedFunc),
    /* 195 */ SyscallDesc("epoll_wait", unimplementedFunc),
    /* 196 */ SyscallDesc("ioprio_set", unimplementedFunc),
    /* 197 */ SyscallDesc("getppid", getppidFunc),
    /* 198 */ SyscallDesc("sigaction", ignoreFunc),
    /* 199 */ SyscallDesc("sgetmask", unimplementedFunc),
    /* 200 */ SyscallDesc("ssetmask", unimplementedFunc),
    /* 201 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 202 */ SyscallDesc("oldlstat", unimplementedFunc),
    /* 203 */ SyscallDesc("uselib", unimplementedFunc),
    /* 204 */ SyscallDesc("readdir", unimplementedFunc),
    /* 205 */ SyscallDesc("readahead", unimplementedFunc),
    /* 206 */ SyscallDesc("socketcall", unimplementedFunc),
    /* 207 */ SyscallDesc("syslog", unimplementedFunc),
    /* 208 */ SyscallDesc("lookup_dcookie", unimplementedFunc),
    /* 209 */ SyscallDesc("fadvise64", unimplementedFunc),
    /* 210 */ SyscallDesc("fadvise64_64", unimplementedFunc),
    /* 211 */ SyscallDesc("tgkill", unimplementedFunc),
    /* 212 */ SyscallDesc("waitpid", unimplementedFunc),
    /* 213 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 214 */ SyscallDesc("sysinfo", sysinfoFunc<SparcLinux>),
    /* 215 */ SyscallDesc("ipc", unimplementedFunc),
    /* 216 */ SyscallDesc("sigreturn", unimplementedFunc),
    /* 217 */ SyscallDesc("clone", cloneFunc),
    /* 218 */ SyscallDesc("ioprio_get", unimplementedFunc),
    /* 219 */ SyscallDesc("adjtimex", unimplementedFunc),
    /* 220 */ SyscallDesc("sigprocmask", unimplementedFunc),
    /* 221 */ SyscallDesc("create_module", unimplementedFunc),
    /* 222 */ SyscallDesc("delete_module", unimplementedFunc),
    /* 223 */ SyscallDesc("get_kernel_syms", unimplementedFunc),
    /* 224 */ SyscallDesc("getpgid", unimplementedFunc),
    /* 225 */ SyscallDesc("bdflush", unimplementedFunc),
    /* 226 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 227 */ SyscallDesc("afs_syscall", unimplementedFunc),
    /* 228 */ SyscallDesc("setfsuid", unimplementedFunc),
    /* 229 */ SyscallDesc("setfsgid", unimplementedFunc),
    /* 230 */ SyscallDesc("_newselect", unimplementedFunc),
    /* 231 */ SyscallDesc("time", ignoreFunc),
    /* 232 */ SyscallDesc("oldstat", unimplementedFunc),
    /* 233 */ SyscallDesc("stime", unimplementedFunc),
    /* 234 */ SyscallDesc("statfs64", unimplementedFunc),
    /* 235 */ SyscallDesc("fstatfs64", unimplementedFunc),
    /* 236 */ SyscallDesc("_llseek", _llseekFunc),
    /* 237 */ SyscallDesc("mlock", unimplementedFunc),
    /* 238 */ SyscallDesc("munlock", unimplementedFunc),
    /* 239 */ SyscallDesc("mlockall", unimplementedFunc),
    /* 240 */ SyscallDesc("munlockall", unimplementedFunc),
    /* 241 */ SyscallDesc("sched_setparam", unimplementedFunc),
    /* 242 */ SyscallDesc("sched_getparam", unimplementedFunc),
    /* 243 */ SyscallDesc("sched_setscheduler", unimplementedFunc),
    /* 244 */ SyscallDesc("sched_getscheduler", unimplementedFunc),
    /* 245 */ SyscallDesc("sched_yield", unimplementedFunc),
    /* 246 */ SyscallDesc("sched_get_priority_max", unimplementedFunc),
    /* 247 */ SyscallDesc("sched_get_priority_min", unimplementedFunc),
    /* 248 */ SyscallDesc("sched_rr_get_interval", unimplementedFunc),
    /* 249 */ SyscallDesc("nanosleep", unimplementedFunc),
    /* 250 */ SyscallDesc("mremap", mremapFunc<SparcLinux>),
    /* 251 */ SyscallDesc("_sysctl", unimplementedFunc),
    /* 252 */ SyscallDesc("getsid", unimplementedFunc),
    /* 253 */ SyscallDesc("fdatasync", unimplementedFunc),
    /* 254 */ SyscallDesc("nfsservctl", unimplementedFunc),
    /* 255 */ SyscallDesc("aplib", unimplementedFunc),
    /* 256 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 257 */ SyscallDesc("clock_gettime", unimplementedFunc),
    /* 258 */ SyscallDesc("clock_getres", unimplementedFunc),
    /* 259 */ SyscallDesc("clock_nanosleep", unimplementedFunc),
    /* 260 */ SyscallDesc("sched_getaffinity", unimplementedFunc),
    /* 261 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 262 */ SyscallDesc("timer_settime", unimplementedFunc),
    /* 263 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 264 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 265 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 266 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 267 */ SyscallDesc("vserver", unimplementedFunc),
    /* 268 */ SyscallDesc("io_setup", unimplementedFunc),
    /* 269 */ SyscallDesc("io_destroy", unimplementedFunc),
    /* 270 */ SyscallDesc("io_submit", unimplementedFunc),
    /* 271 */ SyscallDesc("io_cancel", unimplementedFunc),
    /* 272 */ SyscallDesc("io_getevents", unimplementedFunc),
    /* 273 */ SyscallDesc("mq_open", unimplementedFunc),
    /* 274 */ SyscallDesc("mq_unlink", unimplementedFunc),
    /* 275 */ SyscallDesc("mq_timedsend", unimplementedFunc),
    /* 276 */ SyscallDesc("mq_timedreceive", unimplementedFunc),
    /* 277 */ SyscallDesc("mq_notify", unimplementedFunc),
    /* 278 */ SyscallDesc("mq_getsetattr", unimplementedFunc),
    /* 279 */ SyscallDesc("waitid", unimplementedFunc),
    /* 280 */ SyscallDesc("sys_setaltroot", unimplementedFunc),
    /* 281 */ SyscallDesc("add_key", unimplementedFunc),
    /* 282 */ SyscallDesc("request_key", unimplementedFunc),
    /* 283 */ SyscallDesc("keyctl", unimplementedFunc)
};

const int SparcLinuxProcess::Num_Syscall_Descs =
    sizeof(SparcLinuxProcess::syscallDescs) / sizeof(SyscallDesc);

} // namespace SparcISA
