/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#include "arch/x86/linux/process.hh"
#include "arch/x86/linux/linux.hh"
#include "kern/linux/linux.hh"
#include "sim/syscall_emul.hh"

using namespace X86ISA;

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
          ThreadContext *tc)
{
    TypedBufferArg<Linux::utsname> name(tc->getSyscallArg(0));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "m5.eecs.umich.edu");
    strcpy(name->release, "2.6.12");
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "x86_64");

    name.copyOut(tc->getMemPort());

    return 0;
}

SyscallDesc X86LinuxProcess::syscallDescs[] = {
    /*   0 */ SyscallDesc("read", unimplementedFunc),
    /*   1 */ SyscallDesc("write", unimplementedFunc),
    /*   2 */ SyscallDesc("open", openFunc<X86Linux64>),
    /*   3 */ SyscallDesc("close", unimplementedFunc),
    /*   4 */ SyscallDesc("stat", unimplementedFunc),
    /*   5 */ SyscallDesc("fstat", unimplementedFunc),
    /*   6 */ SyscallDesc("lstat", unimplementedFunc),
    /*   7 */ SyscallDesc("poll", unimplementedFunc),
    /*   8 */ SyscallDesc("lseek", unimplementedFunc),
    /*   9 */ SyscallDesc("mmap", unimplementedFunc),
    /*  10 */ SyscallDesc("mprotect", unimplementedFunc),
    /*  11 */ SyscallDesc("munmap", unimplementedFunc),
    /*  12 */ SyscallDesc("brk", unimplementedFunc),
    /*  13 */ SyscallDesc("rt_sigaction", unimplementedFunc),
    /*  14 */ SyscallDesc("rt_sigprocmask", unimplementedFunc),
    /*  15 */ SyscallDesc("rt_sigreturn", unimplementedFunc),
    /*  16 */ SyscallDesc("ioctl", unimplementedFunc),
    /*  17 */ SyscallDesc("pread64", unimplementedFunc),
    /*  18 */ SyscallDesc("pwrite64", unimplementedFunc),
    /*  19 */ SyscallDesc("readv", unimplementedFunc),
    /*  20 */ SyscallDesc("writev", writevFunc<X86Linux64>),
    /*  21 */ SyscallDesc("access", unimplementedFunc),
    /*  22 */ SyscallDesc("pipe", unimplementedFunc),
    /*  23 */ SyscallDesc("select", unimplementedFunc),
    /*  24 */ SyscallDesc("sched_yield", unimplementedFunc),
    /*  25 */ SyscallDesc("mremap", unimplementedFunc),
    /*  26 */ SyscallDesc("msync", unimplementedFunc),
    /*  27 */ SyscallDesc("mincore", unimplementedFunc),
    /*  28 */ SyscallDesc("madvise", unimplementedFunc),
    /*  29 */ SyscallDesc("shmget", unimplementedFunc),
    /*  30 */ SyscallDesc("shmat", unimplementedFunc),
    /*  31 */ SyscallDesc("shmctl", unimplementedFunc),
    /*  32 */ SyscallDesc("dup", unimplementedFunc),
    /*  33 */ SyscallDesc("dup2", unimplementedFunc),
    /*  34 */ SyscallDesc("pause", unimplementedFunc),
    /*  35 */ SyscallDesc("nanosleep", unimplementedFunc),
    /*  36 */ SyscallDesc("getitimer", unimplementedFunc),
    /*  37 */ SyscallDesc("alarm", unimplementedFunc),
    /*  38 */ SyscallDesc("setitimer", unimplementedFunc),
    /*  39 */ SyscallDesc("getpid", unimplementedFunc),
    /*  40 */ SyscallDesc("sendfile", unimplementedFunc),
    /*  41 */ SyscallDesc("socket", unimplementedFunc),
    /*  42 */ SyscallDesc("connect", unimplementedFunc),
    /*  43 */ SyscallDesc("accept", unimplementedFunc),
    /*  44 */ SyscallDesc("sendto", unimplementedFunc),
    /*  45 */ SyscallDesc("recvfrom", unimplementedFunc),
    /*  46 */ SyscallDesc("sendmsg", unimplementedFunc),
    /*  47 */ SyscallDesc("recvmsg", unimplementedFunc),
    /*  48 */ SyscallDesc("shutdown", unimplementedFunc),
    /*  49 */ SyscallDesc("bind", unimplementedFunc),
    /*  50 */ SyscallDesc("listen", unimplementedFunc),
    /*  51 */ SyscallDesc("getsockname", unimplementedFunc),
    /*  52 */ SyscallDesc("getpeername", unimplementedFunc),
    /*  53 */ SyscallDesc("socketpair", unimplementedFunc),
    /*  54 */ SyscallDesc("setsockopt", unimplementedFunc),
    /*  55 */ SyscallDesc("getsockopt", unimplementedFunc),
    /*  56 */ SyscallDesc("clone", unimplementedFunc),
    /*  57 */ SyscallDesc("fork", unimplementedFunc),
    /*  58 */ SyscallDesc("vfork", unimplementedFunc),
    /*  59 */ SyscallDesc("execve", unimplementedFunc),
    /*  60 */ SyscallDesc("exit", unimplementedFunc),
    /*  61 */ SyscallDesc("wait4", unimplementedFunc),
    /*  62 */ SyscallDesc("kill", unimplementedFunc),
    /*  63 */ SyscallDesc("uname", unameFunc),
    /*  64 */ SyscallDesc("semget", unimplementedFunc),
    /*  65 */ SyscallDesc("semop", unimplementedFunc),
    /*  66 */ SyscallDesc("semctl", unimplementedFunc),
    /*  67 */ SyscallDesc("shmdt", unimplementedFunc),
    /*  68 */ SyscallDesc("msgget", unimplementedFunc),
    /*  69 */ SyscallDesc("msgsnd", unimplementedFunc),
    /*  70 */ SyscallDesc("msgrcv", unimplementedFunc),
    /*  71 */ SyscallDesc("msgctl", unimplementedFunc),
    /*  72 */ SyscallDesc("fcntl", unimplementedFunc),
    /*  73 */ SyscallDesc("flock", unimplementedFunc),
    /*  74 */ SyscallDesc("fsync", unimplementedFunc),
    /*  75 */ SyscallDesc("fdatasync", unimplementedFunc),
    /*  76 */ SyscallDesc("truncate", unimplementedFunc),
    /*  77 */ SyscallDesc("ftruncate", unimplementedFunc),
    /*  78 */ SyscallDesc("getdents", unimplementedFunc),
    /*  79 */ SyscallDesc("getcwd", unimplementedFunc),
    /*  80 */ SyscallDesc("chdir", unimplementedFunc),
    /*  81 */ SyscallDesc("fchdir", unimplementedFunc),
    /*  82 */ SyscallDesc("rename", unimplementedFunc),
    /*  83 */ SyscallDesc("mkdir", unimplementedFunc),
    /*  84 */ SyscallDesc("rmdir", unimplementedFunc),
    /*  85 */ SyscallDesc("creat", unimplementedFunc),
    /*  86 */ SyscallDesc("link", unimplementedFunc),
    /*  87 */ SyscallDesc("unlink", unimplementedFunc),
    /*  88 */ SyscallDesc("symlink", unimplementedFunc),
    /*  89 */ SyscallDesc("readlink", unimplementedFunc),
    /*  90 */ SyscallDesc("chmod", unimplementedFunc),
    /*  91 */ SyscallDesc("fchmod", unimplementedFunc),
    /*  92 */ SyscallDesc("chown", unimplementedFunc),
    /*  93 */ SyscallDesc("fchown", unimplementedFunc),
    /*  94 */ SyscallDesc("lchown", unimplementedFunc),
    /*  95 */ SyscallDesc("umask", unimplementedFunc),
    /*  96 */ SyscallDesc("gettimeofday", unimplementedFunc),
    /*  97 */ SyscallDesc("getrlimit", unimplementedFunc),
    /*  98 */ SyscallDesc("getrusage", unimplementedFunc),
    /*  99 */ SyscallDesc("sysinfo", unimplementedFunc),
    /* 100 */ SyscallDesc("times", unimplementedFunc),
    /* 101 */ SyscallDesc("ptrace", unimplementedFunc),
    /* 102 */ SyscallDesc("getuid", getuidFunc),
    /* 103 */ SyscallDesc("syslog", unimplementedFunc),
    /* 104 */ SyscallDesc("getgid", getgidFunc),
    /* 105 */ SyscallDesc("setuid", unimplementedFunc),
    /* 106 */ SyscallDesc("setgid", unimplementedFunc),
    /* 107 */ SyscallDesc("geteuid", geteuidFunc),
    /* 108 */ SyscallDesc("getegid", getegidFunc),
    /* 109 */ SyscallDesc("setpgid", unimplementedFunc),
    /* 110 */ SyscallDesc("getppid", unimplementedFunc),
    /* 111 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 112 */ SyscallDesc("setsid", unimplementedFunc),
    /* 113 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 114 */ SyscallDesc("setregid", unimplementedFunc),
    /* 115 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 116 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 117 */ SyscallDesc("setresuid", unimplementedFunc),
    /* 118 */ SyscallDesc("getresuid", unimplementedFunc),
    /* 119 */ SyscallDesc("setresgid", unimplementedFunc),
    /* 120 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 121 */ SyscallDesc("getpgid", unimplementedFunc),
    /* 122 */ SyscallDesc("setfsuid", unimplementedFunc),
    /* 123 */ SyscallDesc("setfsgid", unimplementedFunc),
    /* 124 */ SyscallDesc("getsid", unimplementedFunc),
    /* 125 */ SyscallDesc("capget", unimplementedFunc),
    /* 126 */ SyscallDesc("capset", unimplementedFunc),
    /* 127 */ SyscallDesc("rt_sigpending", unimplementedFunc),
    /* 128 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 129 */ SyscallDesc("rt_sigqueueinfo", unimplementedFunc),
    /* 130 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 131 */ SyscallDesc("sigaltstack", unimplementedFunc),
    /* 132 */ SyscallDesc("utime", unimplementedFunc),
    /* 133 */ SyscallDesc("mknod", unimplementedFunc),
    /* 134 */ SyscallDesc("uselib", unimplementedFunc),
    /* 135 */ SyscallDesc("personality", unimplementedFunc),
    /* 136 */ SyscallDesc("ustat", unimplementedFunc),
    /* 137 */ SyscallDesc("statfs", unimplementedFunc),
    /* 138 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 139 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 140 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 141 */ SyscallDesc("setpriority", unimplementedFunc),
    /* 142 */ SyscallDesc("sched_setparam", unimplementedFunc),
    /* 143 */ SyscallDesc("sched_getparam", unimplementedFunc),
    /* 144 */ SyscallDesc("sched_setscheduler", unimplementedFunc),
    /* 145 */ SyscallDesc("sched_getscheduler", unimplementedFunc),
    /* 146 */ SyscallDesc("sched_get_priority_max", unimplementedFunc),
    /* 147 */ SyscallDesc("sched_get_priority_min", unimplementedFunc),
    /* 148 */ SyscallDesc("sched_rr_get_interval", unimplementedFunc),
    /* 149 */ SyscallDesc("mlock", unimplementedFunc),
    /* 150 */ SyscallDesc("munlock", unimplementedFunc),
    /* 151 */ SyscallDesc("mlockall", unimplementedFunc),
    /* 152 */ SyscallDesc("munlockall", unimplementedFunc),
    /* 153 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 154 */ SyscallDesc("modify_ldt", unimplementedFunc),
    /* 155 */ SyscallDesc("pivot_root", unimplementedFunc),
    /* 156 */ SyscallDesc("_sysctl", unimplementedFunc),
    /* 157 */ SyscallDesc("prctl", unimplementedFunc),
    /* 158 */ SyscallDesc("arch_prctl", unimplementedFunc),
    /* 159 */ SyscallDesc("adjtimex", unimplementedFunc),
    /* 160 */ SyscallDesc("setrlimit", unimplementedFunc),
    /* 161 */ SyscallDesc("chroot", unimplementedFunc),
    /* 162 */ SyscallDesc("sync", unimplementedFunc),
    /* 163 */ SyscallDesc("acct", unimplementedFunc),
    /* 164 */ SyscallDesc("settimeofday", unimplementedFunc),
    /* 165 */ SyscallDesc("mount", unimplementedFunc),
    /* 166 */ SyscallDesc("umount2", unimplementedFunc),
    /* 167 */ SyscallDesc("swapon", unimplementedFunc),
    /* 168 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 169 */ SyscallDesc("reboot", unimplementedFunc),
    /* 170 */ SyscallDesc("sethostname", unimplementedFunc),
    /* 171 */ SyscallDesc("setdomainname", unimplementedFunc),
    /* 172 */ SyscallDesc("iopl", unimplementedFunc),
    /* 173 */ SyscallDesc("ioperm", unimplementedFunc),
    /* 174 */ SyscallDesc("create_module", unimplementedFunc),
    /* 175 */ SyscallDesc("init_module", unimplementedFunc),
    /* 176 */ SyscallDesc("delete_module", unimplementedFunc),
    /* 177 */ SyscallDesc("get_kernel_syms", unimplementedFunc),
    /* 178 */ SyscallDesc("query_module", unimplementedFunc),
    /* 179 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 180 */ SyscallDesc("nfsservctl", unimplementedFunc),
    /* 181 */ SyscallDesc("getpmsg", unimplementedFunc),
    /* 182 */ SyscallDesc("putpmsg", unimplementedFunc),
    /* 183 */ SyscallDesc("afs_syscall", unimplementedFunc),
    /* 184 */ SyscallDesc("tuxcall", unimplementedFunc),
    /* 185 */ SyscallDesc("security", unimplementedFunc),
    /* 186 */ SyscallDesc("gettid", unimplementedFunc),
    /* 187 */ SyscallDesc("readahead", unimplementedFunc),
    /* 188 */ SyscallDesc("setxattr", unimplementedFunc),
    /* 189 */ SyscallDesc("lsetxattr", unimplementedFunc),
    /* 190 */ SyscallDesc("fsetxattr", unimplementedFunc),
    /* 191 */ SyscallDesc("getxattr", unimplementedFunc),
    /* 192 */ SyscallDesc("lgetxattr", unimplementedFunc),
    /* 193 */ SyscallDesc("fgetxattr", unimplementedFunc),
    /* 194 */ SyscallDesc("listxattr", unimplementedFunc),
    /* 195 */ SyscallDesc("llistxattr", unimplementedFunc),
    /* 196 */ SyscallDesc("flistxattr", unimplementedFunc),
    /* 197 */ SyscallDesc("removexattr", unimplementedFunc),
    /* 198 */ SyscallDesc("lremovexattr", unimplementedFunc),
    /* 199 */ SyscallDesc("fremovexattr", unimplementedFunc),
    /* 200 */ SyscallDesc("tkill", unimplementedFunc),
    /* 201 */ SyscallDesc("time", unimplementedFunc),
    /* 202 */ SyscallDesc("futex", unimplementedFunc),
    /* 203 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 204 */ SyscallDesc("sched_getaffinity", unimplementedFunc),
    /* 205 */ SyscallDesc("set_thread_area", unimplementedFunc),
    /* 206 */ SyscallDesc("io_setup", unimplementedFunc),
    /* 207 */ SyscallDesc("io_destroy", unimplementedFunc),
    /* 208 */ SyscallDesc("io_getevents", unimplementedFunc),
    /* 209 */ SyscallDesc("io_submit", unimplementedFunc),
    /* 210 */ SyscallDesc("io_cancel", unimplementedFunc),
    /* 211 */ SyscallDesc("get_thread_area", unimplementedFunc),
    /* 212 */ SyscallDesc("lookup_dcookie", unimplementedFunc),
    /* 213 */ SyscallDesc("epoll_create", unimplementedFunc),
    /* 214 */ SyscallDesc("epoll_ctl_old", unimplementedFunc),
    /* 215 */ SyscallDesc("epoll_wait_old", unimplementedFunc),
    /* 216 */ SyscallDesc("remap_file_pages", unimplementedFunc),
    /* 217 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 218 */ SyscallDesc("set_tid_address", unimplementedFunc),
    /* 219 */ SyscallDesc("restart_syscall", unimplementedFunc),
    /* 220 */ SyscallDesc("semtimedop", unimplementedFunc),
    /* 221 */ SyscallDesc("fadvise64", unimplementedFunc),
    /* 222 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 223 */ SyscallDesc("timer_settime", unimplementedFunc),
    /* 224 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 225 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 226 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 227 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 228 */ SyscallDesc("clock_gettime", unimplementedFunc),
    /* 229 */ SyscallDesc("clock_getres", unimplementedFunc),
    /* 230 */ SyscallDesc("clock_nanosleep", unimplementedFunc),
    /* 231 */ SyscallDesc("exit_group", unimplementedFunc),
    /* 232 */ SyscallDesc("epoll_wait", unimplementedFunc),
    /* 233 */ SyscallDesc("epoll_ctl", unimplementedFunc),
    /* 234 */ SyscallDesc("tgkill", unimplementedFunc),
    /* 235 */ SyscallDesc("utimes", unimplementedFunc),
    /* 236 */ SyscallDesc("vserver", unimplementedFunc),
    /* 237 */ SyscallDesc("mbind", unimplementedFunc),
    /* 238 */ SyscallDesc("set_mempolicy", unimplementedFunc),
    /* 239 */ SyscallDesc("get_mempolicy", unimplementedFunc),
    /* 240 */ SyscallDesc("mq_open", unimplementedFunc),
    /* 241 */ SyscallDesc("mq_unlink", unimplementedFunc),
    /* 242 */ SyscallDesc("mq_timedsend", unimplementedFunc),
    /* 243 */ SyscallDesc("mq_timedreceive", unimplementedFunc),
    /* 244 */ SyscallDesc("mq_notify", unimplementedFunc),
    /* 245 */ SyscallDesc("mq_getsetattr", unimplementedFunc),
    /* 246 */ SyscallDesc("kexec_load", unimplementedFunc),
    /* 247 */ SyscallDesc("waitid", unimplementedFunc),
    /* 248 */ SyscallDesc("add_key", unimplementedFunc),
    /* 249 */ SyscallDesc("request_key", unimplementedFunc),
    /* 250 */ SyscallDesc("keyctl", unimplementedFunc),
    /* 251 */ SyscallDesc("ioprio_set", unimplementedFunc),
    /* 252 */ SyscallDesc("ioprio_get", unimplementedFunc),
    /* 253 */ SyscallDesc("inotify_init", unimplementedFunc),
    /* 254 */ SyscallDesc("inotify_add_watch", unimplementedFunc),
    /* 255 */ SyscallDesc("inotify_rm_watch", unimplementedFunc),
    /* 256 */ SyscallDesc("migrate_pages", unimplementedFunc),
    /* 257 */ SyscallDesc("openat", unimplementedFunc),
    /* 258 */ SyscallDesc("mkdirat", unimplementedFunc),
    /* 259 */ SyscallDesc("mknodat", unimplementedFunc),
    /* 260 */ SyscallDesc("fchownat", unimplementedFunc),
    /* 261 */ SyscallDesc("futimesat", unimplementedFunc),
    /* 262 */ SyscallDesc("newfstatat", unimplementedFunc),
    /* 263 */ SyscallDesc("unlinkat", unimplementedFunc),
    /* 264 */ SyscallDesc("renameat", unimplementedFunc),
    /* 265 */ SyscallDesc("linkat", unimplementedFunc),
    /* 266 */ SyscallDesc("symlinkat", unimplementedFunc),
    /* 267 */ SyscallDesc("readlinkat", unimplementedFunc),
    /* 268 */ SyscallDesc("fchmodat", unimplementedFunc),
    /* 269 */ SyscallDesc("faccessat", unimplementedFunc),
    /* 270 */ SyscallDesc("pselect6", unimplementedFunc),
    /* 271 */ SyscallDesc("ppoll", unimplementedFunc),
    /* 272 */ SyscallDesc("unshare", unimplementedFunc)
};
