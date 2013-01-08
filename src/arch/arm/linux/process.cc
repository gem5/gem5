/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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
 *          Stephen Hines
 *          Ali Saidi
 */

#include "arch/arm/linux/linux.hh"
#include "arch/arm/linux/process.hh"
#include "arch/arm/isa_traits.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace ArmISA;

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
          ThreadContext *tc)
{
    int index = 0;
    TypedBufferArg<Linux::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "m5.eecs.umich.edu");
    strcpy(name->release, "3.0.0");
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "armv7l");

    name.copyOut(tc->getMemProxy());
    return 0;
}

SyscallDesc ArmLinuxProcess::syscallDescs[] = {
    /*  0 */ SyscallDesc("syscall", unimplementedFunc),
    /*  1 */ SyscallDesc("exit", exitFunc),
    /*  2 */ SyscallDesc("fork", unimplementedFunc),
    /*  3 */ SyscallDesc("read", readFunc),
    /*  4 */ SyscallDesc("write", writeFunc),
    /*  5 */ SyscallDesc("open", openFunc<ArmLinux>),
    /*  6 */ SyscallDesc("close", closeFunc),
    /*  7 */ SyscallDesc("unused#7", unimplementedFunc),
    /*  8 */ SyscallDesc("creat", unimplementedFunc),
    /*  9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unlinkFunc),
    /* 11 */ SyscallDesc("execve", unimplementedFunc),
    /* 12 */ SyscallDesc("chdir", unimplementedFunc),
    /* 13 */ SyscallDesc("time", timeFunc<ArmLinux>),
    /* 14 */ SyscallDesc("mknod", unimplementedFunc),
    /* 15 */ SyscallDesc("chmod", chmodFunc<ArmLinux>),
    /* 16 */ SyscallDesc("lchown", chownFunc),
    /* 17 */ SyscallDesc("unused#17", unimplementedFunc),
    /* 18 */ SyscallDesc("unused#18", unimplementedFunc),
    /* 19 */ SyscallDesc("lseek", lseekFunc),
    /* 20 */ SyscallDesc("getpid", getpidFunc),
    /* 21 */ SyscallDesc("mount", unimplementedFunc),
    /* 22 */ SyscallDesc("umount", unimplementedFunc),
    /* 23 */ SyscallDesc("setuid", setuidFunc),
    /* 24 */ SyscallDesc("getuid", getuidFunc),
    /* 25 */ SyscallDesc("stime", unimplementedFunc),
    /* 26 */ SyscallDesc("ptrace", unimplementedFunc),
    /* 27 */ SyscallDesc("alarm", unimplementedFunc),
    /* 28 */ SyscallDesc("unused#28", unimplementedFunc),
    /* 29 */ SyscallDesc("pause", unimplementedFunc),
    /* 30 */ SyscallDesc("utime", unimplementedFunc),
    /* 31 */ SyscallDesc("unused#31", unimplementedFunc),
    /* 32 */ SyscallDesc("unused#32", unimplementedFunc),
    /* 33 */ SyscallDesc("access", accessFunc),
    /* 34 */ SyscallDesc("nice", unimplementedFunc),
    /* 35 */ SyscallDesc("unused#35", unimplementedFunc),
    /* 36 */ SyscallDesc("sync", unimplementedFunc),
    /* 37 */ SyscallDesc("kill", ignoreFunc),
    /* 38 */ SyscallDesc("rename", renameFunc),
    /* 39 */ SyscallDesc("mkdir", mkdirFunc),
    /* 40 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 41 */ SyscallDesc("dup", dupFunc),
    /* 42 */ SyscallDesc("pipe", pipePseudoFunc),
    /* 43 */ SyscallDesc("times", timesFunc<ArmLinux>),
    /* 44 */ SyscallDesc("unused#44", unimplementedFunc),
    /* 45 */ SyscallDesc("brk", brkFunc),
    /* 46 */ SyscallDesc("setgid", unimplementedFunc),
    /* 47 */ SyscallDesc("getgid", getgidFunc),
    /* 48 */ SyscallDesc("unused#48", unimplementedFunc),
    /* 49 */ SyscallDesc("geteuid", geteuidFunc),
    /* 50 */ SyscallDesc("getegid", getegidFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("umount2", unimplementedFunc),
    /* 53 */ SyscallDesc("unused#53", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", ioctlFunc<ArmLinux>),
    /* 55 */ SyscallDesc("fcntl", fcntlFunc),
    /* 56 */ SyscallDesc("unused#56", unimplementedFunc),
    /* 57 */ SyscallDesc("setpgid", unimplementedFunc),
    /* 58 */ SyscallDesc("unused#58", unimplementedFunc),
    /* 59 */ SyscallDesc("unused#59", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", unimplementedFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("ustat", unimplementedFunc),
    /* 63 */ SyscallDesc("dup2", unimplementedFunc),
    /* 64 */ SyscallDesc("getppid", getppidFunc),
    /* 65 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 66 */ SyscallDesc("setsid", unimplementedFunc),
    /* 67 */ SyscallDesc("sigaction",unimplementedFunc),
    /* 68 */ SyscallDesc("unused#68", unimplementedFunc),
    /* 69 */ SyscallDesc("unused#69", unimplementedFunc),
    /* 70 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 71 */ SyscallDesc("setregid", unimplementedFunc),
    /* 72 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 73 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 74 */ SyscallDesc("sethostname", ignoreFunc),
    /* 75 */ SyscallDesc("setrlimit", ignoreFunc),
    /* 76 */ SyscallDesc("getrlimit", getrlimitFunc<ArmLinux>),
    /* 77 */ SyscallDesc("getrusage", getrusageFunc<ArmLinux>),
    /* 78 */ SyscallDesc("gettimeofday", gettimeofdayFunc<ArmLinux>),
    /* 79 */ SyscallDesc("settimeofday", unimplementedFunc),
    /* 80 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 81 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 82 */ SyscallDesc("reserved#82", unimplementedFunc),
    /* 83 */ SyscallDesc("symlink", unimplementedFunc),
    /* 84 */ SyscallDesc("unused#84", unimplementedFunc),
    /* 85 */ SyscallDesc("readlink", readlinkFunc),
    /* 86 */ SyscallDesc("uselib", unimplementedFunc),
    /* 87 */ SyscallDesc("swapon", unimplementedFunc),
    /* 88 */ SyscallDesc("reboot", unimplementedFunc),
    /* 89 */ SyscallDesc("readdir", unimplementedFunc),
    /* 90 */ SyscallDesc("mmap", mmapFunc<ArmLinux>),
    /* 91 */ SyscallDesc("munmap", munmapFunc),
    /* 92 */ SyscallDesc("truncate", truncateFunc),
    /* 93 */ SyscallDesc("ftruncate", ftruncateFunc),
    /* 94 */ SyscallDesc("fchmod", unimplementedFunc),
    /* 95 */ SyscallDesc("fchown", unimplementedFunc),
    /* 96 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 97 */ SyscallDesc("setpriority", unimplementedFunc),
    /* 98 */ SyscallDesc("unused#98", unimplementedFunc),
    /* 99 */ SyscallDesc("statfs", unimplementedFunc),
    /* 100 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 101 */ SyscallDesc("unused#101", unimplementedFunc),
    /* 102 */ SyscallDesc("socketcall", unimplementedFunc),
    /* 103 */ SyscallDesc("syslog", unimplementedFunc),
    /* 104 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 105 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 106 */ SyscallDesc("stat",  statFunc<ArmLinux>),
    /* 107 */ SyscallDesc("lstat", unimplementedFunc),
    /* 108 */ SyscallDesc("fstat", fstatFunc<ArmLinux>),
    /* 109 */ SyscallDesc("unused#109", unimplementedFunc),
    /* 110 */ SyscallDesc("unused#101", unimplementedFunc),
    /* 111 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 112 */ SyscallDesc("unused#112", unimplementedFunc),
    /* 113 */ SyscallDesc("syscall", unimplementedFunc),
    /* 114 */ SyscallDesc("wait4", unimplementedFunc),
    /* 115 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 116 */ SyscallDesc("sysinfo", sysinfoFunc<ArmLinux>),
    /* 117 */ SyscallDesc("ipc", unimplementedFunc),
    /* 118 */ SyscallDesc("fsync", unimplementedFunc),
    /* 119 */ SyscallDesc("sigreturn", unimplementedFunc),
    /* 120 */ SyscallDesc("clone", cloneFunc),
    /* 121 */ SyscallDesc("setdomainname", unimplementedFunc),
    /* 122 */ SyscallDesc("uname", unameFunc),
    /* 123 */ SyscallDesc("unused#123", unimplementedFunc),
    /* 124 */ SyscallDesc("adjtimex", unimplementedFunc),
    /* 125 */ SyscallDesc("mprotect", ignoreFunc),
    /* 126 */ SyscallDesc("sigprocmask", unimplementedFunc),
    /* 127 */ SyscallDesc("unused#127", unimplementedFunc),
    /* 128 */ SyscallDesc("init_module", unimplementedFunc),
    /* 129 */ SyscallDesc("delete_module", unimplementedFunc),
    /* 130 */ SyscallDesc("unused#130", unimplementedFunc),
    /* 131 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 132 */ SyscallDesc("getpgid", unimplementedFunc),
    /* 133 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 134 */ SyscallDesc("bdflush", unimplementedFunc),
    /* 135 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 136 */ SyscallDesc("personality", unimplementedFunc),
    /* 137 */ SyscallDesc("reserved#138", unimplementedFunc),
    /* 138 */ SyscallDesc("setfsuid", unimplementedFunc),
    /* 139 */ SyscallDesc("setfsgid", unimplementedFunc),
    /* 140 */ SyscallDesc("llseek", _llseekFunc),
    /* 141 */ SyscallDesc("getdents", unimplementedFunc),
    /* 142 */ SyscallDesc("newselect", unimplementedFunc),
    /* 143 */ SyscallDesc("flock", unimplementedFunc),
    /* 144 */ SyscallDesc("msync", unimplementedFunc),
    /* 145 */ SyscallDesc("readv", unimplementedFunc),
    /* 146 */ SyscallDesc("writev", writevFunc<ArmLinux>),
    /* 147 */ SyscallDesc("getsid", unimplementedFunc),
    /* 148 */ SyscallDesc("fdatasync", unimplementedFunc),
    /* 149 */ SyscallDesc("sysctl", unimplementedFunc),
    /* 150 */ SyscallDesc("mlock", unimplementedFunc),
    /* 151 */ SyscallDesc("munlock", unimplementedFunc),
    /* 152 */ SyscallDesc("mlockall", unimplementedFunc),
    /* 153 */ SyscallDesc("munlockall", unimplementedFunc),
    /* 154 */ SyscallDesc("sched_setparam", unimplementedFunc),
    /* 155 */ SyscallDesc("sched_getparam", unimplementedFunc),
    /* 156 */ SyscallDesc("sched_setscheduler", unimplementedFunc),
    /* 157 */ SyscallDesc("sched_getscheduler", unimplementedFunc),
    /* 158 */ SyscallDesc("sched_yield", unimplementedFunc),
    /* 159 */ SyscallDesc("sched_get_priority_max", unimplementedFunc),
    /* 160 */ SyscallDesc("sched_get_priority_min", unimplementedFunc),
    /* 161 */ SyscallDesc("sched_rr_get_interval", unimplementedFunc),
    /* 162 */ SyscallDesc("nanosleep", ignoreWarnOnceFunc),
    /* 163 */ SyscallDesc("mremap", mremapFunc<ArmLinux>), // ARM-specific
    /* 164 */ SyscallDesc("setresuid", unimplementedFunc),
    /* 165 */ SyscallDesc("getresuid", unimplementedFunc),
    /* 166 */ SyscallDesc("unused#166", unimplementedFunc),
    /* 167 */ SyscallDesc("unused#167", unimplementedFunc),
    /* 168 */ SyscallDesc("poll", unimplementedFunc),
    /* 169 */ SyscallDesc("nfsservctl", unimplementedFunc),
    /* 170 */ SyscallDesc("setresgid", unimplementedFunc),
    /* 171 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 172 */ SyscallDesc("prctl", unimplementedFunc),
    /* 173 */ SyscallDesc("rt_sigreturn", unimplementedFunc),
    /* 174 */ SyscallDesc("rt_sigaction", ignoreWarnOnceFunc),
    /* 175 */ SyscallDesc("rt_sigprocmask", ignoreWarnOnceFunc),
    /* 176 */ SyscallDesc("rt_sigpending", unimplementedFunc),
    /* 177 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 178 */ SyscallDesc("rt_sigqueueinfo", ignoreFunc),
    /* 179 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 180 */ SyscallDesc("pread64", unimplementedFunc),
    /* 181 */ SyscallDesc("pwrite64", unimplementedFunc),
    /* 182 */ SyscallDesc("chown", unimplementedFunc),
    /* 183 */ SyscallDesc("getcwd", getcwdFunc),
    /* 184 */ SyscallDesc("capget", unimplementedFunc),
    /* 185 */ SyscallDesc("capset", unimplementedFunc),
    /* 186 */ SyscallDesc("sigaltstack", unimplementedFunc),
    /* 187 */ SyscallDesc("sendfile", unimplementedFunc),
    /* 188 */ SyscallDesc("unused#188", unimplementedFunc),
    /* 189 */ SyscallDesc("unused#189", unimplementedFunc),
    /* 190 */ SyscallDesc("vfork", unimplementedFunc),
    /* 191 */ SyscallDesc("getrlimit", getrlimitFunc<ArmLinux>),
    /* 192 */ SyscallDesc("mmap2", mmapFunc<ArmLinux>),
    /* 193 */ SyscallDesc("truncate64", unimplementedFunc),
    /* 194 */ SyscallDesc("ftruncate64", ftruncate64Func),
    /* 195 */ SyscallDesc("stat64", stat64Func<ArmLinux>),
    /* 196 */ SyscallDesc("lstat64", lstat64Func<ArmLinux>),
    /* 197 */ SyscallDesc("fstat64", fstat64Func<ArmLinux>),
    /* 198 */ SyscallDesc("lchown", unimplementedFunc),
    /* 199 */ SyscallDesc("getuid", getuidFunc),
    /* 200 */ SyscallDesc("getgid", getgidFunc),
    /* 201 */ SyscallDesc("geteuid", geteuidFunc),
    /* 202 */ SyscallDesc("getegid", getegidFunc),
    /* 203 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 204 */ SyscallDesc("setregid", unimplementedFunc),
    /* 205 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 206 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 207 */ SyscallDesc("fchown", unimplementedFunc),
    /* 208 */ SyscallDesc("setresuid", unimplementedFunc),
    /* 209 */ SyscallDesc("getresuid", unimplementedFunc),
    /* 210 */ SyscallDesc("setresgid", unimplementedFunc),
    /* 211 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 212 */ SyscallDesc("chown", unimplementedFunc),
    /* 213 */ SyscallDesc("setuid", unimplementedFunc),
    /* 214 */ SyscallDesc("setgid", unimplementedFunc),
    /* 215 */ SyscallDesc("setfsuid", unimplementedFunc),
    /* 216 */ SyscallDesc("setfsgid", unimplementedFunc),
    /* 217 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 218 */ SyscallDesc("pivot_root", unimplementedFunc),
    /* 219 */ SyscallDesc("mincore", unimplementedFunc),
    /* 220 */ SyscallDesc("madvise", unimplementedFunc),
    /* 221 */ SyscallDesc("fcntl64", fcntl64Func),
    /* 222 */ SyscallDesc("unused#222", unimplementedFunc),
    /* 223 */ SyscallDesc("unknown#223", unimplementedFunc),
    /* 224 */ SyscallDesc("gettid", unimplementedFunc),
    /* 225 */ SyscallDesc("readahead", unimplementedFunc),
    /* 226 */ SyscallDesc("setxattr", unimplementedFunc),
    /* 227 */ SyscallDesc("lsetxattr", unimplementedFunc),
    /* 228 */ SyscallDesc("fsetxattr", unimplementedFunc),
    /* 229 */ SyscallDesc("getxattr", unimplementedFunc),
    /* 230 */ SyscallDesc("lgetxattr", unimplementedFunc),
    /* 231 */ SyscallDesc("fgetxattr", unimplementedFunc),
    /* 232 */ SyscallDesc("listxattr", unimplementedFunc),
    /* 233 */ SyscallDesc("llistxattr", unimplementedFunc),
    /* 234 */ SyscallDesc("flistxattr", unimplementedFunc),
    /* 235 */ SyscallDesc("removexattr", unimplementedFunc),
    /* 236 */ SyscallDesc("lremovexattr", unimplementedFunc),
    /* 237 */ SyscallDesc("fremovexattr", unimplementedFunc),
    /* 238 */ SyscallDesc("tkill", unimplementedFunc),
    /* 239 */ SyscallDesc("sendfile64", unimplementedFunc),
    /* 240 */ SyscallDesc("futex", ignoreWarnOnceFunc),
    /* 241 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 242 */ SyscallDesc("sched_getaffinity", unimplementedFunc),
    /* 243 */ SyscallDesc("io_setup", unimplementedFunc),
    /* 244 */ SyscallDesc("io_destory", unimplementedFunc),
    /* 245 */ SyscallDesc("io_getevents", unimplementedFunc),
    /* 246 */ SyscallDesc("io_submit", unimplementedFunc),
    /* 247 */ SyscallDesc("io_cancel", unimplementedFunc),
    /* 248 */ SyscallDesc("exit_group", exitGroupFunc),
    /* 249 */ SyscallDesc("lookup_dcookie", unimplementedFunc),
    /* 250 */ SyscallDesc("epoll_create", unimplementedFunc),
    /* 251 */ SyscallDesc("epoll_ctl", unimplementedFunc),
    /* 252 */ SyscallDesc("epoll_wait", unimplementedFunc),
    /* 253 */ SyscallDesc("remap_file_pages", unimplementedFunc),
    /* 254 */ SyscallDesc("unused#254", unimplementedFunc),
    /* 255 */ SyscallDesc("unused#255", unimplementedFunc),
    /* 256 */ SyscallDesc("set_tid_address", unimplementedFunc),
    /* 257 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 258 */ SyscallDesc("timer_settime", unimplementedFunc),
    /* 259 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 260 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 261 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 262 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 263 */ SyscallDesc("clock_gettime", unimplementedFunc),
    /* 264 */ SyscallDesc("clock_getres", unimplementedFunc),
    /* 265 */ SyscallDesc("clock_nanosleep", unimplementedFunc),
    /* 266 */ SyscallDesc("statfs64", unimplementedFunc),
    /* 267 */ SyscallDesc("fstatfs64", unimplementedFunc),
    /* 268 */ SyscallDesc("tgkill", unimplementedFunc),
    /* 269 */ SyscallDesc("utimes", unimplementedFunc),
    /* 270 */ SyscallDesc("arm_fadvise64_64", unimplementedFunc),
    /* 271 */ SyscallDesc("pciconfig_iobase", unimplementedFunc),
    /* 272 */ SyscallDesc("pciconfig_read", unimplementedFunc),
    /* 273 */ SyscallDesc("pciconfig_write", unimplementedFunc),
    /* 274 */ SyscallDesc("mq_open", unimplementedFunc),
    /* 275 */ SyscallDesc("mq_unlink", unimplementedFunc),
    /* 276 */ SyscallDesc("mq_timedsend", unimplementedFunc),
    /* 277 */ SyscallDesc("mq_timedreceive", unimplementedFunc),
    /* 278 */ SyscallDesc("mq_notify", unimplementedFunc),
    /* 279 */ SyscallDesc("mq_getsetattr", unimplementedFunc),
    /* 280 */ SyscallDesc("waitid", unimplementedFunc),
    /* 281 */ SyscallDesc("socket", unimplementedFunc),
    /* 282 */ SyscallDesc("bind", unimplementedFunc),
    /* 283 */ SyscallDesc("connect", unimplementedFunc),
    /* 284 */ SyscallDesc("listen", unimplementedFunc),
    /* 285 */ SyscallDesc("accept", unimplementedFunc),
    /* 286 */ SyscallDesc("getsockname", unimplementedFunc),
    /* 287 */ SyscallDesc("getpeername", unimplementedFunc),
    /* 288 */ SyscallDesc("socketpair", unimplementedFunc),
    /* 289 */ SyscallDesc("send", unimplementedFunc),
    /* 290 */ SyscallDesc("sendto", unimplementedFunc),
    /* 291 */ SyscallDesc("recv", unimplementedFunc),
    /* 292 */ SyscallDesc("recvfrom", unimplementedFunc),
    /* 293 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 294 */ SyscallDesc("setsockopt", unimplementedFunc),
    /* 295 */ SyscallDesc("getsockopt", unimplementedFunc),
    /* 296 */ SyscallDesc("sendmsg", unimplementedFunc),
    /* 297 */ SyscallDesc("rcvmsg", unimplementedFunc),
    /* 298 */ SyscallDesc("semop", unimplementedFunc),
    /* 299 */ SyscallDesc("semget", unimplementedFunc),
    /* 300 */ SyscallDesc("semctl", unimplementedFunc),
    /* 301 */ SyscallDesc("msgsend", unimplementedFunc),
    /* 302 */ SyscallDesc("msgrcv", unimplementedFunc),
    /* 303 */ SyscallDesc("msgget", unimplementedFunc),
    /* 304 */ SyscallDesc("msgctl", unimplementedFunc),
    /* 305 */ SyscallDesc("shmat", unimplementedFunc),
    /* 306 */ SyscallDesc("shmdt", unimplementedFunc),
    /* 307 */ SyscallDesc("shmget", unimplementedFunc),
    /* 308 */ SyscallDesc("shmctl", unimplementedFunc),
    /* 309 */ SyscallDesc("add_key", unimplementedFunc),
    /* 310 */ SyscallDesc("request_key", unimplementedFunc),
    /* 311 */ SyscallDesc("keyctl", unimplementedFunc),
    /* 312 */ SyscallDesc("semtimedop", unimplementedFunc),
    /* 313 */ SyscallDesc("unused#313", unimplementedFunc),
    /* 314 */ SyscallDesc("ioprio_set", unimplementedFunc),
    /* 315 */ SyscallDesc("ioprio_get", unimplementedFunc),
    /* 316 */ SyscallDesc("inotify_init", unimplementedFunc),
    /* 317 */ SyscallDesc("inotify_add_watch", unimplementedFunc),
    /* 318 */ SyscallDesc("inotify_rm_watch", unimplementedFunc),
    /* 319 */ SyscallDesc("mbind", unimplementedFunc),
    /* 320 */ SyscallDesc("get_mempolicy", unimplementedFunc),
    /* 321 */ SyscallDesc("set_mempolicy", unimplementedFunc),
    /* 322 */ SyscallDesc("openat", unimplementedFunc),
    /* 323 */ SyscallDesc("mkdirat", unimplementedFunc),
    /* 324 */ SyscallDesc("mknodat", unimplementedFunc),
    /* 325 */ SyscallDesc("fchownat", unimplementedFunc),
    /* 326 */ SyscallDesc("futimesat", unimplementedFunc),
    /* 327 */ SyscallDesc("fstatat64", unimplementedFunc),
    /* 328 */ SyscallDesc("unlinkat", unimplementedFunc),
    /* 329 */ SyscallDesc("renameat", unimplementedFunc),
    /* 330 */ SyscallDesc("linkat", unimplementedFunc),
    /* 331 */ SyscallDesc("symlinkat", unimplementedFunc),
    /* 332 */ SyscallDesc("readlinkat", unimplementedFunc),
    /* 333 */ SyscallDesc("fchmodat", unimplementedFunc),
    /* 334 */ SyscallDesc("faccessat", unimplementedFunc),
    /* 335 */ SyscallDesc("pselect6", unimplementedFunc),
    /* 336 */ SyscallDesc("ppoll", unimplementedFunc),
    /* 337 */ SyscallDesc("unshare", unimplementedFunc),
    /* 338 */ SyscallDesc("set_robust_list", unimplementedFunc),
    /* 339 */ SyscallDesc("get_robust_list", unimplementedFunc),
    /* 340 */ SyscallDesc("splice", unimplementedFunc),
    /* 341 */ SyscallDesc("arm_sync_file_range", unimplementedFunc),
    /* 342 */ SyscallDesc("tee", unimplementedFunc),
    /* 343 */ SyscallDesc("vmsplice", unimplementedFunc),
    /* 344 */ SyscallDesc("move_pages", unimplementedFunc),
    /* 345 */ SyscallDesc("getcpu", unimplementedFunc),
    /* 346 */ SyscallDesc("epoll_pwait", unimplementedFunc),
    /* 347 */ SyscallDesc("sys_kexec_load", unimplementedFunc),
    /* 348 */ SyscallDesc("sys_utimensat", unimplementedFunc),
    /* 349 */ SyscallDesc("sys_signalfd", unimplementedFunc),
    /* 350 */ SyscallDesc("sys_timerfd_create", unimplementedFunc),
    /* 351 */ SyscallDesc("sys_eventfd", unimplementedFunc),
    /* 352 */ SyscallDesc("sys_fallocate", unimplementedFunc),
    /* 353 */ SyscallDesc("sys_timerfd_settime", unimplementedFunc),
    /* 354 */ SyscallDesc("sys_timerfd_gettime", unimplementedFunc),
    /* 355 */ SyscallDesc("sys_signalfd4", unimplementedFunc),
    /* 356 */ SyscallDesc("sys_eventfd2", unimplementedFunc),
    /* 357 */ SyscallDesc("sys_epoll_create1", unimplementedFunc),
    /* 358 */ SyscallDesc("sys_dup3", unimplementedFunc),
    /* 359 */ SyscallDesc("sys_pipe2", unimplementedFunc),
    /* 360 */ SyscallDesc("sys_inotify_init1", unimplementedFunc),
    /* 361 */ SyscallDesc("sys_preadv", unimplementedFunc),
    /* 362 */ SyscallDesc("sys_pwritev", unimplementedFunc),
    /* 363 */ SyscallDesc("sys_rt_tgsigqueueinfo", unimplementedFunc),
    /* 364 */ SyscallDesc("sys_perf_event_open", unimplementedFunc),
    /* 365 */ SyscallDesc("sys_recvmmsg", unimplementedFunc),

};

/// Target set_tls() handler.
static SyscallReturn
setTLSFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
          ThreadContext *tc)
{
    int index = 0;
    uint32_t tlsPtr = process->getSyscallArg(tc, index);

    tc->getMemProxy().writeBlob(ArmLinuxProcess::commPage + 0x0ff0,
                                (uint8_t *)&tlsPtr, sizeof(tlsPtr));
    tc->setMiscReg(MISCREG_TPIDRURO,tlsPtr);
    return 0;
}

SyscallDesc ArmLinuxProcess::privSyscallDescs[] = {
    /*  1 */ SyscallDesc("breakpoint", unimplementedFunc),
    /*  2 */ SyscallDesc("cacheflush", unimplementedFunc),
    /*  3 */ SyscallDesc("usr26", unimplementedFunc),
    /*  4 */ SyscallDesc("usr32", unimplementedFunc),
    /*  5 */ SyscallDesc("set_tls", setTLSFunc)
};

ArmLinuxProcess::ArmLinuxProcess(LiveProcessParams * params,
        ObjectFile *objFile, ObjectFile::Arch _arch)
    : ArmLiveProcess(params, objFile, _arch),
     Num_Syscall_Descs(sizeof(syscallDescs) / sizeof(SyscallDesc)),
     Num_Priv_Syscall_Descs(sizeof(privSyscallDescs) / sizeof(SyscallDesc))
{ }

const Addr ArmLinuxProcess::commPage = 0xffff0000;

SyscallDesc*
ArmLinuxProcess::getDesc(int callnum)
{
    // Angel SWI syscalls are unsupported in this release
    if (callnum == 0x123456) {
        panic("Attempt to execute an ANGEL_SWI system call (newlib-related)");
    } else if ((callnum & 0x00f00000) == 0x00900000 || 
            (callnum & 0xf0000) == 0xf0000) {
        callnum &= 0x000fffff;
        if ((callnum & 0x0f0000) == 0xf0000) {
            callnum -= 0x0f0001;
            if (callnum < 0 || callnum > Num_Priv_Syscall_Descs)
                return NULL;
            return &privSyscallDescs[callnum];
        }
    }
    // Linux syscalls have to strip off the 0x00900000

    if (callnum < 0 || callnum > Num_Syscall_Descs)
        return NULL;

    return &syscallDescs[callnum];
}

void
ArmLinuxProcess::initState()
{
    ArmLiveProcess::initState();
    allocateMem(commPage, PageBytes);
    ThreadContext *tc = system->getThreadContext(contextIds[0]);

    uint8_t swiNeg1[] = {
        0xff, 0xff, 0xff, 0xef  // swi -1
    };

    // Fill this page with swi -1 so we'll no if we land in it somewhere.
    for (Addr addr = 0; addr < PageBytes; addr += sizeof(swiNeg1)) {
        tc->getMemProxy().writeBlob(commPage + addr,
                                    swiNeg1, sizeof(swiNeg1));
    }

    uint8_t memory_barrier[] =
    {
        0x5f, 0xf0, 0x7f, 0xf5, // dmb
        0x0e, 0xf0, 0xa0, 0xe1  // return
    };
    tc->getMemProxy().writeBlob(commPage + 0x0fa0, memory_barrier,
                                sizeof(memory_barrier));

    uint8_t cmpxchg[] =
    {
        0x9f, 0x3f, 0x92, 0xe1,  // ldrex    r3, [r2]
        0x00, 0x30, 0x53, 0xe0,  // subs     r3, r3, r0
        0x91, 0x3f, 0x82, 0x01,  // strexeq  r3, r1, [r2]
        0x01, 0x00, 0x33, 0x03,  // teqeq    r3, #1
        0xfa, 0xff, 0xff, 0x0a,  // beq 1b
        0x00, 0x00, 0x73, 0xe2,  // rsbs r0, r3, #0
        0x5f, 0xf0, 0x7f, 0xf5,  // dmb
        0x0e, 0xf0, 0xa0, 0xe1   // return
    };
    tc->getMemProxy().writeBlob(commPage + 0x0fc0, cmpxchg, sizeof(cmpxchg));

    uint8_t get_tls[] =
    {
                                // read user read-only thread id register
        0x70, 0x0f, 0x1d, 0xee, // mrc p15, 0, r0, c13, c0, 3
        0x0e, 0xf0, 0xa0, 0xe1  // return
    };
    tc->getMemProxy().writeBlob(commPage + 0x0fe0, get_tls, sizeof(get_tls));
}

ArmISA::IntReg
ArmLinuxProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    // Linux apparently allows more parameter than the ABI says it should.
    // This limit may need to be increased even further.
    assert(i < 6);
    return tc->readIntReg(ArgumentReg0 + i++);
}

void
ArmLinuxProcess::setSyscallArg(ThreadContext *tc, int i, ArmISA::IntReg val)
{
    // Linux apparently allows more parameter than the ABI says it should.
    // This limit may need to be increased even further.
    assert(i < 6);
    tc->setIntReg(ArgumentReg0 + i, val);
}
