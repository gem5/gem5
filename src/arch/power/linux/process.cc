/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
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
 *          Timothy M. Jones
 */

#include "arch/power/linux/process.hh"

#include "arch/power/isa_traits.hh"
#include "arch/power/linux/linux.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace PowerISA;

namespace
{

class PowerLinuxObjectFileLoader : public ObjectFile::Loader
{
  public:
    Process *
    load(ProcessParams *params, ObjectFile *obj_file) override
    {
        if (obj_file->getArch() != ObjectFile::Power)
            return nullptr;

        auto opsys = obj_file->getOpSys();

        if (opsys == ObjectFile::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = ObjectFile::Linux;
        }

        if (opsys != ObjectFile::Linux)
            return nullptr;

        return new PowerLinuxProcess(params, obj_file);
    }
};

PowerLinuxObjectFileLoader loader;

} // anonymous namespace

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, ThreadContext *tc)
{
    int index = 0;
    auto process = tc->getProcessPtr();
    TypedBufferArg<Linux::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "power");

    name.copyOut(tc->getVirtProxy());
    return 0;
}

SyscallDesc PowerLinuxProcess::syscallDescs[] = {
    /*  0 */ SyscallDesc("syscall", unimplementedFunc),
    /*  1 */ SyscallDesc("exit", exitFunc),
    /*  2 */ SyscallDesc("fork", unimplementedFunc),
    /*  3 */ SyscallDesc("read", readFunc<PowerLinux>),
    /*  4 */ SyscallDesc("write", writeFunc<PowerLinux>),
    /*  5 */ SyscallDesc("open", openFunc<PowerLinux>),
    /*  6 */ SyscallDesc("close", closeFunc),
    /*  7 */ SyscallDesc("waitpid", unimplementedFunc), //???
    /*  8 */ SyscallDesc("creat", unimplementedFunc),
    /*  9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unlinkFunc),
    /* 11 */ SyscallDesc("execve", unimplementedFunc),
    /* 12 */ SyscallDesc("chdir", unimplementedFunc),
    /* 13 */ SyscallDesc("time", timeFunc<PowerLinux>),
    /* 14 */ SyscallDesc("mknod", unimplementedFunc),
    /* 15 */ SyscallDesc("chmod", chmodFunc<PowerLinux>),
    /* 16 */ SyscallDesc("lchown", chownFunc),
    /* 17 */ SyscallDesc("break", brkFunc), //???
    /* 18 */ SyscallDesc("unused#18", unimplementedFunc), //???
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
    /* 31 */ SyscallDesc("stty", unimplementedFunc),
    /* 32 */ SyscallDesc("gtty", unimplementedFunc),
    /* 33 */ SyscallDesc("access", unimplementedFunc),
    /* 34 */ SyscallDesc("nice", unimplementedFunc),
    /* 35 */ SyscallDesc("ftime", unimplementedFunc),
    /* 36 */ SyscallDesc("sync", unimplementedFunc),
    /* 37 */ SyscallDesc("kill", ignoreFunc),
    /* 38 */ SyscallDesc("rename", renameFunc),
    /* 39 */ SyscallDesc("mkdir", unimplementedFunc),
    /* 40 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 41 */ SyscallDesc("dup", dupFunc),
    /* 42 */ SyscallDesc("pipe", unimplementedFunc),
    /* 43 */ SyscallDesc("times", timesFunc<PowerLinux>),
    /* 44 */ SyscallDesc("prof", unimplementedFunc),
    /* 45 */ SyscallDesc("brk", brkFunc),
    /* 46 */ SyscallDesc("setgid", unimplementedFunc),
    /* 47 */ SyscallDesc("getgid", getgidFunc),
    /* 48 */ SyscallDesc("signal", ignoreFunc),
    /* 49 */ SyscallDesc("geteuid", geteuidFunc),
    /* 50 */ SyscallDesc("getegid", getegidFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("umount2", unimplementedFunc),
    /* 53 */ SyscallDesc("lock", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", ioctlFunc<PowerLinux>),
    /* 55 */ SyscallDesc("fcntl", fcntlFunc),
    /* 56 */ SyscallDesc("mpx", unimplementedFunc),
    /* 57 */ SyscallDesc("setpgid", unimplementedFunc),
    /* 58 */ SyscallDesc("ulimit", unimplementedFunc),
    /* 59 */ SyscallDesc("unused#59", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", umaskFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("ustat", unimplementedFunc),
    /* 63 */ SyscallDesc("dup2", unimplementedFunc),
    /* 64 */ SyscallDesc("getppid", getpagesizeFunc),
    /* 65 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 66 */ SyscallDesc("setsid", unimplementedFunc),
    /* 67 */ SyscallDesc("sigaction",unimplementedFunc),
    /* 68 */ SyscallDesc("sgetmask", unimplementedFunc),
    /* 69 */ SyscallDesc("ssetmask", unimplementedFunc),
    /* 70 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 71 */ SyscallDesc("setregid", unimplementedFunc),
    /* 72 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 73 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 74 */ SyscallDesc("sethostname", ignoreFunc),
    /* 75 */ SyscallDesc("setrlimit", ignoreFunc),
    /* 76 */ SyscallDesc("getrlimit", unimplementedFunc),
    /* 77 */ SyscallDesc("getrusage", ignoreFunc),
    /* 78 */ SyscallDesc("gettimeofday", unimplementedFunc),
    /* 79 */ SyscallDesc("settimeofday", unimplementedFunc),
    /* 80 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 81 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 82 */ SyscallDesc("reserved#82", unimplementedFunc),
    /* 83 */ SyscallDesc("symlink", unimplementedFunc),
    /* 84 */ SyscallDesc("unused#84", unimplementedFunc),
    /* 85 */ SyscallDesc("readlink", unimplementedFunc),
    /* 86 */ SyscallDesc("uselib", unimplementedFunc),
    /* 87 */ SyscallDesc("swapon", gethostnameFunc),
    /* 88 */ SyscallDesc("reboot", unimplementedFunc),
    /* 89 */ SyscallDesc("readdir", unimplementedFunc),
    /* 90 */ SyscallDesc("mmap", mmapFunc<PowerLinux>),
    /* 91 */ SyscallDesc("munmap",munmapFunc),
    /* 92 */ SyscallDesc("truncate", truncateFunc),
    /* 93 */ SyscallDesc("ftruncate", ftruncateFunc),
    /* 94 */ SyscallDesc("fchmod", unimplementedFunc),
    /* 95 */ SyscallDesc("fchown", unimplementedFunc),
    /* 96 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 97 */ SyscallDesc("setpriority", unimplementedFunc),
    /* 98 */ SyscallDesc("profil", unimplementedFunc),
    /* 99 */ SyscallDesc("statfs", unimplementedFunc),
    /* 100 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 101 */ SyscallDesc("ioperm", unimplementedFunc),
    /* 102 */ SyscallDesc("socketcall", unimplementedFunc),
    /* 103 */ SyscallDesc("syslog", unimplementedFunc),
    /* 104 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 105 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 106 */ SyscallDesc("stat",  statFunc<PowerLinux>),
    /* 107 */ SyscallDesc("lstat", unimplementedFunc),
    /* 108 */ SyscallDesc("fstat", fstatFunc<PowerLinux>),
    /* 109 */ SyscallDesc("unused#109", unimplementedFunc),
    /* 110 */ SyscallDesc("iopl", unimplementedFunc),
    /* 111 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 112 */ SyscallDesc("idle", ignoreFunc),
    /* 113 */ SyscallDesc("vm86", unimplementedFunc),
    /* 114 */ SyscallDesc("wait4", unimplementedFunc),
    /* 115 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 116 */ SyscallDesc("sysinfo", unimplementedFunc),
    /* 117 */ SyscallDesc("ipc", unimplementedFunc),
    /* 118 */ SyscallDesc("fsync", unimplementedFunc),
    /* 119 */ SyscallDesc("sigreturn", unimplementedFunc),
    /* 120 */ SyscallDesc("clone", unimplementedFunc),
    /* 121 */ SyscallDesc("setdomainname", unimplementedFunc),
    /* 122 */ SyscallDesc("uname", unameFunc),
    /* 123 */ SyscallDesc("modify_ldt", unimplementedFunc),
    /* 124 */ SyscallDesc("adjtimex", unimplementedFunc),
    /* 125 */ SyscallDesc("mprotect", ignoreFunc),
    /* 126 */ SyscallDesc("sigprocmask", unimplementedFunc),
    /* 127 */ SyscallDesc("create_module", unimplementedFunc),
    /* 128 */ SyscallDesc("init_module", unimplementedFunc),
    /* 129 */ SyscallDesc("delete_module", unimplementedFunc),
    /* 130 */ SyscallDesc("get_kernel_syms", unimplementedFunc),
    /* 131 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 132 */ SyscallDesc("getpgid", unimplementedFunc),
    /* 133 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 134 */ SyscallDesc("bdflush", unimplementedFunc),
    /* 135 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 136 */ SyscallDesc("personality", unimplementedFunc),
    /* 137 */ SyscallDesc("afs_syscall", unimplementedFunc),
    /* 138 */ SyscallDesc("setfsuid", unimplementedFunc),
    /* 139 */ SyscallDesc("setfsgid", unimplementedFunc),
    /* 140 */ SyscallDesc("llseek", _llseekFunc),
    /* 141 */ SyscallDesc("getdents", unimplementedFunc),
    /* 142 */ SyscallDesc("newselect", unimplementedFunc),
    /* 143 */ SyscallDesc("flock", unimplementedFunc),
    /* 144 */ SyscallDesc("msync", unimplementedFunc),
    /* 145 */ SyscallDesc("readv", unimplementedFunc),
    /* 146 */ SyscallDesc("writev", writevFunc<PowerLinux>),
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
    /* 162 */ SyscallDesc("nanosleep", unimplementedFunc),
    /* 163 */ SyscallDesc("mremap", unimplementedFunc),
    /* 164 */ SyscallDesc("setresuid", unimplementedFunc),
    /* 165 */ SyscallDesc("getresuid", unimplementedFunc),
    /* 166 */ SyscallDesc("vm862", unimplementedFunc),
    /* 167 */ SyscallDesc("query_module", unimplementedFunc),
    /* 168 */ SyscallDesc("poll", unimplementedFunc),
    /* 169 */ SyscallDesc("nfsservctl", unimplementedFunc),
    /* 170 */ SyscallDesc("setresgid", unimplementedFunc),
    /* 171 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 172 */ SyscallDesc("prctl", unimplementedFunc),
    /* 173 */ SyscallDesc("rt_sigaction", ignoreFunc),
    /* 174 */ SyscallDesc("rt_sigprocmask", unimplementedFunc),
    /* 175 */ SyscallDesc("unknown#175", unimplementedFunc),
    /* 176 */ SyscallDesc("rt_sigpending", unimplementedFunc),
    /* 177 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 178 */ SyscallDesc("rt_sigqueueinfo", ignoreFunc),
    /* 179 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 180 */ SyscallDesc("pread64", unimplementedFunc),
    /* 181 */ SyscallDesc("pwrite64", unimplementedFunc),
    /* 182 */ SyscallDesc("chown", unimplementedFunc),
    /* 183 */ SyscallDesc("getcwd", unimplementedFunc),
    /* 184 */ SyscallDesc("capget", unimplementedFunc),
    /* 185 */ SyscallDesc("capset", unimplementedFunc),
    /* 186 */ SyscallDesc("sigaltstack", unimplementedFunc),
    /* 187 */ SyscallDesc("sendfile", unimplementedFunc),
    /* 188 */ SyscallDesc("getpmsg", unimplementedFunc),
    /* 189 */ SyscallDesc("putpmsg", unimplementedFunc),
    /* 190 */ SyscallDesc("ugetrlimit", ignoreFunc),
    /* 191 */ SyscallDesc("getrlimit", unimplementedFunc),
    /* 192 */ SyscallDesc("mmap2", mmapFunc<PowerLinux>),
    /* 193 */ SyscallDesc("truncate64", unimplementedFunc),
    /* 194 */ SyscallDesc("ftruncate64", ftruncate64Func),
    /* 195 */ SyscallDesc("stat64", stat64Func<PowerLinux>),
    /* 196 */ SyscallDesc("lstat64", lstat64Func<PowerLinux>),
    /* 197 */ SyscallDesc("fstat64", fstat64Func<PowerLinux>),
    /* 198 */ SyscallDesc("lchown", unimplementedFunc),
    /* 199 */ SyscallDesc("getuid", getuidFunc),
    /* 200 */ SyscallDesc("getgid", getgidFunc),
    /* 201 */ SyscallDesc("geteuid", geteuidFunc),
    /* 202 */ SyscallDesc("getegid", getegidFunc),
    /* 203 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 204 */ SyscallDesc("fcntl64", fcntl64Func),
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
    /* 221 */ SyscallDesc("unknown#221", unimplementedFunc),
    /* 222 */ SyscallDesc("tux", unimplementedFunc),
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
    /* 234 */ SyscallDesc("exit_group", exitGroupFunc),
    /* 235 */ SyscallDesc("removexattr", unimplementedFunc),
    /* 236 */ SyscallDesc("lremovexattr", unimplementedFunc),
    /* 237 */ SyscallDesc("fremovexattr", unimplementedFunc),
    /* 238 */ SyscallDesc("tkill", unimplementedFunc),
    /* 239 */ SyscallDesc("sendfile64", unimplementedFunc),
    /* 240 */ SyscallDesc("futex", unimplementedFunc),
    /* 241 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 242 */ SyscallDesc("sched_getaffinity", unimplementedFunc),
    /* 243 */ SyscallDesc("io_setup", unimplementedFunc),
    /* 244 */ SyscallDesc("io_destory", unimplementedFunc),
    /* 245 */ SyscallDesc("io_getevents", unimplementedFunc),
    /* 246 */ SyscallDesc("io_submit", unimplementedFunc),
    /* 247 */ SyscallDesc("io_cancel", unimplementedFunc),
    /* 248 */ SyscallDesc("unknown#248", unimplementedFunc),
    /* 249 */ SyscallDesc("lookup_dcookie", unimplementedFunc),
    /* 250 */ SyscallDesc("epoll_create", unimplementedFunc),
    /* 251 */ SyscallDesc("epoll_ctl", unimplementedFunc),
    /* 252 */ SyscallDesc("epoll_wait", unimplementedFunc),
    /* 253 */ SyscallDesc("remap_file_pages", unimplementedFunc),
    /* 254 */ SyscallDesc("set_thread_area", unimplementedFunc),
    /* 255 */ SyscallDesc("get_thread_area", unimplementedFunc),
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
    /* 313 */ SyscallDesc("vserver", unimplementedFunc),
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
};

PowerLinuxProcess::PowerLinuxProcess(ProcessParams * params,
        ObjectFile *objFile)
    : PowerProcess(params, objFile),
      Num_Syscall_Descs(sizeof(syscallDescs) / sizeof(SyscallDesc))
{
}

SyscallDesc*
PowerLinuxProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum > Num_Syscall_Descs)
        return NULL;

    return &syscallDescs[callnum];
}

void
PowerLinuxProcess::initState()
{
    PowerProcess::initState();
}

RegVal
PowerLinuxProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    // Linux apparently allows more parameter than the ABI says it should.
    // This limit may need to be increased even further.
    assert(i < 6);
    return tc->readIntReg(ArgumentReg0 + i++);
}

void
PowerLinuxProcess::setSyscallArg(ThreadContext *tc, int i, RegVal val)
{
    // Linux apparently allows more parameter than the ABI says it should.
    // This limit may need to be increased even further.
    assert(i < 6);
    tc->setIntReg(ArgumentReg0 + i, val);
}
