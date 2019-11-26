/*
 * Copyright (c) 2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 *          Korey Sewell
 */

#include "arch/mips/linux/process.hh"

#include "arch/mips/isa_traits.hh"
#include "arch/mips/linux/linux.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/SyscallVerbose.hh"
#include "kern/linux/linux.hh"
#include "sim/eventq.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace MipsISA;

namespace
{

class MipsLinuxObjectFileLoader : public Process::Loader
{
  public:
    Process *
    load(ProcessParams *params, ObjectFile *obj_file) override
    {
        if (obj_file->getArch() != ObjectFile::Mips)
            return nullptr;

        auto opsys = obj_file->getOpSys();

        if (opsys == ObjectFile::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = ObjectFile::Linux;
        }

        if (opsys != ObjectFile::Linux)
            return nullptr;

        return new MipsLinuxProcess(params, obj_file);
    }
};

MipsLinuxObjectFileLoader loader;

} // anonymous namespace

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, ThreadContext *tc)
{
    int index = 0;
    auto process = tc->getProcessPtr();
    TypedBufferArg<Linux::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename,"sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "mips");

    name.copyOut(tc->getVirtProxy());
    return 0;
}

/// Target sys_getsysyinfo() handler.  Even though this call is
/// borrowed from Tru64, the subcases that get used appear to be
/// different in practice from those used by Tru64 processes.
static SyscallReturn
sys_getsysinfoFunc(SyscallDesc *desc, int callnum, ThreadContext *tc)
{
    int index = 0;
    auto process = tc->getProcessPtr();
    unsigned op = process->getSyscallArg(tc, index);
    unsigned bufPtr = process->getSyscallArg(tc, index);
    // unsigned nbytes = process->getSyscallArg(tc, index);

    switch (op) {
      case 45:
        {
            // GSI_IEEE_FP_CONTROL
            TypedBufferArg<uint64_t> fpcr(bufPtr);
            // I don't think this exactly matches the HW FPCR
            *fpcr = 0;
            fpcr.copyOut(tc->getVirtProxy());
            return 0;
        }
      default:
        cerr << "sys_getsysinfo: unknown op " << op << endl;
        abort();
        break;
    }

    return 1;
}

/// Target sys_setsysinfo() handler.
static SyscallReturn
sys_setsysinfoFunc(SyscallDesc *desc, int callnum, ThreadContext *tc)
{
    int index = 0;
    auto process = tc->getProcessPtr();
    unsigned op = process->getSyscallArg(tc, index);
    Addr bufPtr = process->getSyscallArg(tc, index);
    // unsigned nbytes = process->getSyscallArg(tc, index);

    switch (op) {

      case 14:
        {
            // SSI_IEEE_FP_CONTROL
            TypedBufferArg<uint64_t> fpcr(bufPtr);
            // I don't think this exactly matches the HW FPCR
            fpcr.copyIn(tc->getVirtProxy());
            DPRINTFR(SyscallVerbose, "sys_setsysinfo(SSI_IEEE_FP_CONTROL): "
                   " setting FPCR to 0x%x\n", letoh(*(uint64_t*)fpcr));
            return 0;
        }
      default:
        cerr << "sys_setsysinfo: unknown op " << op << endl;
        abort();
        break;
    }

    return 1;
}

static SyscallReturn
setThreadAreaFunc(SyscallDesc *desc, int callnum, ThreadContext *tc)
{
    int index = 0;
    auto process = tc->getProcessPtr();
    Addr addr = process->getSyscallArg(tc, index);
    tc->setMiscRegNoEffect(MISCREG_TP_VALUE, addr);
    return 0;
}

SyscallDesc MipsLinuxProcess::syscallDescs[] = {
    /*  0 */ { "syscall" },
    /*  1 */ { "exit", exitFunc },
    /*  2 */ { "fork" },
    /*  3 */ { "read", readFunc<MipsLinux> },
    /*  4 */ { "write", writeFunc<MipsLinux> },
    /*  5 */ { "open", openFunc<MipsLinux> },
    /*  6 */ { "close", closeFunc },
    /*  7 */ { "waitpid" },
    /*  8 */ { "creat" },
    /*  9 */ { "link" },
    /* 10 */ { "unlink", unlinkFunc },
    /* 11 */ { "execve" },
    /* 12 */ { "chdir" },
    /* 13 */ { "time" },
    /* 14 */ { "mknod" },
    /* 15 */ { "chmod", chmodFunc<MipsLinux> },
    /* 16 */ { "lchown", chownFunc },
    /* 17 */ { "break", brkFunc },
    /* 18 */ { "unused#18" },
    /* 19 */ { "lseek", lseekFunc },
    /* 20 */ { "getpid", getpidFunc },
    /* 21 */ { "mount" },
    /* 22 */ { "umount" },
    /* 23 */ { "setuid", setuidFunc },
    /* 24 */ { "getuid", getuidFunc },
    /* 25 */ { "stime" },
    /* 26 */ { "ptrace" },
    /* 27 */ { "alarm" },
    /* 28 */ { "unused#28" },
    /* 29 */ { "pause" },
    /* 30 */ { "utime" },
    /* 31 */ { "stty" },
    /* 32 */ { "gtty" },
    /* 33 */ { "access" },
    /* 34 */ { "nice" },
    /* 35 */ { "ftime" },
    /* 36 */ { "sync" },
    /* 37 */ { "kill" },
    /* 38 */ { "rename" },
    /* 39 */ { "mkdir", mkdirFunc },
    /* 40 */ { "rmdir" },
    /* 41 */ { "dup" },
    /* 42 */ { "pipe", pipePseudoFunc },
    /* 43 */ { "times" },
    /* 44 */ { "prof" },
    /* 45 */ { "brk", brkFunc },
    /* 46 */ { "setgid" },
    /* 47 */ { "getgid", getgidFunc },
    /* 48 */ { "signal", ignoreFunc },
    /* 49 */ { "geteuid", geteuidFunc },
    /* 50 */ { "getegid", getegidFunc },
    /* 51 */ { "acct" },
    /* 52 */ { "umount2" },
    /* 53 */ { "lock" },
    /* 54 */ { "ioctl", ioctlFunc<MipsLinux> },
    /* 55 */ { "fcntl", fcntlFunc },
    /* 56 */ { "mpx" },
    /* 57 */ { "setpgid" },
    /* 58 */ { "ulimit" },
    /* 59 */ { "unused#59" },
    /* 60 */ { "umask", umaskFunc },
    /* 61 */ { "chroot" },
    /* 62 */ { "ustat" },
    /* 63 */ { "dup2" },
    /* 64 */ { "getppid", getpagesizeFunc },
    /* 65 */ { "getpgrp" },
    /* 66 */ { "setsid" },
    /* 67 */ { "sigaction" },
    /* 68 */ { "sgetmask" },
    /* 69 */ { "ssetmask" },
    /* 70 */ { "setreuid" },
    /* 71 */ { "setregid" },
    /* 72 */ { "sigsuspend" },
    /* 73 */ { "sigpending" },
    /* 74 */ { "sethostname", ignoreFunc },
    /* 75 */ { "setrlimit" },
    /* 76 */ { "getrlimit" },
    /* 77 */ { "getrusage", getrusageFunc<MipsLinux> },
    /* 78 */ { "gettimeofday" },
    /* 79 */ { "settimeofday" },
    /* 80 */ { "getgroups" },
    /* 81 */ { "setgroups" },
    /* 82 */ { "reserved#82" },
    /* 83 */ { "symlink" },
    /* 84 */ { "unused#84" },
    /* 85 */ { "readlink", readlinkFunc },
    /* 86 */ { "uselib" },
    /* 87 */ { "swapon", gethostnameFunc },
    /* 88 */ { "reboot" },
    /* 89 */ { "readdir" },
    /* 90 */ { "mmap", mmapFunc<MipsLinux> },
    /* 91 */ { "munmap",munmapFunc },
    /* 92 */ { "truncate", truncateFunc },
    /* 93 */ { "ftruncate", ftruncateFunc },
    /* 94 */ { "fchmod", fchmodFunc<MipsLinux> },
    /* 95 */ { "fchown", fchownFunc },
    /* 96 */ { "getpriority" },
    /* 97 */ { "setpriority" },
    /* 98 */ { "profil" },
    /* 99 */ { "statfs" },
    /* 100 */ { "fstatfs" },
    /* 101 */ { "ioperm" },
    /* 102 */ { "socketcall" },
    /* 103 */ { "syslog" },
    /* 104 */ { "setitimer" },
    /* 105 */ { "getitimer" },
    /* 106 */ { "stat",  statFunc<MipsLinux> },
    /* 107 */ { "lstat" },
    /* 108 */ { "fstat", fstatFunc<MipsLinux> },
    /* 109 */ { "unused#109" },
    /* 110 */ { "iopl" },
    /* 111 */ { "vhangup" },
    /* 112 */ { "idle", ignoreFunc },
    /* 113 */ { "vm86" },
    /* 114 */ { "wait4" },
    /* 115 */ { "swapoff" },
    /* 116 */ { "sysinfo", sysinfoFunc<MipsLinux> },
    /* 117 */ { "ipc" },
    /* 118 */ { "fsync" },
    /* 119 */ { "sigreturn" },
    /* 120 */ { "clone" },
    /* 121 */ { "setdomainname" },
    /* 122 */ { "uname", unameFunc },
    /* 123 */ { "modify_ldt" },
    /* 124 */ { "adjtimex" },
    /* 125 */ { "mprotect", ignoreFunc },
    /* 126 */ { "sigprocmask" },
    /* 127 */ { "create_module" },
    /* 128 */ { "init_module" },
    /* 129 */ { "delete_module" },
    /* 130 */ { "get_kernel_syms" },
    /* 131 */ { "quotactl" },
    /* 132 */ { "getpgid" },
    /* 133 */ { "fchdir" },
    /* 134 */ { "bdflush" },
    /* 135 */ { "sysfs" },
    /* 136 */ { "personality" },
    /* 137 */ { "afs_syscall" },
    /* 138 */ { "setfsuid" },
    /* 139 */ { "setfsgid" },
    /* 140 */ { "llseek" },
    /* 141 */ { "getdents" },
    /* 142 */ { "newselect" },
    /* 143 */ { "flock" },
    /* 144 */ { "msync" },
    /* 145 */ { "readv" },
    /* 146 */ { "writev", writevFunc<MipsLinux> },
    /* 147 */ { "cacheflush" },
    /* 148 */ { "cachectl" },
    /* 149 */ { "sysmips" },
    /* 150 */ { "unused#150" },
    /* 151 */ { "getsid" },
    /* 152 */ { "fdatasync" },
    /* 153 */ { "sysctl", ignoreFunc },
    /* 154 */ { "mlock" },
    /* 155 */ { "munlock" },
    /* 156 */ { "mlockall" },
    /* 157 */ { "munlockall" },
    /* 158 */ { "sched_setparam" },
    /* 159 */ { "sched_getparam" },
    /* 160 */ { "sched_setscheduler" },
    /* 161 */ { "sched_getscheduler" },
    /* 162 */ { "sched_yield" },
    /* 163 */ { "sched_get_prioritymax" },
    /* 164 */ { "sched_get_priority_min" },
    /* 165 */ { "sched_rr_get_interval" },
    /* 166 */ { "nanosleep" },
    /* 167 */ { "mremap", mremapFunc<MipsLinux> },
    /* 168 */ { "accept" },
    /* 169 */ { "bind" },
    /* 170 */ { "connect" },
    /* 171 */ { "getpeername" },
    /* 172 */ { "getsockname" },
    /* 173 */ { "getsockopt" },
    /* 174 */ { "listen" },
    /* 175 */ { "recv" },
    /* 176 */ { "recvmsg" },
    /* 177 */ { "send" },
    /* 178 */ { "sendmsg", ignoreFunc },
    /* 179 */ { "sendto" },
    /* 180 */ { "setsockopt" },
    /* 181 */ { "shutdown" },
    /* 182 */ { "unknown #182" },
    /* 183 */ { "socket", ignoreFunc },
    /* 184 */ { "socketpair" },
    /* 185 */ { "setresuid" },
    /* 186 */ { "getresuid" },
    /* 187 */ { "query_module" },
    /* 188 */ { "poll" },
    /* 189 */ { "nfsservctl" },
    /* 190 */ { "setresgid" },
    /* 191 */ { "getresgid" },
    /* 192 */ { "prctl" },
    /* 193 */ { "rt_sigreturn" },
    /* 194 */ { "rt_sigaction" },
    /* 195 */ { "rt_sigprocmask" },
    /* 196 */ { "rt_sigpending" },
    /* 197 */ { "rt_sigtimedwait" },
    /* 198 */ { "rt_sigqueueinfo", ignoreFunc },
    /* 199 */ { "rt_sigsuspend" },
    /* 200 */ { "pread64" },
    /* 201 */ { "pwrite64" },
    /* 202 */ { "chown" },
    /* 203 */ { "getcwd", getcwdFunc },
    /* 204 */ { "capget" },
    /* 205 */ { "capset" },
    /* 206 */ { "sigalstack" },
    /* 207 */ { "sendfile" },
    /* 208 */ { "getpmsg" },
    /* 209 */ { "putpmsg" },
    /* 210 */ { "mmap2" },
    /* 211 */ { "truncate64" },
    /* 212 */ { "ftruncate64" },
    /* 213 */ { "stat64" },
    /* 214 */ { "lstat64", lstat64Func<MipsLinux> },
    /* 215 */ { "fstat64", fstat64Func<MipsLinux> },
    /* 216 */ { "pivot_root" },
    /* 217 */ { "mincore" },
    /* 218 */ { "madvise" },
    /* 219 */ { "getdents64" },
    /* 220 */ { "fcntl64", fcntl64Func },
    /* 221 */ { "reserved#221" },
    /* 222 */ { "gettid" },
    /* 223 */ { "readahead" },
    /* 224 */ { "setxattr" },
    /* 225 */ { "lsetxattr" },
    /* 226 */ { "fsetxattr" },
    /* 227 */ { "getxattr" },
    /* 228 */ { "lgetxattr" },
    /* 229 */ { "fgetxattr" },
    /* 230 */ { "listxattr" },
    /* 231 */ { "llistxattr" },
    /* 232 */ { "flistxattr" },
    /* 233 */ { "removexattr" },
    /* 234 */ { "lremovexattr" },
    /* 235 */ { "fremovexattr", ignoreFunc },
    /* 236 */ { "tkill" },
    /* 237 */ { "sendfile64" },
    /* 238 */ { "futex" },
    /* 239 */ { "sched_setaffinity" },
    /* 240 */ { "sched_getaffinity" },
    /* 241 */ { "io_setup" },
    /* 242 */ { "io_destroy" },
    /* 243 */ { "io_getevents" },
    /* 244 */ { "io_submit" },
    /* 245 */ { "io_cancel" },
    /* 246 */ { "exit_group", exitFunc },
    /* 247 */ { "lookup_dcookie" },
    /* 248 */ { "epoll_create" },
    /* 249 */ { "epoll_ctl" },
    /* 250 */ { "epoll_wait" },
    /* 251 */ { "remap_file_pages" },
    /* 252 */ { "set_tid_address" },
    /* 253 */ { "restart_syscall" },
    /* 254 */ { "fadvise64" },
    /* 255 */ { "statfs64" },
    /* 256 */ { "fstafs64" },
    /* 257 */ { "timer_create", sys_getsysinfoFunc },
    /* 258 */ { "timer_settime", sys_setsysinfoFunc },
    /* 259 */ { "timer_gettime" },
    /* 260 */ { "timer_getoverrun" },
    /* 261 */ { "timer_delete" },
    /* 262 */ { "clock_settime" },
    /* 263 */ { "clock_gettime" },
    /* 264 */ { "clock_getres" },
    /* 265 */ { "clock_nanosleep" },
    /* 266 */ { "tgkill" },
    /* 267 */ { "utimes" },
    /* 268 */ { "mbind" },
    /* 269 */ { "get_mempolicy" },
    /* 270 */ { "set_mempolicy" },
    /* 271 */ { "mq_open" },
    /* 272 */ { "mq_unlink" },
    /* 273 */ { "mq_timedsend" },
    /* 274 */ { "mq_timedreceive" },
    /* 275 */ { "mq_notify" },
    /* 276 */ { "mq_getsetattr" },
    /* 277 */ { "vserver" },
    /* 278 */ { "waitid" },
    /* 279 */ { "unknown #279" },
    /* 280 */ { "add_key" },
    /* 281 */ { "request_key" },
    /* 282 */ { "keyctl" },
    /* 283 */ { "set_thread_area", setThreadAreaFunc },
    /* 284 */ { "inotify_init" },
    /* 285 */ { "inotify_add_watch" },
    /* 286 */ { "inotify_rm_watch" },
    /* 287 */ { "migrate_pages" },
    /* 288 */ { "openat" },
    /* 289 */ { "mkdirat" },
    /* 290 */ { "mknodat" },
    /* 291 */ { "fchownat" },
    /* 292 */ { "futimesat" },
    /* 293 */ { "fstatat64" },
    /* 294 */ { "unlinkat" },
    /* 295 */ { "renameat" },
    /* 296 */ { "linkat" },
    /* 297 */ { "symlinkat" },
    /* 298 */ { "readlinkat" },
    /* 299 */ { "fchmodat" },
    /* 300 */ { "faccessat" },
    /* 301 */ { "pselect6" },
    /* 302 */ { "ppoll" },
    /* 303 */ { "unshare" },
    /* 304 */ { "splice" },
    /* 305 */ { "sync_file_range" },
    /* 306 */ { "tee" },
    /* 307 */ { "vmsplice" },
    /* 308 */ { "move_pages" },
    /* 309 */ { "set_robust_list" },
    /* 310 */ { "get_robust_list" },
    /* 311 */ { "kexec_load" },
    /* 312 */ { "getcpu" },
    /* 313 */ { "epoll_pwait" },
    /* 314 */ { "ioprio_set" },
    /* 315 */ { "ioprio_get" },
    /* 316 */ { "utimensat" },
    /* 317 */ { "signalfd" },
    /* 318 */ { "timerfd" },
    /* 319 */ { "eventfd" }
};

MipsLinuxProcess::MipsLinuxProcess(ProcessParams * params,
                                   ObjectFile *objFile)
    : MipsProcess(params, objFile),
      Num_Syscall_Descs(sizeof(syscallDescs) / sizeof(SyscallDesc))
{  }

SyscallDesc*
MipsLinuxProcess::getDesc(int callnum)
{
    //MIPS32 syscalls are in the range of 4000 - 4999
    int m5_sys_idx = callnum - 4000;

    if (m5_sys_idx < 0 || m5_sys_idx >= Num_Syscall_Descs)
        return NULL;

    return &syscallDescs[m5_sys_idx];
}

void
MipsLinuxProcess::syscall(ThreadContext *tc, Fault *fault)
{
    doSyscall(tc->readIntReg(2), tc, fault);
}
