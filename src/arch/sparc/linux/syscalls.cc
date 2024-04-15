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

#include "arch/sparc/linux/se_workload.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

namespace gem5
{

class Process;
class ThreadContext;

namespace SparcISA
{

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "sparc");

    return 0;
}

static SyscallReturn
getresuidFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> ruid, VPtr<> euid,
              VPtr<> suid)
{
    SETranslatingPortProxy proxy(tc);
    const uint64_t id = htobe(100);
    // Handle the EFAULT case
    // Set the ruid
    if (ruid) {
        BufferArg ruidBuff(ruid, sizeof(uint64_t));
        memcpy(ruidBuff.bufferPtr(), &id, sizeof(uint64_t));
        ruidBuff.copyOut(proxy);
    }
    // Set the euid
    if (euid) {
        BufferArg euidBuff(euid, sizeof(uint64_t));
        memcpy(euidBuff.bufferPtr(), &id, sizeof(uint64_t));
        euidBuff.copyOut(proxy);
    }
    // Set the suid
    if (suid) {
        BufferArg suidBuff(suid, sizeof(uint64_t));
        memcpy(suidBuff.bufferPtr(), &id, sizeof(uint64_t));
        suidBuff.copyOut(proxy);
    }
    return 0;
}

SyscallDescTable<SEWorkload::SyscallABI32> EmuLinux::syscall32Descs = {
    { 0, "restart_syscall" },
    { 1, "exit", exitFunc }, // 32 bit
    { 2, "fork" },
    { 3, "read", readFunc<Sparc32Linux> },
    { 4, "write", writeFunc<Sparc32Linux> },
    { 5, "open", openFunc<Sparc32Linux> }, // 32 bit
    { 6, "close", closeFunc },
    { 7, "wait4" }, // 32 bit
    { 8, "creat" }, // 32 bit
    { 9, "link" },
    { 10, "unlink", unlinkFunc },
    { 11, "execv" },
    { 12, "chdir" },
    { 13, "chown", chownFunc }, // 32 bit
    { 14, "mknod" },
    { 15, "chmod" },
    { 16, "lchown" }, // 32 bit
    { 17, "brk", brkFunc },
    { 18, "perfctr" },          // 32 bit
    { 19, "lseek", lseekFunc }, // 32 bit
    { 20, "getpid", getpidFunc },
    { 21, "capget" },
    { 22, "capset" },
    { 23, "setuid", ignoreFunc }, // 32 bit
    { 24, "getuid", getuidFunc }, // 32 bit
    { 25, "time" },
    { 26, "ptrace" },
    { 27, "alarm" },
    { 28, "sigaltstack" }, // 32 bit
    { 29, "pause" },       // 32 bit
    { 30, "utime" },
    { 31, "lchown32" },
    { 32, "fchown32" },
    { 33, "access" }, // 32 bit
    { 34, "nice" },   // 32 bit
    { 35, "chown32" },
    { 36, "sync" },
    { 37, "kill" }, // 32 bit
    { 38, "stat" },
    { 39, "sendfile" }, // 32 bit
    { 40, "lstat" },
    { 41, "dup" },
    { 42, "pipe", pipePseudoFunc },
    { 43, "times", ignoreFunc },
    { 44, "getuid32" },
    { 45, "umount2" },              // 32 bit
    { 46, "setgid" },               // 32 bit
    { 47, "getgid", getgidFunc },   // 32 bit
    { 48, "signal" },               // 32 bit
    { 49, "geteuid", geteuidFunc }, // 32 bit
    { 50, "getegid", getegidFunc }, // 32 bit
    { 51, "acct" },
    { 52, "memory_ordering" },
    { 53, "getgid32" },
    { 54, "ioctl" },
    { 55, "reboot" }, // 32 bit
    { 56, "mmap2" },  // 32 bit
    { 57, "symlink" },
    { 58, "readlink", readlinkFunc<Sparc32Linux> }, // 32 bit
    { 59, "execve" },                               // 32 bit
    { 60, "umask" },                                // 32 bit
    { 61, "chroot" },
    { 62, "fstat" },
    { 63, "fstat64", fstat64Func<Sparc32Linux> },
    { 64, "getpagesize" },
    { 65, "msync" }, // 32 bit
    { 66, "vfork" },
    { 67, "pread64" },  // 32 bit
    { 68, "pwrite64" }, // 32 bit
    { 69, "geteuid32" },
    { 70, "getegid32" },
    { 71, "mmap", mmapFunc<Sparc32Linux> },
    { 72, "setreuid32" },
    { 73, "munmap", munmapFunc<Sparc32Linux> },
    { 74, "mprotect", ignoreFunc },
    { 75, "madvise" },
    { 76, "vhangup" },
    { 77, "truncate64" }, // 32 bit
    { 78, "mincore" },
    { 79, "getgroups" }, // 32 bit
    { 80, "setgroups" }, // 32 bit
    { 81, "getpgrp" },
    { 82, "setgroups32" }, // 32 bit
    { 83, "setitimer" },   // 32 bit
    { 84, "ftruncate64" }, // 32 bit
    { 85, "swapon" },      // 32 bit
    { 86, "getitimer" },   // 32 bit
    { 87, "setuid32" },
    { 88, "sethostname" }, // 32 bit
    { 89, "setgid32" },
    { 90, "dup2" },
    { 91, "setfsuid32" },
    { 92, "fcntl" },
    { 93, "select" }, // 32 bit
    { 94, "setfsgid32" },
    { 95, "fsync" },
    { 96, "setpriority" }, // 32 bit
    { 97, "socket" },
    { 98, "connect" },
    { 99, "accept" },
    { 100, "getpriority" },                // 32 bit
    { 101, "rt_sigreturn" },               // 32 bit
    { 102, "rt_sigaction", ignoreFunc },   // 32 bit
    { 103, "rt_sigprocmask", ignoreFunc }, // 32 bit
    { 104, "rt_sigpending" },              // 32 bit
    { 105, "rt_sigtimedwait" },
    { 106, "rt_sigqueueinfo" }, // 32 bit
    { 107, "rt_sigsuspend" },
    { 108, "setresuid32" },
    { 109, "getresuid32", getresuidFunc },
    { 110, "setresgid32", ignoreFunc },
    { 111, "getresgid32" },
    { 112, "setregid32" },
    { 113, "revcmsg" },
    { 114, "sendmsg" },
    { 115, "getgroups32" },                                  // 32 bit
    { 116, "gettimeofday", gettimeofdayFunc<Sparc32Linux> }, // 32 bit
    { 117, "getrusage" },                                    // 32 bit
    { 118, "getsockopt" },
    { 119, "getcwd", getcwdFunc },
    { 120, "readv" },
    { 121, "writev" },
    { 122, "settimeofday" }, // 32 bit
    { 123, "fchown" },       // 32 bit
    { 124, "fchmod" },
    { 125, "recvfrom" },
    { 126, "setreuid" }, // 32 bit
    { 127, "setregid" }, // 32 bit
    { 128, "rename", renameFunc },
    { 129, "truncate" },
    { 130, "ftruncate" },
    { 131, "flock" },
    { 132, "lstat64" },
    { 133, "sendto" },
    { 134, "shutdown" },
    { 135, "socketpair" },
    { 136, "mkdir", mkdirFunc }, // 32 bit
    { 137, "rmdir" },
    { 138, "utimes" }, // 32 bit
    { 139, "stat64" },
    { 140, "sendfile64" }, // 32 bit
    { 141, "getpeername" },
    { 142, "futex" }, // 32 bit
    { 143, "gettid" },
    { 144, "getrlimit" },
    { 145, "setrlimit" },
    { 146, "pivot_root" },
    { 147, "prctl" }, // 32 bit
    { 148, "pciconfig_read" },
    { 149, "pciconfig_write" },
    { 150, "getsockname" },
    { 151, "inotify_init" },
    { 152, "inotify_add_watch" },
    { 153, "poll" },
    { 154, "getdents64" },
    { 155, "fcntl64" },
    { 156, "inotify_rm_watch" },
    { 157, "statfs" },
    { 158, "fstatfs" },
    { 159, "umount" },
    { 160, "sched_setaffinity" },
    { 161, "sched_getaffinity" },
    { 162, "getdomainname" }, // 32 bit
    { 163, "setdomainname" }, // 32 bit
    { 164, "ni_syscall" },
    { 165, "quotactl" },
    { 166, "set_tid_address" },
    { 167, "mount" },
    { 168, "ustat" },
    { 169, "setxattr" },  // 32 bit
    { 170, "lsetxattr" }, // 32 bit
    { 171, "fsetxattr" }, // 32 bit
    { 172, "getxattr" },
    { 173, "lgetxattr" },
    { 174, "getdents" },
    { 175, "setsid" },
    { 176, "fchdir" },
    { 177, "fgetxattr" }, // 32 bit
    { 178, "listxattr" },
    { 179, "llistxattr" },
    { 180, "flistxattr" }, // 32 bit
    { 181, "removexattr" },
    { 182, "lremovexattr" },
    { 183, "sigpending" },
    { 184, "query_module" },
    { 185, "setpgid" },                   // 32 bit
    { 186, "fremovexattr" },              // 32 bit
    { 187, "tkill" },                     // 32 bit
    { 188, "exit_group", exitGroupFunc }, // 32 bit
    { 189, "uname", unameFunc },
    { 190, "init_module" }, // 32 bit
    { 191, "personality" },
    { 192, "remap_file_pages" },
    { 193, "epoll_create" }, // 32 bit
    { 194, "epoll_ctl" },    // 32 bit
    { 195, "epoll_wait" },   // 32 bit
    { 196, "ioprio_set" },   // 32 bit
    { 197, "getppid", getppidFunc },
    { 198, "sigaction" }, // 32 bit
    { 199, "sgetmask" },
    { 200, "ssetmask" },
    { 201, "sigsuspend" },
    { 202, "oldlstat" },
    { 203, "uselib" },
    { 204, "readdir" },
    { 205, "readahead" },      // 32 bit
    { 206, "socketcall" },     // 32 bit
    { 207, "syslog" },         // 32 bit
    { 208, "lookup_dcookie" }, // 32 bit
    { 209, "fadvise64" },      // 32 bit
    { 210, "fadvise64_64" },   // 32 bit
    { 211, "tgkill" },         // 32 bit
    { 212, "waitpid" },        // 32 bit
    { 213, "swapoff" },
    { 214, "sysinfo", sysinfoFunc<Sparc32Linux> }, // 32 bit
    { 215, "ipc" },                                // 32 bit
    { 216, "sigreturn" },                          // 32 bit
    { 217, "clone", cloneFunc<Sparc32Linux> },
    { 218, "ioprio_get" },  // 32 bit
    { 219, "adjtimex" },    // 32 bit
    { 220, "sigprocmask" }, // 32 bit
    { 221, "create_module" },
    { 222, "delete_module" }, // 32 bit
    { 223, "get_kernel_syms" },
    { 224, "getpgid" }, // 32 bit
    { 225, "bdflush" }, // 32 bit
    { 226, "sysfs" },   // 32 bit
    { 227, "afs_syscall" },
    { 228, "setfsuid" },   // 32 bit
    { 229, "setfsgid" },   // 32 bit
    { 230, "_newselect" }, // 32 bit
    { 231, "time", ignoreFunc },
    { 232, "oldstat" },
    { 233, "stime" },
    { 234, "statfs64" },
    { 235, "fstatfs64" },
    { 236, "_llseek", _llseekFunc },
    { 237, "mlock" },
    { 238, "munlock" },
    { 239, "mlockall" }, // 32 bit
    { 240, "munlockall" },
    { 241, "sched_setparam" },     // 32 bit
    { 242, "sched_getparam" },     // 32 bit
    { 243, "sched_setscheduler" }, // 32 bit
    { 244, "sched_getscheduler" }, // 32 bit
    { 245, "sched_yield" },
    { 246, "sched_get_priority_max" }, // 32 bit
    { 247, "sched_get_priority_min" }, // 32 bit
    { 248, "sched_rr_get_interval" },  // 32 bit
    { 249, "nanosleep" },
    { 250, "mremap", mremapFunc<Sparc32Linux> }, // 32 bit
    { 251, "_sysctl" },                          // 32 bit
    { 252, "getsid" },                           // 32 bit
    { 253, "fdatasync" },
    { 254, "nfsservctl" }, // 32 bit
    { 255, "aplib" },
    { 256, "clock_settime" },
    { 257, "clock_gettime" },
    { 258, "clock_getres" },
    { 259, "clock_nanosleep" }, // 32 bit
    { 260, "sched_getaffinity" },
    { 261, "sched_setaffinity" },
    { 262, "timer_settime" }, // 32 bit
    { 263, "timer_gettime" },
    { 264, "timer_getoverrun" },
    { 265, "timer_delete" },
    { 266, "timer_create" },
    { 267, "vserver" },
    { 268, "io_setup" },
    { 269, "io_destroy" },
    { 270, "io_submit" }, // 32 bit
    { 271, "io_cancel" },
    { 272, "io_getevents" },
    { 273, "mq_open" }, // 32 bit
    { 274, "mq_unlink" },
    { 275, "mq_timedsend" },
    { 276, "mq_timedreceive" },
    { 277, "mq_notify" },
    { 278, "mq_getsetattr" },
    { 279, "waitid" },
    { 280, "sys_setaltroot" },
    { 281, "add_key" },
    { 282, "request_key" },
    { 283, "keyctl" },
    { 284, "openat" },
    { 285, "mkdirat" },
    { 286, "mknodat" },
    { 287, "fchownat" },
    { 288, "futimesat" },
    { 289, "fstatat64" },
    { 290, "unlinkat" },
    { 291, "renameat" },
    { 292, "linkat" },
    { 293, "symlinkat" },
    { 294, "readlinkat" },
    { 295, "fchmodat" },
    { 296, "faccessat" },
    { 297, "pselect6" },
    { 298, "ppoll" },
    { 299, "unshare" }
};

SyscallDescTable<SEWorkload::SyscallABI64> EmuLinux::syscallDescs = {
    { 0, "restart_syscall" },
    { 1, "exit", exitFunc },
    { 2, "fork" },
    { 3, "read", readFunc<SparcLinux> },
    { 4, "write", writeFunc<SparcLinux> },
    { 5, "open", openFunc<SparcLinux> },
    { 6, "close", closeFunc },
    { 7, "wait4" },
    { 8, "creat" },
    { 9, "link" },
    { 10, "unlink", unlinkFunc },
    { 11, "execv" },
    { 12, "chdir" },
    { 13, "chown", chownFunc },
    { 14, "mknod" },
    { 15, "chmod", chmodFunc<Linux> },
    { 16, "lchown" },
    { 17, "brk", brkFunc },
    { 18, "perfctr" },
    { 19, "lseek", lseekFunc },
    { 20, "getpid", getpidFunc },
    { 21, "capget" },
    { 22, "capset" },
    { 23, "setuid", ignoreFunc },
    { 24, "getuid", getuidFunc },
    { 25, "time" },
    { 26, "ptrace" },
    { 27, "alarm" },
    { 28, "sigaltstack" },
    { 29, "pause" },
    { 30, "utime" },
    { 31, "lchown32" },
    { 32, "fchown32" },
    { 33, "access" },
    { 34, "nice" },
    { 35, "chown32" },
    { 36, "sync" },
    { 37, "kill" },
    { 38, "stat" },
    { 39, "sendfile" },
    { 40, "lstat" },
    { 41, "dup" },
    { 42, "pipe", pipePseudoFunc },
    { 43, "times", ignoreFunc },
    { 44, "getuid32" },
    { 45, "umount2" },
    { 46, "setgid" },
    { 47, "getgid", getgidFunc },
    { 48, "signal" },
    { 49, "geteuid", geteuidFunc },
    { 50, "getegid", getegidFunc },
    { 51, "acct" },
    { 52, "memory_ordering" },
    { 53, "getgid32" },
    { 54, "ioctl" },
    { 55, "reboot" },
    { 56, "mmap2" },
    { 57, "symlink" },
    { 58, "readlink", readlinkFunc<SparcLinux> },
    { 59, "execve" },
    { 60, "umask" },
    { 61, "chroot" },
    { 62, "fstat", fstatFunc<SparcLinux> },
    { 63, "fstat64", fstat64Func<SparcLinux> },
    { 64, "getpagesize" },
    { 65, "msync" },
    { 66, "vfork" },
    { 67, "pread64" },
    { 68, "pwrite64" },
    { 69, "geteuid32" },
    { 70, "getegid32" },
    { 71, "mmap", mmapFunc<SparcLinux> },
    { 72, "setreuid32" },
    { 73, "munmap", munmapFunc<SparcLinux> },
    { 74, "mprotect", ignoreFunc },
    { 75, "madvise" },
    { 76, "vhangup" },
    { 77, "truncate64" },
    { 78, "mincore" },
    { 79, "getgroups" },
    { 80, "setgroups" },
    { 81, "getpgrp" },
    { 82, "setgroups32" },
    { 83, "setitimer" },
    { 84, "ftruncate64" },
    { 85, "swapon" },
    { 86, "getitimer" },
    { 87, "setuid32" },
    { 88, "sethostname" },
    { 89, "setgid32" },
    { 90, "dup2" },
    { 91, "setfsuid32" },
    { 92, "fcntl" },
    { 93, "select" },
    { 94, "setfsgid32" },
    { 95, "fsync" },
    { 96, "setpriority" },
    { 97, "socket" },
    { 98, "connect" },
    { 99, "accept" },
    { 100, "getpriority" },
    { 101, "rt_sigreturn" },
    { 102, "rt_sigaction", ignoreFunc },
    { 103, "rt_sigprocmask", ignoreFunc },
    { 104, "rt_sigpending" },
    { 105, "rt_sigtimedwait" },
    { 106, "rt_sigqueueinfo" },
    { 107, "rt_sigsuspend" },
    { 108, "setresuid" },
    { 109, "getresuid", getresuidFunc },
    { 110, "setresgid", ignoreFunc },
    { 111, "getresgid" },
    { 112, "setregid32" },
    { 113, "recvmsg" },
    { 114, "sendmsg" },
    { 115, "getgroups32" },
    { 116, "gettimeofday", gettimeofdayFunc<SparcLinux> },
    { 117, "getrusage" },
    { 118, "getsockopt" },
    { 119, "getcwd" },
    { 120, "readv" },
    { 121, "writev" },
    { 122, "settimeofday" },
    { 123, "fchown" },
    { 124, "fchmod" },
    { 125, "recvfrom" },
    { 126, "setreuid" },
    { 127, "setregid" },
    { 128, "rename", renameFunc },
    { 129, "truncate" },
    { 130, "ftruncate" },
    { 131, "flock" },
    { 132, "lstat64" },
    { 133, "sendto" },
    { 134, "shutdown" },
    { 135, "socketpair" },
    { 136, "mkdir", mkdirFunc },
    { 137, "rmdir" },
    { 138, "utimes" },
    { 139, "stat64" },
    { 140, "sendfile64" },
    { 141, "getpeername" },
    { 142, "futex" },
    { 143, "gettid" },
    { 144, "getrlimit" },
    { 145, "setrlimit" },
    { 146, "pivot_root" },
    { 147, "prctl" },
    { 148, "pciconfig_read" },
    { 149, "pciconfig_write" },
    { 150, "getsockname" },
    { 151, "inotify_init" },
    { 152, "inotify_add_watch" },
    { 153, "poll" },
    { 154, "getdents64" },
    { 155, "fcntl64" },
    { 156, "inotify_rm_watch" },
    { 157, "statfs" },
    { 158, "fstatfs" },
    { 159, "umount" },
    { 160, "sched_set_affinity" },
    { 161, "sched_get_affinity" },
    { 162, "getdomainname" },
    { 163, "setdomainname" },
    { 164, "utrap_install" },
    { 165, "quotactl" },
    { 166, "set_tid_address" },
    { 167, "mount" },
    { 168, "ustat" },
    { 169, "setxattr" },
    { 170, "lsetxattr" },
    { 171, "fsetxattr" },
    { 172, "getxattr" },
    { 173, "lgetxattr" },
    { 174, "getdents" },
    { 175, "setsid" },
    { 176, "fchdir" },
    { 177, "fgetxattr" },
    { 178, "listxattr" },
    { 179, "llistxattr" },
    { 180, "flistxattr" },
    { 181, "removexattr" },
    { 182, "lremovexattr" },
    { 183, "sigpending" },
    { 184, "query_module" },
    { 185, "setpgid" },
    { 186, "fremovexattr" },
    { 187, "tkill" },
    { 188, "exit_group", exitGroupFunc },
    { 189, "uname", unameFunc },
    { 190, "init_module" },
    { 191, "personality" },
    { 192, "remap_file_pages" },
    { 193, "epoll_create" },
    { 194, "epoll_ctl" },
    { 195, "epoll_wait" },
    { 196, "ioprio_set" },
    { 197, "getppid", getppidFunc },
    { 198, "sigaction", ignoreFunc },
    { 199, "sgetmask" },
    { 200, "ssetmask" },
    { 201, "sigsuspend" },
    { 202, "oldlstat" },
    { 203, "uselib" },
    { 204, "readdir" },
    { 205, "readahead" },
    { 206, "socketcall" },
    { 207, "syslog" },
    { 208, "lookup_dcookie" },
    { 209, "fadvise64" },
    { 210, "fadvise64_64" },
    { 211, "tgkill" },
    { 212, "waitpid" },
    { 213, "swapoff" },
    { 214, "sysinfo", sysinfoFunc<SparcLinux> },
    { 215, "ipc" },
    { 216, "sigreturn" },
    { 217, "clone", cloneFunc<SparcLinux> },
    { 218, "ioprio_get" },
    { 219, "adjtimex" },
    { 220, "sigprocmask" },
    { 221, "create_module" },
    { 222, "delete_module" },
    { 223, "get_kernel_syms" },
    { 224, "getpgid" },
    { 225, "bdflush" },
    { 226, "sysfs" },
    { 227, "afs_syscall" },
    { 228, "setfsuid" },
    { 229, "setfsgid" },
    { 230, "_newselect" },
    { 231, "time", ignoreFunc },
    { 232, "oldstat" },
    { 233, "stime" },
    { 234, "statfs64" },
    { 235, "fstatfs64" },
    { 236, "_llseek", _llseekFunc },
    { 237, "mlock" },
    { 238, "munlock" },
    { 239, "mlockall" },
    { 240, "munlockall" },
    { 241, "sched_setparam" },
    { 242, "sched_getparam" },
    { 243, "sched_setscheduler" },
    { 244, "sched_getscheduler" },
    { 245, "sched_yield" },
    { 246, "sched_get_priority_max" },
    { 247, "sched_get_priority_min" },
    { 248, "sched_rr_get_interval" },
    { 249, "nanosleep" },
    { 250, "mremap", mremapFunc<SparcLinux> },
    { 251, "_sysctl" },
    { 252, "getsid" },
    { 253, "fdatasync" },
    { 254, "nfsservctl" },
    { 255, "aplib" },
    { 256, "clock_settime" },
    { 257, "clock_gettime" },
    { 258, "clock_getres" },
    { 259, "clock_nanosleep" },
    { 260, "sched_getaffinity" },
    { 261, "sched_setaffinity" },
    { 262, "timer_settime" },
    { 263, "timer_gettime" },
    { 264, "timer_getoverrun" },
    { 265, "timer_delete" },
    { 266, "timer_create" },
    { 267, "vserver" },
    { 268, "io_setup" },
    { 269, "io_destroy" },
    { 270, "io_submit" },
    { 271, "io_cancel" },
    { 272, "io_getevents" },
    { 273, "mq_open" },
    { 274, "mq_unlink" },
    { 275, "mq_timedsend" },
    { 276, "mq_timedreceive" },
    { 277, "mq_notify" },
    { 278, "mq_getsetattr" },
    { 279, "waitid" },
    { 280, "sys_setaltroot" },
    { 281, "add_key" },
    { 282, "request_key" },
    { 283, "keyctl" }
};

} // namespace SparcISA
} // namespace gem5
