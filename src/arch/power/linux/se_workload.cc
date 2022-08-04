/*
 * Copyright 2003-2005 The Regents of The University of Michigan
 * Copyright 2007-2008 The Florida State University
 * Copyright 2009 The University of Edinburgh
 * Copyright 2020 Google Inc.
 * Copyright 2021 IBM Corporation
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

#include "arch/power/linux/se_workload.hh"

#include <sys/syscall.h>

#include "arch/power/process.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "sim/syscall_emul.hh"

namespace gem5
{

namespace
{

class LinuxLoader : public Process::Loader
{
  public:
    Process *
    load(const ProcessParams &params, loader::ObjectFile *obj) override
    {
        auto arch = obj->getArch();

        if (arch != loader::Power && arch != loader::Power64)
            return nullptr;

        auto opsys = obj->getOpSys();

        if (opsys == loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = loader::Linux;
        }

        if ((arch == loader::Power && opsys != loader::Linux) ||
            (arch == loader::Power64 &&
             opsys != loader::LinuxPower64ABIv1 &&
             opsys != loader::LinuxPower64ABIv2))
            return nullptr;

        return new PowerProcess(params, obj);
    }
};

LinuxLoader linuxLoader;

} // anonymous namespace

namespace PowerISA
{

void
EmuLinux::syscall(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    syscallDescs.get(tc->getReg(int_reg::R0))->doSyscall(tc);
}

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "power");

    return 0;
}

SyscallDescTable<PowerISA::SEWorkload::SyscallABI> EmuLinux::syscallDescs = {
    {  0, "syscall" },
    {  1, "exit", exitFunc },
    {  2, "fork" },
    {  3, "read", readFunc<PowerLinux> },
    {  4, "write", writeFunc<PowerLinux> },
    {  5, "open", openFunc<PowerLinux> },
    {  6, "close", closeFunc },
    {  7, "waitpid" }, //???
    {  8, "creat" },
    {  9, "link" },
    { 10, "unlink", unlinkFunc },
    { 11, "execve" },
    { 12, "chdir" },
    { 13, "time", timeFunc<PowerLinux> },
    { 14, "mknod" },
    { 15, "chmod", chmodFunc<PowerLinux> },
    { 16, "lchown", chownFunc },
    { 17, "break", brkFunc }, //???
    { 18, "unused#18" }, //???
    { 19, "lseek", lseekFunc },
    { 20, "getpid", getpidFunc },
    { 21, "mount" },
    { 22, "umount" },
    { 23, "setuid", ignoreFunc },
    { 24, "getuid", getuidFunc },
    { 25, "stime" },
    { 26, "ptrace" },
    { 27, "alarm" },
    { 28, "unused#28" },
    { 29, "pause" },
    { 30, "utime" },
    { 31, "stty" },
    { 32, "gtty" },
    { 33, "access" },
    { 34, "nice" },
    { 35, "ftime" },
    { 36, "sync" },
    { 37, "kill", ignoreFunc },
    { 38, "rename", renameFunc },
    { 39, "mkdir" },
    { 40, "rmdir" },
    { 41, "dup", dupFunc },
    { 42, "pipe" },
    { 43, "times", timesFunc<PowerLinux> },
    { 44, "prof" },
    { 45, "brk", brkFunc },
    { 46, "setgid" },
    { 47, "getgid", getgidFunc },
    { 48, "signal", ignoreFunc },
    { 49, "geteuid", geteuidFunc },
    { 50, "getegid", getegidFunc },
    { 51, "acct" },
    { 52, "umount2" },
    { 53, "lock" },
    { 54, "ioctl", ioctlFunc<PowerLinux> },
    { 55, "fcntl", fcntlFunc },
    { 56, "mpx" },
    { 57, "setpgid" },
    { 58, "ulimit" },
    { 59, "unused#59" },
    { 60, "umask", umaskFunc },
    { 61, "chroot" },
    { 62, "ustat" },
    { 63, "dup2" },
    { 64, "getppid", getpagesizeFunc },
    { 65, "getpgrp" },
    { 66, "setsid" },
    { 67, "sigaction" },
    { 68, "sgetmask" },
    { 69, "ssetmask" },
    { 70, "setreuid" },
    { 71, "setregid" },
    { 72, "sigsuspend" },
    { 73, "sigpending" },
    { 74, "sethostname", ignoreFunc },
    { 75, "setrlimit", ignoreFunc },
    { 76, "getrlimit" },
    { 77, "getrusage", ignoreFunc },
    { 78, "gettimeofday" },
    { 79, "settimeofday" },
    { 80, "getgroups" },
    { 81, "setgroups" },
    { 82, "reserved#82" },
    { 83, "symlink" },
    { 84, "unused#84" },
    { 85, "readlink", readlinkFunc<PowerLinux> },
    { 86, "uselib" },
    { 87, "swapon", gethostnameFunc },
    { 88, "reboot" },
    { 89, "readdir" },
    { 90, "mmap", mmapFunc<PowerLinux> },
    { 91, "munmap",munmapFunc<PowerLinux> },
    { 92, "truncate", truncateFunc<PowerLinux> },
    { 93, "ftruncate", ftruncateFunc<PowerLinux> },
    { 94, "fchmod" },
    { 95, "fchown" },
    { 96, "getpriority" },
    { 97, "setpriority" },
    { 98, "profil" },
    { 99, "statfs" },
    { 100, "fstatfs" },
    { 101, "ioperm" },
    { 102, "socketcall" },
    { 103, "syslog" },
    { 104, "setitimer" },
    { 105, "getitimer" },
    { 106, "stat",  statFunc<PowerLinux> },
    { 107, "lstat" },
    { 108, "fstat", fstatFunc<PowerLinux> },
    { 109, "unused#109" },
    { 110, "iopl" },
    { 111, "vhangup" },
    { 112, "idle", ignoreFunc },
    { 113, "vm86" },
    { 114, "wait4" },
    { 115, "swapoff" },
    { 116, "sysinfo" },
    { 117, "ipc" },
    { 118, "fsync" },
    { 119, "sigreturn" },
    { 120, "clone", cloneBackwardsFunc<PowerLinux> },
    { 121, "setdomainname" },
    { 122, "uname", unameFunc },
    { 123, "modify_ldt" },
    { 124, "adjtimex" },
    { 125, "mprotect", ignoreFunc },
    { 126, "sigprocmask" },
    { 127, "create_module" },
    { 128, "init_module" },
    { 129, "delete_module" },
    { 130, "get_kernel_syms" },
    { 131, "quotactl" },
    { 132, "getpgid" },
    { 133, "fchdir" },
    { 134, "bdflush" },
    { 135, "sysfs" },
    { 136, "personality" },
    { 137, "afs_syscall" },
    { 138, "setfsuid" },
    { 139, "setfsgid" },
    { 140, "llseek", _llseekFunc },
    { 141, "getdents" },
    { 142, "newselect" },
    { 143, "flock" },
    { 144, "msync" },
    { 145, "readv" },
    { 146, "writev", writevFunc<PowerLinux> },
    { 147, "getsid" },
    { 148, "fdatasync" },
    { 149, "sysctl" },
    { 150, "mlock" },
    { 151, "munlock" },
    { 152, "mlockall" },
    { 153, "munlockall" },
    { 154, "sched_setparam" },
    { 155, "sched_getparam" },
    { 156, "sched_setscheduler" },
    { 157, "sched_getscheduler" },
    { 158, "sched_yield" },
    { 159, "sched_get_priority_max" },
    { 160, "sched_get_priority_min" },
    { 161, "sched_rr_get_interval" },
    { 162, "nanosleep" },
    { 163, "mremap" },
    { 164, "setresuid" },
    { 165, "getresuid" },
    { 166, "vm862" },
    { 167, "query_module" },
    { 168, "poll" },
    { 169, "nfsservctl" },
    { 170, "setresgid" },
    { 171, "getresgid" },
    { 172, "prctl" },
    { 173, "rt_sigaction", ignoreFunc },
    { 174, "rt_sigprocmask" },
    { 175, "unknown#175" },
    { 176, "rt_sigpending" },
    { 177, "rt_sigtimedwait" },
    { 178, "rt_sigqueueinfo", ignoreFunc },
    { 179, "rt_sigsuspend" },
    { 180, "pread64" },
    { 181, "pwrite64" },
    { 182, "chown" },
    { 183, "getcwd" },
    { 184, "capget" },
    { 185, "capset" },
    { 186, "sigaltstack" },
    { 187, "sendfile" },
    { 188, "getpmsg" },
    { 189, "putpmsg" },
    { 190, "ugetrlimit", ignoreFunc },
    { 191, "getrlimit" },
    { 192, "mmap2", mmapFunc<PowerLinux> },
    { 193, "truncate64" },
    { 194, "ftruncate64", ftruncate64Func },
    { 195, "stat64", stat64Func<PowerLinux> },
    { 196, "lstat64", lstat64Func<PowerLinux> },
    { 197, "fstat64", fstat64Func<PowerLinux> },
    { 198, "lchown" },
    { 199, "getuid", getuidFunc },
    { 200, "getgid", getgidFunc },
    { 201, "geteuid", geteuidFunc },
    { 202, "getegid", getegidFunc },
    { 203, "setreuid" },
    { 204, "fcntl64", fcntl64Func },
    { 205, "getgroups" },
    { 206, "setgroups" },
    { 207, "fchown" },
    { 208, "setresuid" },
    { 209, "getresuid" },
    { 210, "setresgid" },
    { 211, "getresgid" },
    { 212, "chown" },
    { 213, "setuid" },
    { 214, "setgid" },
    { 215, "setfsuid" },
    { 216, "setfsgid" },
    { 217, "getdents64" },
    { 218, "pivot_root" },
    { 219, "mincore" },
    { 220, "madvise" },
    { 221, "unknown#221" },
    { 222, "tux" },
    { 223, "unknown#223" },
    { 224, "gettid" },
    { 225, "readahead" },
    { 226, "setxattr" },
    { 227, "lsetxattr" },
    { 228, "fsetxattr" },
    { 229, "getxattr" },
    { 230, "lgetxattr" },
    { 231, "fgetxattr" },
    { 232, "listxattr" },
    { 233, "llistxattr" },
    { 234, "exit_group", exitGroupFunc },
    { 235, "removexattr" },
    { 236, "lremovexattr" },
    { 237, "fremovexattr" },
    { 238, "tkill" },
    { 239, "sendfile64" },
    { 240, "futex" },
    { 241, "sched_setaffinity" },
    { 242, "sched_getaffinity" },
    { 243, "io_setup" },
    { 244, "io_destory" },
    { 245, "io_getevents" },
    { 246, "io_submit" },
    { 247, "io_cancel" },
    { 248, "unknown#248" },
    { 249, "lookup_dcookie" },
    { 250, "epoll_create" },
    { 251, "epoll_ctl" },
    { 252, "epoll_wait" },
    { 253, "remap_file_pages" },
    { 254, "set_thread_area" },
    { 255, "get_thread_area" },
    { 256, "set_tid_address" },
    { 257, "timer_create" },
    { 258, "timer_settime" },
    { 259, "timer_gettime" },
    { 260, "timer_getoverrun" },
    { 261, "timer_delete" },
    { 262, "clock_settime" },
    { 263, "clock_gettime" },
    { 264, "clock_getres" },
    { 265, "clock_nanosleep" },
    { 266, "statfs64" },
    { 267, "fstatfs64" },
    { 268, "tgkill" },
    { 269, "utimes" },
    { 270, "arm_fadvise64_64" },
    { 271, "pciconfig_iobase" },
    { 272, "pciconfig_read" },
    { 273, "pciconfig_write" },
    { 274, "mq_open" },
    { 275, "mq_unlink" },
    { 276, "mq_timedsend" },
    { 277, "mq_timedreceive" },
    { 278, "mq_notify" },
    { 279, "mq_getsetattr" },
    { 280, "waitid" },
    { 281, "socket" },
    { 282, "bind" },
    { 283, "connect" },
    { 284, "listen" },
    { 285, "accept" },
    { 286, "getsockname" },
    { 287, "getpeername" },
    { 288, "socketpair" },
    { 289, "send" },
    { 290, "sendto" },
    { 291, "recv" },
    { 292, "recvfrom" },
    { 293, "shutdown" },
    { 294, "setsockopt" },
    { 295, "getsockopt" },
    { 296, "sendmsg" },
    { 297, "rcvmsg" },
    { 298, "semop" },
    { 299, "semget" },
    { 300, "semctl" },
    { 301, "msgsend" },
    { 302, "msgrcv" },
    { 303, "msgget" },
    { 304, "msgctl" },
    { 305, "shmat" },
    { 306, "shmdt" },
    { 307, "shmget" },
    { 308, "shmctl" },
    { 309, "add_key" },
    { 310, "request_key" },
    { 311, "keyctl" },
    { 312, "semtimedop" },
    { 313, "vserver" },
    { 314, "ioprio_set" },
    { 315, "ioprio_get" },
    { 316, "inotify_init" },
    { 317, "inotify_add_watch" },
    { 318, "inotify_rm_watch" },
    { 319, "mbind" },
    { 320, "get_mempolicy" },
    { 321, "set_mempolicy" },
    { 322, "openat" },
    { 323, "mkdirat" },
    { 324, "mknodat" },
    { 325, "fchownat" },
    { 326, "futimesat" },
    { 327, "fstatat64" },
    { 328, "unlinkat" },
    { 329, "renameat" },
    { 330, "linkat" },
    { 331, "symlinkat" },
    { 332, "readlinkat" },
    { 333, "fchmodat" },
    { 334, "faccessat" },
    { 335, "pselect6" },
    { 336, "ppoll" },
    { 337, "unshare" },
    { 338, "set_robust_list" },
    { 339, "get_robust_list" },
    { 340, "splice" },
    { 341, "arm_sync_file_range" },
    { 342, "tee" },
    { 343, "vmsplice" },
    { 344, "move_pages" },
    { 345, "getcpu" },
    { 346, "epoll_pwait" },
};

} // namespace PowerISA
} // namespace gem5
