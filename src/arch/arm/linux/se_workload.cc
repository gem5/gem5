/*
 * Copyright (c) 2010-2013, 2015, 2020, 2024 ARM Limited
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
 * Copyright 2003-2005 The Regents of The University of Michigan
 * Copyright 2007-2008 The Florida State University
 * Copyright 2020 Google Inc.
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

#include "arch/arm/linux/se_workload.hh"

#include <sys/syscall.h>

#include "arch/arm/linux/process.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/se_translating_port_proxy.hh"
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
        auto opsys = obj->getOpSys();

        if (arch != loader::Arm && arch != loader::Thumb &&
                arch != loader::Arm64) {
            return nullptr;
        }

        if (opsys == loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = loader::Linux;
        }

        if (opsys == loader::LinuxArmOABI) {
            fatal("gem5 does not support ARM OABI binaries. Please recompile "
                    "with an EABI compiler.");
        }

        if (opsys != loader::Linux)
            return nullptr;

        if (arch == loader::Arm64)
            return new ArmLinuxProcess64(params, obj, arch);
        else
            return new ArmLinuxProcess32(params, obj, arch);
    }
};

LinuxLoader linuxLoader;

} // anonymous namespace

namespace ArmISA
{

/// Target uname() handler.
static SyscallReturn
unameFunc32(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "m5.eecs.umich.edu");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 SMP Sat Dec  1 00:00:00 GMT 2012");
    strcpy(name->machine, "armv7l");

    return 0;
}

/// Target uname() handler.
static SyscallReturn
unameFunc64(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "gem5");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 SMP Sat Dec  1 00:00:00 GMT 2012");
    strcpy(name->machine, "armv8l");

    return 0;
}

/// Target set_tls() handler.
static SyscallReturn
setTLSFunc32(SyscallDesc *desc, ThreadContext *tc, uint32_t tlsPtr)
{
    SETranslatingPortProxy(tc).writeBlob(
            ArmLinuxProcess32::commPage + 0x0ff0, &tlsPtr, sizeof(tlsPtr));
    tc->setMiscReg(MISCREG_TPIDRURO, tlsPtr);
    return 0;
}

static SyscallReturn
setTLSFunc64(SyscallDesc *desc, ThreadContext *tc, uint32_t tlsPtr)
{
    tc->setMiscReg(MISCREG_TPIDRRO_EL0, tlsPtr);
    return 0;
}

class SyscallTable32 : public SyscallDescTable<EmuLinux::SyscallABI32>
{
  public:
    SyscallTable32(int base) : SyscallDescTable<EmuLinux::SyscallABI32>({
        {  base + 0, "syscall" },
        {  base + 1, "exit", exitFunc },
        {  base + 2, "fork" },
        {  base + 3, "read", readFunc<ArmLinux32> },
        {  base + 4, "write", writeFunc<ArmLinux32> },
        {  base + 5, "open", openFunc<ArmLinux32> },
        {  base + 6, "close", closeFunc },
        {  base + 8, "creat" },
        {  base + 9, "link" },
        { base + 10, "unlink", unlinkFunc },
        { base + 11, "execve", execveFunc<ArmLinux32> },
        { base + 12, "chdir", chdirFunc },
        { base + 13, "time", timeFunc<ArmLinux32> },
        { base + 14, "mknod", mknodFunc },
        { base + 15, "chmod", chmodFunc<ArmLinux32> },
        { base + 16, "lchown", chownFunc },
        { base + 19, "lseek", lseekFunc<ArmLinux32> },
        { base + 20, "getpid", getpidFunc },
        { base + 21, "mount" },
        { base + 22, "umount" },
        { base + 23, "setuid", ignoreFunc },
        { base + 24, "getuid", getuidFunc },
        { base + 25, "stime" },
        { base + 26, "ptrace" },
        { base + 27, "alarm" },
        { base + 29, "pause" },
        { base + 30, "utime" },
        { base + 33, "access", accessFunc },
        { base + 34, "nice" },
        { base + 36, "sync" },
        { base + 37, "kill", ignoreFunc },
        { base + 38, "rename", renameFunc },
        { base + 39, "mkdir", mkdirFunc },
        { base + 40, "rmdir" },
        { base + 41, "dup", dupFunc },
        { base + 42, "pipe", pipePseudoFunc },
        { base + 43, "times", timesFunc<ArmLinux32> },
        { base + 45, "brk", brkFunc },
        { base + 46, "setgid", ignoreFunc },
        { base + 47, "getgid", getgidFunc },
        { base + 49, "geteuid", geteuidFunc },
        { base + 50, "getegid", getegidFunc },
        { base + 51, "acct" },
        { base + 52, "umount2" },
        { base + 54, "ioctl", ioctlFunc<ArmLinux32> },
        { base + 55, "fcntl", fcntlFunc },
        { base + 57, "setpgid" },
        { base + 60, "umask", umaskFunc },
        { base + 61, "chroot" },
        { base + 62, "ustat" },
        { base + 63, "dup2" },
        { base + 64, "getppid", getppidFunc },
        { base + 65, "getpgrp" },
        { base + 66, "setsid" },
        { base + 67, "sigaction" },
        { base + 70, "setreuid" },
        { base + 71, "setregid" },
        { base + 72, "sigsuspend" },
        { base + 73, "sigpending" },
        { base + 74, "sethostname", ignoreFunc },
        { base + 75, "setrlimit", ignoreFunc },
        { base + 76, "getrlimit", getrlimitFunc<ArmLinux32> },
        { base + 77, "getrusage", getrusageFunc<ArmLinux32> },
        { base + 78, "gettimeofday", gettimeofdayFunc<ArmLinux32> },
        { base + 79, "settimeofday" },
        { base + 80, "getgroups" },
        { base + 81, "setgroups" },
        { base + 82, "reserved#82" },
        { base + 83, "symlink" },
        { base + 85, "readlink", readlinkFunc<ArmLinux32> },
        { base + 86, "uselib" },
        { base + 87, "swapon" },
        { base + 88, "reboot" },
        { base + 89, "readdir" },
        { base + 90, "mmap", mmapFunc<ArmLinux32> },
        { base + 91, "munmap", munmapFunc<ArmLinux32> },
        { base + 92, "truncate", truncateFunc<ArmLinux32> },
        { base + 93, "ftruncate", ftruncateFunc<ArmLinux32> },
        { base + 94, "fchmod" },
        { base + 95, "fchown", fchownFunc },
        { base + 96, "getpriority" },
        { base + 97, "setpriority" },
        { base + 99, "statfs" },
        { base + 100, "fstatfs" },
        { base + 102, "socketcall" },
        { base + 103, "syslog" },
        { base + 104, "setitimer" },
        { base + 105, "getitimer" },
        { base + 106, "stat",  statFunc<ArmLinux32> },
        { base + 107, "lstat" },
        { base + 108, "fstat", fstatFunc<ArmLinux32> },
        { base + 111, "vhangup" },
        { base + 113, "syscall" },
        { base + 114, "wait4" },
        { base + 115, "swapoff" },
        { base + 116, "sysinfo", sysinfoFunc<ArmLinux32> },
        { base + 117, "ipc" },
        { base + 118, "fsync" },
        { base + 119, "sigreturn" },
        { base + 120, "clone", cloneBackwardsFunc<ArmLinux32> },
        { base + 121, "setdomainname" },
        { base + 122, "uname", unameFunc32 },
        { base + 124, "adjtimex" },
        { base + 125, "mprotect", ignoreFunc },
        { base + 126, "sigprocmask", ignoreWarnOnceFunc },
        { base + 128, "init_module" },
        { base + 129, "delete_module" },
        { base + 131, "quotactl" },
        { base + 132, "getpgid" },
        { base + 133, "fchdir" },
        { base + 134, "bdflush" },
        { base + 135, "sysfs" },
        { base + 136, "personality" },
        { base + 137, "reserved#138" },
        { base + 138, "setfsuid" },
        { base + 139, "setfsgid" },
        { base + 140, "llseek", _llseekFunc },
#if defined(SYS_getdents)
        { base + 141, "getdents", getdentsFunc },
#else
        { base + 141, "getdents" },
#endif
        { base + 142, "newselect" },
        { base + 143, "flock" },
        { base + 144, "msync" },
        { base + 145, "readv", readvFunc<ArmLinux32> },
        { base + 146, "writev", writevFunc<ArmLinux32> },
        { base + 147, "getsid" },
        { base + 148, "fdatasync" },
        { base + 149, "sysctl" },
        { base + 150, "mlock" },
        { base + 151, "munlock" },
        { base + 152, "mlockall" },
        { base + 153, "munlockall" },
        { base + 154, "sched_setparam", ignoreWarnOnceFunc },
        { base + 155, "sched_getparam", sched_getparamFunc },
        { base + 156, "sched_setscheduler", ignoreWarnOnceFunc },
        { base + 157, "sched_getscheduler", ignoreWarnOnceFunc },
        { base + 158, "sched_yield", ignoreWarnOnceFunc },
        { base + 159, "sched_get_priority_max", ignoreWarnOnceFunc },
        { base + 160, "sched_get_priority_min", ignoreWarnOnceFunc },
        { base + 161, "sched_rr_get_interval", ignoreWarnOnceFunc },
        { base + 162, "nanosleep", ignoreWarnOnceFunc },
        { base + 163, "mremap", mremapFunc<ArmLinux32> }, // ARM-specific
        { base + 164, "setresuid" },
        { base + 165, "getresuid" },
        { base + 168, "poll", pollFunc<ArmLinux32> },
        { base + 169, "nfsservctl" },
        { base + 170, "setresgid" },
        { base + 171, "getresgid" },
        { base + 172, "prctl" },
        { base + 173, "rt_sigreturn" },
        { base + 174, "rt_sigaction", ignoreWarnOnceFunc },
        { base + 175, "rt_sigprocmask", ignoreWarnOnceFunc },
        { base + 176, "rt_sigpending" },
        { base + 177, "rt_sigtimedwait" },
        { base + 178, "rt_sigqueueinfo", ignoreFunc },
        { base + 179, "rt_sigsuspend" },
        { base + 180, "pread64", pread64Func<ArmLinux32> },
        { base + 181, "pwrite64", pwrite64Func<ArmLinux32> },
        { base + 182, "chown" },
        { base + 183, "getcwd", getcwdFunc<ArmLinux32> },
        { base + 184, "capget" },
        { base + 185, "capset" },
        { base + 186, "sigaltstack" },
        { base + 187, "sendfile" },
        { base + 190, "vfork" },
        { base + 191, "getrlimit", getrlimitFunc<ArmLinux32> },
        { base + 192, "mmap2", mmapFunc<ArmLinux32> },
        { base + 193, "truncate64", truncate64Func },
        { base + 194, "ftruncate64", ftruncate64Func },
        { base + 195, "stat64", stat64Func<ArmLinux32> },
        { base + 196, "lstat64", lstat64Func<ArmLinux32> },
        { base + 197, "fstat64", fstat64Func<ArmLinux32> },
        { base + 198, "lchown" },
        { base + 199, "getuid", getuidFunc },
        { base + 200, "getgid", getgidFunc },
        { base + 201, "geteuid", geteuidFunc },
        { base + 202, "getegid", getegidFunc },
        { base + 203, "setreuid" },
        { base + 204, "setregid" },
        { base + 205, "getgroups" },
        { base + 206, "setgroups" },
        { base + 207, "fchown", fchownFunc },
        { base + 208, "setresuid" },
        { base + 209, "getresuid" },
        { base + 210, "setresgid" },
        { base + 211, "getresgid" },
        { base + 212, "chown" },
        { base + 213, "setuid", ignoreFunc },
        { base + 214, "setgid", ignoreFunc },
        { base + 215, "setfsuid" },
        { base + 216, "setfsgid" },
#if defined(SYS_getdents64)
        { base + 217, "getdents64", getdents64Func },
#else
        { base + 217, "getdents64" },
#endif
        { base + 218, "pivot_root" },
        { base + 219, "mincore" },
        { base + 220, "madvise", ignoreFunc },
        { base + 221, "fcntl64", fcntl64Func },
        { base + 224, "gettid", gettidFunc },
        { base + 225, "readahead" },
        { base + 226, "setxattr" },
        { base + 227, "lsetxattr" },
        { base + 228, "fsetxattr" },
        { base + 229, "getxattr" },
        { base + 230, "lgetxattr" },
        { base + 231, "fgetxattr" },
        { base + 232, "listxattr" },
        { base + 233, "llistxattr" },
        { base + 234, "flistxattr" },
        { base + 235, "removexattr" },
        { base + 236, "lremovexattr" },
        { base + 237, "fremovexattr" },
        { base + 238, "tkill" },
        { base + 239, "sendfile64" },
        { base + 240, "futex", futexFunc<ArmLinux32> },
        { base + 241, "sched_setaffinity", ignoreWarnOnceFunc },
        { base + 242, "sched_getaffinity", ignoreFunc },
        { base + 243, "io_setup" },
        { base + 244, "io_destroy" },
        { base + 245, "io_getevents" },
        { base + 246, "io_submit" },
        { base + 247, "io_cancel" },
        { base + 248, "exit_group", exitGroupFunc },
        { base + 249, "lookup_dcookie" },
        { base + 250, "epoll_create" },
        { base + 251, "epoll_ctl" },
        { base + 252, "epoll_wait" },
        { base + 253, "remap_file_pages" },
        { base + 256, "set_tid_address", setTidAddressFunc },
        { base + 257, "timer_create" },
        { base + 258, "timer_settime" },
        { base + 259, "timer_gettime" },
        { base + 260, "timer_getoverrun" },
        { base + 261, "timer_delete" },
        { base + 262, "clock_settime" },
        { base + 263, "clock_gettime", clock_gettimeFunc<ArmLinux32> },
        { base + 264, "clock_getres", clock_getresFunc<ArmLinux32> },
        { base + 265, "clock_nanosleep" },
        { base + 266, "statfs64" },
        { base + 267, "fstatfs64" },
        { base + 268, "tgkill", tgkillFunc<ArmLinux32> },
        { base + 269, "utimes", utimesFunc<ArmLinux32> },
        { base + 270, "arm_fadvise64_64" },
        { base + 271, "pciconfig_iobase" },
        { base + 272, "pciconfig_read" },
        { base + 273, "pciconfig_write" },
        { base + 274, "mq_open" },
        { base + 275, "mq_unlink" },
        { base + 276, "mq_timedsend" },
        { base + 277, "mq_timedreceive" },
        { base + 278, "mq_notify" },
        { base + 279, "mq_getsetattr" },
        { base + 280, "waitid" },
        { base + 281, "socket" },
        { base + 282, "bind" },
        { base + 283, "connect" },
        { base + 284, "listen" },
        { base + 285, "accept" },
        { base + 286, "getsockname" },
        { base + 287, "getpeername" },
        { base + 288, "socketpair" },
        { base + 289, "send" },
        { base + 290, "sendto", sendtoFunc<ArmLinux32> },
        { base + 291, "recv" },
        { base + 292, "recvfrom", recvfromFunc<ArmLinux32> },
        { base + 293, "shutdown" },
        { base + 294, "setsockopt" },
        { base + 295, "getsockopt" },
        { base + 296, "sendmsg" },
        { base + 297, "rcvmsg" },
        { base + 298, "semop" },
        { base + 299, "semget" },
        { base + 300, "semctl" },
        { base + 301, "msgsend" },
        { base + 302, "msgrcv" },
        { base + 303, "msgget" },
        { base + 304, "msgctl" },
        { base + 305, "shmat" },
        { base + 306, "shmdt" },
        { base + 307, "shmget" },
        { base + 308, "shmctl" },
        { base + 309, "add_key" },
        { base + 310, "request_key" },
        { base + 311, "keyctl" },
        { base + 312, "semtimedop" },
        { base + 314, "ioprio_set" },
        { base + 315, "ioprio_get" },
        { base + 316, "inotify_init" },
        { base + 317, "inotify_add_watch" },
        { base + 318, "inotify_rm_watch" },
        { base + 319, "mbind" },
        { base + 320, "get_mempolicy" },
        { base + 321, "set_mempolicy" },
        { base + 322, "openat", openatFunc<ArmLinux32> },
        { base + 323, "mkdirat", mkdiratFunc<ArmLinux32> },
        { base + 324, "mknodat", mknodatFunc<ArmLinux32> },
        { base + 325, "fchownat", fchownatFunc<ArmLinux32> },
        { base + 326, "futimesat", futimesatFunc<ArmLinux32> },
        { base + 327, "fstatat64" },
        { base + 328, "unlinkat", unlinkatFunc<ArmLinux32> },
        { base + 329, "renameat", renameatFunc<ArmLinux32> },
        { base + 330, "linkat" },
        { base + 331, "symlinkat" },
        { base + 332, "readlinkat", readlinkatFunc<ArmLinux32> },
        { base + 333, "fchmodat", fchmodatFunc<ArmLinux32> },
        { base + 334, "faccessat", faccessatFunc<ArmLinux32> },
        { base + 335, "pselect6" },
        { base + 336, "ppoll" },
        { base + 337, "unshare" },
        { base + 338, "set_robust_list", ignoreFunc },
        { base + 339, "get_robust_list" },
        { base + 340, "splice" },
        { base + 341, "arm_sync_file_range" },
        { base + 342, "tee" },
        { base + 343, "vmsplice" },
        { base + 344, "move_pages" },
        { base + 345, "getcpu", getcpuFunc },
        { base + 346, "epoll_pwait" },
        { base + 347, "sys_kexec_load" },
        { base + 348, "sys_utimensat" },
        { base + 349, "sys_signalfd" },
        { base + 350, "sys_timerfd_create" },
        { base + 351, "sys_eventfd" },
        { base + 352, "sys_fallocate" },
        { base + 353, "sys_timerfd_settime" },
        { base + 354, "sys_timerfd_gettime" },
        { base + 355, "sys_signalfd4" },
        { base + 356, "sys_eventfd2" },
        { base + 357, "sys_epoll_create1" },
        { base + 358, "sys_dup3" },
        { base + 359, "sys_pipe2" },
        { base + 360, "sys_inotify_init1" },
        { base + 361, "sys_preadv" },
        { base + 362, "sys_pwritev" },
        { base + 363, "sys_rt_tgsigqueueinfo" },
        { base + 364, "sys_perf_event_open" },
        { base + 365, "sys_recvmmsg" },
        { base + 384, "getrandom", getrandomFunc<ArmLinux32> },
        { base + 397, "sys_statx", ignoreFunc },
        { base + 398, "sys_rseq", ignoreFunc },
        { base + 435, "clone3", clone3Func<ArmLinux32> },
    })
    {}
};

static SyscallTable32 syscallDescs32Low(0), syscallDescs32High(0x900000);

class SyscallTable64 : public SyscallDescTable<EmuLinux::SyscallABI64>
{
  public:
    SyscallTable64(int base) : SyscallDescTable<EmuLinux::SyscallABI64>({
        {    base + 0, "io_setup" },
        {    base + 1, "io_destroy" },
        {    base + 2, "io_submit" },
        {    base + 3, "io_cancel" },
        {    base + 4, "io_getevents" },
        {    base + 5, "setxattr" },
        {    base + 6, "lsetxattr" },
        {    base + 7, "fsetxattr" },
        {    base + 8, "getxattr" },
        {    base + 9, "lgetxattr" },
        {   base + 10, "fgetxattr" },
        {   base + 11, "listxattr" },
        {   base + 12, "llistxattr" },
        {   base + 13, "flistxattr" },
        {   base + 14, "removexattr" },
        {   base + 15, "lremovexattr" },
        {   base + 16, "fremovexattr" },
        {   base + 17, "getcwd", getcwdFunc<ArmLinux64> },
        {   base + 18, "lookup_dcookie" },
        {   base + 19, "eventfd2" },
        {   base + 20, "epoll_create1" },
        {   base + 21, "epoll_ctl" },
        {   base + 22, "epoll_pwait" },
        {   base + 23, "dup", dupFunc },
        {   base + 24, "dup3" },
        {   base + 25, "fcntl64", fcntl64Func },
        {   base + 26, "inotify_init1" },
        {   base + 27, "inotify_add_watch" },
        {   base + 28, "inotify_rm_watch" },
        {   base + 29, "ioctl", ioctlFunc<ArmLinux64> },
        {   base + 30, "ioprio_set" },
        {   base + 31, "ioprio_get" },
        {   base + 32, "flock" },
        {   base + 33, "mknodat", mknodatFunc<ArmLinux64> },
        {   base + 34, "mkdirat", mkdiratFunc<ArmLinux64> },
        {   base + 35, "unlinkat", unlinkatFunc<ArmLinux64> },
        {   base + 36, "symlinkat" },
        {   base + 37, "linkat" },
        {   base + 38, "renameat", renameatFunc<ArmLinux64> },
        {   base + 39, "umount2" },
        {   base + 40, "mount" },
        {   base + 41, "pivot_root" },
        {   base + 42, "nfsservctl" },
        {   base + 43, "statfs64" },
        {   base + 44, "fstatfs64" },
        {   base + 45, "truncate64", truncate64Func },
        {   base + 46, "ftruncate64", ftruncate64Func },
        {   base + 47, "fallocate", fallocateFunc<ArmLinux64> },
        {   base + 48, "faccessat", faccessatFunc<ArmLinux64> },
        {   base + 49, "chdir", chdirFunc },
        {   base + 50, "fchdir" },
        {   base + 51, "chroot" },
        {   base + 52, "fchmod" },
        {   base + 53, "fchmodat", fchmodatFunc<ArmLinux64> },
        {   base + 54, "fchownat", fchownatFunc<ArmLinux64> },
        {   base + 55, "fchown", fchownFunc },
        {   base + 56, "openat", openatFunc<ArmLinux64> },
        {   base + 57, "close", closeFunc },
        {   base + 58, "vhangup" },
        {   base + 59, "pipe2" },
        {   base + 60, "quotactl" },
#if defined(SYS_getdents64)
        {   base + 61, "getdents64", getdents64Func },
#else
        {   base + 61, "getdents64" },
#endif
        {   base + 62, "lseek", lseekFunc<ArmLinux64> },
        {   base + 63, "read", readFunc<ArmLinux64> },
        {   base + 64, "write", writeFunc<ArmLinux64> },
        {   base + 65, "readv", readvFunc<ArmLinux64> },
        {   base + 66, "writev", writevFunc<ArmLinux64> },
        {   base + 67, "pread64", pread64Func<ArmLinux64> },
        {   base + 68, "pwrite64", pwrite64Func<ArmLinux64> },
        {   base + 69, "preadv" },
        {   base + 70, "pwritev" },
        {   base + 71, "sendfile64" },
        {   base + 72, "pselect6" },
        {   base + 73, "ppoll" },
        {   base + 74, "signalfd4" },
        {   base + 75, "vmsplice" },
        {   base + 76, "splice" },
        {   base + 77, "tee" },
        {   base + 78, "readlinkat", readlinkatFunc<ArmLinux64> },
        {   base + 79, "fstatat64", fstatat64Func<ArmLinux64> },
        {   base + 80, "fstat64", fstat64Func<ArmLinux64> },
        {   base + 81, "sync" },
        {   base + 82, "fsync" },
        {   base + 83, "fdatasync" },
        {   base + 84, "sync_file_range" },
        {   base + 85, "timerfd_create" },
        {   base + 86, "timerfd_settime" },
        {   base + 87, "timerfd_gettime" },
        {   base + 88, "utimensat" },
        {   base + 89, "acct" },
        {   base + 90, "capget" },
        {   base + 91, "capset" },
        {   base + 92, "personality" },
        {   base + 93, "exit", exitFunc },
        {   base + 94, "exit_group", exitGroupFunc },
        {   base + 95, "waitid" },
        {   base + 96, "set_tid_address", setTidAddressFunc },
        {   base + 97, "unshare" },
        {   base + 98, "futex", futexFunc<ArmLinux64> },
        {   base + 99, "set_robust_list", ignoreFunc },
        {  base + 100, "get_robust_list" },
        {  base + 101, "nanosleep", ignoreWarnOnceFunc },
        {  base + 102, "getitimer" },
        {  base + 103, "setitimer" },
        {  base + 104, "kexec_load" },
        {  base + 105, "init_module" },
        {  base + 106, "delete_module" },
        {  base + 107, "timer_create" },
        {  base + 108, "timer_gettime" },
        {  base + 109, "timer_getoverrun" },
        {  base + 110, "timer_settime" },
        {  base + 111, "timer_delete" },
        {  base + 112, "clock_settime" },
        {  base + 113, "clock_gettime", clock_gettimeFunc<ArmLinux64> },
        {  base + 114, "clock_getres" },
        {  base + 115, "clock_nanosleep" },
        {  base + 116, "syslog" },
        {  base + 117, "ptrace" },
        {  base + 118, "sched_setparam", ignoreWarnOnceFunc },
        {  base + 119, "sched_setscheduler", ignoreWarnOnceFunc },
        {  base + 120, "sched_getscheduler", ignoreWarnOnceFunc },
        {  base + 121, "sched_getparam", sched_getparamFunc },
        {  base + 122, "sched_setaffinity", ignoreWarnOnceFunc },
        {  base + 123, "sched_getaffinity", ignoreFunc },
        {  base + 124, "sched_yield", ignoreWarnOnceFunc },
        {  base + 125, "sched_get_priority_max", ignoreWarnOnceFunc },
        {  base + 126, "sched_get_priority_min", ignoreWarnOnceFunc },
        {  base + 127, "sched_rr_get_interval", ignoreWarnOnceFunc },
        {  base + 128, "restart_syscall" },
        {  base + 129, "kill", ignoreFunc },
        {  base + 130, "tkill" },
        {  base + 131, "tgkill", tgkillFunc<ArmLinux64> },
        {  base + 132, "sigaltstack" },
        {  base + 133, "rt_sigsuspend" },
        {  base + 134, "rt_sigaction", ignoreFunc },
        {  base + 135, "rt_sigprocmask", ignoreWarnOnceFunc },
        {  base + 136, "rt_sigpending" },
        {  base + 137, "rt_sigtimedwait" },
        {  base + 138, "rt_sigqueueinfo", ignoreFunc },
        {  base + 139, "rt_sigreturn" },
        {  base + 140, "setpriority" },
        {  base + 141, "getpriority" },
        {  base + 142, "reboot" },
        {  base + 143, "setregid" },
        {  base + 144, "setgid" },
        {  base + 145, "setreuid" },
        {  base + 146, "setuid" },
        {  base + 147, "setresuid" },
        {  base + 148, "getresuid" },
        {  base + 149, "setresgid" },
        {  base + 150, "getresgid" },
        {  base + 151, "setfsuid" },
        {  base + 152, "setfsgid" },
        {  base + 153, "times", timesFunc<ArmLinux64> },
        {  base + 154, "setpgid" },
        {  base + 155, "getpgid" },
        {  base + 156, "getsid" },
        {  base + 157, "setsid" },
        {  base + 158, "getgroups" },
        {  base + 159, "setgroups" },
        {  base + 160, "uname", unameFunc64 },
        {  base + 161, "sethostname", ignoreFunc },
        {  base + 162, "setdomainname" },
        {  base + 163, "getrlimit", getrlimitFunc<ArmLinux64> },
        {  base + 164, "setrlimit", ignoreFunc },
        {  base + 165, "getrusage", getrusageFunc<ArmLinux64> },
        {  base + 166, "umask" },
        {  base + 167, "prctl" },
        {  base + 168, "getcpu", getcpuFunc },
        {  base + 169, "gettimeofday", gettimeofdayFunc<ArmLinux64> },
        {  base + 170, "settimeofday" },
        {  base + 171, "adjtimex" },
        {  base + 172, "getpid", getpidFunc },
        {  base + 173, "getppid", getppidFunc },
        {  base + 174, "getuid", getuidFunc },
        {  base + 175, "geteuid", geteuidFunc },
        {  base + 176, "getgid", getgidFunc },
        {  base + 177, "getegid", getegidFunc },
        {  base + 178, "gettid", gettidFunc },
        {  base + 179, "sysinfo", sysinfoFunc<ArmLinux64> },
        {  base + 180, "mq_open" },
        {  base + 181, "mq_unlink" },
        {  base + 182, "mq_timedsend" },
        {  base + 183, "mq_timedreceive" },
        {  base + 184, "mq_notify" },
        {  base + 185, "mq_getsetattr" },
        {  base + 186, "msgget" },
        {  base + 187, "msgctl" },
        {  base + 188, "msgrcv" },
        {  base + 189, "msgsnd" },
        {  base + 190, "semget" },
        {  base + 191, "semctl" },
        {  base + 192, "semtimedop" },
        {  base + 193, "semop" },
        {  base + 194, "shmget" },
        {  base + 195, "shmctl" },
        {  base + 196, "shmat" },
        {  base + 197, "shmdt" },
        {  base + 198, "socket" },
        {  base + 199, "socketpair" },
        {  base + 200, "bind" },
        {  base + 201, "listen" },
        {  base + 202, "accept" },
        {  base + 203, "connect" },
        {  base + 204, "getsockname" },
        {  base + 205, "getpeername" },
        {  base + 206, "sendto", sendtoFunc<ArmLinux64> },
        {  base + 207, "recvfrom", recvfromFunc<ArmLinux64> },
        {  base + 208, "setsockopt" },
        {  base + 209, "getsockopt" },
        {  base + 210, "shutdown" },
        {  base + 211, "sendmsg" },
        {  base + 212, "recvmsg" },
        {  base + 213, "readahead" },
        {  base + 214, "brk", brkFunc },
        {  base + 215, "munmap", munmapFunc<ArmLinux64> },
        {  base + 216, "mremap", mremapFunc<ArmLinux64> },
        {  base + 217, "add_key" },
        {  base + 218, "request_key" },
        {  base + 219, "keyctl" },
        {  base + 220, "clone", cloneBackwardsFunc<ArmLinux64> },
        {  base + 221, "execve", execveFunc<ArmLinux64> },
        {  base + 222, "mmap2", mmapFunc<ArmLinux64> },
        {  base + 223, "fadvise64_64" },
        {  base + 224, "swapon" },
        {  base + 225, "swapoff" },
        {  base + 226, "mprotect", ignoreFunc },
        {  base + 227, "msync" },
        {  base + 228, "mlock" },
        {  base + 229, "munlock" },
        {  base + 230, "mlockall" },
        {  base + 231, "munlockall" },
        {  base + 232, "mincore" },
        {  base + 233, "madvise", ignoreFunc },
        {  base + 234, "remap_file_pages" },
        {  base + 235, "mbind" },
        {  base + 236, "get_mempolicy" },
        {  base + 237, "set_mempolicy" },
        {  base + 238, "migrate_pages" },
        {  base + 239, "move_pages" },
        {  base + 240, "rt_tgsigqueueinfo" },
        {  base + 241, "perf_event_open" },
        {  base + 242, "accept4" },
        {  base + 243, "recvmmsg" },
        {  base + 260, "wait4" },
        {  base + 261, "prlimit64", prlimitFunc<ArmLinux64> },
        {  base + 262, "fanotify_init" },
        {  base + 263, "fanotify_mark" },
        {  base + 264, "name_to_handle_at" },
        {  base + 265, "open_by_handle_at" },
        {  base + 266, "clock_adjtime" },
        {  base + 267, "syncfs" },
        {  base + 268, "setns" },
        {  base + 269, "sendmmsg" },
        {  base + 270, "process_vm_readv" },
        {  base + 271, "process_vm_writev" },
        {  base + 272, "kcmp" },
        {  base + 273, "finit_module" },
        {  base + 274, "sched_setattr"},
        {  base + 275, "sched_getattr"},
        {  base + 276, "renameat2"},
        {  base + 277, "seccomp"},
        {  base + 278, "getrandom", getrandomFunc<ArmLinux64> },
        {  base + 279, "memfd_create" },
        {  base + 280, "bpf" },
        {  base + 281, "execveat"},
        {  base + 282, "userfaultfd"},
        {  base + 283, "membarrier"},
        {  base + 284, "mlock2"},
        {  base + 285, "copy_file_range"},
        {  base + 286, "preadv2"},
        {  base + 287, "pwritev2"},
        {  base + 288, "pkey_mprotect"},
        {  base + 289, "pkey_alloc"},
        {  base + 290, "pkey_free"},
        {  base + 291, "statx"},
        {  base + 292, "io_pgetevents"},
        {  base + 293, "rseq", ignoreWarnOnceFunc },
        {  base + 294, "kexec_file_load"},
        {  base + 435, "clone3", clone3Func<ArmLinux64> },
        { base + 1024, "open", openFunc<ArmLinux64> },
        { base + 1025, "link" },
        { base + 1026, "unlink", unlinkFunc },
        { base + 1027, "mknod", mknodFunc },
        { base + 1028, "chmod", chmodFunc<ArmLinux64> },
        { base + 1029, "chown" },
        { base + 1030, "mkdir", mkdirFunc },
        { base + 1031, "rmdir" },
        { base + 1032, "lchown" },
        { base + 1033, "access", accessFunc },
        { base + 1034, "rename", renameFunc },
        { base + 1035, "readlink", readlinkFunc<ArmLinux64> },
        { base + 1036, "symlink" },
        { base + 1037, "utimes", utimesFunc<ArmLinux64> },
        { base + 1038, "stat64", stat64Func<ArmLinux64> },
        { base + 1039, "lstat64", lstat64Func<ArmLinux64> },
        { base + 1040, "pipe", pipePseudoFunc },
        { base + 1041, "dup2" },
        { base + 1042, "epoll_create" },
        { base + 1043, "inotify_init" },
        { base + 1044, "eventfd" },
        { base + 1045, "signalfd" },
        { base + 1046, "sendfile" },
        { base + 1047, "ftruncate", ftruncateFunc<ArmLinux64> },
        { base + 1048, "truncate", truncateFunc<ArmLinux64> },
        { base + 1049, "stat", statFunc<ArmLinux64> },
        { base + 1050, "lstat" },
        { base + 1051, "fstat", fstatFunc<ArmLinux64> },
        { base + 1052, "fcntl", fcntlFunc },
        { base + 1053, "fadvise64" },
        { base + 1054, "newfstatat" },
        { base + 1055, "fstatfs" },
        { base + 1056, "statfs" },
        { base + 1057, "lseek", lseekFunc<ArmLinux64> },
        { base + 1058, "mmap", mmapFunc<ArmLinux64> },
        { base + 1059, "alarm" },
        { base + 1060, "getpgrp" },
        { base + 1061, "pause" },
        { base + 1062, "time", timeFunc<ArmLinux64> },
        { base + 1063, "utime" },
        { base + 1064, "creat" },
#if defined(SYS_getdents)
        { base + 1065, "getdents", getdentsFunc },
#else
        { base + 1065, "getdents" },
#endif
        { base + 1066, "futimesat", futimesatFunc<ArmLinux64> },
        { base + 1067, "select" },
        { base + 1068, "poll", pollFunc<ArmLinux64> },
        { base + 1069, "epoll_wait" },
        { base + 1070, "ustat" },
        { base + 1071, "vfork" },
        { base + 1072, "oldwait4" },
        { base + 1073, "recv" },
        { base + 1074, "send" },
        { base + 1075, "bdflush" },
        { base + 1076, "umount" },
        { base + 1077, "uselib" },
        { base + 1078, "_sysctl" },
        { base + 1079, "fork" }
    })
    {}
};

static SyscallTable64 syscallDescs64Low(0), syscallDescs64High(0x900000);

static SyscallDescTable<EmuLinux::SyscallABI32> privSyscallDescs32 = {
    { 0xf0001, "breakpoint" },
    { 0xf0002, "cacheflush" },
    { 0xf0003, "usr26" },
    { 0xf0004, "usr32" },
    { 0xf0005, "set_tls", setTLSFunc32 },
};

// Indices 1, 3 and 4 are unallocated.
static SyscallDescTable<EmuLinux::SyscallABI64> privSyscallDescs64 = {
    { 0x1002, "cacheflush" },
    { 0x1005, "set_tls", setTLSFunc64 }
};

void
EmuLinux::syscall(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    SyscallDesc *desc = nullptr;
    if (dynamic_cast<ArmLinuxProcess64 *>(process)) {
        int num = tc->getReg(int_reg::X8);
        desc = syscallDescs64Low.get(num, false);
        if (!desc)
            desc = syscallDescs64Low.get(num, false);
        if (!desc)
            desc = privSyscallDescs64.get(num);
    } else {
        int num = tc->getReg(int_reg::R7);
        desc = syscallDescs32Low.get(num, false);
        if (!desc)
            desc = syscallDescs32Low.get(num, false);
        if (!desc)
            desc = privSyscallDescs32.get(num);
    }
    assert(desc);
    desc->doSyscall(tc);
}

} // namespace ArmISA
} // namespace gem5
