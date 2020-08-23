/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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

#include "arch/x86/linux/process.hh"

#include <sys/syscall.h>

#include "arch/x86/isa_traits.hh"
#include "arch/x86/linux/linux.hh"
#include "arch/x86/registers.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace X86ISA;

namespace
{

class X86LinuxObjectFileLoader : public Process::Loader
{
  public:
    Process *
    load(ProcessParams *params, ::Loader::ObjectFile *obj_file) override
    {
        auto arch = obj_file->getArch();
        auto opsys = obj_file->getOpSys();

        if (arch != ::Loader::X86_64 && arch != ::Loader::I386)
            return nullptr;

        if (opsys == ::Loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = ::Loader::Linux;
        }

        if (opsys != ::Loader::Linux)
            return nullptr;

        if (arch == ::Loader::X86_64)
            return new X86_64LinuxProcess(params, obj_file);
        else
            return new I386LinuxProcess(params, obj_file);
    }
};

X86LinuxObjectFileLoader loader;

} // anonymous namespace

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "x86_64");

    return 0;
}

static SyscallReturn
archPrctlFunc(SyscallDesc *desc, ThreadContext *tc, int code, uint64_t addr)
{
    enum ArchPrctlCodes
    {
        SetFS = 0x1002,
        GetFS = 0x1003,
        SetGS = 0x1001,
        GetGS = 0x1004
    };

    uint64_t fsBase, gsBase;
    PortProxy &p = tc->getVirtProxy();
    switch(code)
    {
      // Each of these valid options should actually check addr.
      case SetFS:
        tc->setMiscRegNoEffect(MISCREG_FS_BASE, addr);
        tc->setMiscRegNoEffect(MISCREG_FS_EFF_BASE, addr);
        return 0;
      case GetFS:
        fsBase = tc->readMiscRegNoEffect(MISCREG_FS_BASE);
        p.write(addr, fsBase);
        return 0;
      case SetGS:
        tc->setMiscRegNoEffect(MISCREG_GS_BASE, addr);
        tc->setMiscRegNoEffect(MISCREG_GS_EFF_BASE, addr);
        return 0;
      case GetGS:
        gsBase = tc->readMiscRegNoEffect(MISCREG_GS_BASE);
        p.write(addr, gsBase);
        return 0;
      default:
        return -EINVAL;
    }
}

BitUnion32(UserDescFlags)
    Bitfield<0> seg_32bit;
    Bitfield<2, 1> contents;
    Bitfield<3> read_exec_only;
    Bitfield<4> limit_in_pages;
    Bitfield<5> seg_not_present;
    Bitfield<6> useable;
EndBitUnion(UserDescFlags)

struct UserDesc32 {
    uint32_t entry_number;
    uint32_t base_addr;
    uint32_t limit;
    uint32_t flags;
};

struct UserDesc64 {
    uint32_t entry_number;
    uint32_t __padding1;
    uint64_t base_addr;
    uint32_t limit;
    uint32_t flags;
};

static SyscallReturn
setThreadArea32Func(SyscallDesc *desc, ThreadContext *tc,
                    VPtr<UserDesc32> userDesc)
{
    const int minTLSEntry = 6;
    const int numTLSEntries = 3;
    const int maxTLSEntry = minTLSEntry + numTLSEntries - 1;

    auto process = tc->getProcessPtr();

    X86Process *x86p = dynamic_cast<X86Process *>(process);
    assert(x86p);

    assert((maxTLSEntry + 1) * sizeof(uint64_t) <= x86p->gdtSize());

    TypedBufferArg<uint64_t>
        gdt(x86p->gdtStart() + minTLSEntry * sizeof(uint64_t),
            numTLSEntries * sizeof(uint64_t));

    if (!gdt.copyIn(tc->getVirtProxy()))
        panic("Failed to copy in GDT for %s.\n", desc->name());

    if (userDesc->entry_number == (uint32_t)(-1)) {
        // Find a free TLS entry.
        for (int i = 0; i < numTLSEntries; i++) {
            if (gdt[i] == 0) {
                userDesc->entry_number = i + minTLSEntry;
                break;
            }
        }
        // We failed to find one.
        if (userDesc->entry_number == (uint32_t)(-1))
            return -ESRCH;
    }

    int index = userDesc->entry_number;

    if (index < minTLSEntry || index > maxTLSEntry)
        return -EINVAL;

    index -= minTLSEntry;

    // Build the entry we're going to add.
    SegDescriptor segDesc = 0;
    UserDescFlags flags = userDesc->flags;

    segDesc.limitLow = bits(userDesc->limit, 15, 0);
    segDesc.baseLow = bits(userDesc->base_addr, 23, 0);
    segDesc.type.a = 1;
    if (!flags.read_exec_only)
        segDesc.type.w = 1;
    if (bits((uint8_t)flags.contents, 0))
        segDesc.type.e = 1;
    if (bits((uint8_t)flags.contents, 1))
        segDesc.type.codeOrData = 1;
    segDesc.s = 1;
    segDesc.dpl = 3;
    if (!flags.seg_not_present)
        segDesc.p = 1;
    segDesc.limitHigh = bits(userDesc->limit, 19, 16);
    if (flags.useable)
        segDesc.avl = 1;
    segDesc.l = 0;
    if (flags.seg_32bit)
        segDesc.d = 1;
    if (flags.limit_in_pages)
        segDesc.g = 1;
    segDesc.baseHigh = bits(userDesc->base_addr, 31, 24);

    gdt[index] = (uint64_t)segDesc;

    if (!gdt.copyOut(tc->getVirtProxy()))
        panic("Failed to copy out GDT for %s.\n", desc->name());

    return 0;
}

const std::vector<IntRegIndex> X86_64LinuxProcess::SyscallABI::ArgumentRegs = {
    INTREG_RDI, INTREG_RSI, INTREG_RDX, INTREG_R10W, INTREG_R8W, INTREG_R9W
};

static SyscallDescTable<X86_64LinuxProcess::SyscallABI> syscallDescs64 = {
    {   0, "read", readFunc<X86Linux64> },
    {   1, "write", writeFunc<X86Linux64> },
    {   2, "open", openFunc<X86Linux64> },
    {   3, "close", closeFunc },
    {   4, "stat", stat64Func<X86Linux64> },
    {   5, "fstat", fstat64Func<X86Linux64> },
    {   6, "lstat", lstat64Func<X86Linux64> },
    {   7, "poll", pollFunc<X86Linux64> },
    {   8, "lseek", lseekFunc },
    {   9, "mmap", mmapFunc<X86Linux64> },
    {  10, "mprotect", ignoreFunc },
    {  11, "munmap", munmapFunc },
    {  12, "brk", brkFunc },
    {  13, "rt_sigaction", ignoreWarnOnceFunc },
    {  14, "rt_sigprocmask", ignoreWarnOnceFunc },
    {  15, "rt_sigreturn" },
    {  16, "ioctl", ioctlFunc<X86Linux64> },
    {  17, "pread64", pread64Func<X86Linux64> },
    {  18, "pwrite64", pwrite64Func<X86Linux64> },
    {  19, "readv", readvFunc<X86Linux64> },
    {  20, "writev", writevFunc<X86Linux64> },
    {  21, "access", ignoreFunc },
    {  22, "pipe", pipeFunc },
    {  23, "select", selectFunc<X86Linux64> },
    {  24, "sched_yield", ignoreFunc },
    {  25, "mremap", mremapFunc<X86Linux64> },
    {  26, "msync" },
    {  27, "mincore" },
    {  28, "madvise", ignoreFunc },
    {  29, "shmget" },
    {  30, "shmat" },
    {  31, "shmctl" },
    {  32, "dup", dupFunc },
    {  33, "dup2", dup2Func },
    {  34, "pause" },
    {  35, "nanosleep", ignoreWarnOnceFunc },
    {  36, "getitimer" },
    {  37, "alarm" },
    {  38, "setitimer" },
    {  39, "getpid", getpidFunc },
    {  40, "sendfile" },
    {  41, "socket", socketFunc<X86Linux64> },
    {  42, "connect", connectFunc },
    {  43, "accept", acceptFunc<X86Linux64> },
    {  44, "sendto", sendtoFunc },
    {  45, "recvfrom", recvfromFunc },
    {  46, "sendmsg", sendmsgFunc },
    {  47, "recvmsg", recvmsgFunc },
    {  48, "shutdown", shutdownFunc },
    {  49, "bind", bindFunc },
    {  50, "listen", listenFunc },
    {  51, "getsockname", getsocknameFunc },
    {  52, "getpeername", getpeernameFunc },
    {  53, "socketpair", socketpairFunc<X86Linux64> },
    {  54, "setsockopt", setsockoptFunc },
    {  55, "getsockopt", getsockoptFunc },
    {  56, "clone", cloneFunc<X86Linux64> },
    {  57, "fork" },
    {  58, "vfork" },
    {  59, "execve", execveFunc<X86Linux64> },
    {  60, "exit", exitFunc },
    {  61, "wait4", wait4Func<X86Linux64> },
    {  62, "kill" },
    {  63, "uname", unameFunc },
    {  64, "semget" },
    {  65, "semop" },
    {  66, "semctl" },
    {  67, "shmdt" },
    {  68, "msgget" },
    {  69, "msgsnd" },
    {  70, "msgrcv" },
    {  71, "msgctl" },
    {  72, "fcntl", fcntlFunc },
    {  73, "flock" },
    {  74, "fsync" },
    {  75, "fdatasync" },
    {  76, "truncate", truncateFunc },
    {  77, "ftruncate", ftruncateFunc },
#if defined(SYS_getdents)
    {  78, "getdents", getdentsFunc },
#else
    {  78, "getdents" },
#endif
    {  79, "getcwd", getcwdFunc },
    {  80, "chdir", chdirFunc },
    {  81, "fchdir" },
    {  82, "rename", renameFunc },
    {  83, "mkdir", mkdirFunc },
    {  84, "rmdir", rmdirFunc },
    {  85, "creat" },
    {  86, "link", linkFunc },
    {  87, "unlink", unlinkFunc },
    {  88, "symlink", symlinkFunc },
    {  89, "readlink", readlinkFunc },
    {  90, "chmod", ignoreFunc },
    {  91, "fchmod" },
    {  92, "chown" },
    {  93, "fchown" },
    {  94, "lchown" },
    {  95, "umask", umaskFunc },
    {  96, "gettimeofday", gettimeofdayFunc<X86Linux64> },
    {  97, "getrlimit", getrlimitFunc<X86Linux64> },
    {  98, "getrusage", getrusageFunc<X86Linux64> },
    {  99, "sysinfo", sysinfoFunc<X86Linux64> },
    { 100, "times", timesFunc<X86Linux64> },
    { 101, "ptrace" },
    { 102, "getuid", getuidFunc },
    { 103, "syslog" },
    { 104, "getgid", getgidFunc },
    { 105, "setuid" },
    { 106, "setgid" },
    { 107, "geteuid", geteuidFunc },
    { 108, "getegid", getegidFunc },
    { 109, "setpgid", setpgidFunc },
    { 110, "getppid", getppidFunc },
    { 111, "getpgrp", getpgrpFunc },
    { 112, "setsid" },
    { 113, "setreuid" },
    { 114, "setregid" },
    { 115, "getgroups" },
    { 116, "setgroups" },
    { 117, "setresuid", ignoreFunc },
    { 118, "getresuid" },
    { 119, "setresgid" },
    { 120, "getresgid" },
    { 121, "getpgid" },
    { 122, "setfsuid" },
    { 123, "setfsgid" },
    { 124, "getsid" },
    { 125, "capget" },
    { 126, "capset" },
    { 127, "rt_sigpending" },
    { 128, "rt_sigtimedwait" },
    { 129, "rt_sigqueueinfo" },
    { 130, "rt_sigsuspend" },
    { 131, "sigaltstack" },
    { 132, "utime" },
    { 133, "mknod", mknodFunc },
    { 134, "uselib" },
    { 135, "personality" },
    { 136, "ustat" },
    { 137, "statfs", statfsFunc<X86Linux64> },
    { 138, "fstatfs", fstatfsFunc<X86Linux64> },
    { 139, "sysfs" },
    { 140, "getpriority" },
    { 141, "setpriority", ignoreFunc },
    { 142, "sched_setparam" },
    { 143, "sched_getparam" },
    { 144, "sched_setscheduler" },
    { 145, "sched_getscheduler" },
    { 146, "sched_get_priority_max" },
    { 147, "sched_get_priority_min" },
    { 148, "sched_rr_get_interval" },
    { 149, "mlock" },
    { 150, "munlock" },
    { 151, "mlockall" },
    { 152, "munlockall" },
    { 153, "vhangup" },
    { 154, "modify_ldt" },
    { 155, "pivot_root" },
    { 156, "_sysctl" },
    { 157, "prctl" },
    { 158, "arch_prctl", archPrctlFunc },
    { 159, "adjtimex" },
    { 160, "setrlimit", ignoreFunc },
    { 161, "chroot" },
    { 162, "sync" },
    { 163, "acct" },
    { 164, "settimeofday" },
    { 165, "mount" },
    { 166, "umount2" },
    { 167, "swapon" },
    { 168, "swapoff" },
    { 169, "reboot" },
    { 170, "sethostname" },
    { 171, "setdomainname" },
    { 172, "iopl" },
    { 173, "ioperm" },
    { 174, "create_module" },
    { 175, "init_module" },
    { 176, "delete_module" },
    { 177, "get_kernel_syms" },
    { 178, "query_module" },
    { 179, "quotactl" },
    { 180, "nfsservctl" },
    { 181, "getpmsg" },
    { 182, "putpmsg" },
    { 183, "afs_syscall" },
    { 184, "tuxcall" },
    { 185, "security" },
    { 186, "gettid", gettidFunc },
    { 187, "readahead" },
    { 188, "setxattr" },
    { 189, "lsetxattr" },
    { 190, "fsetxattr" },
    { 191, "getxattr" },
    { 192, "lgetxattr" },
    { 193, "fgetxattr" },
    { 194, "listxattr" },
    { 195, "llistxattr" },
    { 196, "flistxattr" },
    { 197, "removexattr" },
    { 198, "lremovexattr" },
    { 199, "fremovexattr" },
    { 200, "tkill" },
    { 201, "time", timeFunc<X86Linux64> },
    { 202, "futex", futexFunc<X86Linux64> },
    { 203, "sched_setaffinity", ignoreFunc },
    { 204, "sched_getaffinity", ignoreFunc },
    { 205, "set_thread_area" },
    { 206, "io_setup" },
    { 207, "io_destroy" },
    { 208, "io_getevents" },
    { 209, "io_submit" },
    { 210, "io_cancel" },
    { 211, "get_thread_area" },
    { 212, "lookup_dcookie" },
    { 213, "epoll_create" },
    { 214, "epoll_ctl_old" },
    { 215, "epoll_wait_old" },
    { 216, "remap_file_pages" },
    { 217, "getdents64" },
    { 218, "set_tid_address", setTidAddressFunc },
    { 219, "restart_syscall" },
    { 220, "semtimedop" },
    { 221, "fadvise64", ignoreFunc },
    { 222, "timer_create" },
    { 223, "timer_settime" },
    { 224, "timer_gettime" },
    { 225, "timer_getoverrun" },
    { 226, "timer_delete" },
    { 227, "clock_settime" },
    { 228, "clock_gettime", clock_gettimeFunc<X86Linux64> },
    { 229, "clock_getres", clock_getresFunc<X86Linux64> },
    { 230, "clock_nanosleep" },
    { 231, "exit_group", exitGroupFunc },
    { 232, "epoll_wait" },
    { 233, "epoll_ctl" },
    { 234, "tgkill", tgkillFunc<X86Linux64> },
    { 235, "utimes" },
    { 236, "vserver" },
    { 237, "mbind" },
    { 238, "set_mempolicy" },
    { 239, "get_mempolicy", ignoreFunc },
    { 240, "mq_open" },
    { 241, "mq_unlink" },
    { 242, "mq_timedsend" },
    { 243, "mq_timedreceive" },
    { 244, "mq_notify" },
    { 245, "mq_getsetattr" },
    { 246, "kexec_load" },
    { 247, "waitid" },
    { 248, "add_key" },
    { 249, "request_key" },
    { 250, "keyctl" },
    { 251, "ioprio_set" },
    { 252, "ioprio_get" },
    { 253, "inotify_init" },
    { 254, "inotify_add_watch" },
    { 255, "inotify_rm_watch" },
    { 256, "migrate_pages" },
    { 257, "openat", openatFunc<X86Linux64> },
    { 258, "mkdirat" },
    { 259, "mknodat" },
    { 260, "fchownat" },
    { 261, "futimesat" },
    { 262, "newfstatat" },
    { 263, "unlinkat" },
    { 264, "renameat" },
    { 265, "linkat" },
    { 266, "symlinkat" },
    { 267, "readlinkat", readlinkFunc },
    { 268, "fchmodat" },
    { 269, "faccessat" },
    { 270, "pselect6" },
    { 271, "ppoll" },
    { 272, "unshare" },
    { 273, "set_robust_list", ignoreFunc },
    { 274, "get_robust_list" },
    { 275, "splice" },
    { 276, "tee" },
    { 277, "sync_file_range" },
    { 278, "vmsplice" },
    { 279, "move_pages" },
    { 280, "utimensat" },
    { 281, "epoll_pwait" },
    { 282, "signalfd" },
    { 283, "timerfd_create" },
    { 284, "eventfd", eventfdFunc<X86Linux64> },
    { 285, "fallocate", fallocateFunc },
    { 286, "timerfd_settime" },
    { 287, "timerfd_gettime" },
    { 288, "accept4" },
    { 289, "signalfd4" },
    { 290, "eventfd2", eventfdFunc<X86Linux64> },
    { 291, "epoll_create1" },
    { 292, "dup3" },
    { 293, "pipe2", pipe2Func },
    { 294, "inotify_init1" },
    { 295, "preadv" },
    { 296, "pwritev" },
    { 297, "rt_tgsigqueueinfo" },
    { 298, "perf_event_open" },
    { 299, "recvmmsg" },
    { 300, "fanotify_init" },
    { 301, "fanotify_mark" },
    { 302, "prlimit64", prlimitFunc<X86Linux64> },
    { 303, "name_to_handle_at" },
    { 304, "open_by_handle_at" },
    { 305, "clock_adjtime" },
    { 306, "syncfs" },
    { 307, "sendmmsg" },
    { 308, "setns" },
    { 309, "getcpu", getcpuFunc },
    { 310, "proess_vm_readv" },
    { 311, "proess_vm_writev" },
    { 312, "kcmp" },
    { 313, "finit_module" },
};

void
X86_64LinuxProcess::syscall(ThreadContext *tc)
{
    X86_64Process::syscall(tc);
    syscallDescs64.get(tc->readIntReg(INTREG_RAX))->doSyscall(tc);
}

void
X86_64LinuxProcess::clone(ThreadContext *old_tc, ThreadContext *new_tc,
                          Process *process, RegVal flags)
{
    X86_64Process::clone(old_tc, new_tc, (X86_64Process*)process, flags);
}

const std::vector<IntRegIndex> I386LinuxProcess::SyscallABI::ArgumentRegs = {
    INTREG_EBX, INTREG_ECX, INTREG_EDX, INTREG_ESI, INTREG_EDI, INTREG_EBP
};

static SyscallDescTable<I386LinuxProcess::SyscallABI> syscallDescs32 = {
    {   0, "restart_syscall" },
    {   1, "exit", exitFunc },
    {   2, "fork" },
    {   3, "read", readFunc<X86Linux32> },
    {   4, "write", writeFunc<X86Linux32> },
    {   5, "open", openFunc<X86Linux32> },
    {   6, "close", closeFunc },
    {   7, "waitpid" },
    {   8, "creat" },
    {   9, "link" },
    {  10, "unlink" },
    {  11, "execve", execveFunc<X86Linux32> },
    {  12, "chdir", chdirFunc },
    {  13, "time", timeFunc<X86Linux32> },
    {  14, "mknod", mknodFunc },
    {  15, "chmod" },
    {  16, "lchown" },
    {  17, "break" },
    {  18, "oldstat" },
    {  19, "lseek" },
    {  20, "getpid", getpidFunc },
    {  21, "mount" },
    {  22, "umount" },
    {  23, "setuid" },
    {  24, "getuid", getuidFunc },
    {  25, "stime" },
    {  26, "ptrace" },
    {  27, "alarm" },
    {  28, "oldfstat" },
    {  29, "pause" },
    {  30, "utime" },
    {  31, "stty" },
    {  32, "gtty" },
    {  33, "access", ignoreFunc },
    {  34, "nice" },
    {  35, "ftime" },
    {  36, "sync" },
    {  37, "kill" },
    {  38, "rename" },
    {  39, "mkdir", mkdirFunc },
    {  40, "rmdir", mkdirFunc },
    {  41, "dup", dupFunc },
    {  42, "pipe", pipeFunc },
    {  43, "times", timesFunc<X86Linux32> },
    {  44, "prof" },
    {  45, "brk", brkFunc },
    {  46, "setgid" },
    {  47, "getgid", getgidFunc },
    {  48, "signal" },
    {  49, "geteuid", geteuidFunc },
    {  50, "getegid", getegidFunc },
    {  51, "acct" },
    {  52, "umount2" },
    {  53, "lock" },
    {  54, "ioctl", ioctlFunc<X86Linux32> },
    {  55, "fcntl", fcntlFunc },
    {  56, "mpx" },
    {  57, "setpgid", setpgidFunc },
    {  58, "ulimit" },
    {  59, "oldolduname" },
    {  60, "umask", umaskFunc },
    {  61, "chroot" },
    {  62, "ustat" },
    {  63, "dup2", dup2Func },
    {  64, "getppid" },
    {  65, "getpgrp" },
    {  66, "setsid" },
    {  67, "sigaction" },
    {  68, "sgetmask" },
    {  69, "ssetmask" },
    {  70, "setreuid" },
    {  71, "setregid" },
    {  72, "sigsuspend" },
    {  73, "sigpending" },
    {  74, "sethostname" },
    {  75, "setrlimit", ignoreFunc },
    {  76, "getrlimit", getrlimitFunc<X86Linux32> },
    {  77, "getrusage", getrusageFunc<X86Linux32> },
    {  78, "gettimeofday" },
    {  79, "settimeofday" },
    {  80, "getgroups" },
    {  81, "setgroups" },
    {  82, "select", selectFunc<X86Linux32> },
    {  83, "symlink" },
    {  84, "oldlstat" },
    {  85, "readlink", readlinkFunc },
    {  86, "uselib" },
    {  87, "swapon" },
    {  88, "reboot" },
    {  89, "readdir" },
    {  90, "mmap" },
    {  91, "munmap", munmapFunc },
    {  92, "truncate", truncateFunc },
    {  93, "ftruncate", ftruncateFunc },
    {  94, "fchmod" },
    {  95, "fchown" },
    {  96, "getpriority" },
    {  97, "setpriority", ignoreFunc },
    {  98, "profil" },
    {  99, "statfs", ignoreFunc },
    { 100, "fstatfs" },
    { 101, "ioperm" },
    { 102, "socketcall" },
    { 103, "syslog" },
    { 104, "setitimer" },
    { 105, "getitimer" },
    { 106, "stat" },
    { 107, "lstat" },
    { 108, "fstat" },
    { 109, "olduname" },
    { 110, "iopl" },
    { 111, "vhangup" },
    { 112, "idle" },
    { 113, "vm86old" },
    { 114, "wait4", wait4Func<X86Linux32> },
    { 115, "swapoff" },
    { 116, "sysinfo", sysinfoFunc<X86Linux32> },
    { 117, "ipc" },
    { 118, "fsync" },
    { 119, "sigreturn" },
    { 120, "clone", cloneFunc<X86Linux32> },
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
    { 140, "_llseek", _llseekFunc },
#if defined(SYS_getdents)
    { 141, "getdents", getdentsFunc },
#else
    { 141, "getdents" },
#endif
    { 142, "_newselect" },
    { 143, "flock" },
    { 144, "msync" },
    { 145, "readv", readvFunc<X86Linux32> },
    { 146, "writev", writevFunc<X86Linux32> },
    { 147, "getsid" },
    { 148, "fdatasync" },
    { 149, "_sysctl" },
    { 150, "mlock" },
    { 151, "munlock" },
    { 152, "mlockall" },
    { 153, "munlockall" },
    { 154, "sched_setparam" },
    { 155, "sched_getparam" },
    { 156, "sched_setscheduler" },
    { 157, "sched_getscheduler" },
    { 158, "sched_yield", ignoreFunc },
    { 159, "sched_get_priority_max" },
    { 160, "sched_get_priority_min" },
    { 161, "sched_rr_get_interval" },
    { 162, "nanosleep", ignoreFunc },
    { 163, "mremap" },
    { 164, "setresuid", ignoreFunc },
    { 165, "getresuid" },
    { 166, "vm86" },
    { 167, "query_module" },
    { 168, "poll", pollFunc<X86Linux32> },
    { 169, "nfsservctl" },
    { 170, "setresgid" },
    { 171, "getresgid" },
    { 172, "prctl" },
    { 173, "rt_sigreturn" },
    { 174, "rt_sigaction", ignoreFunc },
    { 175, "rt_sigprocmask", ignoreFunc },
    { 176, "rt_sigpending" },
    { 177, "rt_sigtimedwait" },
    { 178, "rt_sigqueueinfo" },
    { 179, "rt_sigsuspend" },
    { 180, "pread64", pread64Func<X86Linux64> },
    { 181, "pwrite64", pwrite64Func<X86Linux64> },
    { 182, "chown" },
    { 183, "getcwd", getcwdFunc },
    { 184, "capget" },
    { 185, "capset" },
    { 186, "sigaltstack" },
    { 187, "sendfile" },
    { 188, "getpmsg" },
    { 189, "putpmsg" },
    { 190, "vfork" },
    { 191, "ugetrlimit", ignoreFunc },
    { 192, "mmap2", mmap2Func<X86Linux32> },
    { 193, "truncate64", truncate64Func },
    { 194, "ftruncate64", ftruncate64Func },
    { 195, "stat64", stat64Func<X86Linux32> },
    { 196, "lstat64" },
    { 197, "fstat64", fstat64Func<X86Linux32> },
    { 198, "lchown32" },
    { 199, "getuid32", getuidFunc },
    { 200, "getgid32", getgidFunc },
    { 201, "geteuid32", geteuidFunc },
    { 202, "getegid32", getegidFunc },
    { 203, "setreuid32" },
    { 204, "setregid32" },
    { 205, "getgroups32" },
    { 206, "setgroups32" },
    { 207, "fchown32" },
    { 208, "setresuid32" },
    { 209, "getresuid32" },
    { 210, "setresgid32" },
    { 211, "getresgid32" },
    { 212, "chown32" },
    { 213, "setuid32" },
    { 214, "setgid32" },
    { 215, "setfsuid32" },
    { 216, "setfsgid32" },
    { 217, "pivot_root" },
    { 218, "mincore" },
    { 219, "madvise", ignoreFunc },
    { 220, "madvise1" },
    { 221, "getdents64" },
    { 222, "fcntl64" },
    { 223, "unused" },
    { 224, "gettid", gettidFunc },
    { 225, "readahead" },
    { 226, "setxattr" },
    { 227, "lsetxattr" },
    { 228, "fsetxattr" },
    { 229, "getxattr" },
    { 230, "lgetxattr" },
    { 231, "fgetxattr" },
    { 232, "listxattr" },
    { 233, "llistxattr" },
    { 234, "flistxattr" },
    { 235, "removexattr" },
    { 236, "lremovexattr" },
    { 237, "fremovexattr" },
    { 238, "tkill" },
    { 239, "sendfile64" },
    { 240, "futex" },
    { 241, "sched_setaffinity", ignoreFunc },
    { 242, "sched_getaffinity", ignoreFunc },
    { 243, "set_thread_area", setThreadArea32Func },
    { 244, "get_thread_area" },
    { 245, "io_setup" },
    { 246, "io_destroy" },
    { 247, "io_getevents" },
    { 248, "io_submit" },
    { 249, "io_cancel" },
    { 250, "fadvise64" },
    { 251, "unused" },
    { 252, "exit_group", exitFunc },
    { 253, "lookup_dcookie" },
    { 254, "epoll_create" },
    { 255, "epoll_ctl" },
    { 256, "epoll_wait" },
    { 257, "remap_file_pages" },
    { 258, "set_tid_address", setTidAddressFunc },
    { 259, "timer_create" },
    { 260, "timer_settime" },
    { 261, "timer_gettime" },
    { 262, "timer_getoverrun" },
    { 263, "timer_delete" },
    { 264, "clock_settime" },
    { 265, "clock_gettime", clock_gettimeFunc<X86Linux32> },
    { 266, "clock_getres" },
    { 267, "clock_nanosleep" },
    { 268, "statfs64" },
    { 269, "fstatfs64" },
    { 270, "tgkill", tgkillFunc<X86Linux32> },
    { 271, "utimes" },
    { 272, "fadvise64_64" },
    { 273, "vserver" },
    { 274, "mbind" },
    { 275, "get_mempolicy", ignoreFunc },
    { 276, "set_mempolicy" },
    { 277, "mq_open" },
    { 278, "mq_unlink" },
    { 279, "mq_timedsend" },
    { 280, "mq_timedreceive" },
    { 281, "mq_notify" },
    { 282, "mq_getsetattr" },
    { 283, "kexec_load" },
    { 284, "waitid" },
    { 285, "sys_setaltroot" },
    { 286, "add_key" },
    { 287, "request_key" },
    { 288, "keyctl" },
    { 289, "ioprio_set" },
    { 290, "ioprio_get" },
    { 291, "inotify_init" },
    { 292, "inotify_add_watch" },
    { 293, "inotify_rm_watch" },
    { 294, "migrate_pages" },
    { 295, "openat", openatFunc<X86Linux32> },
    { 296, "mkdirat" },
    { 297, "mknodat" },
    { 298, "fchownat" },
    { 299, "futimesat" },
    { 300, "fstatat64" },
    { 301, "unlinkat" },
    { 302, "renameat" },
    { 303, "linkat" },
    { 304, "symlinkat" },
    { 305, "readlinkat", readlinkFunc },
    { 306, "fchmodat" },
    { 307, "faccessat" },
    { 308, "pselect6" },
    { 309, "ppoll" },
    { 310, "unshare" },
    { 311, "set_robust_list", ignoreFunc },
    { 312, "get_robust_list", ignoreFunc },
    { 313, "splice" },
    { 314, "sync_file_range" },
    { 315, "tee" },
    { 316, "vmsplice" },
    { 317, "move_pages" },
    { 318, "getcpu", getcpuFunc },
    { 319, "epoll_pwait" },
    { 320, "utimensat" },
    { 321, "signalfd" },
    { 322, "timerfd" },
    { 323, "eventfd", eventfdFunc<X86Linux32> }
};

void
I386LinuxProcess::syscall(ThreadContext *tc)
{
    I386Process::syscall(tc);
    PCState pc = tc->pcState();
    Addr eip = pc.pc();
    if (eip >= vsyscallPage.base &&
            eip < vsyscallPage.base + vsyscallPage.size) {
        pc.npc(vsyscallPage.base + vsyscallPage.vsysexitOffset);
        tc->pcState(pc);
    }
    syscallDescs32.get(tc->readIntReg(INTREG_RAX))->doSyscall(tc);
}

void
I386LinuxProcess::clone(ThreadContext *old_tc, ThreadContext *new_tc,
                        Process *process, RegVal flags)
{
    I386Process::clone(old_tc, new_tc, (I386Process*)process, flags);
}
