/*
 * Copyright (c) 2024 Arm Limited
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
 * Copyright 2005 The Regents of The University of Michigan
 * Copyright 2007 MIPS Technologies, Inc.
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

#include "arch/mips/linux/se_workload.hh"

#include <sys/syscall.h>

#include "arch/mips/process.hh"
#include "arch/mips/regs/misc.hh"
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
        if (obj->getArch() != loader::Mips)
            return nullptr;

        auto opsys = obj->getOpSys();

        if (opsys == loader::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = loader::Linux;
        }

        if (opsys != loader::Linux)
            return nullptr;

        return new MipsProcess(params, obj);
    }
};

LinuxLoader linuxLoader;

} // anonymous namespace

namespace MipsISA
{

void
EmuLinux::syscall(ThreadContext *tc)
{
    Process *process = tc->getProcessPtr();
    // Call the syscall function in the base Process class to update stats.
    // This will move into the base SEWorkload function at some point.
    process->Process::syscall(tc);

    syscallDescs.get(tc->getReg(int_reg::V0))->doSyscall(tc);
}

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename,"sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "mips");

    return 0;
}

/// Target sys_getsysyinfo() handler.  Even though this call is
/// borrowed from Tru64, the subcases that get used appear to be
/// different in practice from those used by Tru64 processes.
static SyscallReturn
sys_getsysinfoFunc(SyscallDesc *desc, ThreadContext *tc, unsigned op,
                   unsigned bufPtr, unsigned nbytes)
{
    switch (op) {
      case 45:
        {
            // GSI_IEEE_FP_CONTROL
            VPtr<uint64_t> fpcr(bufPtr, tc);
            // I don't think this exactly matches the HW FPCR
            *fpcr = 0;
            return 0;
        }
      default:
        std::cerr << "sys_getsysinfo: unknown op " << op << std::endl;
        abort();
        break;
    }

    return 1;
}

/// Target sys_setsysinfo() handler.
static SyscallReturn
sys_setsysinfoFunc(SyscallDesc *desc, ThreadContext *tc, unsigned op,
                   VPtr<> bufPtr, unsigned nbytes)
{
    switch (op) {

      case 14:
        {
            // SSI_IEEE_FP_CONTROL
            ConstVPtr<uint64_t> fpcr(bufPtr, tc);
            // I don't think this exactly matches the HW FPCR
            DPRINTFR(SyscallVerbose, "sys_setsysinfo(SSI_IEEE_FP_CONTROL): "
                   " setting FPCR to 0x%x\n", letoh(*fpcr));
            return 0;
        }
      default:
        std::cerr << "sys_setsysinfo: unknown op " << op << std::endl;
        abort();
        break;
    }

    return 1;
}

static SyscallReturn
setThreadAreaFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<> addr)
{
    tc->setMiscRegNoEffect(misc_reg::TpValue, addr);
    return 0;
}

SyscallDescTable<MipsISA::SEWorkload::SyscallABI> EmuLinux::syscallDescs = {
    { 4000, "syscall" },
    { 4001, "exit", exitFunc },
    { 4002, "fork" },
    { 4003, "read", readFunc<MipsLinux> },
    { 4004, "write", writeFunc<MipsLinux> },
    { 4005, "open", openFunc<MipsLinux> },
    { 4006, "close", closeFunc },
    { 4007, "waitpid" },
    { 4008, "creat" },
    { 4009, "link" },
    { 4010, "unlink", unlinkFunc },
    { 4011, "execve" },
    { 4012, "chdir" },
    { 4013, "time" },
    { 4014, "mknod" },
    { 4015, "chmod", chmodFunc<MipsLinux> },
    { 4016, "lchown", chownFunc },
    { 4017, "break", brkFunc },
    { 4018, "unused#18" },
    { 4019, "lseek", lseekFunc<MipsLinux> },
    { 4020, "getpid", getpidFunc },
    { 4021, "mount" },
    { 4022, "umount" },
    { 4023, "setuid", ignoreFunc },
    { 4024, "getuid", getuidFunc },
    { 4025, "stime" },
    { 4026, "ptrace" },
    { 4027, "alarm" },
    { 4028, "unused#28" },
    { 4029, "pause" },
    { 4030, "utime" },
    { 4031, "stty" },
    { 4032, "gtty" },
    { 4033, "access" },
    { 4034, "nice" },
    { 4035, "ftime" },
    { 4036, "sync" },
    { 4037, "kill" },
    { 4038, "rename" },
    { 4039, "mkdir", mkdirFunc },
    { 4040, "rmdir" },
    { 4041, "dup" },
    { 4042, "pipe", pipePseudoFunc },
    { 4043, "times" },
    { 4044, "prof" },
    { 4045, "brk", brkFunc },
    { 4046, "setgid" },
    { 4047, "getgid", getgidFunc },
    { 4048, "signal", ignoreFunc },
    { 4049, "geteuid", geteuidFunc },
    { 4050, "getegid", getegidFunc },
    { 4051, "acct" },
    { 4052, "umount2" },
    { 4053, "lock" },
    { 4054, "ioctl", ioctlFunc<MipsLinux> },
    { 4055, "fcntl", fcntlFunc },
    { 4056, "mpx" },
    { 4057, "setpgid" },
    { 4058, "ulimit" },
    { 4059, "unused#59" },
    { 4060, "umask", umaskFunc },
    { 4061, "chroot" },
    { 4062, "ustat" },
    { 4063, "dup2" },
    { 4064, "getppid", getpagesizeFunc },
    { 4065, "getpgrp" },
    { 4066, "setsid" },
    { 4067, "sigaction" },
    { 4068, "sgetmask" },
    { 4069, "ssetmask" },
    { 4070, "setreuid" },
    { 4071, "setregid" },
    { 4072, "sigsuspend" },
    { 4073, "sigpending" },
    { 4074, "sethostname", ignoreFunc },
    { 4075, "setrlimit" },
    { 4076, "getrlimit" },
    { 4077, "getrusage", getrusageFunc<MipsLinux> },
    { 4078, "gettimeofday" },
    { 4079, "settimeofday" },
    { 4080, "getgroups" },
    { 4081, "setgroups" },
    { 4082, "reserved#82" },
    { 4083, "symlink" },
    { 4084, "unused#84" },
    { 4085, "readlink", readlinkFunc<MipsLinux> },
    { 4086, "uselib" },
    { 4087, "swapon", gethostnameFunc },
    { 4088, "reboot" },
    { 4089, "readdir" },
    { 4090, "mmap", mmapFunc<MipsLinux> },
    { 4091, "munmap",munmapFunc<MipsLinux> },
    { 4092, "truncate", truncateFunc<MipsLinux> },
    { 4093, "ftruncate", ftruncateFunc<MipsLinux> },
    { 4094, "fchmod", fchmodFunc<MipsLinux> },
    { 4095, "fchown", fchownFunc },
    { 4096, "getpriority" },
    { 4097, "setpriority" },
    { 4098, "profil" },
    { 4099, "statfs" },
    { 4100, "fstatfs" },
    { 4101, "ioperm" },
    { 4102, "socketcall" },
    { 4103, "syslog" },
    { 4104, "setitimer" },
    { 4105, "getitimer" },
    { 4106, "stat",  statFunc<MipsLinux> },
    { 4107, "lstat" },
    { 4108, "fstat", fstatFunc<MipsLinux> },
    { 4109, "unused#109" },
    { 4110, "iopl" },
    { 4111, "vhangup" },
    { 4112, "idle", ignoreFunc },
    { 4113, "vm86" },
    { 4114, "wait4" },
    { 4115, "swapoff" },
    { 4116, "sysinfo", sysinfoFunc<MipsLinux> },
    { 4117, "ipc" },
    { 4118, "fsync" },
    { 4119, "sigreturn" },
    { 4120, "clone" },
    { 4121, "setdomainname" },
    { 4122, "uname", unameFunc },
    { 4123, "modify_ldt" },
    { 4124, "adjtimex" },
    { 4125, "mprotect", ignoreFunc },
    { 4126, "sigprocmask" },
    { 4127, "create_module" },
    { 4128, "init_module" },
    { 4129, "delete_module" },
    { 4130, "get_kernel_syms" },
    { 4131, "quotactl" },
    { 4132, "getpgid" },
    { 4133, "fchdir" },
    { 4134, "bdflush" },
    { 4135, "sysfs" },
    { 4136, "personality" },
    { 4137, "afs_syscall" },
    { 4138, "setfsuid" },
    { 4139, "setfsgid" },
    { 4140, "llseek" },
    { 4141, "getdents" },
    { 4142, "newselect" },
    { 4143, "flock" },
    { 4144, "msync" },
    { 4145, "readv" },
    { 4146, "writev", writevFunc<MipsLinux> },
    { 4147, "cacheflush" },
    { 4148, "cachectl" },
    { 4149, "sysmips" },
    { 4150, "unused#150" },
    { 4151, "getsid" },
    { 4152, "fdatasync" },
    { 4153, "sysctl", ignoreFunc },
    { 4154, "mlock" },
    { 4155, "munlock" },
    { 4156, "mlockall" },
    { 4157, "munlockall" },
    { 4158, "sched_setparam" },
    { 4159, "sched_getparam" },
    { 4160, "sched_setscheduler" },
    { 4161, "sched_getscheduler" },
    { 4162, "sched_yield" },
    { 4163, "sched_get_prioritymax" },
    { 4164, "sched_get_priority_min" },
    { 4165, "sched_rr_get_interval" },
    { 4166, "nanosleep" },
    { 4167, "mremap", mremapFunc<MipsLinux> },
    { 4168, "accept" },
    { 4169, "bind" },
    { 4170, "connect" },
    { 4171, "getpeername" },
    { 4172, "getsockname" },
    { 4173, "getsockopt" },
    { 4174, "listen" },
    { 4175, "recv" },
    { 4176, "recvmsg" },
    { 4177, "send" },
    { 4178, "sendmsg", ignoreFunc },
    { 4179, "sendto" },
    { 4180, "setsockopt" },
    { 4181, "shutdown" },
    { 4182, "unknown #182" },
    { 4183, "socket", ignoreFunc },
    { 4184, "socketpair" },
    { 4185, "setresuid" },
    { 4186, "getresuid" },
    { 4187, "query_module" },
    { 4188, "poll" },
    { 4189, "nfsservctl" },
    { 4190, "setresgid" },
    { 4191, "getresgid" },
    { 4192, "prctl" },
    { 4193, "rt_sigreturn" },
    { 4194, "rt_sigaction" },
    { 4195, "rt_sigprocmask" },
    { 4196, "rt_sigpending" },
    { 4197, "rt_sigtimedwait" },
    { 4198, "rt_sigqueueinfo", ignoreFunc },
    { 4199, "rt_sigsuspend" },
    { 4200, "pread64" },
    { 4201, "pwrite64" },
    { 4202, "chown" },
    { 4203, "getcwd", getcwdFunc<MipsLinux> },
    { 4204, "capget" },
    { 4205, "capset" },
    { 4206, "sigalstack" },
    { 4207, "sendfile" },
    { 4208, "getpmsg" },
    { 4209, "putpmsg" },
    { 4210, "mmap2" },
    { 4211, "truncate64" },
    { 4212, "ftruncate64" },
    { 4213, "stat64" },
    { 4214, "lstat64", lstat64Func<MipsLinux> },
    { 4215, "fstat64", fstat64Func<MipsLinux> },
    { 4216, "pivot_root" },
    { 4217, "mincore" },
    { 4218, "madvise" },
    { 4219, "getdents64" },
    { 4220, "fcntl64", fcntl64Func },
    { 4221, "reserved#221" },
    { 4222, "gettid" },
    { 4223, "readahead" },
    { 4224, "setxattr" },
    { 4225, "lsetxattr" },
    { 4226, "fsetxattr" },
    { 4227, "getxattr" },
    { 4228, "lgetxattr" },
    { 4229, "fgetxattr" },
    { 4230, "listxattr" },
    { 4231, "llistxattr" },
    { 4232, "flistxattr" },
    { 4233, "removexattr" },
    { 4234, "lremovexattr" },
    { 4235, "fremovexattr", ignoreFunc },
    { 4236, "tkill" },
    { 4237, "sendfile64" },
    { 4238, "futex" },
    { 4239, "sched_setaffinity" },
    { 4240, "sched_getaffinity" },
    { 4241, "io_setup" },
    { 4242, "io_destroy" },
    { 4243, "io_getevents" },
    { 4244, "io_submit" },
    { 4245, "io_cancel" },
    { 4246, "exit_group", exitFunc },
    { 4247, "lookup_dcookie" },
    { 4248, "epoll_create" },
    { 4249, "epoll_ctl" },
    { 4250, "epoll_wait" },
    { 4251, "remap_file_pages" },
    { 4252, "set_tid_address" },
    { 4253, "restart_syscall" },
    { 4254, "fadvise64" },
    { 4255, "statfs64" },
    { 4256, "fstafs64" },
    { 4257, "timer_create", sys_getsysinfoFunc },
    { 4258, "timer_settime", sys_setsysinfoFunc },
    { 4259, "timer_gettime" },
    { 4260, "timer_getoverrun" },
    { 4261, "timer_delete" },
    { 4262, "clock_settime" },
    { 4263, "clock_gettime" },
    { 4264, "clock_getres" },
    { 4265, "clock_nanosleep" },
    { 4266, "tgkill" },
    { 4267, "utimes" },
    { 4268, "mbind" },
    { 4269, "get_mempolicy" },
    { 4270, "set_mempolicy" },
    { 4271, "mq_open" },
    { 4272, "mq_unlink" },
    { 4273, "mq_timedsend" },
    { 4274, "mq_timedreceive" },
    { 4275, "mq_notify" },
    { 4276, "mq_getsetattr" },
    { 4277, "vserver" },
    { 4278, "waitid" },
    { 4279, "unknown #279" },
    { 4280, "add_key" },
    { 4281, "request_key" },
    { 4282, "keyctl" },
    { 4283, "set_thread_area", setThreadAreaFunc },
    { 4284, "inotify_init" },
    { 4285, "inotify_add_watch" },
    { 4286, "inotify_rm_watch" },
    { 4287, "migrate_pages" },
    { 4288, "openat" },
    { 4289, "mkdirat" },
    { 4290, "mknodat" },
    { 4291, "fchownat" },
    { 4292, "futimesat" },
    { 4293, "fstatat64" },
    { 4294, "unlinkat" },
    { 4295, "renameat" },
    { 4296, "linkat" },
    { 4297, "symlinkat" },
    { 4298, "readlinkat" },
    { 4299, "fchmodat" },
    { 4300, "faccessat" },
    { 4301, "pselect6" },
    { 4302, "ppoll" },
    { 4303, "unshare" },
    { 4304, "splice" },
    { 4305, "sync_file_range" },
    { 4306, "tee" },
    { 4307, "vmsplice" },
    { 4308, "move_pages" },
    { 4309, "set_robust_list" },
    { 4310, "get_robust_list" },
    { 4311, "kexec_load" },
    { 4312, "getcpu" },
    { 4313, "epoll_pwait" },
    { 4314, "ioprio_set" },
    { 4315, "ioprio_get" },
    { 4316, "utimensat" },
    { 4317, "signalfd" },
    { 4318, "timerfd" },
    { 4319, "eventfd" }
};

} // namespace MipsISA
} // namespace gem5
