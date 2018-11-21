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
 *
 * Authors: Gabe Black
 */

#include "arch/x86/linux/process.hh"

#include <sys/syscall.h>

#include "arch/x86/isa_traits.hh"
#include "arch/x86/linux/linux.hh"
#include "arch/x86/registers.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace X86ISA;

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, Process *process,
          ThreadContext *tc)
{
    int index = 0;
    TypedBufferArg<Linux::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, "3.2.0");
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "x86_64");

    name.copyOut(tc->getMemProxy());

    return 0;
}

static SyscallReturn
archPrctlFunc(SyscallDesc *desc, int callnum, Process *process,
              ThreadContext *tc)
{
    enum ArchPrctlCodes
    {
        SetFS = 0x1002,
        GetFS = 0x1003,
        SetGS = 0x1001,
        GetGS = 0x1004
    };

    // First argument is the code, second is the address
    int index = 0;
    int code = process->getSyscallArg(tc, index);
    uint64_t addr = process->getSyscallArg(tc, index);
    uint64_t fsBase, gsBase;
    SETranslatingPortProxy &p = tc->getMemProxy();
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
setThreadArea32Func(SyscallDesc *desc, int callnum,
                    Process *process, ThreadContext *tc)
{
    const int minTLSEntry = 6;
    const int numTLSEntries = 3;
    const int maxTLSEntry = minTLSEntry + numTLSEntries - 1;

    X86Process *x86p = dynamic_cast<X86Process *>(process);
    assert(x86p);

    assert((maxTLSEntry + 1) * sizeof(uint64_t) <= x86p->gdtSize());

    int argIndex = 0;
    TypedBufferArg<UserDesc32> userDesc(process->getSyscallArg(tc, argIndex));
    TypedBufferArg<uint64_t>
        gdt(x86p->gdtStart() + minTLSEntry * sizeof(uint64_t),
            numTLSEntries * sizeof(uint64_t));

    if (!userDesc.copyIn(tc->getMemProxy()))
        return -EFAULT;

    if (!gdt.copyIn(tc->getMemProxy()))
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

    if (!userDesc.copyOut(tc->getMemProxy()))
        return -EFAULT;
    if (!gdt.copyOut(tc->getMemProxy()))
        panic("Failed to copy out GDT for %s.\n", desc->name());

    return 0;
}

static SyscallDesc syscallDescs64[] = {
    /*   0 */ SyscallDesc("read", readFunc<X86Linux64>),
    /*   1 */ SyscallDesc("write", writeFunc<X86Linux64>),
    /*   2 */ SyscallDesc("open", openFunc<X86Linux64>),
    /*   3 */ SyscallDesc("close", closeFunc),
    /*   4 */ SyscallDesc("stat", stat64Func<X86Linux64>),
    /*   5 */ SyscallDesc("fstat", fstat64Func<X86Linux64>),
    /*   6 */ SyscallDesc("lstat", lstat64Func<X86Linux64>),
    /*   7 */ SyscallDesc("poll", pollFunc<X86Linux64>),
    /*   8 */ SyscallDesc("lseek", lseekFunc),
    /*   9 */ SyscallDesc("mmap", mmapFunc<X86Linux64>),
    /*  10 */ SyscallDesc("mprotect", ignoreFunc),
    /*  11 */ SyscallDesc("munmap", munmapFunc),
    /*  12 */ SyscallDesc("brk", brkFunc),
    /*  13 */ SyscallDesc("rt_sigaction", ignoreFunc, SyscallDesc::WarnOnce),
    /*  14 */ SyscallDesc("rt_sigprocmask", ignoreFunc, SyscallDesc::WarnOnce),
    /*  15 */ SyscallDesc("rt_sigreturn", unimplementedFunc),
    /*  16 */ SyscallDesc("ioctl", ioctlFunc<X86Linux64>),
    /*  17 */ SyscallDesc("pread64", unimplementedFunc),
    /*  18 */ SyscallDesc("pwrite64", pwrite64Func<X86Linux64>),
    /*  19 */ SyscallDesc("readv", readvFunc<X86Linux64>),
    /*  20 */ SyscallDesc("writev", writevFunc<X86Linux64>),
    /*  21 */ SyscallDesc("access", ignoreFunc),
    /*  22 */ SyscallDesc("pipe", pipeFunc),
    /*  23 */ SyscallDesc("select", selectFunc<X86Linux64>),
    /*  24 */ SyscallDesc("sched_yield", ignoreFunc),
    /*  25 */ SyscallDesc("mremap", mremapFunc<X86Linux64>),
    /*  26 */ SyscallDesc("msync", unimplementedFunc),
    /*  27 */ SyscallDesc("mincore", unimplementedFunc),
    /*  28 */ SyscallDesc("madvise", ignoreFunc),
    /*  29 */ SyscallDesc("shmget", unimplementedFunc),
    /*  30 */ SyscallDesc("shmat", unimplementedFunc),
    /*  31 */ SyscallDesc("shmctl", unimplementedFunc),
    /*  32 */ SyscallDesc("dup", dupFunc),
    /*  33 */ SyscallDesc("dup2", dup2Func),
    /*  34 */ SyscallDesc("pause", unimplementedFunc),
    /*  35 */ SyscallDesc("nanosleep", ignoreFunc, SyscallDesc::WarnOnce),
    /*  36 */ SyscallDesc("getitimer", unimplementedFunc),
    /*  37 */ SyscallDesc("alarm", unimplementedFunc),
    /*  38 */ SyscallDesc("setitimer", unimplementedFunc),
    /*  39 */ SyscallDesc("getpid", getpidFunc),
    /*  40 */ SyscallDesc("sendfile", unimplementedFunc),
    /*  41 */ SyscallDesc("socket", socketFunc<X86Linux64>),
    /*  42 */ SyscallDesc("connect", connectFunc),
    /*  43 */ SyscallDesc("accept", acceptFunc<X86Linux64>),
    /*  44 */ SyscallDesc("sendto", sendtoFunc),
    /*  45 */ SyscallDesc("recvfrom", recvfromFunc),
    /*  46 */ SyscallDesc("sendmsg", sendmsgFunc),
    /*  47 */ SyscallDesc("recvmsg", recvmsgFunc),
    /*  48 */ SyscallDesc("shutdown", shutdownFunc),
    /*  49 */ SyscallDesc("bind", bindFunc),
    /*  50 */ SyscallDesc("listen", listenFunc),
    /*  51 */ SyscallDesc("getsockname", getsocknameFunc),
    /*  52 */ SyscallDesc("getpeername", getpeernameFunc),
    /*  53 */ SyscallDesc("socketpair", socketpairFunc<X86Linux64>),
    /*  54 */ SyscallDesc("setsockopt", setsockoptFunc),
    /*  55 */ SyscallDesc("getsockopt", getsockoptFunc),
    /*  56 */ SyscallDesc("clone", cloneFunc<X86Linux64>),
    /*  57 */ SyscallDesc("fork", unimplementedFunc),
    /*  58 */ SyscallDesc("vfork", unimplementedFunc),
    /*  59 */ SyscallDesc("execve", execveFunc<X86Linux64>),
    /*  60 */ SyscallDesc("exit", exitFunc),
    /*  61 */ SyscallDesc("wait4", wait4Func<X86Linux64>),
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
    /*  72 */ SyscallDesc("fcntl", fcntlFunc),
    /*  73 */ SyscallDesc("flock", unimplementedFunc),
    /*  74 */ SyscallDesc("fsync", unimplementedFunc),
    /*  75 */ SyscallDesc("fdatasync", unimplementedFunc),
    /*  76 */ SyscallDesc("truncate", truncateFunc),
    /*  77 */ SyscallDesc("ftruncate", ftruncateFunc),
#if defined(SYS_getdents)
    /*  78 */ SyscallDesc("getdents", getdentsFunc),
#else
    /*  78 */ SyscallDesc("getdents", unimplementedFunc),
#endif
    /*  79 */ SyscallDesc("getcwd", getcwdFunc),
    /*  80 */ SyscallDesc("chdir", chdirFunc),
    /*  81 */ SyscallDesc("fchdir", unimplementedFunc),
    /*  82 */ SyscallDesc("rename", renameFunc),
    /*  83 */ SyscallDesc("mkdir", mkdirFunc),
    /*  84 */ SyscallDesc("rmdir", rmdirFunc),
    /*  85 */ SyscallDesc("creat", unimplementedFunc),
    /*  86 */ SyscallDesc("link", linkFunc),
    /*  87 */ SyscallDesc("unlink", unlinkFunc),
    /*  88 */ SyscallDesc("symlink", symlinkFunc),
    /*  89 */ SyscallDesc("readlink", readlinkFunc),
    /*  90 */ SyscallDesc("chmod", unimplementedFunc),
    /*  91 */ SyscallDesc("fchmod", unimplementedFunc),
    /*  92 */ SyscallDesc("chown", unimplementedFunc),
    /*  93 */ SyscallDesc("fchown", unimplementedFunc),
    /*  94 */ SyscallDesc("lchown", unimplementedFunc),
    /*  95 */ SyscallDesc("umask", umaskFunc),
    /*  96 */ SyscallDesc("gettimeofday", gettimeofdayFunc<X86Linux64>),
    /*  97 */ SyscallDesc("getrlimit", getrlimitFunc<X86Linux64>),
    /*  98 */ SyscallDesc("getrusage", getrusageFunc<X86Linux64>),
    /*  99 */ SyscallDesc("sysinfo", sysinfoFunc<X86Linux64>),
    /* 100 */ SyscallDesc("times", timesFunc<X86Linux64>),
    /* 101 */ SyscallDesc("ptrace", unimplementedFunc),
    /* 102 */ SyscallDesc("getuid", getuidFunc),
    /* 103 */ SyscallDesc("syslog", unimplementedFunc),
    /* 104 */ SyscallDesc("getgid", getgidFunc),
    /* 105 */ SyscallDesc("setuid", unimplementedFunc),
    /* 106 */ SyscallDesc("setgid", unimplementedFunc),
    /* 107 */ SyscallDesc("geteuid", geteuidFunc),
    /* 108 */ SyscallDesc("getegid", getegidFunc),
    /* 109 */ SyscallDesc("setpgid", setpgidFunc),
    /* 110 */ SyscallDesc("getppid", getppidFunc),
    /* 111 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 112 */ SyscallDesc("setsid", unimplementedFunc),
    /* 113 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 114 */ SyscallDesc("setregid", unimplementedFunc),
    /* 115 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 116 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 117 */ SyscallDesc("setresuid", ignoreFunc),
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
    /* 133 */ SyscallDesc("mknod", mknodFunc),
    /* 134 */ SyscallDesc("uselib", unimplementedFunc),
    /* 135 */ SyscallDesc("personality", unimplementedFunc),
    /* 136 */ SyscallDesc("ustat", unimplementedFunc),
    /* 137 */ SyscallDesc("statfs", statfsFunc<X86Linux64>),
    /* 138 */ SyscallDesc("fstatfs", fstatfsFunc<X86Linux64>),
    /* 139 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 140 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 141 */ SyscallDesc("setpriority", ignoreFunc),
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
    /* 158 */ SyscallDesc("arch_prctl", archPrctlFunc),
    /* 159 */ SyscallDesc("adjtimex", unimplementedFunc),
    /* 160 */ SyscallDesc("setrlimit", ignoreFunc),
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
    /* 186 */ SyscallDesc("gettid", gettidFunc),
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
    /* 201 */ SyscallDesc("time", timeFunc<X86Linux64>),
    /* 202 */ SyscallDesc("futex", futexFunc<X86Linux64>),
    /* 203 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 204 */ SyscallDesc("sched_getaffinity", ignoreFunc),
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
    /* 218 */ SyscallDesc("set_tid_address", setTidAddressFunc),
    /* 219 */ SyscallDesc("restart_syscall", unimplementedFunc),
    /* 220 */ SyscallDesc("semtimedop", unimplementedFunc),
    /* 221 */ SyscallDesc("fadvise64", unimplementedFunc),
    /* 222 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 223 */ SyscallDesc("timer_settime", unimplementedFunc),
    /* 224 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 225 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 226 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 227 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 228 */ SyscallDesc("clock_gettime", clock_gettimeFunc<X86Linux64>),
    /* 229 */ SyscallDesc("clock_getres", clock_getresFunc<X86Linux64>),
    /* 230 */ SyscallDesc("clock_nanosleep", unimplementedFunc),
    /* 231 */ SyscallDesc("exit_group", exitGroupFunc),
    /* 232 */ SyscallDesc("epoll_wait", unimplementedFunc),
    /* 233 */ SyscallDesc("epoll_ctl", unimplementedFunc),
    /* 234 */ SyscallDesc("tgkill", tgkillFunc<X86Linux64>),
    /* 235 */ SyscallDesc("utimes", unimplementedFunc),
    /* 236 */ SyscallDesc("vserver", unimplementedFunc),
    /* 237 */ SyscallDesc("mbind", unimplementedFunc),
    /* 238 */ SyscallDesc("set_mempolicy", unimplementedFunc),
    /* 239 */ SyscallDesc("get_mempolicy", ignoreFunc),
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
    /* 257 */ SyscallDesc("openat", openatFunc<X86Linux64>),
    /* 258 */ SyscallDesc("mkdirat", unimplementedFunc),
    /* 259 */ SyscallDesc("mknodat", unimplementedFunc),
    /* 260 */ SyscallDesc("fchownat", unimplementedFunc),
    /* 261 */ SyscallDesc("futimesat", unimplementedFunc),
    /* 262 */ SyscallDesc("newfstatat", unimplementedFunc),
    /* 263 */ SyscallDesc("unlinkat", unimplementedFunc),
    /* 264 */ SyscallDesc("renameat", unimplementedFunc),
    /* 265 */ SyscallDesc("linkat", unimplementedFunc),
    /* 266 */ SyscallDesc("symlinkat", unimplementedFunc),
    /* 267 */ SyscallDesc("readlinkat", readlinkFunc),
    /* 268 */ SyscallDesc("fchmodat", unimplementedFunc),
    /* 269 */ SyscallDesc("faccessat", unimplementedFunc),
    /* 270 */ SyscallDesc("pselect6", unimplementedFunc),
    /* 271 */ SyscallDesc("ppoll", unimplementedFunc),
    /* 272 */ SyscallDesc("unshare", unimplementedFunc),
    /* 273 */ SyscallDesc("set_robust_list", ignoreFunc),
    /* 274 */ SyscallDesc("get_robust_list", unimplementedFunc),
    /* 275 */ SyscallDesc("splice", unimplementedFunc),
    /* 276 */ SyscallDesc("tee", unimplementedFunc),
    /* 277 */ SyscallDesc("sync_file_range", unimplementedFunc),
    /* 278 */ SyscallDesc("vmsplice", unimplementedFunc),
    /* 279 */ SyscallDesc("move_pages", unimplementedFunc),
    /* 280 */ SyscallDesc("utimensat", unimplementedFunc),
    /* 281 */ SyscallDesc("epoll_pwait", unimplementedFunc),
    /* 282 */ SyscallDesc("signalfd", unimplementedFunc),
    /* 283 */ SyscallDesc("timerfd_create", unimplementedFunc),
    /* 284 */ SyscallDesc("eventfd", unimplementedFunc),
    /* 285 */ SyscallDesc("fallocate", fallocateFunc),
    /* 286 */ SyscallDesc("timerfd_settime", unimplementedFunc),
    /* 287 */ SyscallDesc("timerfd_gettime", unimplementedFunc),
    /* 288 */ SyscallDesc("accept4", unimplementedFunc),
    /* 289 */ SyscallDesc("signalfd4", unimplementedFunc),
    /* 290 */ SyscallDesc("eventfd2", unimplementedFunc),
    /* 291 */ SyscallDesc("epoll_create1", unimplementedFunc),
    /* 292 */ SyscallDesc("dup3", unimplementedFunc),
    /* 293 */ SyscallDesc("pipe2", unimplementedFunc),
    /* 294 */ SyscallDesc("inotify_init1", unimplementedFunc),
    /* 295 */ SyscallDesc("preadv", unimplementedFunc),
    /* 296 */ SyscallDesc("pwritev", unimplementedFunc),
    /* 297 */ SyscallDesc("rt_tgsigqueueinfo", unimplementedFunc),
    /* 298 */ SyscallDesc("perf_event_open", unimplementedFunc),
    /* 299 */ SyscallDesc("recvmmsg", unimplementedFunc),
    /* 300 */ SyscallDesc("fanotify_init", unimplementedFunc),
    /* 301 */ SyscallDesc("fanotify_mark", unimplementedFunc),
    /* 302 */ SyscallDesc("prlimit64", prlimitFunc<X86Linux64>),
    /* 303 */ SyscallDesc("name_to_handle_at", unimplementedFunc),
    /* 304 */ SyscallDesc("open_by_handle_at", unimplementedFunc),
    /* 305 */ SyscallDesc("clock_adjtime", unimplementedFunc),
    /* 306 */ SyscallDesc("syncfs", unimplementedFunc),
    /* 307 */ SyscallDesc("sendmmsg", unimplementedFunc),
    /* 308 */ SyscallDesc("setns", unimplementedFunc),
    /* 309 */ SyscallDesc("getcpu", unimplementedFunc),
    /* 310 */ SyscallDesc("proess_vm_readv", unimplementedFunc),
    /* 311 */ SyscallDesc("proess_vm_writev", unimplementedFunc),
    /* 312 */ SyscallDesc("kcmp", unimplementedFunc),
    /* 313 */ SyscallDesc("finit_module", unimplementedFunc),
};

X86_64LinuxProcess::X86_64LinuxProcess(ProcessParams * params,
                                       ObjectFile *objFile)
    : X86_64Process(params, objFile, syscallDescs64,
                    sizeof(syscallDescs64) / sizeof(SyscallDesc))
{}

void X86_64LinuxProcess::clone(ThreadContext *old_tc, ThreadContext *new_tc,
                               Process *process, RegVal flags)
{
    X86_64Process::clone(old_tc, new_tc, (X86_64Process*)process, flags);
}

static SyscallDesc syscallDescs32[] = {
    /*   0 */ SyscallDesc("restart_syscall", unimplementedFunc),
    /*   1 */ SyscallDesc("exit", exitFunc),
    /*   2 */ SyscallDesc("fork", unimplementedFunc),
    /*   3 */ SyscallDesc("read", readFunc<X86Linux32>),
    /*   4 */ SyscallDesc("write", writeFunc<X86Linux32>),
    /*   5 */ SyscallDesc("open", openFunc<X86Linux32>),
    /*   6 */ SyscallDesc("close", closeFunc),
    /*   7 */ SyscallDesc("waitpid", unimplementedFunc),
    /*   8 */ SyscallDesc("creat", unimplementedFunc),
    /*   9 */ SyscallDesc("link", unimplementedFunc),
    /*  10 */ SyscallDesc("unlink", unimplementedFunc),
    /*  11 */ SyscallDesc("execve", execveFunc<X86Linux32>),
    /*  12 */ SyscallDesc("chdir", chdirFunc),
    /*  13 */ SyscallDesc("time", timeFunc<X86Linux32>),
    /*  14 */ SyscallDesc("mknod", mknodFunc),
    /*  15 */ SyscallDesc("chmod", unimplementedFunc),
    /*  16 */ SyscallDesc("lchown", unimplementedFunc),
    /*  17 */ SyscallDesc("break", unimplementedFunc),
    /*  18 */ SyscallDesc("oldstat", unimplementedFunc),
    /*  19 */ SyscallDesc("lseek", unimplementedFunc),
    /*  20 */ SyscallDesc("getpid", getpidFunc),
    /*  21 */ SyscallDesc("mount", unimplementedFunc),
    /*  22 */ SyscallDesc("umount", unimplementedFunc),
    /*  23 */ SyscallDesc("setuid", unimplementedFunc),
    /*  24 */ SyscallDesc("getuid", getuidFunc),
    /*  25 */ SyscallDesc("stime", unimplementedFunc),
    /*  26 */ SyscallDesc("ptrace", unimplementedFunc),
    /*  27 */ SyscallDesc("alarm", unimplementedFunc),
    /*  28 */ SyscallDesc("oldfstat", unimplementedFunc),
    /*  29 */ SyscallDesc("pause", unimplementedFunc),
    /*  30 */ SyscallDesc("utime", unimplementedFunc),
    /*  31 */ SyscallDesc("stty", unimplementedFunc),
    /*  32 */ SyscallDesc("gtty", unimplementedFunc),
    /*  33 */ SyscallDesc("access", ignoreFunc),
    /*  34 */ SyscallDesc("nice", unimplementedFunc),
    /*  35 */ SyscallDesc("ftime", unimplementedFunc),
    /*  36 */ SyscallDesc("sync", unimplementedFunc),
    /*  37 */ SyscallDesc("kill", unimplementedFunc),
    /*  38 */ SyscallDesc("rename", unimplementedFunc),
    /*  39 */ SyscallDesc("mkdir", mkdirFunc),
    /*  40 */ SyscallDesc("rmdir", mkdirFunc),
    /*  41 */ SyscallDesc("dup", dupFunc),
    /*  42 */ SyscallDesc("pipe", pipeFunc),
    /*  43 */ SyscallDesc("times", timesFunc<X86Linux32>),
    /*  44 */ SyscallDesc("prof", unimplementedFunc),
    /*  45 */ SyscallDesc("brk", brkFunc),
    /*  46 */ SyscallDesc("setgid", unimplementedFunc),
    /*  47 */ SyscallDesc("getgid", getgidFunc),
    /*  48 */ SyscallDesc("signal", unimplementedFunc),
    /*  49 */ SyscallDesc("geteuid", geteuidFunc),
    /*  50 */ SyscallDesc("getegid", getegidFunc),
    /*  51 */ SyscallDesc("acct", unimplementedFunc),
    /*  52 */ SyscallDesc("umount2", unimplementedFunc),
    /*  53 */ SyscallDesc("lock", unimplementedFunc),
    /*  54 */ SyscallDesc("ioctl", ioctlFunc<X86Linux32>),
    /*  55 */ SyscallDesc("fcntl", fcntlFunc),
    /*  56 */ SyscallDesc("mpx", unimplementedFunc),
    /*  57 */ SyscallDesc("setpgid", setpgidFunc),
    /*  58 */ SyscallDesc("ulimit", unimplementedFunc),
    /*  59 */ SyscallDesc("oldolduname", unimplementedFunc),
    /*  60 */ SyscallDesc("umask", umaskFunc),
    /*  61 */ SyscallDesc("chroot", unimplementedFunc),
    /*  62 */ SyscallDesc("ustat", unimplementedFunc),
    /*  63 */ SyscallDesc("dup2", dup2Func),
    /*  64 */ SyscallDesc("getppid", unimplementedFunc),
    /*  65 */ SyscallDesc("getpgrp", unimplementedFunc),
    /*  66 */ SyscallDesc("setsid", unimplementedFunc),
    /*  67 */ SyscallDesc("sigaction", unimplementedFunc),
    /*  68 */ SyscallDesc("sgetmask", unimplementedFunc),
    /*  69 */ SyscallDesc("ssetmask", unimplementedFunc),
    /*  70 */ SyscallDesc("setreuid", unimplementedFunc),
    /*  71 */ SyscallDesc("setregid", unimplementedFunc),
    /*  72 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /*  73 */ SyscallDesc("sigpending", unimplementedFunc),
    /*  74 */ SyscallDesc("sethostname", unimplementedFunc),
    /*  75 */ SyscallDesc("setrlimit", ignoreFunc),
    /*  76 */ SyscallDesc("getrlimit", getrlimitFunc<X86Linux32>),
    /*  77 */ SyscallDesc("getrusage", getrusageFunc<X86Linux32>),
    /*  78 */ SyscallDesc("gettimeofday", unimplementedFunc),
    /*  79 */ SyscallDesc("settimeofday", unimplementedFunc),
    /*  80 */ SyscallDesc("getgroups", unimplementedFunc),
    /*  81 */ SyscallDesc("setgroups", unimplementedFunc),
    /*  82 */ SyscallDesc("select", selectFunc<X86Linux32>),
    /*  83 */ SyscallDesc("symlink", unimplementedFunc),
    /*  84 */ SyscallDesc("oldlstat", unimplementedFunc),
    /*  85 */ SyscallDesc("readlink", readlinkFunc),
    /*  86 */ SyscallDesc("uselib", unimplementedFunc),
    /*  87 */ SyscallDesc("swapon", unimplementedFunc),
    /*  88 */ SyscallDesc("reboot", unimplementedFunc),
    /*  89 */ SyscallDesc("readdir", unimplementedFunc),
    /*  90 */ SyscallDesc("mmap", unimplementedFunc),
    /*  91 */ SyscallDesc("munmap", munmapFunc),
    /*  92 */ SyscallDesc("truncate", truncateFunc),
    /*  93 */ SyscallDesc("ftruncate", ftruncateFunc),
    /*  94 */ SyscallDesc("fchmod", unimplementedFunc),
    /*  95 */ SyscallDesc("fchown", unimplementedFunc),
    /*  96 */ SyscallDesc("getpriority", unimplementedFunc),
    /*  97 */ SyscallDesc("setpriority", ignoreFunc),
    /*  98 */ SyscallDesc("profil", unimplementedFunc),
    /*  99 */ SyscallDesc("statfs", ignoreFunc),
    /* 100 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 101 */ SyscallDesc("ioperm", unimplementedFunc),
    /* 102 */ SyscallDesc("socketcall", unimplementedFunc),
    /* 103 */ SyscallDesc("syslog", unimplementedFunc),
    /* 104 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 105 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 106 */ SyscallDesc("stat", unimplementedFunc),
    /* 107 */ SyscallDesc("lstat", unimplementedFunc),
    /* 108 */ SyscallDesc("fstat", unimplementedFunc),
    /* 109 */ SyscallDesc("olduname", unimplementedFunc),
    /* 110 */ SyscallDesc("iopl", unimplementedFunc),
    /* 111 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 112 */ SyscallDesc("idle", unimplementedFunc),
    /* 113 */ SyscallDesc("vm86old", unimplementedFunc),
    /* 114 */ SyscallDesc("wait4", wait4Func<X86Linux32>),
    /* 115 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 116 */ SyscallDesc("sysinfo", sysinfoFunc<X86Linux32>),
    /* 117 */ SyscallDesc("ipc", unimplementedFunc),
    /* 118 */ SyscallDesc("fsync", unimplementedFunc),
    /* 119 */ SyscallDesc("sigreturn", unimplementedFunc),
    /* 120 */ SyscallDesc("clone", cloneFunc<X86Linux32>),
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
    /* 140 */ SyscallDesc("_llseek", _llseekFunc),
#if defined(SYS_getdents)
    /* 141 */ SyscallDesc("getdents", getdentsFunc),
#else
    /* 141 */ SyscallDesc("getdents", unimplementedFunc),
#endif
    /* 142 */ SyscallDesc("_newselect", unimplementedFunc),
    /* 143 */ SyscallDesc("flock", unimplementedFunc),
    /* 144 */ SyscallDesc("msync", unimplementedFunc),
    /* 145 */ SyscallDesc("readv", readvFunc<X86Linux32>),
    /* 146 */ SyscallDesc("writev", writevFunc<X86Linux32>),
    /* 147 */ SyscallDesc("getsid", unimplementedFunc),
    /* 148 */ SyscallDesc("fdatasync", unimplementedFunc),
    /* 149 */ SyscallDesc("_sysctl", unimplementedFunc),
    /* 150 */ SyscallDesc("mlock", unimplementedFunc),
    /* 151 */ SyscallDesc("munlock", unimplementedFunc),
    /* 152 */ SyscallDesc("mlockall", unimplementedFunc),
    /* 153 */ SyscallDesc("munlockall", unimplementedFunc),
    /* 154 */ SyscallDesc("sched_setparam", unimplementedFunc),
    /* 155 */ SyscallDesc("sched_getparam", unimplementedFunc),
    /* 156 */ SyscallDesc("sched_setscheduler", unimplementedFunc),
    /* 157 */ SyscallDesc("sched_getscheduler", unimplementedFunc),
    /* 158 */ SyscallDesc("sched_yield", ignoreFunc),
    /* 159 */ SyscallDesc("sched_get_priority_max", unimplementedFunc),
    /* 160 */ SyscallDesc("sched_get_priority_min", unimplementedFunc),
    /* 161 */ SyscallDesc("sched_rr_get_interval", unimplementedFunc),
    /* 162 */ SyscallDesc("nanosleep", ignoreFunc),
    /* 163 */ SyscallDesc("mremap", unimplementedFunc),
    /* 164 */ SyscallDesc("setresuid", ignoreFunc),
    /* 165 */ SyscallDesc("getresuid", unimplementedFunc),
    /* 166 */ SyscallDesc("vm86", unimplementedFunc),
    /* 167 */ SyscallDesc("query_module", unimplementedFunc),
    /* 168 */ SyscallDesc("poll", pollFunc<X86Linux32>),
    /* 169 */ SyscallDesc("nfsservctl", unimplementedFunc),
    /* 170 */ SyscallDesc("setresgid", unimplementedFunc),
    /* 171 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 172 */ SyscallDesc("prctl", unimplementedFunc),
    /* 173 */ SyscallDesc("rt_sigreturn", unimplementedFunc),
    /* 174 */ SyscallDesc("rt_sigaction", ignoreFunc),
    /* 175 */ SyscallDesc("rt_sigprocmask", ignoreFunc),
    /* 176 */ SyscallDesc("rt_sigpending", unimplementedFunc),
    /* 177 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 178 */ SyscallDesc("rt_sigqueueinfo", unimplementedFunc),
    /* 179 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 180 */ SyscallDesc("pread64", unimplementedFunc),
    /* 181 */ SyscallDesc("pwrite64", unimplementedFunc),
    /* 182 */ SyscallDesc("chown", unimplementedFunc),
    /* 183 */ SyscallDesc("getcwd", getcwdFunc),
    /* 184 */ SyscallDesc("capget", unimplementedFunc),
    /* 185 */ SyscallDesc("capset", unimplementedFunc),
    /* 186 */ SyscallDesc("sigaltstack", unimplementedFunc),
    /* 187 */ SyscallDesc("sendfile", unimplementedFunc),
    /* 188 */ SyscallDesc("getpmsg", unimplementedFunc),
    /* 189 */ SyscallDesc("putpmsg", unimplementedFunc),
    /* 190 */ SyscallDesc("vfork", unimplementedFunc),
    /* 191 */ SyscallDesc("ugetrlimit", ignoreFunc),
    /* 192 */ SyscallDesc("mmap2", mmap2Func<X86Linux32>),
    /* 193 */ SyscallDesc("truncate64", truncate64Func),
    /* 194 */ SyscallDesc("ftruncate64", ftruncate64Func),
    /* 195 */ SyscallDesc("stat64", stat64Func<X86Linux32>),
    /* 196 */ SyscallDesc("lstat64", unimplementedFunc),
    /* 197 */ SyscallDesc("fstat64", fstat64Func<X86Linux32>),
    /* 198 */ SyscallDesc("lchown32", unimplementedFunc),
    /* 199 */ SyscallDesc("getuid32", getuidFunc),
    /* 200 */ SyscallDesc("getgid32", getgidFunc),
    /* 201 */ SyscallDesc("geteuid32", geteuidFunc),
    /* 202 */ SyscallDesc("getegid32", getegidFunc),
    /* 203 */ SyscallDesc("setreuid32", unimplementedFunc),
    /* 204 */ SyscallDesc("setregid32", unimplementedFunc),
    /* 205 */ SyscallDesc("getgroups32", unimplementedFunc),
    /* 206 */ SyscallDesc("setgroups32", unimplementedFunc),
    /* 207 */ SyscallDesc("fchown32", unimplementedFunc),
    /* 208 */ SyscallDesc("setresuid32", unimplementedFunc),
    /* 209 */ SyscallDesc("getresuid32", unimplementedFunc),
    /* 210 */ SyscallDesc("setresgid32", unimplementedFunc),
    /* 211 */ SyscallDesc("getresgid32", unimplementedFunc),
    /* 212 */ SyscallDesc("chown32", unimplementedFunc),
    /* 213 */ SyscallDesc("setuid32", unimplementedFunc),
    /* 214 */ SyscallDesc("setgid32", unimplementedFunc),
    /* 215 */ SyscallDesc("setfsuid32", unimplementedFunc),
    /* 216 */ SyscallDesc("setfsgid32", unimplementedFunc),
    /* 217 */ SyscallDesc("pivot_root", unimplementedFunc),
    /* 218 */ SyscallDesc("mincore", unimplementedFunc),
    /* 219 */ SyscallDesc("madvise", ignoreFunc),
    /* 220 */ SyscallDesc("madvise1", unimplementedFunc),
    /* 221 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 222 */ SyscallDesc("fcntl64", unimplementedFunc),
    /* 223 */ SyscallDesc("unused", unimplementedFunc),
    /* 224 */ SyscallDesc("gettid", gettidFunc),
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
    /* 240 */ SyscallDesc("futex", unimplementedFunc),
    /* 241 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 242 */ SyscallDesc("sched_getaffinity", ignoreFunc),
    /* 243 */ SyscallDesc("set_thread_area", setThreadArea32Func),
    /* 244 */ SyscallDesc("get_thread_area", unimplementedFunc),
    /* 245 */ SyscallDesc("io_setup", unimplementedFunc),
    /* 246 */ SyscallDesc("io_destroy", unimplementedFunc),
    /* 247 */ SyscallDesc("io_getevents", unimplementedFunc),
    /* 248 */ SyscallDesc("io_submit", unimplementedFunc),
    /* 249 */ SyscallDesc("io_cancel", unimplementedFunc),
    /* 250 */ SyscallDesc("fadvise64", unimplementedFunc),
    /* 251 */ SyscallDesc("unused", unimplementedFunc),
    /* 252 */ SyscallDesc("exit_group", exitFunc),
    /* 253 */ SyscallDesc("lookup_dcookie", unimplementedFunc),
    /* 254 */ SyscallDesc("epoll_create", unimplementedFunc),
    /* 255 */ SyscallDesc("epoll_ctl", unimplementedFunc),
    /* 256 */ SyscallDesc("epoll_wait", unimplementedFunc),
    /* 257 */ SyscallDesc("remap_file_pages", unimplementedFunc),
    /* 258 */ SyscallDesc("set_tid_address", setTidAddressFunc),
    /* 259 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 260 */ SyscallDesc("timer_settime", unimplementedFunc),
    /* 261 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 262 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 263 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 264 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 265 */ SyscallDesc("clock_gettime", clock_gettimeFunc<X86Linux32>),
    /* 266 */ SyscallDesc("clock_getres", unimplementedFunc),
    /* 267 */ SyscallDesc("clock_nanosleep", unimplementedFunc),
    /* 268 */ SyscallDesc("statfs64", unimplementedFunc),
    /* 269 */ SyscallDesc("fstatfs64", unimplementedFunc),
    /* 270 */ SyscallDesc("tgkill", tgkillFunc<X86Linux32>),
    /* 271 */ SyscallDesc("utimes", unimplementedFunc),
    /* 272 */ SyscallDesc("fadvise64_64", unimplementedFunc),
    /* 273 */ SyscallDesc("vserver", unimplementedFunc),
    /* 274 */ SyscallDesc("mbind", unimplementedFunc),
    /* 275 */ SyscallDesc("get_mempolicy", ignoreFunc),
    /* 276 */ SyscallDesc("set_mempolicy", unimplementedFunc),
    /* 277 */ SyscallDesc("mq_open", unimplementedFunc),
    /* 278 */ SyscallDesc("mq_unlink", unimplementedFunc),
    /* 279 */ SyscallDesc("mq_timedsend", unimplementedFunc),
    /* 280 */ SyscallDesc("mq_timedreceive", unimplementedFunc),
    /* 281 */ SyscallDesc("mq_notify", unimplementedFunc),
    /* 282 */ SyscallDesc("mq_getsetattr", unimplementedFunc),
    /* 283 */ SyscallDesc("kexec_load", unimplementedFunc),
    /* 284 */ SyscallDesc("waitid", unimplementedFunc),
    /* 285 */ SyscallDesc("sys_setaltroot", unimplementedFunc),
    /* 286 */ SyscallDesc("add_key", unimplementedFunc),
    /* 287 */ SyscallDesc("request_key", unimplementedFunc),
    /* 288 */ SyscallDesc("keyctl", unimplementedFunc),
    /* 289 */ SyscallDesc("ioprio_set", unimplementedFunc),
    /* 290 */ SyscallDesc("ioprio_get", unimplementedFunc),
    /* 291 */ SyscallDesc("inotify_init", unimplementedFunc),
    /* 292 */ SyscallDesc("inotify_add_watch", unimplementedFunc),
    /* 293 */ SyscallDesc("inotify_rm_watch", unimplementedFunc),
    /* 294 */ SyscallDesc("migrate_pages", unimplementedFunc),
    /* 295 */ SyscallDesc("openat", openatFunc<X86Linux32>),
    /* 296 */ SyscallDesc("mkdirat", unimplementedFunc),
    /* 297 */ SyscallDesc("mknodat", unimplementedFunc),
    /* 298 */ SyscallDesc("fchownat", unimplementedFunc),
    /* 299 */ SyscallDesc("futimesat", unimplementedFunc),
    /* 300 */ SyscallDesc("fstatat64", unimplementedFunc),
    /* 301 */ SyscallDesc("unlinkat", unimplementedFunc),
    /* 302 */ SyscallDesc("renameat", unimplementedFunc),
    /* 303 */ SyscallDesc("linkat", unimplementedFunc),
    /* 304 */ SyscallDesc("symlinkat", unimplementedFunc),
    /* 305 */ SyscallDesc("readlinkat", readlinkFunc),
    /* 306 */ SyscallDesc("fchmodat", unimplementedFunc),
    /* 307 */ SyscallDesc("faccessat", unimplementedFunc),
    /* 308 */ SyscallDesc("pselect6", unimplementedFunc),
    /* 309 */ SyscallDesc("ppoll", unimplementedFunc),
    /* 310 */ SyscallDesc("unshare", unimplementedFunc),
    /* 311 */ SyscallDesc("set_robust_list", ignoreFunc),
    /* 312 */ SyscallDesc("get_robust_list", ignoreFunc),
    /* 313 */ SyscallDesc("splice", unimplementedFunc),
    /* 314 */ SyscallDesc("sync_file_range", unimplementedFunc),
    /* 315 */ SyscallDesc("tee", unimplementedFunc),
    /* 316 */ SyscallDesc("vmsplice", unimplementedFunc),
    /* 317 */ SyscallDesc("move_pages", unimplementedFunc),
    /* 318 */ SyscallDesc("getcpu", unimplementedFunc),
    /* 319 */ SyscallDesc("epoll_pwait", unimplementedFunc),
    /* 320 */ SyscallDesc("utimensat", unimplementedFunc),
    /* 321 */ SyscallDesc("signalfd", unimplementedFunc),
    /* 322 */ SyscallDesc("timerfd", unimplementedFunc),
    /* 323 */ SyscallDesc("eventfd", unimplementedFunc)
};

I386LinuxProcess::I386LinuxProcess(ProcessParams * params, ObjectFile *objFile)
    : I386Process(params, objFile, syscallDescs32,
                  sizeof(syscallDescs32) / sizeof(SyscallDesc))
{}

void I386LinuxProcess::clone(ThreadContext *old_tc, ThreadContext *new_tc,
                             Process *process, RegVal flags)
{
    I386Process::clone(old_tc, new_tc, (I386Process*)process, flags);
}
