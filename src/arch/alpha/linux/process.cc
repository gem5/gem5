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
 *
 * Authors: Steve Reinhardt
 *          Ali Saidi
 */

#include "arch/alpha/linux/process.hh"

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/linux/linux.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/SyscallVerbose.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace AlphaISA;

namespace
{

class AlphaLinuxObjectFileLoader : public Process::Loader
{
  public:
    Process *
    load(ProcessParams *params, ObjectFile *obj_file) override
    {
        if (obj_file->getArch() != ObjectFile::Alpha)
            return nullptr;

        auto opsys = obj_file->getOpSys();

        if (opsys == ObjectFile::UnknownOpSys) {
            warn("Unknown operating system; assuming Linux.");
            opsys = ObjectFile::Linux;
        }

        if (opsys != ObjectFile::Linux)
            return nullptr;

        return new AlphaLinuxProcess(params, obj_file);
    }
};

AlphaLinuxObjectFileLoader loader;

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
    strcpy(name->machine, "alpha");

    name.copyOut(tc->getVirtProxy());
    return 0;
}

/// Target osf_getsysyinfo() handler.  Even though this call is
/// borrowed from Tru64, the subcases that get used appear to be
/// different in practice from those used by Tru64 processes.
static SyscallReturn
osf_getsysinfoFunc(SyscallDesc *desc, int callnum, ThreadContext *tc)
{
    int index = 0;
    auto process = tc->getProcessPtr();
    unsigned op = process->getSyscallArg(tc, index);
    Addr bufPtr = process->getSyscallArg(tc, index);
    // unsigned nbytes = process->getSyscallArg(tc, 2);

    switch (op) {

      case 45: { // GSI_IEEE_FP_CONTROL
          TypedBufferArg<uint64_t> fpcr(bufPtr);
          // I don't think this exactly matches the HW FPCR
          *fpcr = 0;
          fpcr.copyOut(tc->getVirtProxy());
          return 0;
      }

      default:
        cerr << "osf_getsysinfo: unknown op " << op << endl;
        abort();
        break;
    }

    return 1;
}

/// Target osf_setsysinfo() handler.
static SyscallReturn
osf_setsysinfoFunc(SyscallDesc *desc, int callnum, ThreadContext *tc)
{
    int index = 0;
    auto process = tc->getProcessPtr();
    unsigned op = process->getSyscallArg(tc, index);
    Addr bufPtr = process->getSyscallArg(tc, index);
    // unsigned nbytes = process->getSyscallArg(tc, 2);

    switch (op) {

      case 14: { // SSI_IEEE_FP_CONTROL
          TypedBufferArg<uint64_t> fpcr(bufPtr);
          // I don't think this exactly matches the HW FPCR
          fpcr.copyIn(tc->getVirtProxy());
          DPRINTFR(SyscallVerbose, "osf_setsysinfo(SSI_IEEE_FP_CONTROL): "
                   " setting FPCR to 0x%x\n", letoh(*(uint64_t*)fpcr));
          return 0;
      }

      default:
        cerr << "osf_setsysinfo: unknown op " << op << endl;
        abort();
        break;
    }

    return 1;
}


SyscallDesc AlphaLinuxProcess::syscallDescs[] = {
    /*  0 */ { "osf_syscall" },
    /*  1 */ { "exit", exitFunc },
    /*  2 */ { "fork" },
    /*  3 */ { "read", readFunc<AlphaLinux> },
    /*  4 */ { "write", writeFunc<AlphaLinux> },
    /*  5 */ { "osf_old_open" },
    /*  6 */ { "close", closeFunc },
    /*  7 */ { "osf_wait4" },
    /*  8 */ { "osf_old_creat" },
    /*  9 */ { "link" },
    /* 10 */ { "unlink", unlinkFunc },
    /* 11 */ { "osf_execve" },
    /* 12 */ { "chdir" },
    /* 13 */ { "fchdir" },
    /* 14 */ { "mknod" },
    /* 15 */ { "chmod", chmodFunc<AlphaLinux> },
    /* 16 */ { "chown", chownFunc },
    /* 17 */ { "brk", brkFunc },
    /* 18 */ { "osf_getfsstat" },
    /* 19 */ { "lseek", lseekFunc },
    /* 20 */ { "getxpid", getpidPseudoFunc },
    /* 21 */ { "osf_mount" },
    /* 22 */ { "umount" },
    /* 23 */ { "setuid", setuidFunc },
    /* 24 */ { "getxuid", getuidPseudoFunc },
    /* 25 */ { "exec_with_loader" },
    /* 26 */ { "osf_ptrace" },
    /* 27 */ { "osf_nrecvmsg" },
    /* 28 */ { "osf_nsendmsg" },
    /* 29 */ { "osf_nrecvfrom" },
    /* 30 */ { "osf_naccept" },
    /* 31 */ { "osf_ngetpeername" },
    /* 32 */ { "osf_ngetsockname" },
    /* 33 */ { "access" },
    /* 34 */ { "osf_chflags" },
    /* 35 */ { "osf_fchflags" },
    /* 36 */ { "sync" },
    /* 37 */ { "kill" },
    /* 38 */ { "osf_old_stat" },
    /* 39 */ { "setpgid" },
    /* 40 */ { "osf_old_lstat" },
    /* 41 */ { "dup", dupFunc },
    /* 42 */ { "pipe", pipePseudoFunc },
    /* 43 */ { "osf_set_program_attributes" },
    /* 44 */ { "osf_profil" },
    /* 45 */ { "open", openFunc<AlphaLinux> },
    /* 46 */ { "osf_old_sigaction" },
    /* 47 */ { "getxgid", getgidPseudoFunc },
    /* 48 */ { "osf_sigprocmask", ignoreFunc },
    /* 49 */ { "osf_getlogin" },
    /* 50 */ { "osf_setlogin" },
    /* 51 */ { "acct" },
    /* 52 */ { "sigpending" },
    /* 53 */ { "osf_classcntl" },
    /* 54 */ { "ioctl", ioctlFunc<AlphaLinux> },
    /* 55 */ { "osf_reboot" },
    /* 56 */ { "osf_revoke" },
    /* 57 */ { "symlink" },
    /* 58 */ { "readlink", readlinkFunc },
    /* 59 */ { "execve" },
    /* 60 */ { "umask", umaskFunc },
    /* 61 */ { "chroot" },
    /* 62 */ { "osf_old_fstat" },
    /* 63 */ { "getpgrp" },
    /* 64 */ { "getpagesize", getpagesizeFunc },
    /* 65 */ { "osf_mremap" },
    /* 66 */ { "vfork" },
    /* 67 */ { "stat", statFunc<AlphaLinux> },
    /* 68 */ { "lstat", lstatFunc<AlphaLinux> },
    /* 69 */ { "osf_sbrk" },
    /* 70 */ { "osf_sstk" },
    /* 71 */ { "mmap", mmapFunc<AlphaLinux> },
    /* 72 */ { "osf_old_vadvise" },
    /* 73 */ { "munmap", munmapFunc },
    /* 74 */ { "mprotect", ignoreFunc },
    /* 75 */ { "madvise" },
    /* 76 */ { "vhangup" },
    /* 77 */ { "osf_kmodcall" },
    /* 78 */ { "osf_mincore" },
    /* 79 */ { "getgroups" },
    /* 80 */ { "setgroups" },
    /* 81 */ { "osf_old_getpgrp" },
    /* 82 */ { "setpgrp" },
    /* 83 */ { "osf_setitimer" },
    /* 84 */ { "osf_old_wait" },
    /* 85 */ { "osf_table" },
    /* 86 */ { "osf_getitimer" },
    /* 87 */ { "gethostname", gethostnameFunc },
    /* 88 */ { "sethostname" },
    /* 89 */ { "getdtablesize" },
    /* 90 */ { "dup2" },
    /* 91 */ { "fstat", fstatFunc<AlphaLinux> },
    /* 92 */ { "fcntl", fcntlFunc },
    /* 93 */ { "osf_select" },
    /* 94 */ { "poll" },
    /* 95 */ { "fsync" },
    /* 96 */ { "setpriority" },
    /* 97 */ { "socket" },
    /* 98 */ { "connect" },
    /* 99 */ { "accept" },
    /* 100 */ { "getpriority" },
    /* 101 */ { "send" },
    /* 102 */ { "recv" },
    /* 103 */ { "sigreturn" },
    /* 104 */ { "bind" },
    /* 105 */ { "setsockopt" },
    /* 106 */ { "listen" },
    /* 107 */ { "osf_plock" },
    /* 108 */ { "osf_old_sigvec" },
    /* 109 */ { "osf_old_sigblock" },
    /* 110 */ { "osf_old_sigsetmask" },
    /* 111 */ { "sigsuspend" },
    /* 112 */ { "osf_sigstack", ignoreFunc },
    /* 113 */ { "recvmsg" },
    /* 114 */ { "sendmsg" },
    /* 115 */ { "osf_old_vtrace" },
    /* 116 */ { "osf_gettimeofday" },
    /* 117 */ { "osf_getrusage" },
    /* 118 */ { "getsockopt" },
    /* 119 */ { "numa_syscalls" },
    /* 120 */ { "readv" },
    /* 121 */ { "writev", writevFunc<AlphaLinux> },
    /* 122 */ { "osf_settimeofday" },
    /* 123 */ { "fchown", fchownFunc },
    /* 124 */ { "fchmod", fchmodFunc<AlphaLinux> },
    /* 125 */ { "recvfrom" },
    /* 126 */ { "setreuid" },
    /* 127 */ { "setregid" },
    /* 128 */ { "rename", renameFunc },
    /* 129 */ { "truncate", truncateFunc },
    /* 130 */ { "ftruncate", ftruncateFunc },
    /* 131 */ { "flock" },
    /* 132 */ { "setgid" },
    /* 133 */ { "sendto" },
    /* 134 */ { "shutdown" },
    /* 135 */ { "socketpair" },
    /* 136 */ { "mkdir", mkdirFunc },
    /* 137 */ { "rmdir" },
    /* 138 */ { "osf_utimes" },
    /* 139 */ { "osf_old_sigreturn" },
    /* 140 */ { "osf_adjtime" },
    /* 141 */ { "getpeername" },
    /* 142 */ { "osf_gethostid" },
    /* 143 */ { "osf_sethostid" },
    /* 144 */ { "getrlimit", getrlimitFunc<AlphaLinux> },
    /* 145 */ { "setrlimit", ignoreFunc },
    /* 146 */ { "osf_old_killpg" },
    /* 147 */ { "setsid" },
    /* 148 */ { "quotactl" },
    /* 149 */ { "osf_oldquota" },
    /* 150 */ { "getsockname" },
    /* 151 */ { "osf_pread" },
    /* 152 */ { "osf_pwrite" },
    /* 153 */ { "osf_pid_block" },
    /* 154 */ { "osf_pid_unblock" },
    /* 155 */ { "osf_signal_urti" },
    /* 156 */ { "sigaction", ignoreFunc },
    /* 157 */ { "osf_sigwaitprim" },
    /* 158 */ { "osf_nfssvc" },
    /* 159 */ { "osf_getdirentries" },
    /* 160 */ { "osf_statfs" },
    /* 161 */ { "osf_fstatfs" },
    /* 162 */ { "unknown #162" },
    /* 163 */ { "osf_async_daemon" },
    /* 164 */ { "osf_getfh" },
    /* 165 */ { "osf_getdomainname" },
    /* 166 */ { "setdomainname" },
    /* 167 */ { "unknown #167" },
    /* 168 */ { "unknown #168" },
    /* 169 */ { "osf_exportfs" },
    /* 170 */ { "unknown #170" },
    /* 171 */ { "unknown #171" },
    /* 172 */ { "unknown #172" },
    /* 173 */ { "unknown #173" },
    /* 174 */ { "unknown #174" },
    /* 175 */ { "unknown #175" },
    /* 176 */ { "unknown #176" },
    /* 177 */ { "unknown #177" },
    /* 178 */ { "unknown #178" },
    /* 179 */ { "unknown #179" },
    /* 180 */ { "unknown #180" },
    /* 181 */ { "osf_alt_plock" },
    /* 182 */ { "unknown #182" },
    /* 183 */ { "unknown #183" },
    /* 184 */ { "osf_getmnt" },
    /* 185 */ { "unknown #185" },
    /* 186 */ { "unknown #186" },
    /* 187 */ { "osf_alt_sigpending" },
    /* 188 */ { "osf_alt_setsid" },
    /* 189 */ { "unknown #189" },
    /* 190 */ { "unknown #190" },
    /* 191 */ { "unknown #191" },
    /* 192 */ { "unknown #192" },
    /* 193 */ { "unknown #193" },
    /* 194 */ { "unknown #194" },
    /* 195 */ { "unknown #195" },
    /* 196 */ { "unknown #196" },
    /* 197 */ { "unknown #197" },
    /* 198 */ { "unknown #198" },
    /* 199 */ { "osf_swapon" },
    /* 200 */ { "msgctl" },
    /* 201 */ { "msgget" },
    /* 202 */ { "msgrcv" },
    /* 203 */ { "msgsnd" },
    /* 204 */ { "semctl" },
    /* 205 */ { "semget" },
    /* 206 */ { "semop" },
    /* 207 */ { "osf_utsname" },
    /* 208 */ { "lchown" },
    /* 209 */ { "osf_shmat" },
    /* 210 */ { "shmctl" },
    /* 211 */ { "shmdt" },
    /* 212 */ { "shmget" },
    /* 213 */ { "osf_mvalid" },
    /* 214 */ { "osf_getaddressconf" },
    /* 215 */ { "osf_msleep" },
    /* 216 */ { "osf_mwakeup" },
    /* 217 */ { "msync" },
    /* 218 */ { "osf_signal" },
    /* 219 */ { "osf_utc_gettime" },
    /* 220 */ { "osf_utc_adjtime" },
    /* 221 */ { "unknown #221" },
    /* 222 */ { "osf_security" },
    /* 223 */ { "osf_kloadcall" },
    /* 224 */ { "unknown #224" },
    /* 225 */ { "unknown #225" },
    /* 226 */ { "unknown #226" },
    /* 227 */ { "unknown #227" },
    /* 228 */ { "unknown #228" },
    /* 229 */ { "unknown #229" },
    /* 230 */ { "unknown #230" },
    /* 231 */ { "unknown #231" },
    /* 232 */ { "unknown #232" },
    /* 233 */ { "getpgid" },
    /* 234 */ { "getsid" },
    /* 235 */ { "sigaltstack", ignoreFunc },
    /* 236 */ { "osf_waitid" },
    /* 237 */ { "osf_priocntlset" },
    /* 238 */ { "osf_sigsendset" },
    /* 239 */ { "osf_set_speculative" },
    /* 240 */ { "osf_msfs_syscall" },
    /* 241 */ { "osf_sysinfo" },
    /* 242 */ { "osf_uadmin" },
    /* 243 */ { "osf_fuser" },
    /* 244 */ { "osf_proplist_syscall" },
    /* 245 */ { "osf_ntp_adjtime" },
    /* 246 */ { "osf_ntp_gettime" },
    /* 247 */ { "osf_pathconf" },
    /* 248 */ { "osf_fpathconf" },
    /* 249 */ { "unknown #249" },
    /* 250 */ { "osf_uswitch" },
    /* 251 */ { "osf_usleep_thread" },
    /* 252 */ { "osf_audcntl" },
    /* 253 */ { "osf_audgen" },
    /* 254 */ { "sysfs" },
    /* 255 */ { "osf_subsys_info" },
    /* 256 */ { "osf_getsysinfo", osf_getsysinfoFunc },
    /* 257 */ { "osf_setsysinfo", osf_setsysinfoFunc },
    /* 258 */ { "osf_afs_syscall" },
    /* 259 */ { "osf_swapctl" },
    /* 260 */ { "osf_memcntl" },
    /* 261 */ { "osf_fdatasync" },
    /* 262 */ { "unknown #262" },
    /* 263 */ { "unknown #263" },
    /* 264 */ { "unknown #264" },
    /* 265 */ { "unknown #265" },
    /* 266 */ { "unknown #266" },
    /* 267 */ { "unknown #267" },
    /* 268 */ { "unknown #268" },
    /* 269 */ { "unknown #269" },
    /* 270 */ { "unknown #270" },
    /* 271 */ { "unknown #271" },
    /* 272 */ { "unknown #272" },
    /* 273 */ { "unknown #273" },
    /* 274 */ { "unknown #274" },
    /* 275 */ { "unknown #275" },
    /* 276 */ { "unknown #276" },
    /* 277 */ { "unknown #277" },
    /* 278 */ { "unknown #278" },
    /* 279 */ { "unknown #279" },
    /* 280 */ { "unknown #280" },
    /* 281 */ { "unknown #281" },
    /* 282 */ { "unknown #282" },
    /* 283 */ { "unknown #283" },
    /* 284 */ { "unknown #284" },
    /* 285 */ { "unknown #285" },
    /* 286 */ { "unknown #286" },
    /* 287 */ { "unknown #287" },
    /* 288 */ { "unknown #288" },
    /* 289 */ { "unknown #289" },
    /* 290 */ { "unknown #290" },
    /* 291 */ { "unknown #291" },
    /* 292 */ { "unknown #292" },
    /* 293 */ { "unknown #293" },
    /* 294 */ { "unknown #294" },
    /* 295 */ { "unknown #295" },
    /* 296 */ { "unknown #296" },
    /* 297 */ { "unknown #297" },
    /* 298 */ { "unknown #298" },
    /* 299 */ { "unknown #299" },
/*
 * Linux-specific system calls begin at 300
 */
    /* 300 */ { "bdflush" },
    /* 301 */ { "sethae" },
    /* 302 */ { "mount" },
    /* 303 */ { "old_adjtimex" },
    /* 304 */ { "swapoff" },
    /* 305 */ { "getdents" },
    /* 306 */ { "create_module" },
    /* 307 */ { "init_module" },
    /* 308 */ { "delete_module" },
    /* 309 */ { "get_kernel_syms" },
    /* 310 */ { "syslog" },
    /* 311 */ { "reboot" },
    /* 312 */ { "clone", cloneFunc<AlphaLinux> },
    /* 313 */ { "uselib" },
    /* 314 */ { "mlock" },
    /* 315 */ { "munlock" },
    /* 316 */ { "mlockall" },
    /* 317 */ { "munlockall" },
    /* 318 */ { "sysinfo", sysinfoFunc<AlphaLinux> },
    /* 319 */ { "_sysctl" },
    /* 320 */ { "was sys_idle" },
    /* 321 */ { "oldumount" },
    /* 322 */ { "swapon" },
    /* 323 */ { "times", ignoreFunc },
    /* 324 */ { "personality" },
    /* 325 */ { "setfsuid" },
    /* 326 */ { "setfsgid" },
    /* 327 */ { "ustat" },
    /* 328 */ { "statfs" },
    /* 329 */ { "fstatfs" },
    /* 330 */ { "sched_setparam" },
    /* 331 */ { "sched_getparam" },
    /* 332 */ { "sched_setscheduler" },
    /* 333 */ { "sched_getscheduler" },
    /* 334 */ { "sched_yield" },
    /* 335 */ { "sched_get_priority_max" },
    /* 336 */ { "sched_get_priority_min" },
    /* 337 */ { "sched_rr_get_interval" },
    /* 338 */ { "afs_syscall" },
    /* 339 */ { "uname", unameFunc },
    /* 340 */ { "nanosleep" },
    /* 341 */ { "mremap", mremapFunc<AlphaLinux> },
    /* 342 */ { "nfsservctl" },
    /* 343 */ { "setresuid" },
    /* 344 */ { "getresuid" },
    /* 345 */ { "pciconfig_read" },
    /* 346 */ { "pciconfig_write" },
    /* 347 */ { "query_module" },
    /* 348 */ { "prctl" },
    /* 349 */ { "pread" },
    /* 350 */ { "pwrite" },
    /* 351 */ { "rt_sigreturn" },
    /* 352 */ { "rt_sigaction", ignoreFunc },
    /* 353 */ { "rt_sigprocmask" },
    /* 354 */ { "rt_sigpending" },
    /* 355 */ { "rt_sigtimedwait" },
    /* 356 */ { "rt_sigqueueinfo" },
    /* 357 */ { "rt_sigsuspend" },
    /* 358 */ { "select" },
    /* 359 */ { "gettimeofday", gettimeofdayFunc<AlphaLinux> },
    /* 360 */ { "settimeofday" },
    /* 361 */ { "getitimer" },
    /* 362 */ { "setitimer" },
    /* 363 */ { "utimes", utimesFunc<AlphaLinux> },
    /* 364 */ { "getrusage", getrusageFunc<AlphaLinux> },
    /* 365 */ { "wait4" },
    /* 366 */ { "adjtimex" },
    /* 367 */ { "getcwd", getcwdFunc },
    /* 368 */ { "capget" },
    /* 369 */ { "capset" },
    /* 370 */ { "sendfile" },
    /* 371 */ { "setresgid" },
    /* 372 */ { "getresgid" },
    /* 373 */ { "dipc" },
    /* 374 */ { "pivot_root" },
    /* 375 */ { "mincore" },
    /* 376 */ { "pciconfig_iobase" },
    /* 377 */ { "getdents64" },
    /* 378 */ { "gettid" },
    /* 379 */ { "readahead" },
    /* 380 */ { "security" },
    /* 381 */ { "tkill" },
    /* 382 */ { "setxattr" },
    /* 383 */ { "lsetxattr" },
    /* 384 */ { "fsetxattr" },
    /* 385 */ { "getxattr" },
    /* 386 */ { "lgetxattr" },
    /* 387 */ { "fgetxattr" },
    /* 388 */ { "listxattr" },
    /* 389 */ { "llistxattr" },
    /* 390 */ { "flistxattr" },
    /* 391 */ { "removexattr" },
    /* 392 */ { "lremovexattr" },
    /* 393 */ { "fremovexattr" },
    /* 394 */ { "futex" },
    /* 395 */ { "sched_setaffinity" },
    /* 396 */ { "sched_getaffinity" },
    /* 397 */ { "tuxcall" },
    /* 398 */ { "io_setup" },
    /* 399 */ { "io_destroy" },
    /* 400 */ { "io_getevents" },
    /* 401 */ { "io_submit" },
    /* 402 */ { "io_cancel" },
    /* 403 */ { "unknown #403" },
    /* 404 */ { "unknown #404" },
    /* 405 */ { "exit_group", exitGroupFunc }, // exit all threads...
    /* 406 */ { "lookup_dcookie" },
    /* 407 */ { "sys_epoll_create" },
    /* 408 */ { "sys_epoll_ctl" },
    /* 409 */ { "sys_epoll_wait" },
    /* 410 */ { "remap_file_pages" },
    /* 411 */ { "set_tid_address" },
    /* 412 */ { "restart_syscall" },
    /* 413 */ { "fadvise64" },
    /* 414 */ { "timer_create" },
    /* 415 */ { "timer_settime" },
    /* 416 */ { "timer_gettime" },
    /* 417 */ { "timer_getoverrun" },
    /* 418 */ { "timer_delete" },
    /* 419 */ { "clock_settime" },
    /* 420 */ { "clock_gettime" },
    /* 421 */ { "clock_getres" },
    /* 422 */ { "clock_nanosleep" },
    /* 423 */ { "semtimedop" },
    /* 424 */ { "tgkill" },
    /* 425 */ { "stat64", stat64Func<AlphaLinux> },
    /* 426 */ { "lstat64", lstat64Func<AlphaLinux> },
    /* 427 */ { "fstat64", fstat64Func<AlphaLinux> },
    /* 428 */ { "vserver" },
    /* 429 */ { "mbind" },
    /* 430 */ { "get_mempolicy" },
    /* 431 */ { "set_mempolicy" },
    /* 432 */ { "mq_open" },
    /* 433 */ { "mq_unlink" },
    /* 434 */ { "mq_timedsend" },
    /* 435 */ { "mq_timedreceive" },
    /* 436 */ { "mq_notify" },
    /* 437 */ { "mq_getsetattr" },
    /* 438 */ { "waitid" },
    /* 439 */ { "add_key" },
    /* 440 */ { "request_key" },
    /* 441 */ { "keyctl" }
};

AlphaLinuxProcess::AlphaLinuxProcess(ProcessParams * params,
                                     ObjectFile *objFile)
    : AlphaProcess(params, objFile),
     Num_Syscall_Descs(sizeof(syscallDescs) / sizeof(SyscallDesc))
{
    //init_regs->intRegFile[0] = 0;
}



SyscallDesc*
AlphaLinuxProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum >= Num_Syscall_Descs)
        return NULL;
    return &syscallDescs[callnum];
}

void
AlphaLinuxProcess::syscall(ThreadContext *tc, Fault *fault)
{
    doSyscall(tc->readIntReg(0), tc, fault);
}

