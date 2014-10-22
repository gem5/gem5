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

#include "arch/alpha/linux/linux.hh"
#include "arch/alpha/linux/process.hh"
#include "arch/alpha/isa_traits.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/SyscallVerbose.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace AlphaISA;

/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
          ThreadContext *tc)
{
    int index = 0;
    TypedBufferArg<Linux::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, "3.0.0");
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "alpha");

    name.copyOut(tc->getMemProxy());
    return 0;
}

/// Target osf_getsysyinfo() handler.  Even though this call is
/// borrowed from Tru64, the subcases that get used appear to be
/// different in practice from those used by Tru64 processes.
static SyscallReturn
osf_getsysinfoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
                   ThreadContext *tc)
{
    int index = 0;
    unsigned op = process->getSyscallArg(tc, index);
    Addr bufPtr = process->getSyscallArg(tc, index);
    // unsigned nbytes = process->getSyscallArg(tc, 2);

    switch (op) {

      case 45: { // GSI_IEEE_FP_CONTROL
          TypedBufferArg<uint64_t> fpcr(bufPtr);
          // I don't think this exactly matches the HW FPCR
          *fpcr = 0;
          fpcr.copyOut(tc->getMemProxy());
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
osf_setsysinfoFunc(SyscallDesc *desc, int callnum, LiveProcess *process,
                   ThreadContext *tc)
{
    int index = 0;
    unsigned op = process->getSyscallArg(tc, index);
    Addr bufPtr = process->getSyscallArg(tc, index);
    // unsigned nbytes = process->getSyscallArg(tc, 2);

    switch (op) {

      case 14: { // SSI_IEEE_FP_CONTROL
          TypedBufferArg<uint64_t> fpcr(bufPtr);
          // I don't think this exactly matches the HW FPCR
          fpcr.copyIn(tc->getMemProxy());
          DPRINTFR(SyscallVerbose, "osf_setsysinfo(SSI_IEEE_FP_CONTROL): "
                   " setting FPCR to 0x%x\n", gtoh(*(uint64_t*)fpcr));
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
    /*  0 */ SyscallDesc("osf_syscall", unimplementedFunc),
    /*  1 */ SyscallDesc("exit", exitFunc),
    /*  2 */ SyscallDesc("fork", unimplementedFunc),
    /*  3 */ SyscallDesc("read", readFunc),
    /*  4 */ SyscallDesc("write", writeFunc),
    /*  5 */ SyscallDesc("osf_old_open", unimplementedFunc),
    /*  6 */ SyscallDesc("close", closeFunc),
    /*  7 */ SyscallDesc("osf_wait4", unimplementedFunc),
    /*  8 */ SyscallDesc("osf_old_creat", unimplementedFunc),
    /*  9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unlinkFunc),
    /* 11 */ SyscallDesc("osf_execve", unimplementedFunc),
    /* 12 */ SyscallDesc("chdir", unimplementedFunc),
    /* 13 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 14 */ SyscallDesc("mknod", unimplementedFunc),
    /* 15 */ SyscallDesc("chmod", chmodFunc<AlphaLinux>),
    /* 16 */ SyscallDesc("chown", chownFunc),
    /* 17 */ SyscallDesc("brk", brkFunc),
    /* 18 */ SyscallDesc("osf_getfsstat", unimplementedFunc),
    /* 19 */ SyscallDesc("lseek", lseekFunc),
    /* 20 */ SyscallDesc("getxpid", getpidPseudoFunc),
    /* 21 */ SyscallDesc("osf_mount", unimplementedFunc),
    /* 22 */ SyscallDesc("umount", unimplementedFunc),
    /* 23 */ SyscallDesc("setuid", setuidFunc),
    /* 24 */ SyscallDesc("getxuid", getuidPseudoFunc),
    /* 25 */ SyscallDesc("exec_with_loader", unimplementedFunc),
    /* 26 */ SyscallDesc("osf_ptrace", unimplementedFunc),
    /* 27 */ SyscallDesc("osf_nrecvmsg", unimplementedFunc),
    /* 28 */ SyscallDesc("osf_nsendmsg", unimplementedFunc),
    /* 29 */ SyscallDesc("osf_nrecvfrom", unimplementedFunc),
    /* 30 */ SyscallDesc("osf_naccept", unimplementedFunc),
    /* 31 */ SyscallDesc("osf_ngetpeername", unimplementedFunc),
    /* 32 */ SyscallDesc("osf_ngetsockname", unimplementedFunc),
    /* 33 */ SyscallDesc("access", unimplementedFunc),
    /* 34 */ SyscallDesc("osf_chflags", unimplementedFunc),
    /* 35 */ SyscallDesc("osf_fchflags", unimplementedFunc),
    /* 36 */ SyscallDesc("sync", unimplementedFunc),
    /* 37 */ SyscallDesc("kill", unimplementedFunc),
    /* 38 */ SyscallDesc("osf_old_stat", unimplementedFunc),
    /* 39 */ SyscallDesc("setpgid", unimplementedFunc),
    /* 40 */ SyscallDesc("osf_old_lstat", unimplementedFunc),
    /* 41 */ SyscallDesc("dup", dupFunc),
    /* 42 */ SyscallDesc("pipe", pipePseudoFunc),
    /* 43 */ SyscallDesc("osf_set_program_attributes", unimplementedFunc),
    /* 44 */ SyscallDesc("osf_profil", unimplementedFunc),
    /* 45 */ SyscallDesc("open", openFunc<AlphaLinux>),
    /* 46 */ SyscallDesc("osf_old_sigaction", unimplementedFunc),
    /* 47 */ SyscallDesc("getxgid", getgidPseudoFunc),
    /* 48 */ SyscallDesc("osf_sigprocmask", ignoreFunc),
    /* 49 */ SyscallDesc("osf_getlogin", unimplementedFunc),
    /* 50 */ SyscallDesc("osf_setlogin", unimplementedFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 53 */ SyscallDesc("osf_classcntl", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", ioctlFunc<AlphaLinux>),
    /* 55 */ SyscallDesc("osf_reboot", unimplementedFunc),
    /* 56 */ SyscallDesc("osf_revoke", unimplementedFunc),
    /* 57 */ SyscallDesc("symlink", unimplementedFunc),
    /* 58 */ SyscallDesc("readlink", readlinkFunc),
    /* 59 */ SyscallDesc("execve", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", umaskFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("osf_old_fstat", unimplementedFunc),
    /* 63 */ SyscallDesc("getpgrp", unimplementedFunc),
    /* 64 */ SyscallDesc("getpagesize", getpagesizeFunc),
    /* 65 */ SyscallDesc("osf_mremap", unimplementedFunc),
    /* 66 */ SyscallDesc("vfork", unimplementedFunc),
    /* 67 */ SyscallDesc("stat", statFunc<AlphaLinux>),
    /* 68 */ SyscallDesc("lstat", lstatFunc<AlphaLinux>),
    /* 69 */ SyscallDesc("osf_sbrk", unimplementedFunc),
    /* 70 */ SyscallDesc("osf_sstk", unimplementedFunc),
    /* 71 */ SyscallDesc("mmap", mmapFunc<AlphaLinux>),
    /* 72 */ SyscallDesc("osf_old_vadvise", unimplementedFunc),
    /* 73 */ SyscallDesc("munmap", munmapFunc),
    /* 74 */ SyscallDesc("mprotect", ignoreFunc),
    /* 75 */ SyscallDesc("madvise", unimplementedFunc),
    /* 76 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 77 */ SyscallDesc("osf_kmodcall", unimplementedFunc),
    /* 78 */ SyscallDesc("osf_mincore", unimplementedFunc),
    /* 79 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 80 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 81 */ SyscallDesc("osf_old_getpgrp", unimplementedFunc),
    /* 82 */ SyscallDesc("setpgrp", unimplementedFunc),
    /* 83 */ SyscallDesc("osf_setitimer", unimplementedFunc),
    /* 84 */ SyscallDesc("osf_old_wait", unimplementedFunc),
    /* 85 */ SyscallDesc("osf_table", unimplementedFunc),
    /* 86 */ SyscallDesc("osf_getitimer", unimplementedFunc),
    /* 87 */ SyscallDesc("gethostname", gethostnameFunc),
    /* 88 */ SyscallDesc("sethostname", unimplementedFunc),
    /* 89 */ SyscallDesc("getdtablesize", unimplementedFunc),
    /* 90 */ SyscallDesc("dup2", unimplementedFunc),
    /* 91 */ SyscallDesc("fstat", fstatFunc<AlphaLinux>),
    /* 92 */ SyscallDesc("fcntl", fcntlFunc),
    /* 93 */ SyscallDesc("osf_select", unimplementedFunc),
    /* 94 */ SyscallDesc("poll", unimplementedFunc),
    /* 95 */ SyscallDesc("fsync", unimplementedFunc),
    /* 96 */ SyscallDesc("setpriority", unimplementedFunc),
    /* 97 */ SyscallDesc("socket", unimplementedFunc),
    /* 98 */ SyscallDesc("connect", unimplementedFunc),
    /* 99 */ SyscallDesc("accept", unimplementedFunc),
    /* 100 */ SyscallDesc("getpriority", unimplementedFunc),
    /* 101 */ SyscallDesc("send", unimplementedFunc),
    /* 102 */ SyscallDesc("recv", unimplementedFunc),
    /* 103 */ SyscallDesc("sigreturn", unimplementedFunc),
    /* 104 */ SyscallDesc("bind", unimplementedFunc),
    /* 105 */ SyscallDesc("setsockopt", unimplementedFunc),
    /* 106 */ SyscallDesc("listen", unimplementedFunc),
    /* 107 */ SyscallDesc("osf_plock", unimplementedFunc),
    /* 108 */ SyscallDesc("osf_old_sigvec", unimplementedFunc),
    /* 109 */ SyscallDesc("osf_old_sigblock", unimplementedFunc),
    /* 110 */ SyscallDesc("osf_old_sigsetmask", unimplementedFunc),
    /* 111 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 112 */ SyscallDesc("osf_sigstack", ignoreFunc),
    /* 113 */ SyscallDesc("recvmsg", unimplementedFunc),
    /* 114 */ SyscallDesc("sendmsg", unimplementedFunc),
    /* 115 */ SyscallDesc("osf_old_vtrace", unimplementedFunc),
    /* 116 */ SyscallDesc("osf_gettimeofday", unimplementedFunc),
    /* 117 */ SyscallDesc("osf_getrusage", unimplementedFunc),
    /* 118 */ SyscallDesc("getsockopt", unimplementedFunc),
    /* 119 */ SyscallDesc("numa_syscalls", unimplementedFunc),
    /* 120 */ SyscallDesc("readv", unimplementedFunc),
    /* 121 */ SyscallDesc("writev", writevFunc<AlphaLinux>),
    /* 122 */ SyscallDesc("osf_settimeofday", unimplementedFunc),
    /* 123 */ SyscallDesc("fchown", fchownFunc),
    /* 124 */ SyscallDesc("fchmod", fchmodFunc<AlphaLinux>),
    /* 125 */ SyscallDesc("recvfrom", unimplementedFunc),
    /* 126 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 127 */ SyscallDesc("setregid", unimplementedFunc),
    /* 128 */ SyscallDesc("rename", renameFunc),
    /* 129 */ SyscallDesc("truncate", truncateFunc),
    /* 130 */ SyscallDesc("ftruncate", ftruncateFunc),
    /* 131 */ SyscallDesc("flock", unimplementedFunc),
    /* 132 */ SyscallDesc("setgid", unimplementedFunc),
    /* 133 */ SyscallDesc("sendto", unimplementedFunc),
    /* 134 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 135 */ SyscallDesc("socketpair", unimplementedFunc),
    /* 136 */ SyscallDesc("mkdir", mkdirFunc),
    /* 137 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 138 */ SyscallDesc("osf_utimes", unimplementedFunc),
    /* 139 */ SyscallDesc("osf_old_sigreturn", unimplementedFunc),
    /* 140 */ SyscallDesc("osf_adjtime", unimplementedFunc),
    /* 141 */ SyscallDesc("getpeername", unimplementedFunc),
    /* 142 */ SyscallDesc("osf_gethostid", unimplementedFunc),
    /* 143 */ SyscallDesc("osf_sethostid", unimplementedFunc),
    /* 144 */ SyscallDesc("getrlimit", getrlimitFunc<AlphaLinux>),
    /* 145 */ SyscallDesc("setrlimit", ignoreFunc),
    /* 146 */ SyscallDesc("osf_old_killpg", unimplementedFunc),
    /* 147 */ SyscallDesc("setsid", unimplementedFunc),
    /* 148 */ SyscallDesc("quotactl", unimplementedFunc),
    /* 149 */ SyscallDesc("osf_oldquota", unimplementedFunc),
    /* 150 */ SyscallDesc("getsockname", unimplementedFunc),
    /* 151 */ SyscallDesc("osf_pread", unimplementedFunc),
    /* 152 */ SyscallDesc("osf_pwrite", unimplementedFunc),
    /* 153 */ SyscallDesc("osf_pid_block", unimplementedFunc),
    /* 154 */ SyscallDesc("osf_pid_unblock", unimplementedFunc),
    /* 155 */ SyscallDesc("osf_signal_urti", unimplementedFunc),
    /* 156 */ SyscallDesc("sigaction", ignoreFunc),
    /* 157 */ SyscallDesc("osf_sigwaitprim", unimplementedFunc),
    /* 158 */ SyscallDesc("osf_nfssvc", unimplementedFunc),
    /* 159 */ SyscallDesc("osf_getdirentries", unimplementedFunc),
    /* 160 */ SyscallDesc("osf_statfs", unimplementedFunc),
    /* 161 */ SyscallDesc("osf_fstatfs", unimplementedFunc),
    /* 162 */ SyscallDesc("unknown #162", unimplementedFunc),
    /* 163 */ SyscallDesc("osf_async_daemon", unimplementedFunc),
    /* 164 */ SyscallDesc("osf_getfh", unimplementedFunc),
    /* 165 */ SyscallDesc("osf_getdomainname", unimplementedFunc),
    /* 166 */ SyscallDesc("setdomainname", unimplementedFunc),
    /* 167 */ SyscallDesc("unknown #167", unimplementedFunc),
    /* 168 */ SyscallDesc("unknown #168", unimplementedFunc),
    /* 169 */ SyscallDesc("osf_exportfs", unimplementedFunc),
    /* 170 */ SyscallDesc("unknown #170", unimplementedFunc),
    /* 171 */ SyscallDesc("unknown #171", unimplementedFunc),
    /* 172 */ SyscallDesc("unknown #172", unimplementedFunc),
    /* 173 */ SyscallDesc("unknown #173", unimplementedFunc),
    /* 174 */ SyscallDesc("unknown #174", unimplementedFunc),
    /* 175 */ SyscallDesc("unknown #175", unimplementedFunc),
    /* 176 */ SyscallDesc("unknown #176", unimplementedFunc),
    /* 177 */ SyscallDesc("unknown #177", unimplementedFunc),
    /* 178 */ SyscallDesc("unknown #178", unimplementedFunc),
    /* 179 */ SyscallDesc("unknown #179", unimplementedFunc),
    /* 180 */ SyscallDesc("unknown #180", unimplementedFunc),
    /* 181 */ SyscallDesc("osf_alt_plock", unimplementedFunc),
    /* 182 */ SyscallDesc("unknown #182", unimplementedFunc),
    /* 183 */ SyscallDesc("unknown #183", unimplementedFunc),
    /* 184 */ SyscallDesc("osf_getmnt", unimplementedFunc),
    /* 185 */ SyscallDesc("unknown #185", unimplementedFunc),
    /* 186 */ SyscallDesc("unknown #186", unimplementedFunc),
    /* 187 */ SyscallDesc("osf_alt_sigpending", unimplementedFunc),
    /* 188 */ SyscallDesc("osf_alt_setsid", unimplementedFunc),
    /* 189 */ SyscallDesc("unknown #189", unimplementedFunc),
    /* 190 */ SyscallDesc("unknown #190", unimplementedFunc),
    /* 191 */ SyscallDesc("unknown #191", unimplementedFunc),
    /* 192 */ SyscallDesc("unknown #192", unimplementedFunc),
    /* 193 */ SyscallDesc("unknown #193", unimplementedFunc),
    /* 194 */ SyscallDesc("unknown #194", unimplementedFunc),
    /* 195 */ SyscallDesc("unknown #195", unimplementedFunc),
    /* 196 */ SyscallDesc("unknown #196", unimplementedFunc),
    /* 197 */ SyscallDesc("unknown #197", unimplementedFunc),
    /* 198 */ SyscallDesc("unknown #198", unimplementedFunc),
    /* 199 */ SyscallDesc("osf_swapon", unimplementedFunc),
    /* 200 */ SyscallDesc("msgctl", unimplementedFunc),
    /* 201 */ SyscallDesc("msgget", unimplementedFunc),
    /* 202 */ SyscallDesc("msgrcv", unimplementedFunc),
    /* 203 */ SyscallDesc("msgsnd", unimplementedFunc),
    /* 204 */ SyscallDesc("semctl", unimplementedFunc),
    /* 205 */ SyscallDesc("semget", unimplementedFunc),
    /* 206 */ SyscallDesc("semop", unimplementedFunc),
    /* 207 */ SyscallDesc("osf_utsname", unimplementedFunc),
    /* 208 */ SyscallDesc("lchown", unimplementedFunc),
    /* 209 */ SyscallDesc("osf_shmat", unimplementedFunc),
    /* 210 */ SyscallDesc("shmctl", unimplementedFunc),
    /* 211 */ SyscallDesc("shmdt", unimplementedFunc),
    /* 212 */ SyscallDesc("shmget", unimplementedFunc),
    /* 213 */ SyscallDesc("osf_mvalid", unimplementedFunc),
    /* 214 */ SyscallDesc("osf_getaddressconf", unimplementedFunc),
    /* 215 */ SyscallDesc("osf_msleep", unimplementedFunc),
    /* 216 */ SyscallDesc("osf_mwakeup", unimplementedFunc),
    /* 217 */ SyscallDesc("msync", unimplementedFunc),
    /* 218 */ SyscallDesc("osf_signal", unimplementedFunc),
    /* 219 */ SyscallDesc("osf_utc_gettime", unimplementedFunc),
    /* 220 */ SyscallDesc("osf_utc_adjtime", unimplementedFunc),
    /* 221 */ SyscallDesc("unknown #221", unimplementedFunc),
    /* 222 */ SyscallDesc("osf_security", unimplementedFunc),
    /* 223 */ SyscallDesc("osf_kloadcall", unimplementedFunc),
    /* 224 */ SyscallDesc("unknown #224", unimplementedFunc),
    /* 225 */ SyscallDesc("unknown #225", unimplementedFunc),
    /* 226 */ SyscallDesc("unknown #226", unimplementedFunc),
    /* 227 */ SyscallDesc("unknown #227", unimplementedFunc),
    /* 228 */ SyscallDesc("unknown #228", unimplementedFunc),
    /* 229 */ SyscallDesc("unknown #229", unimplementedFunc),
    /* 230 */ SyscallDesc("unknown #230", unimplementedFunc),
    /* 231 */ SyscallDesc("unknown #231", unimplementedFunc),
    /* 232 */ SyscallDesc("unknown #232", unimplementedFunc),
    /* 233 */ SyscallDesc("getpgid", unimplementedFunc),
    /* 234 */ SyscallDesc("getsid", unimplementedFunc),
    /* 235 */ SyscallDesc("sigaltstack", ignoreFunc),
    /* 236 */ SyscallDesc("osf_waitid", unimplementedFunc),
    /* 237 */ SyscallDesc("osf_priocntlset", unimplementedFunc),
    /* 238 */ SyscallDesc("osf_sigsendset", unimplementedFunc),
    /* 239 */ SyscallDesc("osf_set_speculative", unimplementedFunc),
    /* 240 */ SyscallDesc("osf_msfs_syscall", unimplementedFunc),
    /* 241 */ SyscallDesc("osf_sysinfo", unimplementedFunc),
    /* 242 */ SyscallDesc("osf_uadmin", unimplementedFunc),
    /* 243 */ SyscallDesc("osf_fuser", unimplementedFunc),
    /* 244 */ SyscallDesc("osf_proplist_syscall", unimplementedFunc),
    /* 245 */ SyscallDesc("osf_ntp_adjtime", unimplementedFunc),
    /* 246 */ SyscallDesc("osf_ntp_gettime", unimplementedFunc),
    /* 247 */ SyscallDesc("osf_pathconf", unimplementedFunc),
    /* 248 */ SyscallDesc("osf_fpathconf", unimplementedFunc),
    /* 249 */ SyscallDesc("unknown #249", unimplementedFunc),
    /* 250 */ SyscallDesc("osf_uswitch", unimplementedFunc),
    /* 251 */ SyscallDesc("osf_usleep_thread", unimplementedFunc),
    /* 252 */ SyscallDesc("osf_audcntl", unimplementedFunc),
    /* 253 */ SyscallDesc("osf_audgen", unimplementedFunc),
    /* 254 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 255 */ SyscallDesc("osf_subsys_info", unimplementedFunc),
    /* 256 */ SyscallDesc("osf_getsysinfo", osf_getsysinfoFunc),
    /* 257 */ SyscallDesc("osf_setsysinfo", osf_setsysinfoFunc),
    /* 258 */ SyscallDesc("osf_afs_syscall", unimplementedFunc),
    /* 259 */ SyscallDesc("osf_swapctl", unimplementedFunc),
    /* 260 */ SyscallDesc("osf_memcntl", unimplementedFunc),
    /* 261 */ SyscallDesc("osf_fdatasync", unimplementedFunc),
    /* 262 */ SyscallDesc("unknown #262", unimplementedFunc),
    /* 263 */ SyscallDesc("unknown #263", unimplementedFunc),
    /* 264 */ SyscallDesc("unknown #264", unimplementedFunc),
    /* 265 */ SyscallDesc("unknown #265", unimplementedFunc),
    /* 266 */ SyscallDesc("unknown #266", unimplementedFunc),
    /* 267 */ SyscallDesc("unknown #267", unimplementedFunc),
    /* 268 */ SyscallDesc("unknown #268", unimplementedFunc),
    /* 269 */ SyscallDesc("unknown #269", unimplementedFunc),
    /* 270 */ SyscallDesc("unknown #270", unimplementedFunc),
    /* 271 */ SyscallDesc("unknown #271", unimplementedFunc),
    /* 272 */ SyscallDesc("unknown #272", unimplementedFunc),
    /* 273 */ SyscallDesc("unknown #273", unimplementedFunc),
    /* 274 */ SyscallDesc("unknown #274", unimplementedFunc),
    /* 275 */ SyscallDesc("unknown #275", unimplementedFunc),
    /* 276 */ SyscallDesc("unknown #276", unimplementedFunc),
    /* 277 */ SyscallDesc("unknown #277", unimplementedFunc),
    /* 278 */ SyscallDesc("unknown #278", unimplementedFunc),
    /* 279 */ SyscallDesc("unknown #279", unimplementedFunc),
    /* 280 */ SyscallDesc("unknown #280", unimplementedFunc),
    /* 281 */ SyscallDesc("unknown #281", unimplementedFunc),
    /* 282 */ SyscallDesc("unknown #282", unimplementedFunc),
    /* 283 */ SyscallDesc("unknown #283", unimplementedFunc),
    /* 284 */ SyscallDesc("unknown #284", unimplementedFunc),
    /* 285 */ SyscallDesc("unknown #285", unimplementedFunc),
    /* 286 */ SyscallDesc("unknown #286", unimplementedFunc),
    /* 287 */ SyscallDesc("unknown #287", unimplementedFunc),
    /* 288 */ SyscallDesc("unknown #288", unimplementedFunc),
    /* 289 */ SyscallDesc("unknown #289", unimplementedFunc),
    /* 290 */ SyscallDesc("unknown #290", unimplementedFunc),
    /* 291 */ SyscallDesc("unknown #291", unimplementedFunc),
    /* 292 */ SyscallDesc("unknown #292", unimplementedFunc),
    /* 293 */ SyscallDesc("unknown #293", unimplementedFunc),
    /* 294 */ SyscallDesc("unknown #294", unimplementedFunc),
    /* 295 */ SyscallDesc("unknown #295", unimplementedFunc),
    /* 296 */ SyscallDesc("unknown #296", unimplementedFunc),
    /* 297 */ SyscallDesc("unknown #297", unimplementedFunc),
    /* 298 */ SyscallDesc("unknown #298", unimplementedFunc),
    /* 299 */ SyscallDesc("unknown #299", unimplementedFunc),
/*
 * Linux-specific system calls begin at 300
 */
    /* 300 */ SyscallDesc("bdflush", unimplementedFunc),
    /* 301 */ SyscallDesc("sethae", unimplementedFunc),
    /* 302 */ SyscallDesc("mount", unimplementedFunc),
    /* 303 */ SyscallDesc("old_adjtimex", unimplementedFunc),
    /* 304 */ SyscallDesc("swapoff", unimplementedFunc),
    /* 305 */ SyscallDesc("getdents", unimplementedFunc),
    /* 306 */ SyscallDesc("create_module", unimplementedFunc),
    /* 307 */ SyscallDesc("init_module", unimplementedFunc),
    /* 308 */ SyscallDesc("delete_module", unimplementedFunc),
    /* 309 */ SyscallDesc("get_kernel_syms", unimplementedFunc),
    /* 310 */ SyscallDesc("syslog", unimplementedFunc),
    /* 311 */ SyscallDesc("reboot", unimplementedFunc),
    /* 312 */ SyscallDesc("clone", cloneFunc),
    /* 313 */ SyscallDesc("uselib", unimplementedFunc),
    /* 314 */ SyscallDesc("mlock", unimplementedFunc),
    /* 315 */ SyscallDesc("munlock", unimplementedFunc),
    /* 316 */ SyscallDesc("mlockall", unimplementedFunc),
    /* 317 */ SyscallDesc("munlockall", unimplementedFunc),
    /* 318 */ SyscallDesc("sysinfo", sysinfoFunc<AlphaLinux>),
    /* 319 */ SyscallDesc("_sysctl", unimplementedFunc),
    /* 320 */ SyscallDesc("was sys_idle", unimplementedFunc),
    /* 321 */ SyscallDesc("oldumount", unimplementedFunc),
    /* 322 */ SyscallDesc("swapon", unimplementedFunc),
    /* 323 */ SyscallDesc("times", ignoreFunc),
    /* 324 */ SyscallDesc("personality", unimplementedFunc),
    /* 325 */ SyscallDesc("setfsuid", unimplementedFunc),
    /* 326 */ SyscallDesc("setfsgid", unimplementedFunc),
    /* 327 */ SyscallDesc("ustat", unimplementedFunc),
    /* 328 */ SyscallDesc("statfs", unimplementedFunc),
    /* 329 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 330 */ SyscallDesc("sched_setparam", unimplementedFunc),
    /* 331 */ SyscallDesc("sched_getparam", unimplementedFunc),
    /* 332 */ SyscallDesc("sched_setscheduler", unimplementedFunc),
    /* 333 */ SyscallDesc("sched_getscheduler", unimplementedFunc),
    /* 334 */ SyscallDesc("sched_yield", unimplementedFunc),
    /* 335 */ SyscallDesc("sched_get_priority_max", unimplementedFunc),
    /* 336 */ SyscallDesc("sched_get_priority_min", unimplementedFunc),
    /* 337 */ SyscallDesc("sched_rr_get_interval", unimplementedFunc),
    /* 338 */ SyscallDesc("afs_syscall", unimplementedFunc),
    /* 339 */ SyscallDesc("uname", unameFunc),
    /* 340 */ SyscallDesc("nanosleep", unimplementedFunc),
    /* 341 */ SyscallDesc("mremap", mremapFunc<AlphaLinux>),
    /* 342 */ SyscallDesc("nfsservctl", unimplementedFunc),
    /* 343 */ SyscallDesc("setresuid", unimplementedFunc),
    /* 344 */ SyscallDesc("getresuid", unimplementedFunc),
    /* 345 */ SyscallDesc("pciconfig_read", unimplementedFunc),
    /* 346 */ SyscallDesc("pciconfig_write", unimplementedFunc),
    /* 347 */ SyscallDesc("query_module", unimplementedFunc),
    /* 348 */ SyscallDesc("prctl", unimplementedFunc),
    /* 349 */ SyscallDesc("pread", unimplementedFunc),
    /* 350 */ SyscallDesc("pwrite", unimplementedFunc),
    /* 351 */ SyscallDesc("rt_sigreturn", unimplementedFunc),
    /* 352 */ SyscallDesc("rt_sigaction", ignoreFunc),
    /* 353 */ SyscallDesc("rt_sigprocmask", unimplementedFunc),
    /* 354 */ SyscallDesc("rt_sigpending", unimplementedFunc),
    /* 355 */ SyscallDesc("rt_sigtimedwait", unimplementedFunc),
    /* 356 */ SyscallDesc("rt_sigqueueinfo", unimplementedFunc),
    /* 357 */ SyscallDesc("rt_sigsuspend", unimplementedFunc),
    /* 358 */ SyscallDesc("select", unimplementedFunc),
    /* 359 */ SyscallDesc("gettimeofday", gettimeofdayFunc<AlphaLinux>),
    /* 360 */ SyscallDesc("settimeofday", unimplementedFunc),
    /* 361 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 362 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 363 */ SyscallDesc("utimes", utimesFunc<AlphaLinux>),
    /* 364 */ SyscallDesc("getrusage", getrusageFunc<AlphaLinux>),
    /* 365 */ SyscallDesc("wait4", unimplementedFunc),
    /* 366 */ SyscallDesc("adjtimex", unimplementedFunc),
    /* 367 */ SyscallDesc("getcwd", getcwdFunc),
    /* 368 */ SyscallDesc("capget", unimplementedFunc),
    /* 369 */ SyscallDesc("capset", unimplementedFunc),
    /* 370 */ SyscallDesc("sendfile", unimplementedFunc),
    /* 371 */ SyscallDesc("setresgid", unimplementedFunc),
    /* 372 */ SyscallDesc("getresgid", unimplementedFunc),
    /* 373 */ SyscallDesc("dipc", unimplementedFunc),
    /* 374 */ SyscallDesc("pivot_root", unimplementedFunc),
    /* 375 */ SyscallDesc("mincore", unimplementedFunc),
    /* 376 */ SyscallDesc("pciconfig_iobase", unimplementedFunc),
    /* 377 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 378 */ SyscallDesc("gettid", unimplementedFunc),
    /* 379 */ SyscallDesc("readahead", unimplementedFunc),
    /* 380 */ SyscallDesc("security", unimplementedFunc),
    /* 381 */ SyscallDesc("tkill", unimplementedFunc),
    /* 382 */ SyscallDesc("setxattr", unimplementedFunc),
    /* 383 */ SyscallDesc("lsetxattr", unimplementedFunc),
    /* 384 */ SyscallDesc("fsetxattr", unimplementedFunc),
    /* 385 */ SyscallDesc("getxattr", unimplementedFunc),
    /* 386 */ SyscallDesc("lgetxattr", unimplementedFunc),
    /* 387 */ SyscallDesc("fgetxattr", unimplementedFunc),
    /* 388 */ SyscallDesc("listxattr", unimplementedFunc),
    /* 389 */ SyscallDesc("llistxattr", unimplementedFunc),
    /* 390 */ SyscallDesc("flistxattr", unimplementedFunc),
    /* 391 */ SyscallDesc("removexattr", unimplementedFunc),
    /* 392 */ SyscallDesc("lremovexattr", unimplementedFunc),
    /* 393 */ SyscallDesc("fremovexattr", unimplementedFunc),
    /* 394 */ SyscallDesc("futex", unimplementedFunc),
    /* 395 */ SyscallDesc("sched_setaffinity", unimplementedFunc),
    /* 396 */ SyscallDesc("sched_getaffinity", unimplementedFunc),
    /* 397 */ SyscallDesc("tuxcall", unimplementedFunc),
    /* 398 */ SyscallDesc("io_setup", unimplementedFunc),
    /* 399 */ SyscallDesc("io_destroy", unimplementedFunc),
    /* 400 */ SyscallDesc("io_getevents", unimplementedFunc),
    /* 401 */ SyscallDesc("io_submit", unimplementedFunc),
    /* 402 */ SyscallDesc("io_cancel", unimplementedFunc),
    /* 403 */ SyscallDesc("unknown #403", unimplementedFunc),
    /* 404 */ SyscallDesc("unknown #404", unimplementedFunc),
    /* 405 */ SyscallDesc("exit_group", exitGroupFunc), // exit all threads...
    /* 406 */ SyscallDesc("lookup_dcookie", unimplementedFunc),
    /* 407 */ SyscallDesc("sys_epoll_create", unimplementedFunc),
    /* 408 */ SyscallDesc("sys_epoll_ctl", unimplementedFunc),
    /* 409 */ SyscallDesc("sys_epoll_wait", unimplementedFunc),
    /* 410 */ SyscallDesc("remap_file_pages", unimplementedFunc),
    /* 411 */ SyscallDesc("set_tid_address", unimplementedFunc),
    /* 412 */ SyscallDesc("restart_syscall", unimplementedFunc),
    /* 413 */ SyscallDesc("fadvise64", unimplementedFunc),
    /* 414 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 415 */ SyscallDesc("timer_settime", unimplementedFunc),
    /* 416 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 417 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 418 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 419 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 420 */ SyscallDesc("clock_gettime", unimplementedFunc),
    /* 421 */ SyscallDesc("clock_getres", unimplementedFunc),
    /* 422 */ SyscallDesc("clock_nanosleep", unimplementedFunc),
    /* 423 */ SyscallDesc("semtimedop", unimplementedFunc),
    /* 424 */ SyscallDesc("tgkill", unimplementedFunc),
    /* 425 */ SyscallDesc("stat64", stat64Func<AlphaLinux>),
    /* 426 */ SyscallDesc("lstat64", lstat64Func<AlphaLinux>),
    /* 427 */ SyscallDesc("fstat64", fstat64Func<AlphaLinux>),
    /* 428 */ SyscallDesc("vserver", unimplementedFunc),
    /* 429 */ SyscallDesc("mbind", unimplementedFunc),
    /* 430 */ SyscallDesc("get_mempolicy", unimplementedFunc),
    /* 431 */ SyscallDesc("set_mempolicy", unimplementedFunc),
    /* 432 */ SyscallDesc("mq_open", unimplementedFunc),
    /* 433 */ SyscallDesc("mq_unlink", unimplementedFunc),
    /* 434 */ SyscallDesc("mq_timedsend", unimplementedFunc),
    /* 435 */ SyscallDesc("mq_timedreceive", unimplementedFunc),
    /* 436 */ SyscallDesc("mq_notify", unimplementedFunc),
    /* 437 */ SyscallDesc("mq_getsetattr", unimplementedFunc),
    /* 438 */ SyscallDesc("waitid", unimplementedFunc),
    /* 439 */ SyscallDesc("add_key", unimplementedFunc),
    /* 440 */ SyscallDesc("request_key", unimplementedFunc),
    /* 441 */ SyscallDesc("keyctl", unimplementedFunc)
};

AlphaLinuxProcess::AlphaLinuxProcess(LiveProcessParams * params,
                                     ObjectFile *objFile)
    : AlphaLiveProcess(params, objFile),
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
