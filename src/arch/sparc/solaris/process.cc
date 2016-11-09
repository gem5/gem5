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
 * Authors: Ali Saidi
 */

#include "arch/sparc/solaris/process.hh"

#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/registers.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/solaris/solaris.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace SparcISA;


/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, int callnum, Process *process,
          ThreadContext *tc)
{
    int index = 0;
    TypedBufferArg<Solaris::utsname> name(process->getSyscallArg(tc, index));

    strcpy(name->sysname, "SunOS");
    strcpy(name->nodename, "m5.eecs.umich.edu");
    strcpy(name->release, "5.9"); //?? do we want this or something newer?
    strcpy(name->version, "Generic_118558-21");
    strcpy(name->machine, "sun4u");

    name.copyOut(tc->getMemProxy());

    return 0;
}


SyscallDesc SparcSolarisProcess::syscallDescs[] = {
    /* 0 */ SyscallDesc("syscall", unimplementedFunc),
    /* 1 */ SyscallDesc("exit", exitFunc),
    /* 2 */ SyscallDesc("fork", unimplementedFunc),
    /* 3 */ SyscallDesc("read", readFunc),
    /* 4 */ SyscallDesc("write", writeFunc),
    /* 5 */ SyscallDesc("open", openFunc<SparcSolaris>),
    /* 6 */ SyscallDesc("close", closeFunc),
    /* 7 */ SyscallDesc("wait", unimplementedFunc),
    /* 8 */ SyscallDesc("creat", unimplementedFunc),
    /* 9 */ SyscallDesc("link", unimplementedFunc),
    /* 10 */ SyscallDesc("unlink", unlinkFunc),
    /* 11 */ SyscallDesc("exec", unimplementedFunc),
    /* 12 */ SyscallDesc("chdir", unimplementedFunc),
    /* 13 */ SyscallDesc("time", unimplementedFunc),
    /* 14 */ SyscallDesc("mknod", unimplementedFunc),
    /* 15 */ SyscallDesc("chmod", chmodFunc<Solaris>),
    /* 16 */ SyscallDesc("chown", chownFunc),
    /* 17 */ SyscallDesc("brk", brkFunc),
    /* 18 */ SyscallDesc("stat", unimplementedFunc),
    /* 19 */ SyscallDesc("lseek", lseekFunc),
    /* 20 */ SyscallDesc("getpid", getpidFunc),
    /* 21 */ SyscallDesc("mount", unimplementedFunc),
    /* 22 */ SyscallDesc("umount", unimplementedFunc),
    /* 23 */ SyscallDesc("setuid", setuidFunc),
    /* 24 */ SyscallDesc("getuid", getuidFunc),
    /* 25 */ SyscallDesc("stime", unimplementedFunc),
    /* 26 */ SyscallDesc("pcsample", unimplementedFunc),
    /* 27 */ SyscallDesc("alarm", unimplementedFunc),
    /* 28 */ SyscallDesc("fstat", fstatFunc<SparcSolaris>),
    /* 29 */ SyscallDesc("pause", unimplementedFunc),
    /* 30 */ SyscallDesc("utime", unimplementedFunc),
    /* 31 */ SyscallDesc("stty", unimplementedFunc),
    /* 32 */ SyscallDesc("gtty", unimplementedFunc),
    /* 33 */ SyscallDesc("access", unimplementedFunc),
    /* 34 */ SyscallDesc("nice", unimplementedFunc),
    /* 35 */ SyscallDesc("statfs", unimplementedFunc),
    /* 36 */ SyscallDesc("sync", unimplementedFunc),
    /* 37 */ SyscallDesc("kill", unimplementedFunc),
    /* 38 */ SyscallDesc("fstatfs", unimplementedFunc),
    /* 39 */ SyscallDesc("pgrpsys", unimplementedFunc),
    /* 40 */ SyscallDesc("xenix", unimplementedFunc),
    /* 41 */ SyscallDesc("dup", unimplementedFunc),
    /* 42 */ SyscallDesc("pipe", pipePseudoFunc),
    /* 43 */ SyscallDesc("times", unimplementedFunc),
    /* 44 */ SyscallDesc("profil", unimplementedFunc),
    /* 45 */ SyscallDesc("plock", unimplementedFunc),
    /* 46 */ SyscallDesc("setgid", unimplementedFunc),
    /* 47 */ SyscallDesc("getgid", getgidFunc),
    /* 48 */ SyscallDesc("signal", unimplementedFunc),
    /* 49 */ SyscallDesc("msgsys", unimplementedFunc),
    /* 50 */ SyscallDesc("syssun", unimplementedFunc),
    /* 51 */ SyscallDesc("acct", unimplementedFunc),
    /* 52 */ SyscallDesc("shmsys", unimplementedFunc),
    /* 53 */ SyscallDesc("semsys", unimplementedFunc),
    /* 54 */ SyscallDesc("ioctl", unimplementedFunc),
    /* 55 */ SyscallDesc("uadmin", unimplementedFunc),
    /* 56 */ SyscallDesc("RESERVED", unimplementedFunc),
    /* 57 */ SyscallDesc("utssys", unimplementedFunc),
    /* 58 */ SyscallDesc("fdsync", unimplementedFunc),
    /* 59 */ SyscallDesc("execve", unimplementedFunc),
    /* 60 */ SyscallDesc("umask", umaskFunc),
    /* 61 */ SyscallDesc("chroot", unimplementedFunc),
    /* 62 */ SyscallDesc("fcntl", unimplementedFunc),
    /* 63 */ SyscallDesc("ulimit", unimplementedFunc),
    /* 64 */ SyscallDesc("reserved_64", unimplementedFunc),
    /* 65 */ SyscallDesc("reserved_65", unimplementedFunc),
    /* 66 */ SyscallDesc("reserved_66", unimplementedFunc),
    /* 67 */ SyscallDesc("reserved_67", unimplementedFunc),
    /* 68 */ SyscallDesc("reserved_68", unimplementedFunc),
    /* 69 */ SyscallDesc("reserved_69", unimplementedFunc),
    /* 70 */ SyscallDesc("tasksys", unimplementedFunc),
    /* 71 */ SyscallDesc("acctctl", unimplementedFunc),
    /* 72 */ SyscallDesc("reserved_72", unimplementedFunc),
    /* 73 */ SyscallDesc("getpagesizes", unimplementedFunc),
    /* 74 */ SyscallDesc("rctlsys", unimplementedFunc),
    /* 75 */ SyscallDesc("issetugid", unimplementedFunc),
    /* 76 */ SyscallDesc("fsat", unimplementedFunc),
    /* 77 */ SyscallDesc("lwp_park", unimplementedFunc),
    /* 78 */ SyscallDesc("sendfilev", unimplementedFunc),
    /* 79 */ SyscallDesc("rmdir", unimplementedFunc),
    /* 80 */ SyscallDesc("mkdir", unimplementedFunc),
    /* 81 */ SyscallDesc("getdents", unimplementedFunc),
    /* 82 */ SyscallDesc("reserved_82", unimplementedFunc),
    /* 83 */ SyscallDesc("reserved_83", unimplementedFunc),
    /* 84 */ SyscallDesc("sysfs", unimplementedFunc),
    /* 85 */ SyscallDesc("getmsg", unimplementedFunc),
    /* 86 */ SyscallDesc("putmsg", unimplementedFunc),
    /* 87 */ SyscallDesc("poll", unimplementedFunc),
    /* 88 */ SyscallDesc("lstat", unimplementedFunc),
    /* 89 */ SyscallDesc("symlink", unimplementedFunc),
    /* 90 */ SyscallDesc("readlink", readlinkFunc),
    /* 91 */ SyscallDesc("setgroups", unimplementedFunc),
    /* 92 */ SyscallDesc("getgroups", unimplementedFunc),
    /* 93 */ SyscallDesc("fchmod", unimplementedFunc),
    /* 94 */ SyscallDesc("fchown", unimplementedFunc),
    /* 95 */ SyscallDesc("sigprocmask", unimplementedFunc),
    /* 96 */ SyscallDesc("sigsuspend", unimplementedFunc),
    /* 97 */ SyscallDesc("sigaltstack", unimplementedFunc),
    /* 98 */ SyscallDesc("sigaction", unimplementedFunc),
    /* 99 */ SyscallDesc("sigpending", unimplementedFunc),
    /* 100 */ SyscallDesc("context", unimplementedFunc),
    /* 101 */ SyscallDesc("evsys", unimplementedFunc),
    /* 102 */ SyscallDesc("evtrapret", unimplementedFunc),
    /* 103 */ SyscallDesc("statvfs", unimplementedFunc),
    /* 104 */ SyscallDesc("fstatvfs", unimplementedFunc),
    /* 105 */ SyscallDesc("getloadavg", unimplementedFunc),
    /* 106 */ SyscallDesc("nfssys", unimplementedFunc),
    /* 107 */ SyscallDesc("waitsys", unimplementedFunc),
    /* 108 */ SyscallDesc("sigsendsys", unimplementedFunc),
    /* 109 */ SyscallDesc("hrtsys", unimplementedFunc),
    /* 110 */ SyscallDesc("acancel", unimplementedFunc),
    /* 111 */ SyscallDesc("async", unimplementedFunc),
    /* 112 */ SyscallDesc("priocntlsys", unimplementedFunc),
    /* 113 */ SyscallDesc("pathconf", unimplementedFunc),
    /* 114 */ SyscallDesc("mincore", unimplementedFunc),
    /* 115 */ SyscallDesc("mmap", mmapFunc<SparcSolaris>),
    /* 116 */ SyscallDesc("mprotect", unimplementedFunc),
    /* 117 */ SyscallDesc("munmap", munmapFunc),
    /* 118 */ SyscallDesc("fpathconf", unimplementedFunc),
    /* 119 */ SyscallDesc("vfork", unimplementedFunc),
    /* 120 */ SyscallDesc("fchdir", unimplementedFunc),
    /* 121 */ SyscallDesc("readv", unimplementedFunc),
    /* 122 */ SyscallDesc("writev", unimplementedFunc),
    /* 123 */ SyscallDesc("xstat", unimplementedFunc),
    /* 124 */ SyscallDesc("lxstat", unimplementedFunc),
    /* 125 */ SyscallDesc("fxstat", unimplementedFunc),
    /* 126 */ SyscallDesc("xmknod", unimplementedFunc),
    /* 127 */ SyscallDesc("clocal", unimplementedFunc),
    /* 128 */ SyscallDesc("setrlimit", unimplementedFunc),
    /* 129 */ SyscallDesc("getrlimit", unimplementedFunc),
    /* 130 */ SyscallDesc("lchown", unimplementedFunc),
    /* 131 */ SyscallDesc("memcntl", unimplementedFunc),
    /* 132 */ SyscallDesc("getpmsg", unimplementedFunc),
    /* 133 */ SyscallDesc("putpmsg", unimplementedFunc),
    /* 134 */ SyscallDesc("rename", unimplementedFunc),
    /* 135 */ SyscallDesc("uname", unameFunc),
    /* 136 */ SyscallDesc("setegid", unimplementedFunc),
    /* 137 */ SyscallDesc("sysconfig", unimplementedFunc),
    /* 138 */ SyscallDesc("adjtime", unimplementedFunc),
    /* 139 */ SyscallDesc("systeminfo", unimplementedFunc),
    /* 140 */ SyscallDesc("reserved_140", unimplementedFunc),
    /* 141 */ SyscallDesc("seteuid", unimplementedFunc),
    /* 142 */ SyscallDesc("vtrace", unimplementedFunc),
    /* 143 */ SyscallDesc("fork1", unimplementedFunc),
    /* 144 */ SyscallDesc("sigtimedwait", unimplementedFunc),
    /* 145 */ SyscallDesc("lwp_info", unimplementedFunc),
    /* 146 */ SyscallDesc("yield", unimplementedFunc),
    /* 147 */ SyscallDesc("lwp_sema_wait", unimplementedFunc),
    /* 148 */ SyscallDesc("lwp_sema_post", unimplementedFunc),
    /* 149 */ SyscallDesc("lwp_sema_trywait", unimplementedFunc),
    /* 150 */ SyscallDesc("lwp_detach", unimplementedFunc),
    /* 151 */ SyscallDesc("corectl", unimplementedFunc),
    /* 152 */ SyscallDesc("modctl", unimplementedFunc),
    /* 153 */ SyscallDesc("fchroot", unimplementedFunc),
    /* 154 */ SyscallDesc("utimes", unimplementedFunc),
    /* 155 */ SyscallDesc("vhangup", unimplementedFunc),
    /* 156 */ SyscallDesc("gettimeofday", unimplementedFunc),
    /* 157 */ SyscallDesc("getitimer", unimplementedFunc),
    /* 158 */ SyscallDesc("setitimer", unimplementedFunc),
    /* 159 */ SyscallDesc("lwp_create", unimplementedFunc),
    /* 160 */ SyscallDesc("lwp_exit", unimplementedFunc),
    /* 161 */ SyscallDesc("lwp_suspend", unimplementedFunc),
    /* 162 */ SyscallDesc("lwp_continue", unimplementedFunc),
    /* 163 */ SyscallDesc("lwp_kill", unimplementedFunc),
    /* 164 */ SyscallDesc("lwp_self", unimplementedFunc),
    /* 165 */ SyscallDesc("lwp_setprivate", unimplementedFunc),
    /* 166 */ SyscallDesc("lwp_getprivate", unimplementedFunc),
    /* 167 */ SyscallDesc("lwp_wait", unimplementedFunc),
    /* 168 */ SyscallDesc("lwp_mutex_wakeup", unimplementedFunc),
    /* 169 */ SyscallDesc("lwp_mutex_lock", unimplementedFunc),
    /* 170 */ SyscallDesc("lwp_cond_wait", unimplementedFunc),
    /* 171 */ SyscallDesc("lwp_cond_signal", unimplementedFunc),
    /* 172 */ SyscallDesc("lwp_cond_broadcast", unimplementedFunc),
    /* 173 */ SyscallDesc("pread", unimplementedFunc),
    /* 174 */ SyscallDesc("pwrite", unimplementedFunc),
    /* 175 */ SyscallDesc("llseek", unimplementedFunc),
    /* 176 */ SyscallDesc("inst_sync", unimplementedFunc),
    /* 177 */ SyscallDesc("srmlimitsys", unimplementedFunc),
    /* 178 */ SyscallDesc("kaio", unimplementedFunc),
    /* 179 */ SyscallDesc("cpc", unimplementedFunc),
    /* 180 */ SyscallDesc("lgrpsys_meminfosys", unimplementedFunc),
    /* 181 */ SyscallDesc("rusagesys", unimplementedFunc),
    /* 182 */ SyscallDesc("reserved_182", unimplementedFunc),
    /* 183 */ SyscallDesc("reserved_183", unimplementedFunc),
    /* 184 */ SyscallDesc("tsolsys", unimplementedFunc),
    /* 185 */ SyscallDesc("acl", unimplementedFunc),
    /* 186 */ SyscallDesc("auditsys", unimplementedFunc),
    /* 187 */ SyscallDesc("processor_bind", unimplementedFunc),
    /* 188 */ SyscallDesc("processor_info", unimplementedFunc),
    /* 189 */ SyscallDesc("p_online", unimplementedFunc),
    /* 190 */ SyscallDesc("sigqueue", unimplementedFunc),
    /* 191 */ SyscallDesc("clock_gettime", unimplementedFunc),
    /* 192 */ SyscallDesc("clock_settime", unimplementedFunc),
    /* 193 */ SyscallDesc("clock_getres", unimplementedFunc),
    /* 194 */ SyscallDesc("timer_create", unimplementedFunc),
    /* 195 */ SyscallDesc("timer_delete", unimplementedFunc),
    /* 196 */ SyscallDesc("timer_settime", unimplementedFunc),
    /* 197 */ SyscallDesc("timer_gettime", unimplementedFunc),
    /* 198 */ SyscallDesc("timer_getoverrun", unimplementedFunc),
    /* 199 */ SyscallDesc("nanosleep", unimplementedFunc),
    /* 200 */ SyscallDesc("facl", unimplementedFunc),
    /* 201 */ SyscallDesc("door", unimplementedFunc),
    /* 202 */ SyscallDesc("setreuid", unimplementedFunc),
    /* 203 */ SyscallDesc("setregid", unimplementedFunc),
    /* 204 */ SyscallDesc("install_utrap", unimplementedFunc),
    /* 205 */ SyscallDesc("signotify", unimplementedFunc),
    /* 206 */ SyscallDesc("schedctl", unimplementedFunc),
    /* 207 */ SyscallDesc("pset", unimplementedFunc),
    /* 208 */ SyscallDesc("sparc_utrap_install", unimplementedFunc),
    /* 209 */ SyscallDesc("resolvepath", unimplementedFunc),
    /* 210 */ SyscallDesc("signotifywait", unimplementedFunc),
    /* 211 */ SyscallDesc("lwp_sigredirect", unimplementedFunc),
    /* 212 */ SyscallDesc("lwp_alarm", unimplementedFunc),
    /* 213 */ SyscallDesc("getdents64", unimplementedFunc),
    /* 214 */ SyscallDesc("mmap64", unimplementedFunc),
    /* 215 */ SyscallDesc("stat64", unimplementedFunc),
    /* 216 */ SyscallDesc("lstat64", unimplementedFunc),
    /* 217 */ SyscallDesc("fstat64", unimplementedFunc),
    /* 218 */ SyscallDesc("statvfs64", unimplementedFunc),
    /* 219 */ SyscallDesc("fstatvfs64", unimplementedFunc),
    /* 220 */ SyscallDesc("setrlimit64", unimplementedFunc),
    /* 221 */ SyscallDesc("getrlimit64", unimplementedFunc),
    /* 222 */ SyscallDesc("pread64", unimplementedFunc),
    /* 223 */ SyscallDesc("pwrite64", unimplementedFunc),
    /* 224 */ SyscallDesc("creat64", unimplementedFunc),
    /* 225 */ SyscallDesc("open64", unimplementedFunc),
    /* 226 */ SyscallDesc("rpcsys", unimplementedFunc),
    /* 227 */ SyscallDesc("reserved_227", unimplementedFunc),
    /* 228 */ SyscallDesc("reserved_228", unimplementedFunc),
    /* 229 */ SyscallDesc("reserved_229", unimplementedFunc),
    /* 230 */ SyscallDesc("so_socket", unimplementedFunc),
    /* 231 */ SyscallDesc("so_socketpair", unimplementedFunc),
    /* 232 */ SyscallDesc("bind", unimplementedFunc),
    /* 233 */ SyscallDesc("listen", unimplementedFunc),
    /* 234 */ SyscallDesc("accept", unimplementedFunc),
    /* 235 */ SyscallDesc("connect", unimplementedFunc),
    /* 236 */ SyscallDesc("shutdown", unimplementedFunc),
    /* 237 */ SyscallDesc("recv", unimplementedFunc),
    /* 238 */ SyscallDesc("recvfrom", unimplementedFunc),
    /* 239 */ SyscallDesc("recvmsg", unimplementedFunc),
    /* 240 */ SyscallDesc("send", unimplementedFunc),
    /* 241 */ SyscallDesc("sendmsg", unimplementedFunc),
    /* 242 */ SyscallDesc("sendto", unimplementedFunc),
    /* 243 */ SyscallDesc("getpeername", unimplementedFunc),
    /* 244 */ SyscallDesc("getsockname", unimplementedFunc),
    /* 245 */ SyscallDesc("getsockopt", unimplementedFunc),
    /* 246 */ SyscallDesc("setsockopt", unimplementedFunc),
    /* 247 */ SyscallDesc("sockconfig", unimplementedFunc),
    /* 248 */ SyscallDesc("ntp_gettime", unimplementedFunc),
    /* 249 */ SyscallDesc("ntp_adjtime", unimplementedFunc),
    /* 250 */ SyscallDesc("lwp_mutex_unlock", unimplementedFunc),
    /* 251 */ SyscallDesc("lwp_mutex_trylock", unimplementedFunc),
    /* 252 */ SyscallDesc("lwp_mutex_init", unimplementedFunc),
    /* 253 */ SyscallDesc("cladm", unimplementedFunc),
    /* 254 */ SyscallDesc("lwp_sigtimedwait", unimplementedFunc),
    /* 255 */ SyscallDesc("umount2", unimplementedFunc)
};

SparcSolarisProcess::SparcSolarisProcess(ProcessParams * params,
                                         ObjectFile *objFile)
    : Sparc64Process(params, objFile),
     Num_Syscall_Descs(sizeof(syscallDescs) / sizeof(SyscallDesc))
{
    // The sparc syscall table must be <= 284 entries because that is all there
    // is space for.
    assert(Num_Syscall_Descs <= 284);
}



SyscallDesc*
SparcSolarisProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum >= Num_Syscall_Descs)
        return NULL;
    return &syscallDescs[callnum];
}
