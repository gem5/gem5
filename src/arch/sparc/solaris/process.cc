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

#include "arch/sparc/solaris/process.hh"

#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/registers.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/solaris/solaris.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

using namespace std;
using namespace SparcISA;

namespace
{

class SparcSolarisObjectFileLoader : public Process::Loader
{
  public:
    Process *
    load(ProcessParams *params, ::Loader::ObjectFile *obj_file) override
    {
        auto arch = obj_file->getArch();
        auto opsys = obj_file->getOpSys();

        if (arch != ::Loader::SPARC64 && arch != ::Loader::SPARC32)
            return nullptr;

        if (opsys != ::Loader::Solaris)
            return nullptr;

        return new SparcSolarisProcess(params, obj_file);
    }
};

SparcSolarisObjectFileLoader loader;

} // anonymous namespace


/// Target uname() handler.
static SyscallReturn
unameFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<Solaris::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "SunOS");
    strcpy(name->nodename, "m5.eecs.umich.edu");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "Generic_118558-21");
    strcpy(name->machine, "sun4u");

    return 0;
}


SyscallDescTable<Sparc64Process::SyscallABI>
    SparcSolarisProcess::syscallDescs = {
    { 0, "syscall" },
    { 1, "exit", exitFunc },
    { 2, "fork" },
    { 3, "read", readFunc<SparcSolaris> },
    { 4, "write", writeFunc<SparcSolaris> },
    { 5, "open", openFunc<SparcSolaris> },
    { 6, "close", closeFunc },
    { 7, "wait" },
    { 8, "creat" },
    { 9, "link" },
    { 10, "unlink", unlinkFunc },
    { 11, "exec" },
    { 12, "chdir" },
    { 13, "time" },
    { 14, "mknod" },
    { 15, "chmod", chmodFunc<Solaris> },
    { 16, "chown", chownFunc },
    { 17, "brk", brkFunc },
    { 18, "stat" },
    { 19, "lseek", lseekFunc },
    { 20, "getpid", getpidFunc },
    { 21, "mount" },
    { 22, "umount" },
    { 23, "setuid", ignoreFunc },
    { 24, "getuid", getuidFunc },
    { 25, "stime" },
    { 26, "pcsample" },
    { 27, "alarm" },
    { 28, "fstat", fstatFunc<SparcSolaris> },
    { 29, "pause" },
    { 30, "utime" },
    { 31, "stty" },
    { 32, "gtty" },
    { 33, "access" },
    { 34, "nice" },
    { 35, "statfs" },
    { 36, "sync" },
    { 37, "kill" },
    { 38, "fstatfs" },
    { 39, "pgrpsys" },
    { 40, "xenix" },
    { 41, "dup" },
    { 42, "pipe", pipePseudoFunc },
    { 43, "times" },
    { 44, "profil" },
    { 45, "plock" },
    { 46, "setgid" },
    { 47, "getgid", getgidFunc },
    { 48, "signal" },
    { 49, "msgsys" },
    { 50, "syssun" },
    { 51, "acct" },
    { 52, "shmsys" },
    { 53, "semsys" },
    { 54, "ioctl" },
    { 55, "uadmin" },
    { 56, "RESERVED" },
    { 57, "utssys" },
    { 58, "fdsync" },
    { 59, "execve" },
    { 60, "umask", umaskFunc },
    { 61, "chroot" },
    { 62, "fcntl" },
    { 63, "ulimit" },
    { 64, "reserved_64" },
    { 65, "reserved_65" },
    { 66, "reserved_66" },
    { 67, "reserved_67" },
    { 68, "reserved_68" },
    { 69, "reserved_69" },
    { 70, "tasksys" },
    { 71, "acctctl" },
    { 72, "reserved_72" },
    { 73, "getpagesizes" },
    { 74, "rctlsys" },
    { 75, "issetugid" },
    { 76, "fsat" },
    { 77, "lwp_park" },
    { 78, "sendfilev" },
    { 79, "rmdir" },
    { 80, "mkdir" },
    { 81, "getdents" },
    { 82, "reserved_82" },
    { 83, "reserved_83" },
    { 84, "sysfs" },
    { 85, "getmsg" },
    { 86, "putmsg" },
    { 87, "poll" },
    { 88, "lstat" },
    { 89, "symlink" },
    { 90, "readlink", readlinkFunc },
    { 91, "setgroups" },
    { 92, "getgroups" },
    { 93, "fchmod" },
    { 94, "fchown" },
    { 95, "sigprocmask" },
    { 96, "sigsuspend" },
    { 97, "sigaltstack" },
    { 98, "sigaction" },
    { 99, "sigpending" },
    { 100, "context" },
    { 101, "evsys" },
    { 102, "evtrapret" },
    { 103, "statvfs" },
    { 104, "fstatvfs" },
    { 105, "getloadavg" },
    { 106, "nfssys" },
    { 107, "waitsys" },
    { 108, "sigsendsys" },
    { 109, "hrtsys" },
    { 110, "acancel" },
    { 111, "async" },
    { 112, "priocntlsys" },
    { 113, "pathconf" },
    { 114, "mincore" },
    { 115, "mmap", mmapFunc<SparcSolaris> },
    { 116, "mprotect" },
    { 117, "munmap", munmapFunc },
    { 118, "fpathconf" },
    { 119, "vfork" },
    { 120, "fchdir" },
    { 121, "readv" },
    { 122, "writev" },
    { 123, "xstat" },
    { 124, "lxstat" },
    { 125, "fxstat" },
    { 126, "xmknod" },
    { 127, "clocal" },
    { 128, "setrlimit" },
    { 129, "getrlimit" },
    { 130, "lchown" },
    { 131, "memcntl" },
    { 132, "getpmsg" },
    { 133, "putpmsg" },
    { 134, "rename" },
    { 135, "uname", unameFunc },
    { 136, "setegid" },
    { 137, "sysconfig" },
    { 138, "adjtime" },
    { 139, "systeminfo" },
    { 140, "reserved_140" },
    { 141, "seteuid" },
    { 142, "vtrace" },
    { 143, "fork1" },
    { 144, "sigtimedwait" },
    { 145, "lwp_info" },
    { 146, "yield" },
    { 147, "lwp_sema_wait" },
    { 148, "lwp_sema_post" },
    { 149, "lwp_sema_trywait" },
    { 150, "lwp_detach" },
    { 151, "corectl" },
    { 152, "modctl" },
    { 153, "fchroot" },
    { 154, "utimes" },
    { 155, "vhangup" },
    { 156, "gettimeofday" },
    { 157, "getitimer" },
    { 158, "setitimer" },
    { 159, "lwp_create" },
    { 160, "lwp_exit" },
    { 161, "lwp_suspend" },
    { 162, "lwp_continue" },
    { 163, "lwp_kill" },
    { 164, "lwp_self" },
    { 165, "lwp_setprivate" },
    { 166, "lwp_getprivate" },
    { 167, "lwp_wait" },
    { 168, "lwp_mutex_wakeup" },
    { 169, "lwp_mutex_lock" },
    { 170, "lwp_cond_wait" },
    { 171, "lwp_cond_signal" },
    { 172, "lwp_cond_broadcast" },
    { 173, "pread" },
    { 174, "pwrite" },
    { 175, "llseek" },
    { 176, "inst_sync" },
    { 177, "srmlimitsys" },
    { 178, "kaio" },
    { 179, "cpc" },
    { 180, "lgrpsys_meminfosys" },
    { 181, "rusagesys" },
    { 182, "reserved_182" },
    { 183, "reserved_183" },
    { 184, "tsolsys" },
    { 185, "acl" },
    { 186, "auditsys" },
    { 187, "processor_bind" },
    { 188, "processor_info" },
    { 189, "p_online" },
    { 190, "sigqueue" },
    { 191, "clock_gettime" },
    { 192, "clock_settime" },
    { 193, "clock_getres" },
    { 194, "timer_create" },
    { 195, "timer_delete" },
    { 196, "timer_settime" },
    { 197, "timer_gettime" },
    { 198, "timer_getoverrun" },
    { 199, "nanosleep" },
    { 200, "facl" },
    { 201, "door" },
    { 202, "setreuid" },
    { 203, "setregid" },
    { 204, "install_utrap" },
    { 205, "signotify" },
    { 206, "schedctl" },
    { 207, "pset" },
    { 208, "sparc_utrap_install" },
    { 209, "resolvepath" },
    { 210, "signotifywait" },
    { 211, "lwp_sigredirect" },
    { 212, "lwp_alarm" },
    { 213, "getdents64" },
    { 214, "mmap64" },
    { 215, "stat64" },
    { 216, "lstat64" },
    { 217, "fstat64" },
    { 218, "statvfs64" },
    { 219, "fstatvfs64" },
    { 220, "setrlimit64" },
    { 221, "getrlimit64" },
    { 222, "pread64" },
    { 223, "pwrite64" },
    { 224, "creat64" },
    { 225, "open64" },
    { 226, "rpcsys" },
    { 227, "reserved_227" },
    { 228, "reserved_228" },
    { 229, "reserved_229" },
    { 230, "so_socket" },
    { 231, "so_socketpair" },
    { 232, "bind" },
    { 233, "listen" },
    { 234, "accept" },
    { 235, "connect" },
    { 236, "shutdown" },
    { 237, "recv" },
    { 238, "recvfrom" },
    { 239, "recvmsg" },
    { 240, "send" },
    { 241, "sendmsg" },
    { 242, "sendto" },
    { 243, "getpeername" },
    { 244, "getsockname" },
    { 245, "getsockopt" },
    { 246, "setsockopt" },
    { 247, "sockconfig" },
    { 248, "ntp_gettime" },
    { 249, "ntp_adjtime" },
    { 250, "lwp_mutex_unlock" },
    { 251, "lwp_mutex_trylock" },
    { 252, "lwp_mutex_init" },
    { 253, "cladm" },
    { 254, "lwp_sigtimedwait" },
    { 255, "umount2" }
};

SparcSolarisProcess::SparcSolarisProcess(ProcessParams *params,
                                         ::Loader::ObjectFile *objFile) :
    Sparc64Process(params, objFile)
{}

void
SparcSolarisProcess::syscall(ThreadContext *tc)
{
    Sparc64Process::syscall(tc);
    syscallDescs.get(tc->readIntReg(1))->doSyscall(tc);
}
