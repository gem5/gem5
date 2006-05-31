/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "kern/linux/linux_syscalls.hh"

namespace {
    const char *
    standard_strings[SystemCalls<Linux>::Number] = {


     "llseek",              //0
     "newselect",           //1
     "sysctl",              //2
     "access",              //3
     "acct",                //4
     "adjtimex",            //5
     "afs_syscall",         //6
     "alarm",               //7
     "bdflush",             //8
     "break",               //9


     "brk",                 //10
     "capget",              //11
     "capset",              //12
     "chdir",               //13
     "chmod",               //14
     "chown",               //15
     "chown32",             //16
     "chroot",              //17
     "clock_getres",        //18
     "clock_gettime",       //19


     "clock_nanosleep",     //20
     "clock_settime",       //21
     "clone",               //22
     "close",               //23
     "creat",               //24
     "create_module",       //25
     "delete_module",       //26
     "dup",                 //27
     "dup2",                //28
     "epoll_create",        //29


     "epoll_ctl",           //30
     "epoll_wait",          //31
     "execve",              //32
     "exit",                //33
     "exit_group",          //34
     "fadvise64",           //35
     "fadvise64_64",        //36
     "fchdir",              //37
     "fchmod",              //38
     "fchown",              //39


     "fchown32",            //40
     "fcntl",               //41
     "fcntl64",             //42
     "fdatasync",           //43
     "fgetxattr",           //44
     "flistxattr",          //45
     "flock",               //46
     "fork",                //47
     "fremovexattr",        //48
     "fsetxattr",           //49


     "fstat",               //50
     "fstat64",             //51
     "fstatfs",             //52
     "fstatfs64",           //53
     "fsync",               //54
     "ftime",               //55
     "ftruncate",           //56
     "ftruncate64",         //57
     "futex",               //58
     "get_kernel_syms",     //59


     "get_thread_area",     //60
     "getcwd",              //61
     "getdents",            //62
     "getdents64",          //63
     "getegid",             //64
     "getegid32",           //65
     "geteuid",             //66
     "geteuid32",           //67
     "getgid",              //68
     "getgid32",            //69


     "getgroups",           //70
     "getgroups32",         //71
     "getitimer",           //72
     "getpgid",             //73
     "getpgrp",             //74
     "getpid",              //75
     "getpmsg",             //76
     "getppid",             //77
     "getpriority",         //78
     "getresgid",           //79


     "getresgid32",         //80
     "getresuid",           //81
     "getresuid32",         //82
     "getrlimit",           //83
     "getrusage",           //84
     "getsid",              //85
     "gettid",              //86
     "gettimeofday",        //87
     "getuid",              //88
     "getuid32",            //89


     "getxattr",            //90
     "gtty",                //91
     "idle",                //92
     "init_module",         //93
     "io_cancel",           //94
     "io_destroy",          //95
     "io_getevents",        //96
     "io_setup",            //97
     "io_submit",           //98
     "ioctl",               //99


     "ioperm",              //100
     "iopl",                //101
     "ipc",                 //102
     "kill",                //103
     "lchown",              //104
     "lchown32",            //105
     "lgetxattr",           //106
     "link",                //107
     "listxattr",           //108
     "llistxattr",          //109


     "lock",                //110
     "lookup_dcookie",      //111
     "lremovexattr",        //112
     "lseek",               //113
     "lsetxattr",           //114
     "lstat",               //115
     "lstat64",             //116
     "madvise",             //117
     "madvise1",            //118
     "mincore",             //119


     "mkdir",               //120
     "mknod",               //121
     "mlock",               //122
     "mlockall",            //123
     "mmap",                //124
     "mmap2",               //125
     "modify_ldt",          //126
     "mount",               //127
     "mprotect",            //128
     "mpx",                 //129


     "mremap",              //130
     "msync",               //131
     "munlock",             //132
     "munlockall",          //133
     "munmap",              //134
     "nanosleep",           //135
     "nfsservctl",          //136
     "nice",                //137
     "oldfstat",            //138
     "oldlstat",            //139


     "oldolduname",         //140
     "oldstat",             //141
     "olduname",            //142
     "open",                //143
     "pause",               //144
     "personality",         //145
     "pipe",                //146
     "pivot_root",          //147
     "poll",                //148
     "prctl",               //149


     "pread64",             //150
     "prof",                //151
     "profil",              //152
     "ptrace",              //153
     "putpmsg",             //154
     "pwrite64",            //155
     "query_module",        //156
     "quotactl",            //157
     "read",                //158
     "readahead",           //159


     "readdir",             //160
     "readlink",            //161
     "readv",               //162
     "reboot",              //163
     "remap_file_pages",    //164
     "removexattr",         //165
     "rename",              //166
     "restart_syscall",     //167
     "rmdir",               //168
     "rt_sigaction",        //169


     "rt_sigpending",       //170
     "rt_sigprocmask",      //171
     "rt_sigqueueinfo",     //172
     "rt_sigreturn",        //173
     "rt_sigsuspend",       //174
     "rt_sigtimedwait",     //175
     "sched_get_priority_max",    //176
     "sched_get_priority_min",    //177
     "sched_getaffinity",   //178
     "sched_getparam",      //179


     "sched_getscheduler",  //180
     "sched_rr_get_interval",     //181
     "sched_setaffinity",   //182
     "sched_setparam",      //183
     "sched_setscheduler",  //184
     "sched_yield",         //185
     "select",              //186
     "sendfile",            //187
     "sendfile64",          //188
     "set_thread_area",     //189


     "set_tid_address",     //190
     "setdomainname",       //191
     "setfsgid",            //192
     "setfsgid32",          //193
     "setfsuid",            //194
     "setfsuid32",          //195
     "setgid",              //196
     "setgid32",            //197
     "setgroups",           //198
     "setgroups32",         //199


     "sethostname",         //200
     "setitimer",           //201
     "setpgid",             //202
     "setpriority",         //203
     "setregid",            //204
     "setregid32",          //205
     "setresgid",           //206
     "setresgid32",         //207
     "setresuid",           //208
     "setresuid32",         //209


     "setreuid",            //210
     "setreuid32",          //211
     "setrlimit",           //212
     "setsid",              //213
     "settimeofday",        //214
     "setuid",              //215
     "setuid32",            //216
     "setxattr",            //217
     "sgetmask",            //218
     "sigaction",           //219


     "sigaltstack",         //220
     "signal",              //221
     "sigpending",          //222
     "sigprocmask",         //223
     "sigreturn",           //224
     "sigsuspend",          //225
     "socketcall",          //226
     "ssetmask",            //227
     "stat",                //228
     "stat64",              //229


     "statfs",              //230
     "statfs64",            //231
     "stime",               //232
     "stty",                //233
     "swapoff",             //234
     "swapon",              //235
     "symlink",             //236
     "sync",                //237
     "sysfs",               //238
     "sysinfo",             //239


     "syslog",              //240
     "tgkill",              //241
     "time",                //242
     "timer_create",        //243
     "timer_delete",        //244
     "timer_getoverrun",    //245
     "timer_gettime",       //246
     "timer_settime",       //247
     "times",               //248
     "tkill",               //249


     "truncate",            //250
     "truncate64",          //251
     "ugetrlimit",          //252
     "ulimit",              //253
     "umask",               //254
     "umount",              //255
     "umount2",             //256
     "uname",               //257
     "unlink",              //258
     "uselib",              //259


     "ustat",               //260
     "utime",               //261
     "utimes",              //262
     "vfork",               //263
     "vhangup",             //264
     "vm86",                //265
     "vm86old",             //266
     "vserver",             //267
     "wait4",               //268
     "waitpid",             //269


     "write",               //270
     "writev",              //271
    };


}

const char *
SystemCalls<Linux>::name(int num)
{
    if ((num >= 0) && (num < Number))
        return standard_strings[num];
    else
        return 0;
}
