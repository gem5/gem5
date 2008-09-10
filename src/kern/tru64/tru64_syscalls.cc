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
 * Authors: Nathan Binkert
 */

#include "kern/tru64/tru64_syscalls.hh"

namespace {
    const char *
    standard_strings[SystemCalls<Tru64>::StandardNumber] = {
        "syscall",                      // 0
        "exit",                         // 1
        "fork",                         // 2
        "read",                         // 3
        "write",                        // 4
        "old_open",                     // 5
        "close",                        // 6
        "wait4",                        // 7
        "old_creat",                    // 8
        "link",                         // 9

        "unlink",                       // 10
        "execv",                        // 11
        "chdir",                        // 12
        "fchdir",                       // 13
        "mknod",                        // 14
        "chmod",                        // 15
        "chown",                        // 16
        "obreak",                       // 17
        "pre_F64_getfsstat",            // 18
        "lseek",                        // 19

        "getpid",                       // 20
        "mount",                        // 21
        "unmount",                      // 22
        "setuid",                       // 23
        "getuid",                       // 24
        "exec_with_loader",             // 25
        "ptrace",                       // 26
        "recvmsg",                      // 27
        "sendmsg",                      // 28
        "recvfrom",                     // 29

        "accept",                       // 30
        "getpeername",                  // 31
        "getsockname",                  // 32
        "access",                       // 33
        "chflags",                      // 34
        "fchflags",                     // 35
        "sync",                         // 36
        "kill",                         // 37
        "old_stat",                     // 38
        "setpgid",                      // 39

        "old_lstat",                    // 40
        "dup",                          // 41
        "pipe",                         // 42
        "set_program_attributes",       // 43
        "profil",                       // 44
        "open",                         // 45
        "obsolete_osigaction",          // 46
        "getgid",                       // 47
        "sigprocmask",                  // 48
        "getlogin",                     // 49

        "setlogin",                     // 50
        "acct",                         // 51
        "sigpending",                   // 52
        "classcntl",                    // 53
        "ioctl",                        // 54
        "reboot",                       // 55
        "revoke",                       // 56
        "symlink",                      // 57
        "readlink",                     // 58
        "execve",                       // 59

        "umask",                        // 60
        "chroot",                       // 61
        "old_fstat",                    // 62
        "getpgrp",                      // 63
        "getpagesize",                  // 64
        "mremap",                       // 65
        "vfork",                        // 66
        "pre_F64_stat",                 // 67
        "pre_F64_lstat",                // 68
        "sbrk",                         // 69

        "sstk",                         // 70
        "mmap",                         // 71
        "ovadvise",                     // 72
        "munmap",                       // 73
        "mprotect",                     // 74
        "madvise",                      // 75
        "old_vhangup",                  // 76
        "kmodcall",                     // 77
        "mincore",                      // 78
        "getgroups",                    // 79

        "setgroups",                    // 80
        "old_getpgrp",                  // 81
        "setpgrp",                      // 82
        "setitimer",                    // 83
        "old_wait",                     // 84
        "table",                        // 85
        "getitimer",                    // 86
        "gethostname",                  // 87
        "sethostname",                  // 88
        "getdtablesize",                // 89

        "dup2",                         // 90
        "pre_F64_fstat",                // 91
        "fcntl",                        // 92
        "select",                       // 93
        "poll",                         // 94
        "fsync",                        // 95
        "setpriority",                  // 96
        "socket",                       // 97
        "connect",                      // 98
        "old_accept",                   // 99

        "getpriority",                  // 100
        "old_send",                     // 101
        "old_recv",                     // 102
        "sigreturn",                    // 103
        "bind",                         // 104
        "setsockopt",                   // 105
        "listen",                       // 106
        "plock",                        // 107
        "old_sigvec",                   // 108
        "old_sigblock",                 // 109

        "old_sigsetmask",               // 110
        "sigsuspend",                   // 111
        "sigstack",                     // 112
        "old_recvmsg",                  // 113
        "old_sendmsg",                  // 114
        "obsolete_vtrcae",              // 115
        "gettimeofday",                 // 116
        "getrusage",                    // 117
        "getsockopt",                   // 118
        "numa_syscalls",                // 119

        "readv",                        // 120
        "writev",                       // 121
        "settimeofday",                 // 122
        "fchown",                       // 123
        "fchmod",                       // 124
        "old_recvfrom",                 // 125
        "setreuid",                     // 126
        "setregid",                     // 127
        "rename",                       // 128
        "truncate",                     // 129

        "ftruncate",                    // 130
        "flock",                        // 131
        "setgid",                       // 132
        "sendto",                       // 133
        "shutdown",                     // 134
        "socketpair",                   // 135
        "mkdir",                        // 136
        "rmdir",                        // 137
        "utimes",                       // 138
        "obsolete_42_sigreturn",        // 139

        "adjtime",                      // 140
        "old_getpeername",              // 141
        "gethostid",                    // 142
        "sethostid",                    // 143
        "getrlimit",                    // 144
        "setrlimit",                    // 145
        "old_killpg",                   // 146
        "setsid",                       // 147
        "quotactl",                     // 148
        "oldquota",                     // 149

        "old_getsockname",              // 150
        "pread",                        // 151
        "pwrite",                       // 152
        "pid_block",                    // 153
        "pid_unblock",                  // 154
        "signal_urti",                  // 155
        "sigaction",                    // 156
        "sigwaitprim",                  // 157
        "nfssvc",                       // 158
        "getdirentries",                // 159

        "pre_F64_statfs",               // 160
        "pre_F64_fstatfs",              // 161
        0,                              // 162
        "async_daemon",                 // 163
        "getfh",                        // 164
        "getdomainname",                // 165
        "setdomainname",                // 166
        0,                              // 167
        0,                              // 168
        "exportfs",                     // 169

        0,                              // 170
        0,                              // 171
        0,                              // 172
        0,                              // 173
        0,                              // 174
        0,                              // 175
        0,                              // 176
        0,                              // 177
        0,                              // 178
        0,                              // 179

        0,                              // 180
        "alt_plock",                    // 181
        0,                              // 182
        0,                              // 183
        "getmnt",                       // 184
        0,                              // 185
        0,                              // 186
        "alt_sigpending",               // 187
        "alt_setsid",                   // 188
        0,                              // 189

        0,                              // 190
        0,                              // 191
        0,                              // 192
        0,                              // 193
        0,                              // 194
        0,                              // 195
        0,                              // 196
        0,                              // 197
        0,                              // 198
        "swapon",                       // 199

        "msgctl",                       // 200
        "msgget",                       // 201
        "msgrcv",                       // 202
        "msgsnd",                       // 203
        "semctl",                       // 204
        "semget",                       // 205
        "semop",                        // 206
        "uname",                        // 207
        "lchown",                       // 208
        "shmat",                        // 209

        "shmctl",                       // 210
        "shmdt",                        // 211
        "shmget",                       // 212
        "mvalid",                       // 213
        "getaddressconf",               // 214
        "msleep",                       // 215
        "mwakeup",                      // 216
        "msync",                        // 217
        "signal",                       // 218
        "utc_gettime",                  // 219

        "utc_adjtime",                  // 220
        0,                              // 221
        "security",                     // 222
        "kloadcall",                    // 223
        "stat",                         // 224
        "lstat",                        // 225
        "fstat",                        // 226
        "statfs",                       // 227
        "fstatfs",                      // 228
        "getfsstat",                    // 229

        "gettimeofday64",               // 230
        "settimeofday64",               // 231
        0,                              // 232
        "getpgid",                      // 233
        "getsid",                       // 234
        "sigaltstack",                  // 235
        "waitid",                       // 236
        "priocntlset",                  // 237
        "sigsendset",                   // 238
        "set_speculative",              // 239

        "msfs_syscall",                 // 240
        "sysinfo",                      // 241
        "uadmin",                       // 242
        "fuser",                        // 243
        "proplist_syscall",             // 244
        "ntp_adjtime",                  // 245
        "ntp_gettime",                  // 246
        "pathconf",                     // 247
        "fpathconf",                    // 248
        "sync2",                        // 249

        "uswitch",                      // 250
        "usleep_thread",                // 251
        "audcntl",                      // 252
        "audgen",                       // 253
        "sysfs",                        // 254
        "subsys_info",                  // 255
        "getsysinfo",                   // 256
        "setsysinfo",                   // 257
        "afs_syscall",                  // 258
        "swapctl",                      // 259

        "memcntl",                      // 260
        "fdatasync",                    // 261
        "oflock",                       // 262
        "_F64_readv",                   // 263
        "_F64_writev",                  // 264
        "cdslxlate",                    // 265
        "sendfile",                     // 266
    };

    const char *
    mach_strings[SystemCalls<Tru64>::MachNumber] = {
        0,                                      // 0
        0,                                      // 1
        0,                                      // 2
        0,                                      // 3
        0,                                      // 4
        0,                                      // 5
        0,                                      // 6
        0,                                      // 7
        0,                                      // 8
        0,                                      // 9

        "task_self",                            // 10
        "thread_reply",                         // 11
        "task_notify",                          // 12
        "thread_self",                          // 13
        0,                                      // 14
        0,                                      // 15
        0,                                      // 16
        0,                                      // 17
        0,                                      // 18
        0,                                      // 19

        "msg_send_trap",                        // 20
        "msg_receive_trap",                     // 21
        "msg_rpc_trap",                         // 22
        0,                                      // 23
        "nxm_block",                            // 24
        "nxm_unblock",                          // 25
        0,                                      // 26
        0,                                      // 27
        0,                                      // 28
        "nxm_thread_destroy",                   // 29

        "lw_wire",                              // 30
        "lw_unwire",                            // 31
        "nxm_thread_create",                    // 32
        "nxm_task_init",                        // 33
        0,                                      // 34
        "nxm_idle",                             // 35
        "nxm_wakeup_idle",                      // 36
        "nxm_set_pthid",                        // 37
        "nxm_thread_kill",                      // 38
        "nxm_thread_block",                     // 39

        "nxm_thread_wakeup",                    // 40
        "init_process",                         // 41
        "nxm_get_binding",                      // 42
        "map_fd",                               // 43
        "nxm_resched",                          // 44
        "nxm_set_cancel",                       // 45
        "nxm_set_binding",                      // 46
        "stack_create",                         // 47
        "nxm_get_state",                        // 48
        "nxm_thread_suspend",                   // 49

        "nxm_thread_resume",                    // 50
        "nxm_signal_check",                     // 51
        "htg_unix_syscall",                     // 52
        0,                                      // 53
        0,                                      // 54
        "host_self",                            // 55
        "host_priv_self",                       // 56
        0,                                      // 57
        0,                                      // 58
        "swtch_pri",                            // 59

        "swtch",                                // 60
        "thread_switch",                        // 61
        "semop_fast",                           // 62
        "nxm_pshared_init",                     // 63
        "nxm_pshared_block",                    // 64
        "nxm_pshared_unblock",                  // 65
        "nxm_pshared_destroy",                  // 66
        "nxm_swtch_pri",                        // 67
        "lw_syscall",                           // 68
        0,                                      // 69

        "mach_sctimes_0",                       // 70
        "mach_sctimes_1",                       // 71
        "mach_sctimes_2",                       // 72
        "mach_sctimes_3",                       // 73
        "mach_sctimes_4",                       // 74
        "mach_sctimes_5",                       // 75
        "mach_sctimes_6",                       // 76
        "mach_sctimes_7",                       // 77
        "mach_sctimes_8",                       // 78
        "mach_sctimes_9",                       // 79

        "mach_sctimes_10",                      // 80
        "mach_sctimes_11",                      // 81
        "mach_sctimes_port_alloc_dealloc",      // 82
    };
}

const char *
SystemCalls<Tru64>::name(int num)
{
    if (num >= Number)
        return 0;
    else if (num >= StandardNumber)
        return mach_strings[num - StandardNumber];
    else if (num >= 0)
        return standard_strings[num];
    else if (num > -MachNumber)
        return mach_strings[-num];
    else
        return 0;
}
