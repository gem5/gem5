#ifndef __LINUX_SCHED_H__
#define __LINUX_SCHED_H__

#include "targetarch/isa_traits.hh"
#include "kern/linux/atomic.hh"
#include "kern/linux/list.hh"
#include "kern/linux/wait.hh"
#include "kern/linux/timer.hh"
#include "kern/linux/pid.hh"
#include "kern/linux/aligned.hh"

namespace Linux {

    struct rlimit {
        uint64_ta rlim_cur;
        uint64_ta rlim_max;
    };

    const uint32_t RLIM_NLIMITS  = 11;

    struct task_struct {
        int64_ta state;    /* -1 unrunnable, 0 runnable, >0 stopped */
        Addr_a    thread_info;
        atomic_t usage;

        uint64_ta flags;    /* per process flags, defined below */
        uint64_ta ptrace;

        int32_t lock_depth;        /* Lock depth */

        int32_t prio, static_prio;

        struct list_head run_list;
        Addr_a array;

        uint64_ta sleep_avg;
        int64_ta interactive_credit;
        uint64_ta timestamp;
        int32_t activated;

        uint64_ta policy;
        uint64_ta cpus_allowed;
        uint32_t time_slice, first_time_slice;

        struct list_head tasks;
        struct list_head ptrace_children;
        struct list_head ptrace_list;

        Addr_a mm, active_mm;

    /* task state */
        Addr_a binfmt;
        int32_t exit_code, exit_signal;
        int32_t pdeath_signal;  /*  The signal sent when the parent dies  */
        /* ??? */
        uint64_ta personality;
        int32_t did_exec:1;
        int32_t pid;
        int32_t __pgrp;        /* Accessed via process_group() */
        int32_t tty_old_pgrp;
        int32_t session;
        int32_t tgid;
        /* boolean value for session group leader */
        int32_t leader;
        /*
         * pointers to (original) parent process, youngest child, younger sibling,
         * older sibling, respectively.  (p->father can be replaced with
         * p->parent->pid)
         */
        Addr_a real_parent; /* real parent process (when being debugged) */
        Addr_a parent;    /* parent process */
        struct list_head children;    /* list of my children */
        struct list_head sibling;    /* linkage in my parent's children list */
        Addr_a group_leader;    /* threadgroup leader */

        /* PID/PID hash table linkage. */
        struct pid_link pids[PIDTYPE_MAX];

        wait_queue_head_t wait_chldexit;    /* for wait4() */
        Addr_a vfork_done;        /* for vfork() */
        Addr_a set_child_tid;        /* CLONE_CHILD_SETTID */
        Addr_a clear_child_tid;        /* CLONE_CHILD_CLEARTID */

        uint64_ta rt_priority;
        uint64_ta it_real_value, it_prof_value, it_virt_value;
        uint64_ta it_real_incr, it_prof_incr, it_virt_incr;
        struct timer_list real_timer;
        struct list_head posix_timers; /* POSIX.1b Interval Timers */
        uint64_ta utime, stime, cutime, cstime;
        uint64_ta nvcsw, nivcsw, cnvcsw, cnivcsw; /* context switch counts */
        uint64_ta start_time;
    /* mm fault and swap info: this can arguably be seen as either mm-specific or thread-specific */
        uint64_ta min_flt, maj_flt, nswap, cmin_flt, cmaj_flt, cnswap;
    /* process credentials */
        uint32_t uid,euid,suid,fsuid;
        uint32_t gid,egid,sgid,fsgid;
        Addr_a group_info;
        uint32_t   cap_effective, cap_inheritable, cap_permitted;
        int32_t keep_capabilities:1;
        Addr user;
    /* limits */
        struct rlimit rlim[RLIM_NLIMITS];
        uint16_t used_math;
        char comm[16];
    };


}

#endif
