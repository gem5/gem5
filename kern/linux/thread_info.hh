#ifndef __ALPHA_THREAD_INFO_H__
#define __ALPHA_THREAD_INFO_H__

#include "kern/linux/hwrpb.hh"
#include "kern/linux/aligned.hh"

namespace Linux {
    struct thread_info {
        struct pcb_struct       pcb;            /* palcode state */

        Addr_a                  task;          /* main task structure */
        uint32_t                flags;          /* low level flags */
        uint32_t                ieee_state;     /* see fpu.h */

        Addr_a                   exec_domain;   /* execution domain */
        uint64_ta                addr_limit;     /* thread address space */
        int64_ta                 cpu;            /* current CPU */
        int32_t                  preempt_count;  /* 0 => preemptable, <0 => BUG */

        int32_t                  bpt_nsaved;
        uint64_ta                bpt_addr[2];    /* breakpoint handling  */
        uint32_t                 bpt_insn[2];

        /*restart_block;*/
    };
}
#endif /* __ALPHA_THREAD_INFO_H__ */
