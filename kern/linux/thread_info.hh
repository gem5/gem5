#ifndef __ALPHA_THREAD_INFO_H__
#define __ALPHA_THREAD_INFO_H__

#include "kern/linux/hwrpb.hh"
#include "kern/linux/aligned.hh"

namespace Linux {
    struct thread_info {
        struct pcb_struct       pcb;
        Addr_a                  task;
    };
}
#endif /* __ALPHA_THREAD_INFO_H__ */
