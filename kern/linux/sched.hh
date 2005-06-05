#ifndef __LINUX_SCHED_H__
#define __LINUX_SCHED_H__

#include "targetarch/isa_traits.hh"
#include "kern/linux/aligned.hh"

namespace Linux {
    struct task_struct {
        uint8_t junk1[0xf4];
        int32_t pid;
        uint8_t junk2[0x190];
        uint64_ta start_time;
        uint8_t junk3[0x5c];
        char comm[16];
    };


}

#endif
