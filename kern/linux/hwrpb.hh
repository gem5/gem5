#ifndef __ALPHA_HWRPB_H__
#define __ALPHA_HWRPB_H__

#include "kern/linux/aligned.hh"

namespace Linux {
    struct pcb_struct {
        uint64_ta ksp;
        uint64_ta usp;
        uint64_ta ptbr;
        uint32_t pcc;
        uint32_t asn;
        uint64_ta unique;
        uint64_ta flags;
        uint64_ta res1, res2;
    };
}
#endif /* __ALPHA_HWRPB_H */
