// Immutable T2/P1 stream contract. Mutable SIT storage remains private to T2.
#ifndef __MEM_CACHE_PREFETCH_T2_STREAM_HH__
#define __MEM_CACHE_PREFETCH_T2_STREAM_HH__
#include "mem/cache/prefetch/instruction_state.hh"
namespace gem5::prefetch {
struct T2StreamHandle { uint64_t id = 0, generation = 0; };
struct T2StreamSnapshot {
    T2StreamHandle handle;
    InstructionSource instruction;
    ContextID context = 0;
    Addr pc = 0, modifiedPC = 0, ras = 0, lastAddress = 0;
    bool rasValid = false, pointerSource = false;
    uint64_t loopGeneration = 0, loopPeriod = 0;
    int64_t stride = 0;
    unsigned width = 0, microPC = 0, state = 0, baseDistance = 1, effectiveDistance = 1;
};
}
#endif
