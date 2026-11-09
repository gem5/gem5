// Owning, typed CPU facts for T/P integration; no O3 types in algorithm state.
#ifndef __MEM_CACHE_PREFETCH_TPC_CPU_EVENTS_HH__
#define __MEM_CACHE_PREFETCH_TPC_CPU_EVENTS_HH__
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "mem/request.hh"
namespace gem5::prefetch {
struct CPUIdentity {
    ContextID context = 0;
    InstSeqNum sequence = 0;
};
struct CPULoadEvent : CPUIdentity {
    Tick tick = 0;
    RequestPtr request;
    Addr address = 0; // original effective VA, not a split fragment
    unsigned size = 0, microPC = 0;
    bool rasValid = false;
    Addr ras = 0;
};
struct CPUBranchEvent : CPUIdentity {
    Tick tick = 0;
    Addr pc = 0, target = 0;
    bool taken = false;
};
struct CPUCompleteEvent : CPUIdentity {
    Tick tick = 0;
    bool success = false;
};
}
#endif
