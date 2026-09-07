#ifndef __SIM_CTX_SWITCH_STATE_HH__
#define __SIM_CTX_SWITCH_STATE_HH__
#include <cstdint>
#include <string>
namespace gem5 {
extern bool inContextSwitch;
/* misses (existing) */
extern uint64_t ctxL1dMisses;
extern uint64_t ctxL2Misses;
extern uint64_t ctxDtlbMisses;
extern uint64_t ctxItlbMisses;
/* accesses (new) */
extern uint64_t ctxL1dAccesses;
extern uint64_t ctxL2Accesses;
extern uint64_t ctxDtlbAccesses;
extern uint64_t ctxItlbAccesses;
/* instructions (new) */
extern uint64_t ctxInstsAtBegin;
extern uint64_t ctxInsts;
extern uint64_t ctxL1iMisses;
extern uint64_t ctxL1iAccesses;

void ctxSwitchBegin();
void ctxSwitchEnd();
void ctxRecordCacheMiss(const std::string& cacheName);
void ctxRecordCacheAccess(const std::string& cacheName);
} // namespace gem5
#endif
