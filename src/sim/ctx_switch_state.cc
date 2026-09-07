#include "sim/ctx_switch_state.hh"
namespace gem5 {
bool inContextSwitch = false;
uint64_t ctxL1dMisses = 0;
uint64_t ctxL2Misses  = 0;
uint64_t ctxDtlbMisses = 0;
uint64_t ctxItlbMisses = 0;
uint64_t ctxL1dAccesses = 0;
uint64_t ctxL2Accesses  = 0;
uint64_t ctxDtlbAccesses = 0;
uint64_t ctxItlbAccesses = 0;
uint64_t ctxInstsAtBegin = 0;
uint64_t ctxInsts = 0;
uint64_t ctxL1iMisses = 0;
uint64_t ctxL1iAccesses = 0;

void ctxRecordCacheMiss(const std::string& cacheName)
{
    if (!inContextSwitch) return;
    if (cacheName.find("dcache") != std::string::npos ||
        cacheName.find(".l1d")  != std::string::npos)
        ctxL1dMisses++;
    else if (cacheName.find(".l2") != std::string::npos)
        ctxL2Misses++;
    else if (cacheName.find("icache") != std::string::npos ||
         cacheName.find(".l1i")  != std::string::npos)
    	 ctxL1iMisses++;
}
void ctxRecordCacheAccess(const std::string& cacheName)
{
    if (!inContextSwitch) return;
    if (cacheName.find("dcache") != std::string::npos ||
        cacheName.find(".l1d")  != std::string::npos)
        ctxL1dAccesses++;
    else if (cacheName.find(".l2") != std::string::npos)
        ctxL2Accesses++;
    else if (cacheName.find("icache") != std::string::npos ||
         cacheName.find(".l1i")  != std::string::npos)
	    ctxL1iAccesses++;
}
void ctxSwitchBegin()
{
    
    ctxL1dMisses = 0;  ctxL2Misses = 0;
    ctxDtlbMisses = 0; ctxItlbMisses = 0;
    ctxL1dAccesses = 0;  ctxL2Accesses = 0;
    ctxDtlbAccesses = 0; ctxItlbAccesses = 0;
    ctxInsts = 0;
    ctxL1iMisses = 0;
    ctxL1iAccesses = 0;
    inContextSwitch = true;
}
void ctxSwitchEnd()
{
    inContextSwitch = false;
}
} // namespace gem5
