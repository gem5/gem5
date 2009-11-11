
#include <arch/arm/utility.hh>
#include <cpu/thread_context.hh>


namespace ArmISA {

void
initCPU(ThreadContext *tc, int cpuId)
{
    // Reset CP15?? What does that mean -- ali
    
    // FPEXC.EN = 0
    
    static Fault reset = new Reset();
    if (cpuId == 0)
        reset->invoke(tc);
}

