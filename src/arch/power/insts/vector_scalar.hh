#ifndef __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__
#define __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__

#include "arch/power/regs/float.hh"
#include "arch/power/regs/int.hh"
#include "arch/power/regs/vec.hh"
#include "cpu/thread_context.hh"

namespace gem5
{
namespace PowerISA
{

void mtvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx);
void mfvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx);

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__
