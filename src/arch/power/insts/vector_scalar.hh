#ifndef __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__
#define __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__

#include "cpu/thread_context.hh"

namespace gem5
{
namespace PowerISA
{

void mtvsrd(ThreadContext *tc, int xt, int ra);
void mfvsrd(ThreadContext *tc, int rt, int xs);

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__
