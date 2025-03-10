#include "arch/power/insts/misc.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "vector_scalar.hh"

void mtvsrd(ThreadContext *tc, int xt, int ra) {
    // Ensure valid register indices
    assert(xt >= 0 && xt < NumVecRegs);
    assert(ra >= 0 && ra < NumIntRegs);

    // Read the integer register value from GPR[RA]
    uint64_t value = tc->getReg(RegId(IntRegClass, ra));

    // Read the current value of the VSR[XT] register (128-bit)
    uint128_t vsr_value = tc->getReg(RegId(VecRegClass, xt));

    // Mask to preserve the upper 64 bits of the VSR
    const uint128_t UPPER_MASK = (uint128_t)0xFFFFFFFFFFFFFFFFULL << 64;

    // Replace only the lower 64-bits of VSR with the new value from GPR
    vsr_value = (vsr_value & UPPER_MASK) | value;

    // Write the updated value back to VSR[XT]
    tc->setReg(RegId(VecRegClass, xt), vsr_value);
}


void mfvsrd(ThreadContext *tc, int rt, int xs) {
    // Ensure valid register indices
    assert(rt >= 0 && rt < NumIntRegs);
    assert(xs >= 0 && xs < NumVecRegs);

    // Read the current value of the VSR[XS] register (128-bit)
    uint128_t vsr_value = tc->getReg(RegId(VecRegClass, xs));

    // Extract only the lower 64 bits of the VSR[XS]
    uint64_t lower_64_bits = static_cast<uint64_t>(
    vsr_value & 0xFFFFFFFFFFFFFFFFULL
    );


    // Write the extracted lower 64 bits to GPR[RT]
    tc->setReg(RegId(IntRegClass, rt), lower_64_bits);
}
