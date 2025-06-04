#include "arch/power/insts/misc.hh"
#include "arch/power/isa.hh"
#include "arch/power/regs/float.hh"  // Defines floatRegClass
#include "arch/power/regs/int.hh"  // Defines intRegClass
#include "arch/power/regs/vec.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "vector_scalar.hh"

using namespace gem5;

// Define 128-bit unsigned int if not already available
typedef __uint128_t uint128_t;

// Register classes are in PowerISA namespace (from src/arch/power/regs/)
using namespace PowerISA;

void mtvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx) {
    // Compute effective VSX register index (0-63)
    const RegIndex vsr_idx = (tx_sx << 5) | t_s;

    // Get integer register value
    uint64_t int_val = tc->getReg(intRegClass[ra_vsx]);

    // For VSR0-VSR31 (vector registers)
    if (vsr_idx < 32) {
        // Read existing VSR value (preserve upper 64 bits)
        uint128_t vsr_val = tc->getReg(vecRegClass[vsr_idx]);
        vsr_val = (vsr_val & ((uint128_t)0xFFFFFFFFFFFFFFFFULL << 64)) |
        int_val;
        tc->setReg(vecRegClass[vsr_idx], vsr_val);
    }
    // For VSR32-VSR63 (map to FPRs)
    else {
        tc->setReg(floatRegClass[vsr_idx - 32], int_val);
    }
}

void mfvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx) {
    // Compute effective VSX register index (0-63)
    const RegIndex vsr_idx = (tx_sx << 5) | t_s;
    uint64_t val;

    // For VSR0-VSR31 (vector registers)
    if (vsr_idx < 32) {
        val = static_cast<uint64_t>(tc->getReg(vecRegClass[vsr_idx]));
    }
    // For VSR32-VSR63 (map to FPRs)
    else {
        val = tc->getReg(floatRegClass[vsr_idx - 32]);
    }

    tc->setReg(intRegClass[ra_vsx], val);
}
