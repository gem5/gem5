
#include "cpu/beta_cpu/alpha_dyn_inst.hh"
#include "cpu/beta_cpu/alpha_impl.hh"
#include "cpu/beta_cpu/inst_queue_impl.hh"

// Force instantiation of InstructionQueue.
template InstructionQueue<AlphaSimpleImpl>;

unsigned
InstructionQueue<AlphaSimpleImpl>::DependencyEntry::mem_alloc_counter = 0;
