
#include "cpu/beta_cpu/alpha_impl.hh"
#include "cpu/beta_cpu/alpha_full_cpu_impl.hh"
#include "cpu/beta_cpu/alpha_dyn_inst.hh"

// Force instantiation of AlphaFullCPU for all the implemntations that are
// needed.  Consider merging this and alpha_dyn_inst.cc, and maybe all
// classes that depend on a certain impl, into one file (alpha_impl.cc?).
template class AlphaFullCPU<AlphaSimpleImpl>;
