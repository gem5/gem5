
#include "cpu/beta_cpu/alpha_dyn_inst_impl.hh"
#include "cpu/beta_cpu/alpha_impl.hh"

// Force instantiation of AlphaDynInst for all the implementations that
// are needed.
template class AlphaDynInst<AlphaSimpleImpl>;
