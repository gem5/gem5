
#include "cpu/beta_cpu/alpha_dyn_inst.hh"
#include "cpu/beta_cpu/alpha_impl.hh"
#include "cpu/beta_cpu/store_set.hh"
#include "cpu/beta_cpu/mem_dep_unit_impl.hh"

// Force instantation of memory dependency unit using store sets and
// AlphaSimpleImpl.
template class MemDepUnit<StoreSet, AlphaSimpleImpl>;
