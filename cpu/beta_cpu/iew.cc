
#include "cpu/beta_cpu/alpha_dyn_inst.hh"
#include "cpu/beta_cpu/alpha_impl.hh"
#include "cpu/beta_cpu/iew_impl.hh"
#include "cpu/beta_cpu/inst_queue.hh"

template class SimpleIEW<AlphaSimpleImpl, AlphaSimpleImpl::CPUPol::IQ>;
