#include "sim/power/power_model_pyfunc.hh"

#include "base/trace.hh"

namespace gem5
{

PowerModelPyFunc::PowerModelPyFunc(const Params &p)
           :  PowerModelState(p), dyn(p.dyn), st(p.st)
        {
           // Bind PyFunc parameters into functions to be called in this SimObj

           dyn_func =
                   pybind11::reinterpret_borrow<pybind11::function>(dyn);
           st_func =
                   pybind11::reinterpret_borrow<pybind11::function>(st);
        }
} // namespace gem5
