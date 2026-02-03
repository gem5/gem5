#ifndef __SIM_POWERMODEL_FUNC_PM_HH__
#define __SIM_POWERMODEL_FUNC_PM_HH__

#include "params/PowerModelPyFunc.hh"
#include "python/pybind11/pybind.hh"
#include "sim/power/power_model.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PowerModelPyFunc : public PowerModelState
{
   private:
     pybind11::object dyn;
     pybind11::object st;
     pybind11::function st_func;
     pybind11::function dyn_func;

   public:
     PARAMS(PowerModelPyFunc);
     PowerModelPyFunc(const Params &p);
     double getDynamicPower() const override {
             pybind11::object result_py = dyn_func();
             return result_py.cast<double>();
     }
     double getStaticPower() const override {
             pybind11::object result_py = st_func();
             return result_py.cast<double>();
     }
};

} // namespace gem5

#endif
