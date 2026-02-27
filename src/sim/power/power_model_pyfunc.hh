/*
 * Copyright (c) 2026 University of Wisconsin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
             assert(pybind11::isinstance<pybind11::float_>(result_py));
             return result_py.cast<double>();
     }
     double getStaticPower() const override {
             pybind11::object result_py = st_func();
             assert(pybind11::isinstance<pybind11::float_>(result_py));
             return result_py.cast<double>();
     }
};

} // namespace gem5

#endif
