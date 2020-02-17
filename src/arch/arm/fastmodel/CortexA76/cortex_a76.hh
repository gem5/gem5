/*
 * Copyright 2019 Google, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_ARM_FASTMODEL_CORTEXA76_CORETEX_A76_HH__
#define __ARCH_ARM_FASTMODEL_CORTEXA76_CORETEX_A76_HH__

#include "arch/arm/fastmodel/CortexA76/thread_context.hh"
#include "arch/arm/fastmodel/amba_ports.hh"
#include "arch/arm/fastmodel/iris/cpu.hh"
#include "params/FastModelCortexA76.hh"
#include "params/FastModelCortexA76Cluster.hh"
#include "scx/scx.h"
#include "sim/port.hh"
#include "systemc/ext/core/sc_module.hh"

class BaseCPU;

namespace FastModel
{

// The fast model exports a class called scx_evs_CortexA76x1 which represents
// the subsystem described in LISA+. This class specializes it to export gem5
// ports and interface with its peer gem5 CPU. The gem5 CPU inherits from the
// gem5 BaseCPU class and implements its API, while this class actually does
// the work.
class CortexA76Cluster;

class CortexA76 : public Iris::CPU<CortexA76TC>
{
  protected:
    typedef FastModelCortexA76Params Params;
    typedef Iris::CPU<CortexA76TC> Base;
    const Params &_params;

    CortexA76Cluster *cluster = nullptr;
    int num = 0;

    const Params &params() { return _params; }

  public:
    CortexA76(Params &p) : Base(&p, scx::scx_get_iris_connection_interface()),
        _params(p)
    {}

    void
    clockPeriodUpdated() override
    {
        Base::clockPeriodUpdated();

        // FIXME(b/139447397): this is a workaround since CNTFRQ_EL0 should not
        // be modified after clock is changed in real hardwares. Remove or
        // modify this after a more reasonable solution is found.
        for (auto *tc : threadContexts) {
            tc->setMiscRegNoEffect(ArmISA::MISCREG_CNTFRQ_EL0, frequency());
        }
    }

    void initState() override;

    template <class T>
    void set_evs_param(const std::string &n, T val);

    void setCluster(CortexA76Cluster *_cluster, int _num);

    Port &getPort(const std::string &if_name,
            PortID idx=InvalidPortID) override;
};

class CortexA76Cluster : public SimObject
{
  private:
    typedef FastModelCortexA76ClusterParams Params;
    const Params &_params;

    std::vector<CortexA76 *> cores;
    sc_core::sc_module *evs;

  public:
    template <class T>
    void
    set_evs_param(const std::string &n, T val)
    {
        scx::scx_set_parameter(evs->name() + std::string(".") + n, val);
    }

    CortexA76 *getCore(int num) { return cores.at(num); }
    sc_core::sc_module *getEvs() { return evs; }

    CortexA76Cluster(Params &p);
    const Params &params() { return _params; }

    Port &getPort(const std::string &if_name,
            PortID idx=InvalidPortID) override;
};

template <class T>
inline void
CortexA76::set_evs_param(const std::string &n, T val)
{
    for (auto &path: params().thread_paths)
        cluster->set_evs_param(path + "." + n, val);
}

} // namespace FastModel

#endif // __ARCH_ARM_FASTMODEL_CORTEXA76_CORETEX_A76_HH__
