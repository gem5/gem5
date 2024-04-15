/*
 * Copyright 2020 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_CORTEXR52_CORETEX_R52_HH__
#define __ARCH_ARM_FASTMODEL_CORTEXR52_CORETEX_R52_HH__

#include "arch/arm/fastmodel/CortexR52/thread_context.hh"
#include "arch/arm/fastmodel/amba_ports.hh"
#include "arch/arm/fastmodel/iris/cpu.hh"
#include "params/FastModelCortexR52.hh"
#include "params/FastModelCortexR52Cluster.hh"
#include "scx/scx.h"
#include "sim/port.hh"
#include "systemc/ext/core/sc_module.hh"

namespace gem5
{

class BaseCPU;

namespace fastmodel
{

// The fast model exports a class called scx_evs_CortexR52x1 which represents
// the subsystem described in LISA+. This class specializes it to export gem5
// ports and interface with its peer gem5 CPU. The gem5 CPU inherits from the
// gem5 BaseCPU class and implements its API, while this class actually does
// the work.
class CortexR52Cluster;

class CortexR52 : public Iris::CPU<CortexR52TC>
{
  protected:
    typedef Iris::CPU<CortexR52TC> Base;

    CortexR52Cluster *cluster = nullptr;
    int num = 0;

  public:
    PARAMS(FastModelCortexR52);

    CortexR52(const Params &p)
        : Base(p, scx::scx_get_iris_connection_interface())
    {}

    template <class T>
    void set_evs_param(const std::string &n, T val);

    void setCluster(CortexR52Cluster *_cluster, int _num);

    void setResetAddr(Addr addr, bool secure = false) override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

class CortexR52Cluster : public SimObject
{
  private:
    std::vector<CortexR52 *> cores;
    sc_core::sc_module *evs;

  public:
    template <class T>
    void
    set_evs_param(const std::string &n, T val)
    {
        scx::scx_set_parameter(evs->name() + std::string(".") + n, val);
    }

    CortexR52 *
    getCore(int num) const
    {
        return cores.at(num);
    }

    sc_core::sc_module *
    getEvs() const
    {
        return evs;
    }

    PARAMS(FastModelCortexR52Cluster);
    CortexR52Cluster(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

template <class T>
inline void
CortexR52::set_evs_param(const std::string &n, T val)
{
    for (auto &path : params().thread_paths)
        cluster->set_evs_param(path + "." + n, val);
}

} // namespace fastmodel
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_CORTEXR52_CORETEX_R52_HH__
