/*
 * Copyright (c) 2016, 2017 Advanced Micro Devices, Inc.
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

#ifndef __REGISTER_MANAGER_HH__
#define __REGISTER_MANAGER_HH__

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gpu-compute/pool_manager.hh"
#include "gpu-compute/register_manager_policy.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

namespace gem5
{

class ComputeUnit;
class Wavefront;

struct RegisterManagerParams;

/*
 * Rename stage.
 */
class RegisterManager : public SimObject
{
  public:
    RegisterManager(const RegisterManagerParams &params);
    ~RegisterManager();
    void setParent(ComputeUnit *cu);
    void exec();

    // lookup virtual to physical register translation
    int mapVgpr(Wavefront *w, int vgprIndex);
    int mapSgpr(Wavefront *w, int sgprIndex);

    // check if we can allocate registers
    bool canAllocateVgprs(int simdId, int nWfs, int demandPerWf);
    bool canAllocateSgprs(int simdId, int nWfs, int demandPerWf);

    // allocate registers
    void allocateRegisters(Wavefront *w, int vectorDemand, int scalarDemand);

    // free all registers used by the WF
    void freeRegisters(Wavefront *w);

    std::vector<PoolManager *> srfPoolMgrs;
    std::vector<PoolManager *> vrfPoolMgrs;

  private:
    RegisterManagerPolicy *policy;

    ComputeUnit *computeUnit;

    std::string _name;
};

} // namespace gem5

#endif // __REGISTER_MANAGER_HH__
