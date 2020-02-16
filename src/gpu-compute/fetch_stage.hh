/*
 * Copyright (c) 2014-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

#ifndef __FETCH_STAGE_HH__
#define __FETCH_STAGE_HH__

#include <string>
#include <vector>

#include "gpu-compute/fetch_unit.hh"

// Instruction fetch stage.
// All dispatched wavefronts for all SIMDS are analyzed for the
// need to fetch instructions. From the fetch eligible waves,
// one wave is selected from each SIMD and fetch is initiated
// for the selected waves.

class ComputeUnit;
class Wavefront;

class FetchStage
{
  public:
    FetchStage(const ComputeUnitParams* params);
    ~FetchStage();
    void init(ComputeUnit *cu);
    void exec();
    void processFetchReturn(PacketPtr pkt);
    void fetch(PacketPtr pkt, Wavefront *wave);

    // Stats related variables and methods
    std::string name() { return _name; }
    void regStats();
    Stats::Distribution instFetchInstReturned;

  private:
    uint32_t numSIMDs;
    ComputeUnit *computeUnit;

    // List of fetch units. A fetch unit is
    // instantiated per SIMD
    std::vector<FetchUnit> fetchUnit;
    std::string _name;
};

#endif // __FETCH_STAGE_HH__
