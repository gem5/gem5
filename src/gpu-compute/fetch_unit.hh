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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 *
 * Author: Brad Beckmann, Sooraj Puthoor
 */

#ifndef __FETCH_UNIT_HH__
#define __FETCH_UNIT_HH__

#include <string>
#include <utility>
#include <vector>

#include "arch/gpu_decoder.hh"
#include "base/statistics.hh"
#include "config/the_gpu_isa.hh"
#include "gpu-compute/scheduler.hh"
#include "mem/packet.hh"

class ComputeUnit;
class Wavefront;

class FetchUnit
{
  public:
    FetchUnit(const ComputeUnitParams* params);
    ~FetchUnit();
    void init(ComputeUnit *cu);
    void exec();
    void bindWaveList(std::vector<Wavefront*> *list);
    void initiateFetch(Wavefront *wavefront);
    void fetch(PacketPtr pkt, Wavefront *wavefront);
    void processFetchReturn(PacketPtr pkt);
    static uint32_t globalFetchUnitID;

  private:
    bool timingSim;
    ComputeUnit *computeUnit;
    TheGpuISA::Decoder decoder;

    // Fetch scheduler; Selects one wave from
    // the fetch queue for instruction fetching.
    // The selection is made according to
    // a scheduling policy
    Scheduler fetchScheduler;

    // Stores the list of waves that are
    // ready to be fetched this cycle
    std::vector<Wavefront*> fetchQueue;

    // Stores the fetch status of all waves dispatched to this SIMD.
    // TRUE implies the wave is ready to fetch and is already
    // moved to fetchQueue
    std::vector<std::pair<Wavefront*, bool>> fetchStatusQueue;

    // Pointer to list of waves dispatched on to this SIMD unit
    std::vector<Wavefront*> *waveList;
};

#endif // __FETCH_UNIT_HH__
