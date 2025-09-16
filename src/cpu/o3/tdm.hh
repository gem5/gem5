/*
 * Copyright (c) 2025 Technical University of Munich
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __CPU_O3_TDA_HH__
#define __CPU_O3_TDA_HH__

#include "base/statistics.hh"

#include "cpu/o3/commit.hh"
#include "cpu/o3/decode.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/o3/iew.hh"
#include "cpu/o3/rename.hh"

namespace gem5
{

namespace o3
{

class CPU;

struct TopDownStats : statistics::Group
{
    TopDownStats(CPU *cpu, Fetch *fetch, Rename *rename, Decode *decode,
                 IEW *iew, Commit *commit);

    struct TopDownL1 : statistics::Group
    {
        TopDownL1(CPU *cpu, Fetch *fetch, Rename *rename, Decode *decode,
                  IEW *iew, Commit *commit);
        statistics::Formula frontendBound;
        statistics::Formula badSpeculation;
        statistics::Formula backendBound;
        statistics::Formula retiring;
    } topDownL1;

    struct TopDownFrontendBoundL2 : statistics::Group
    {
        TopDownFrontendBoundL2(CPU *cpu, Fetch *fetch);
        statistics::Formula fetchLatency;
        statistics::Formula fetchBandwidth;
    } topDownFbL2;

    struct TopDownBadSpeculationL2 : statistics::Group
    {
        TopDownBadSpeculationL2(CPU *cpu, Decode *decode, IEW *iew);
        statistics::Formula branchMissPredicts;
        statistics::Formula machineClears;
    } topDownBsL2;

    struct TopDownBackendBoundL2 : statistics::Group
    {
        TopDownBackendBoundL2(CPU *cpu, Rename *rename, IEW *iew);
        statistics::Formula executionStalls;
        statistics::Formula memoryBound;
        statistics::Formula coreBound;
    } topDownBbL2;

    struct TopDownBackendBoundL3 : statistics::Group
    {
        TopDownBackendBoundL3(CPU *cpu, Rename *rename, IEW *iew);
        statistics::Formula l1Bound;
        statistics::Formula l2Bound;
        statistics::Formula l3Bound;
        statistics::Formula extMemBound;
        statistics::Formula storeBound;
    } topDownBbMem;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_TDA_HH__
