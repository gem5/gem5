/*
 * Copyright (c) 2021 ARM Limited
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

#include <mem/ruby/structures/MN_TBETable.hh>

namespace gem5
{

namespace ruby
{

namespace CHI
{

// Based on the current set of TBEs, choose a new "distributor"
// Can return null -> no distributor
MiscNode_TBE*
MN_TBETable::chooseNewDistributor()
{
    // Run over the current TBEs, gather information
    std::vector<MiscNode_TBE*> ready_sync_tbes;
    std::vector<MiscNode_TBE*> ready_nonsync_tbes;
    std::vector<MiscNode_TBE*> potential_sync_dependency_tbes;
    bool has_waiting_sync = false;
    int waiting_count = 0;
    for (auto& keyValuePair : m_map) {
        MiscNode_TBE& tbe = keyValuePair.second;

        switch (tbe.getstate()) {
            case MiscNode_State_DvmSync_Distributing:
            case MiscNode_State_DvmNonSync_Distributing:
                // If something is still distributing, just return it
                return &tbe;
            case MiscNode_State_DvmSync_ReadyToDist:
                ready_sync_tbes.push_back(&tbe);
                break;
            case MiscNode_State_DvmNonSync_ReadyToDist:
                ready_nonsync_tbes.push_back(&tbe);
                // Sync ops can potentially depend on not-executed NonSync ops
                potential_sync_dependency_tbes.push_back(&tbe);
                break;
            case MiscNode_State_DvmSync_Waiting:
                has_waiting_sync = true;
                waiting_count++;
                break;
            case MiscNode_State_DvmNonSync_Waiting:
                waiting_count++;
                // Sync ops can potentially depend on not-finished NonSync ops
                potential_sync_dependency_tbes.push_back(&tbe);
                break;
            default:
                break;
        }
    }

    // At most ~4 pending snoops at the RN-F
    // => for safety we only allow 4 ops waiting + distributing at a time
    // => if 4 are waiting currently, don't start distributing another one
    assert(waiting_count <= 4);
    if (waiting_count == 4) {
        return nullptr;
    }

    // If there's a waiting Sync op, don't allow other Sync ops to start.
    if (has_waiting_sync) {
        ready_sync_tbes.clear();
    }

    // We need to handle NonSync -> Sync dependencies
    // If we send CompDBIDResp for a Non-Sync that hasn't started,
    // the RN-F can send a dependent Sync immediately afterwards.
    // The Non-Sync must receive all responses before the Sync starts.
    // => ignore Syncs which arrive after unfinished NonSyncs
    auto hasNonSyncDependency = [&](const MiscNode_TBE* sync_tbe) {
        for (const auto* potential_dep : potential_sync_dependency_tbes) {
            if (sync_tbe->gettimestamp() > potential_dep->gettimestamp() &&
                sync_tbe->getrequestor() == potential_dep->getrequestor()) {
                // A NonSync from the same machine arrived before us
                // => we have a dependency
                return true;
            }
        }
        return false;
    };
    // Erase-remove idiom to remove elements at arbitrary indices
    // https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom
    // This calls an O(n) function n times = O(n^2) worst case.
    // TODO this should be improved if n grows > 16
    ready_sync_tbes.erase(
        std::remove_if(ready_sync_tbes.begin(), ready_sync_tbes.end(),
                       hasNonSyncDependency),
        ready_sync_tbes.end()
    );

    // TODO shouldn't use age?

    // Extend ready_nonsync_tbes with the contents of ready_sync_tbes
    ready_nonsync_tbes.insert(ready_nonsync_tbes.end(),
                              ready_sync_tbes.begin(), ready_sync_tbes.end());

    // Check if no candidates
    if (ready_nonsync_tbes.empty())
        return nullptr;

    // Otherwise select the minimum timestamp = oldest element
    auto it = std::min_element(
        ready_nonsync_tbes.begin(), ready_nonsync_tbes.end(),
        [](const MiscNode_TBE* a, const MiscNode_TBE* b) {
            return a->gettimestamp() - b->gettimestamp();
        }
    );
    assert(it != ready_nonsync_tbes.end());
    return *it;
}

} // namespace CHI

} // namespace ruby

} // namespace gem5
