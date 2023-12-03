/*
 * Copyright (c) 2023 The University of Wisconsin
 *
 * All rights reserved.
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

#include "gpu-compute/register_file_cache.hh"

#include <sstream>
#include <string>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/GPURFC.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"
#include "params/RegisterFileCache.hh"

namespace gem5
{

RegisterFileCache::RegisterFileCache(const RegisterFileCacheParams &p)
    : SimObject(p), simdId(p.simd_id), _capacity(p.cache_size)
{
    fatal_if(simdId < 0, "Illegal SIMD id for rfc");
}

RegisterFileCache::~RegisterFileCache() {}

void
RegisterFileCache::setParent(ComputeUnit *_computeUnit)
{
    computeUnit = _computeUnit;
}

bool
RegisterFileCache::inRFC(int regIdx)
{
    return (lruHash.find(regIdx) != lruHash.end());
}

std::string
RegisterFileCache::dumpLL() const
{
    std::stringstream ss;
    ss << "lru_order: ";
    for (auto i = lruHead; i != nullptr; i = i->next) {
        if (i->prev == nullptr) {
            ss << "reg: " << i->regIdx << " ";
        } else {
            ss << "reg: " << i->regIdx << " (prev: " << i->prev->regIdx
               << ") ";
        }
        if (i->next != nullptr) {
            ss << " (next: " << i->next->regIdx << ") ";
        }
    }
    ss << "\n";
    return ss.str();
}

void
RegisterFileCache::markRFC(int regIdx)
{
    if (_capacity == 0) {
        return;
    }
    if (lruHash.find(regIdx) == lruHash.end()) {
        if (lruHead == nullptr) {
            DPRINTF(GPURFC, "RFC SIMD[%d] cache miss inserting physReg[%d]\n",
                    simdId, regIdx);
            OrderedRegs *oreg = new OrderedRegs(regIdx);
            lruHash[regIdx] = oreg;
            lruHead = oreg;
            lruTail = oreg;
            return;
        }

        if (lruHash.size() >= _capacity) {
            int val = lruTail->regIdx;
            DPRINTF(GPURFC,
                    "RFC SIMD[%d] cache miss inserting "
                    "physReg[%d] evicting physReg[%d]\n",
                    simdId, regIdx, val);

            lruTail = lruTail->prev;
            lruTail->next = nullptr;
            lruHash.erase(val);
        } else {
            DPRINTF(GPURFC, "RFC SIMD[%d] cache miss inserting physReg[%d]\n",
                    simdId, regIdx);
        }
    } else { // Exists in cache need to update
        DPRINTF(GPURFC, "RFC SIMD[%d] cache hit physReg[%d]\n", simdId,
                regIdx);

        if (lruHead->regIdx == regIdx) {
            return;
        }
        if (lruHash[regIdx] == lruTail) {
            lruTail = lruHash[regIdx]->prev;
        }
        if (lruHash[regIdx]->next != nullptr) {
            lruHash[regIdx]->next->prev = lruHash[regIdx]->prev;
        }
        lruHash[regIdx]->prev->next = lruHash[regIdx]->next;
        lruHash.erase(regIdx);
    }

    OrderedRegs *oreg = new OrderedRegs(regIdx);
    lruHash[regIdx] = oreg;
    oreg->next = lruHead;
    lruHead->prev = oreg;
    lruHead = oreg;
}

void
RegisterFileCache::waveExecuteInst(Wavefront *w, GPUDynInstPtr ii)
{
    if (!ii->isLoad() && !(ii->isAtomic() || ii->isMemSync())) {
        Cycles delay(computeUnit->rfcLength());
        Tick tickDelay = computeUnit->cyclesToTicks(delay);

        for (const auto &dstVecOp : ii->dstVecRegOperands()) {
            for (const auto &physIdx : dstVecOp.physIndices()) {
                enqCacheInsertEvent(physIdx, tickDelay);
            }
        }
    }
}

void
RegisterFileCache::enqCacheInsertEvent(uint32_t regIdx, uint64_t delay)
{
    schedule(new MarkRegCachedEvent(this, regIdx), curTick() + delay);
}

void
RegisterFileCache::MarkRegCachedEvent::process()
{
    rfc->markRFC(regIdx);
}

} // namespace gem5
