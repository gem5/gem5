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

#ifndef __REGISTER_FILE_CACHE_HH__
#define __REGISTER_FILE_CACHE_HH__

#include <limits>
#include <unordered_set>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "gpu-compute/misc.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ComputeUnit;
class Wavefront;

struct RegisterFileCacheParams;

class RegisterFileCache : public SimObject
{
  public:
    RegisterFileCache(const RegisterFileCacheParams &p);
    virtual ~RegisterFileCache();
    virtual void setParent(ComputeUnit *_computeUnit);

    int
    cacheSize() const
    {
        return _capacity;
    }

    // Debug functions
    virtual std::string dumpLL() const;

    // Abstract Register Event
    class RegisterCacheEvent : public Event
    {
      protected:
        RegisterFileCache *rfc;
        int regIdx;

      public:
        RegisterCacheEvent(RegisterFileCache *rfc, int regIdx)
            : rfc(rfc), regIdx(regIdx)
        {
            setFlags(AutoDelete);
        }
    };

    class MarkRegCachedEvent : public RegisterCacheEvent
    {
      public:
        MarkRegCachedEvent(RegisterFileCache *rfc, int regIdx)
            : RegisterCacheEvent(rfc, regIdx)
        {}

        void process();
    };

    virtual void enqCacheInsertEvent(uint32_t regIdx, uint64_t delay);

    virtual void waveExecuteInst(Wavefront *w, GPUDynInstPtr ii);

    // Add register to rfc using LRU replacement policy
    virtual void markRFC(int regIdx);

    virtual bool inRFC(int regIdx);

  protected:
    ComputeUnit *computeUnit;
    int simdId, _capacity;

    class OrderedRegs
    {
      public:
        int regIdx;

        OrderedRegs *next;
        OrderedRegs *prev;

        OrderedRegs(int val) : regIdx(val), next(nullptr), prev(nullptr) {}
    };

    // Doubly linked list, head is the most recently used
    std::unordered_map<int, OrderedRegs *> lruHash;
    OrderedRegs *lruHead = nullptr;
    OrderedRegs *lruTail = nullptr;
};

} // namespace gem5

#endif // __REGISTER_FILE_CACHE_HH__
