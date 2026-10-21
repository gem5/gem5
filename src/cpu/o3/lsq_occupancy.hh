/*
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

#ifndef __CPU_O3_LSQ_OCCUPANCY_HH__
#define __CPU_O3_LSQ_OCCUPANCY_HH__

#include <vector>

#include "base/types.hh"
#include "cpu/o3/lsq_segments.hh"
#include "cpu/op_class.hh"
#include "params/ConstantOccupancy.hh"
#include "params/LSQOccupancyPolicy.hh"
#include "params/LSQOpDesc.hh"
#include "params/PerBlockOccupancy.hh"
#include "params/PerCacheLineOccupancy.hh"
#include "params/PerSegmentOccupancy.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace o3
{

/**
 * Computes the number of LSQ entries a memory access occupies once its
 * addresses and sizes are known.
 */
class LSQOccupancyPolicy : public SimObject
{
  public:
    PARAMS(LSQOccupancyPolicy);
    LSQOccupancyPolicy(const Params &p) : SimObject(p) {}

    /**
     * Returns the number of LSQ entries the access occupies. The LSQ
     * raises results below one to one.
     */
    virtual unsigned occupiedEntries(const MemAccessInfo &info) const = 0;
};

/** Occupies a fixed number of entries. */
class ConstantOccupancy : public LSQOccupancyPolicy
{
  public:
    PARAMS(ConstantOccupancy);
    ConstantOccupancy(const Params &p);

    unsigned occupiedEntries(const MemAccessInfo &info) const override;

  private:
    const unsigned entries;
};

/** Occupies one entry per accessed segment. */
class PerSegmentOccupancy : public LSQOccupancyPolicy
{
  public:
    PARAMS(PerSegmentOccupancy);
    PerSegmentOccupancy(const Params &p);

    unsigned occupiedEntries(const MemAccessInfo &info) const override;
};

/** Occupies one entry per aligned block touched by the access. */
class PerBlockOccupancy : public LSQOccupancyPolicy
{
  public:
    PARAMS(PerBlockOccupancy);
    PerBlockOccupancy(const Params &p);

    unsigned occupiedEntries(const MemAccessInfo &info) const override;

  private:
    const Addr blockSize;
    /** Count a block touched by several segments only once. */
    const bool coalesce;
};

/** Occupies one entry per cache line touched by the access. */
class PerCacheLineOccupancy : public LSQOccupancyPolicy
{
  public:
    PARAMS(PerCacheLineOccupancy);
    PerCacheLineOccupancy(const Params &p);

    unsigned occupiedEntries(const MemAccessInfo &info) const override;

  private:
    /** Count a cache line touched by several segments only once. */
    const bool coalesce;
};

/**
 * LSQ occupancy of the instructions of a set of op classes. Op classes
 * without a description occupy one entry.
 */
class LSQOpDesc : public SimObject
{
  public:
    std::vector<OpClass> opClasses;
    /** Entries reserved at dispatch. */
    unsigned entries;
    /** Policy computing the entries occupied once the access is known. */
    LSQOccupancyPolicy *policy;

    PARAMS(LSQOpDesc);
    LSQOpDesc(const Params &p)
        : SimObject(p),
          opClasses(p.opClasses),
          entries(p.entries),
          policy(p.policy)
    {}
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_LSQ_OCCUPANCY_HH__
