#ifndef __CPU_PRED_MULTI_LEVEL_BTB_HH__
#define __CPU_PRED_MULTI_LEVEL_BTB_HH__

#include <vector>

#include "base/cache/associative_cache.hh"
#include "cpu/pred/btb.hh"
#include "cpu/pred/btb_entry.hh"
#include "params/BTBLevel.hh"
#include "params/MultiLevelBTB.hh"
#include "sim/sim_object.hh"

namespace gem5::branch_prediction
{

class BTBLevel : public SimObject
{
  public:
    BTBLevel(const BTBLevelParams &params);

  private:
    friend class MultiLevelBTB;

    AssociativeCache<BTBEntry> btb;
    const Cycles latency;
    const bool inclusive;
};

class MultiLevelBTB : public BranchTargetBuffer
{
  public:
    MultiLevelBTB(const MultiLevelBTBParams &params);

    void memInvalidate() override;
    bool valid(ThreadID tid, Addr instPC) override;

    const BTBLookupResult
    lookup(ThreadID tid, Addr instPC,
           BranchType type = BranchType::NoBranch) override;

    void update(ThreadID tid, Addr instPC, const PCStateBase &target_pc,
                BranchType type = BranchType::NoBranch,
                StaticInstPtr inst = nullptr) override;

    const StaticInstPtr getInst(ThreadID tid, Addr instPC) override;

  private:
    BTBEntry *handleEviction(ThreadID tid, Addr instPC,
                             BTBLevel *insertionLevel);

    const std::vector<BTBLevel *> levels;

    struct MultiLevelBTBStats : public statistics::Group
    {
        MultiLevelBTBStats(statistics::Group *parent, unsigned num_levels);

        statistics::Vector levelHits;
    } multilevelstats;
};
} // namespace gem5::branch_prediction

#endif // __CPU_PRED_MULTI_LEVEL_BTB_HH__
