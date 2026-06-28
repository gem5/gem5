#ifndef __CPU_PRED_MULTI_LEVEL_BTB_HH__
#define __CPU_PRED_MULTI_LEVEL_BTB_HH__

#include "base/cache/associative_cache.hh"
#include "cpu/pred/btb.hh"
#include "cpu/pred/btb_entry.hh"
#include "params/MultiLevelBTB.hh"

namespace gem5::branch_prediction
{

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
                             AssociativeCache<BTBEntry> &upper_btb,
                             AssociativeCache<BTBEntry> &lower_btb,
                             bool lowerIsL3 = false);

    AssociativeCache<BTBEntry> l1btb;

    AssociativeCache<BTBEntry> l2btb;

    AssociativeCache<BTBEntry> l3btb;

    const Cycles l1Latency;
    const Cycles l2Latency;
    const Cycles l3Latency;
    const bool threeLevel;
    const bool inclusive;

    struct MultiLevelBTBStats : public statistics::Group
    {
        MultiLevelBTBStats(statistics::Group *parent);

        statistics::Scalar l1Hits;
        statistics::Scalar l2Hits;
        statistics::Scalar l3Hits;
    } multilevelstats;
};
} // namespace gem5::branch_prediction

#endif // __CPU_PRED_MULTI_LEVEL_BTB_HH__
