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

    /** Look up instPC at this level only. */
    BTBEntry *lookup(ThreadID tid, Addr instPC);

    /** Look up instPC at this level, recursing into nextLevel on a miss.
     *  On a hit in a lower level, the entry is refilled into this level
     *  (and any other inclusive level along the way) before returning.
     *  hit_latency/hit_level report the latency and index of the level
     *  that actually produced the hit, for stats/DPRINTF purposes.
     */
    BTBEntry *multiLookup(ThreadID tid, Addr instPC, Cycles &hit_latency,
                          unsigned &hit_level);

    /** Insert/update instPC's entry at this level, allocating (and
     *  possibly evicting) a slot for it if it isn't already present.
     */
    BTBEntry *insertEntry(ThreadID tid, Addr instPC, const PCStateBase &target,
                          StaticInstPtr inst);

    void doWriteback(ThreadID tid, const BTBEntry &upper_victim);

    AssociativeCache<BTBEntry> btb;
    const Cycles latency;
    const bool inclusive;
    BTBLevel *nextLevel = nullptr;
    unsigned level = 0;
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
    const std::vector<BTBLevel *> levels;

    struct MultiLevelBTBStats : public statistics::Group
    {
        MultiLevelBTBStats(statistics::Group *parent, unsigned num_levels);

        statistics::Vector levelHits;
    } multilevelstats;
};
} // namespace gem5::branch_prediction

#endif // __CPU_PRED_MULTI_LEVEL_BTB_HH__
