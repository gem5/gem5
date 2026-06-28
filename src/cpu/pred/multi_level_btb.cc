#include "cpu/pred/multi_level_btb.hh"
#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/BTB.hh"

namespace gem5::branch_prediction
{

MultiLevelBTB::MultiLevelBTBStats::MultiLevelBTBStats(
    statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(l1Hits, statistics::units::Count::get(),
               "Number of L1 BTB hits"),
      ADD_STAT(l2Hits, statistics::units::Count::get(),
               "Number of L2 BTB hits"),
      ADD_STAT(l3Hits, statistics::units::Count::get(),
               "Number of L3 BTB hits")
{
    using namespace statistics;

    l1Hits.flags(total);
    l2Hits.flags(total);
    l3Hits.flags(total);
}

MultiLevelBTB::MultiLevelBTB(const MultiLevelBTBParams &p)
    : BranchTargetBuffer(p),
      l1btb("l1BTB", p.l1NumEntries, p.l1Associativity, p.l1ReplPolicy,
            p.l1IndexingPolicy, BTBEntry(genTagExtractor(p.l1IndexingPolicy))),
      l2btb("l2BTB", p.l2NumEntries, p.l2Associativity, p.l2ReplPolicy,
            p.l2IndexingPolicy, BTBEntry(genTagExtractor(p.l2IndexingPolicy))),
      l3btb("l3BTB", p.threeLevel ? p.l3NumEntries : 0, p.l3Associativity,
            p.l3ReplPolicy, p.l3IndexingPolicy,
            BTBEntry(genTagExtractor(p.l3IndexingPolicy))),
      l1Latency(p.l1Latency),
      l2Latency(p.l2Latency),
      l3Latency(p.l3Latency),
      threeLevel(p.threeLevel),
      inclusive(p.inclusive),
      multilevelstats(this)
{
    if (threeLevel) {
        DPRINTF(BTB,
                "MultiLevelBTB: Creating L1(%d entries, %d cycles) + "
                "L2(%d entries, %d cycles) + L3(%d entries, %d cycles)\n",
                p.l1NumEntries, p.l1Latency, p.l2NumEntries, p.l2Latency,
                p.l3NumEntries, p.l3Latency);
    } else {
        DPRINTF(BTB,
                "MultiLevelBTB: Creating L1(%d entries, %d cycles) + "
                "L2(%d entries, %d cycles)\n",
                p.l1NumEntries, p.l1Latency, p.l2NumEntries, p.l2Latency);
    }

    if (!isPowerOf2(p.l1NumEntries / p.l1Associativity) ||
        !isPowerOf2(p.l2NumEntries / p.l2Associativity) ||
        (threeLevel && !isPowerOf2(p.l3NumEntries / p.l3Associativity))) {
        fatal("BTB sets is not a power of 2!");
    }
}

void
MultiLevelBTB::memInvalidate()
{
    l1btb.clear();
    l2btb.clear();
    if (threeLevel) {
        l3btb.clear();
    }
}

bool
MultiLevelBTB::valid(ThreadID tid, Addr instPC)
{
    BTBEntry *l1_entry = l1btb.findEntry({instPC, tid});
    if (l1_entry != nullptr) {
        DPRINTF(BTB, "L1 BTB valid for PC %#x\n", instPC);
        return true;
    }

    BTBEntry *l2_entry = l2btb.findEntry({instPC, tid});
    if (l2_entry != nullptr) {
        DPRINTF(BTB, "L2 BTB valid for PC %#x\n", instPC);
        return true;
    }
    if (threeLevel) {
        BTBEntry *l3_entry = l3btb.findEntry({instPC, tid});
        if (l3_entry != nullptr) {
            DPRINTF(BTB, "L3 BTB valid for PC %#x\n", instPC);
            return true;
        }
    }
    return false;
}

const BTBLookupResult
MultiLevelBTB::lookup(ThreadID tid, Addr instPC, BranchType type)
{
    stats.lookups[type]++;

    // lookup l1 btb firstly
    BTBEntry *l1_entry = l1btb.accessEntry({instPC, tid});
    if (l1_entry != nullptr) {
        multilevelstats.l1Hits++;
        DPRINTF(BTB, "L1 BTB hit for PC %#x, latency=%d cycles\n", instPC,
                l1Latency);
        return BTBLookupResult(l1_entry->target.get(), l1Latency);
    }

    // L1miss, lookup l2 btb
    BTBEntry *l2_entry = l2btb.accessEntry({instPC, tid});
    if (l2_entry != nullptr) {
        // If L2-BTB is a victim buffer, backup the L2-hit entry in case
        // the l1_victim overwrites the hit entry when they would be mapped to
        // the same set in L2.
        BTBEntry l2_hit_copy(*l2_entry);
        if (!inclusive) {
            // Demote the L2-hit entry to the LRU position
            l2btb.demoteEntry(l2_entry);
        }
        BTBEntry *l1_victim = handleEviction(tid, instPC, l1btb, l2btb);
        l1_victim->update(*l2_hit_copy.target, l2_hit_copy.inst);
        multilevelstats.l2Hits++;
        DPRINTF(BTB,
                "L2 BTB hit for PC %#x, latency=%d cycles, "
                "insert in L1\n",
                instPC, l2Latency);
        return BTBLookupResult(l1_victim->target.get(), l2Latency);
    }

    if (threeLevel) {
        BTBEntry *l3_entry = l3btb.accessEntry({instPC, tid});
        if (l3_entry != nullptr) {
            BTBEntry l3_hit_copy(*l3_entry);
            if (!inclusive) {
                // Demote the L3-hit entry to the LRU position
                l3btb.demoteEntry(l3_entry);
            }
            BTBEntry *l1_victim = handleEviction(tid, instPC, l1btb, l2btb);
            l1_victim->update(*l3_hit_copy.target, l3_hit_copy.inst);
            multilevelstats.l3Hits++;
            // If lower-level BTB is inclusive of upper-level BTB,
            // also insert the hit entry into the mid-level, i.e., L2-BTB.
            if (inclusive) {
                BTBEntry *l2_victim =
                    handleEviction(tid, instPC, l2btb, l3btb, true);
                l2_victim->update(*l3_entry->target, l3_entry->inst);
                DPRINTF(BTB, "L3 BTB hit for PC %#x, latency=%d cycles",
                        "insert in both L1 and L2\n", instPC, l3Latency);
            } else {
                DPRINTF(
                    BTB,
                    "L3 BTB hit for PC %#x, latency=%d cycles, insert in L1\n",
                    instPC, l3Latency);
            }
            return BTBLookupResult(l1_victim->target.get(), l3Latency);
        }
    }
    // Miss all levels
    stats.misses[type]++;
    DPRINTF(BTB, "BTB miss for PC %#x\n", instPC);

    return BTBLookupResult(nullptr, Cycles(0));
}

const StaticInstPtr
MultiLevelBTB::getInst(ThreadID tid, Addr instPC)
{
    BTBEntry *l1_entry = l1btb.findEntry({instPC, tid});
    if (l1_entry) {
        return l1_entry->inst;
    }
    BTBEntry *l2_entry = l2btb.findEntry({instPC, tid});
    if (l2_entry) {
        return l2_entry->inst;
    }
    if (threeLevel) {
        BTBEntry *l3_entry = l3btb.findEntry({instPC, tid});
        if (l3_entry) {
            return l3_entry->inst;
        }
    }
    return nullptr;
}

void
MultiLevelBTB::update(ThreadID tid, Addr instPC, const PCStateBase &target,
                      BranchType type, StaticInstPtr inst)
{
    stats.updates[type]++;

    BTBEntry *l1_entry = l1btb.findEntry({instPC, tid});
    if (l1_entry) {
        l1btb.accessEntry(l1_entry);
    } else {
        // If non-existant make space by evicting a victim in L1-BTB
        l1_entry = handleEviction(tid, instPC, l1btb, l2btb);
    }
    l1_entry->update(target, inst);

    // If lower-level BTB is inclusive of upper-level BTB,
    // insert entry into all levels.
    if (inclusive) {
        BTBEntry *l2_entry = l2btb.findEntry({instPC, tid});
        if (l2_entry) {
            l2btb.accessEntry(l2_entry);
        } else {
            if (threeLevel) {
                l2_entry = handleEviction(tid, instPC, l2btb, l3btb, true);
            } else {
                l2_entry = l2btb.findVictim({instPC, tid});
                l2btb.insertEntry({instPC, tid}, l2_entry);
            }
        }
        l2_entry->update(target, inst);

        if (threeLevel) {
            BTBEntry *l3_entry = l3btb.findEntry({instPC, tid});
            if (l3_entry) {
                l3btb.accessEntry(l3_entry);
            } else {
                l3_entry = l3btb.findVictim({instPC, tid});
                l3btb.insertEntry({instPC, tid}, l3_entry);
            }
            l3_entry->update(target, inst);
        }
    }
}

BTBEntry *
MultiLevelBTB::handleEviction(ThreadID tid, Addr instPC,
                              AssociativeCache<BTBEntry> &upper_btb,
                              AssociativeCache<BTBEntry> &lower_btb,
                              bool lowerIsL3)
{
    // Get victim from the upper-level BTB
    BTBEntry *upper_victim = upper_btb.findVictim({instPC, tid}, false);
    // If lower-level BTB acts as victim buffer of upper-level BTB,
    // settle the evicted entry from upper_btb in lower adjacent level.
    // Otherwise, simply dump the evicted entry because lower_btb is inclusive.
    if (upper_victim->isValid() && !inclusive) {
        BTBEntry *lower_victim =
            lower_btb.findEntry({upper_victim->getBranchAddr(), tid});
        if (lower_victim) {
            lower_btb.accessEntry(lower_victim);
        } else {
            // If the hierarchy is three-level and the lower_btb is not L3,
            // the evicted entry from lower_btb needs to be settled in L3.
            if (threeLevel && !lowerIsL3) {
                lower_victim = lower_btb.findVictim(
                    {upper_victim->getBranchAddr(), tid}, false);
                if (lower_victim->isValid()) {
                    BTBEntry *l3_victim =
                        l3btb.findEntry({lower_victim->getBranchAddr(), tid});
                    if (l3_victim) {
                        l3btb.accessEntry(l3_victim);
                    } else {
                        // L3 is always the last-level BTB,
                        // so the evicted entry from L3 will be dumped.
                        l3_victim = l3btb.findVictim(
                            {lower_victim->getBranchAddr(), tid});
                        l3btb.insertEntry({lower_victim->getBranchAddr(), tid},
                                          l3_victim);
                    }
                    l3_victim->update(*lower_victim->target,
                                      lower_victim->inst);
                }
                lower_victim->invalidate();
            } else {
                lower_victim =
                    lower_btb.findVictim({upper_victim->getBranchAddr(), tid});
            }
            lower_btb.insertEntry({upper_victim->getBranchAddr(), tid},
                                  lower_victim);
        }
        // Copy content from upper-level victim to the lower-level
        lower_victim->update(*upper_victim->target, upper_victim->inst);
    }
    upper_victim->invalidate();
    upper_btb.insertEntry({instPC, tid}, upper_victim);
    return upper_victim;
}
} // namespace gem5::branch_prediction
