#include "cpu/pred/multi_level_btb.hh"

#include <optional>

#include "base/cprintf.hh"
#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/BTB.hh"

namespace gem5::branch_prediction
{

BTBLevel::BTBLevel(const BTBLevelParams &p)
    : SimObject(p),
      btb(name() + ".btb", p.numEntries, p.associativity, p.replPolicy,
          p.indexingPolicy, BTBEntry(genTagExtractor(p.indexingPolicy))),
      latency(p.latency),
      inclusive(p.inclusive)
{
    fatal_if(!isPowerOf2(p.numEntries / p.associativity),
             "Number of BTB sets must be a power of two");
}

MultiLevelBTB::MultiLevelBTBStats::MultiLevelBTBStats(
    statistics::Group *parent, unsigned num_levels)
    : statistics::Group(parent),
      ADD_STAT(levelHits, statistics::units::Count::get(),
               "Number of BTB hits at each level")
{
    using namespace statistics;

    levelHits.init(num_levels).flags(total);
}

MultiLevelBTB::MultiLevelBTB(const MultiLevelBTBParams &p)
    : BranchTargetBuffer(p),
      levels(p.levels),
      multilevelstats(this, levels.size())
{
    fatal_if(levels.empty(), "%s must contain at least one BTB level", name());
    DPRINTF(BTB, "%s: creating a %d-level BTB hierarchy\n", name(),
            levels.size());

    unsigned level_num = 1;
    for (auto *level : levels) {
        fatal_if(!level, "%s: BTB level %d is null", name(), level_num);
        multilevelstats.levelHits.subname(level_num - 1,
                                          csprintf("L%d", level_num));
        ++level_num;
    }
}

void
MultiLevelBTB::memInvalidate()
{
    for (auto *level : levels) {
        level->btb.clear();
    }
}

bool
MultiLevelBTB::valid(ThreadID tid, Addr instPC)
{
    unsigned level_num = 1;
    for (auto *level : levels) {
        if (level->btb.findEntry({instPC, tid})) {
            DPRINTF(BTB, "L%d BTB valid for PC %#x\n", level_num, instPC);
            return true;
        }
        ++level_num;
    }

    return false;
}

const BTBLookupResult
MultiLevelBTB::lookup(ThreadID tid, Addr instPC, BranchType type)
{
    stats.lookups[type]++;

    unsigned level_num = 1;
    for (auto *lookup_level : levels) {
        auto *hit_entry = lookup_level->btb.accessEntry({instPC, tid});
        if (hit_entry) {
            BTBEntry *result = hit_entry;

            if (level_num > 1) {
                // If the hit level is a victum buffer, backup the hit entry,
                // in case the victim from upper level overwrites the hit-entry
                // slot when they are mapped to the same set in the hit level.
                BTBEntry hit_backup(*hit_entry);

                if (!lookup_level->inclusive) {
                    // Demote the hit entry to the LRU position
                    lookup_level->btb.demoteEntry(hit_entry);
                }

                bool first_level = true;
                // Insert the hit entry into the L1 BTB and other
                // upper levels which are inclusive.
                for (auto *promotion_level : levels) {
                    if (promotion_level == lookup_level) {
                        break;
                    }
                    if (first_level || promotion_level->inclusive) {
                        auto *allocated_entry =
                            handleEviction(tid, instPC, promotion_level);
                        allocated_entry->update(*hit_backup.target,
                                                hit_backup.inst);

                        if (first_level) {
                            result = allocated_entry;
                        }
                    }
                    first_level = false;
                }
            }

            multilevelstats.levelHits[level_num - 1]++;
            DPRINTF(BTB, "L%d BTB hit for PC %#x, latency=%d cycles\n",
                    level_num, instPC, lookup_level->latency);
            return BTBLookupResult(result->target.get(),
                                   lookup_level->latency);
        }
        ++level_num;
    }

    // Miss all levels
    // Only the processor with coupled front-end can get here.
    stats.misses[type]++;
    DPRINTF(BTB, "BTB miss for PC %#x\n", instPC);
    return BTBLookupResult(nullptr, Cycles(0));
}

const StaticInstPtr
MultiLevelBTB::getInst(ThreadID tid, Addr instPC)
{
    for (auto *level : levels) {
        if (auto *entry = level->btb.findEntry({instPC, tid})) {
            return entry->inst;
        }
    }

    return nullptr;
}

void
MultiLevelBTB::update(ThreadID tid, Addr instPC, const PCStateBase &target,
                      BranchType type, StaticInstPtr inst)
{
    stats.updates[type]++;

    bool first_level = true;
    for (auto *level : levels) {
        if (first_level || level->inclusive) {
            auto *entry = level->btb.findEntry({instPC, tid});
            if (entry) {
                level->btb.accessEntry(entry);
            } else {
                entry = handleEviction(tid, instPC, level);
            }
            entry->update(target, inst);
        }
        first_level = false;
    }
}

BTBEntry *
MultiLevelBTB::handleEviction(ThreadID tid, Addr instPC,
                              BTBLevel *insertionLevel)
{
    auto &insertion_btb = insertionLevel->btb;
    BTBEntry *victim_entry = insertion_btb.findVictim({instPC, tid}, false);

    BTBEntry cur_victim(*victim_entry);
    bool victim_valid = victim_entry->isValid();

    victim_entry->invalidate();
    insertion_btb.insertEntry({instPC, tid}, victim_entry);

    bool reached_lower_level = false;
    for (auto *level : levels) {
        // Possible writeback only happens at lower levels.
        if (!reached_lower_level) {
            reached_lower_level = level == insertionLevel;
            continue;
        }

        // If this BTB level acts as victim buffer of the upper-level BTB,
        // settle the evicted victim from the upper-level in this level
        // Otherwise, simply dump the evicted victim because this level is
        // inclusive of the immediately upper level.
        if (level->inclusive || !victim_valid) {
            break;
        }

        auto &btb = level->btb;
        Addr victim_pc = cur_victim.getBranchAddr();
        BTBEntry *entry = btb.findEntry({victim_pc, tid});

        std::optional<BTBEntry> next_victim;
        if (entry) {
            btb.accessEntry(entry);
        } else {
            entry = btb.findVictim({victim_pc, tid}, false);
            if (entry->isValid()) {
                next_victim = *entry;
            }

            entry->invalidate();
            btb.insertEntry({victim_pc, tid}, entry);
        }

        entry->update(*cur_victim.target, cur_victim.inst);
        victim_valid = next_victim.has_value();
        if (victim_valid) {
            cur_victim = *next_victim;
        }
    }

    return victim_entry;
}
} // namespace gem5::branch_prediction
