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

BTBEntry *
BTBLevel::lookup(ThreadID tid, Addr instPC)
{
    auto *entry = btb.accessEntry({instPC, tid});
    if (entry && !inclusive) {
        btb.demoteEntry(entry);
    }

    return entry;
}

BTBEntry *
BTBLevel::multiLookup(ThreadID tid, Addr instPC, Cycles &hit_latency,
                      unsigned &hit_level)
{
    auto *entry = lookup(tid, instPC);
    if (entry) {
        hit_latency = latency;
        hit_level = level;
        return entry;
    }

    if (!nextLevel) {
        return nullptr;
    }

    entry = nextLevel->multiLookup(tid, instPC, hit_latency, hit_level);
    if (entry && inclusive) {
        // Refilling into this level can cascade an eviction and overwrite
        // the entry in the hit level. Preserve it before starting the refill.
        BTBEntry hit_copy(*entry);
        entry = insertEntry(tid, instPC, *hit_copy.target, hit_copy.inst);
    }

    return entry;
}

BTBEntry *
BTBLevel::insertEntry(ThreadID tid, Addr instPC, const PCStateBase &target,
                      StaticInstPtr inst)
{
    BTBEntry *entry = btb.findEntry({instPC, tid});
    if (entry) {
        btb.accessEntry(entry);
    } else {
        entry = btb.findVictim({instPC, tid}, false);

        std::optional<BTBEntry> victim;
        if (entry->isValid()) {
            victim = *entry;
        }

        entry->invalidate();
        btb.insertEntry({instPC, tid}, entry);

        if (victim && nextLevel) {
            nextLevel->doWriteback(tid, *victim);
        }
    }

    entry->update(target, inst);
    return entry;
}

void
BTBLevel::doWriteback(ThreadID tid, const BTBEntry &upper_victim)
{
    // An inclusive level already contains entries inserted in its upper
    // level. Only a victim-buffer level needs to accept the writeback.
    if (inclusive) {
        return;
    }

    insertEntry(tid, upper_victim.getBranchPC(), *upper_victim.target,
                upper_victim.inst);
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

    for (unsigned level = 0; level < levels.size(); ++level) {
        levels[level]->level = level;
        if (level + 1 < levels.size()) {
            levels[level]->nextLevel = levels[level + 1];
        }
    }

    fatal_if(!levels.front()->inclusive, "%s: L1 BTB must be inclusive",
             name());
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

    Cycles hit_latency(0);
    unsigned hit_level = 0;
    auto *entry =
        levels.front()->multiLookup(tid, instPC, hit_latency, hit_level);
    if (entry) {
        multilevelstats.levelHits[hit_level]++;
        DPRINTF(BTB, "L%d BTB hit for PC %#x, latency=%d cycles\n",
                hit_level + 1, instPC, hit_latency);
        return BTBLookupResult(entry->target.get(), hit_latency);
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

    for (unsigned level = 0; level < levels.size(); ++level) {
        auto *update_level = levels[level];
        if (update_level->inclusive) {
            update_level->insertEntry(tid, instPC, target, inst);
        }
    }
}
} // namespace gem5::branch_prediction
