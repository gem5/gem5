// Finite C1 monitoring resources. No packet ownership or cache-state storage.
#ifndef __MEM_CACHE_PREFETCH_C1_TABLES_HH__
#define __MEM_CACHE_PREFETCH_C1_TABLES_HH__
#include <algorithm>
#include <vector>
#include "mem/cache/prefetch/instruction_state.hh"
namespace gem5::prefetch {
class C1Tables {
  public:
    struct Instruction {
        bool valid = false;
        InstructionSource source;
        Addr pc = 0;
        ContextID context = 0;
        unsigned total = 0, dense = 0;
    };
    struct Region {
        bool valid = false, secure = false;
        Addr address = 0;
        uint64_t lines = 0, instructions = 0, last = 0;
    };
    struct Decision { InstructionSource source; bool dense; Addr pc; ContextID context; };
    std::vector<Instruction> im;
    std::vector<Region> rm;
    unsigned regionLines, denseLines, trainingRegions;
    uint64_t sequence = 0, allocations = 0, evictions = 0, denseEvictions = 0;
    C1Tables(unsigned ni, unsigned nr, unsigned lines, unsigned dense, unsigned samples)
        : im(ni), rm(nr), regionLines(lines), denseLines(dense), trainingRegions(samples) {}
    Region *find(Addr base, bool secure) {
        for (auto &r : rm)
            if (r.valid && r.address == base && r.secure == secure) return &r;
        return nullptr;
    }
    void release(unsigned slot) {
        for (auto &r : rm) r.instructions &= ~(uint64_t(1) << slot);
        im[slot] = {};
    }
    int freeSlot() const {
        for (unsigned k = 0; k < im.size(); ++k) if (!im[k].valid) return k;
        return -1;
    }
    void allocate(unsigned k, InstructionSource source, Addr pc, ContextID context) {
        im[k] = {}; im[k].valid = true; im[k].source = source;
        im[k].pc = pc; im[k].context = context;
    }
    void associate(InstructionSource source, Addr pc, ContextID context,
                   Addr address, unsigned block, bool secure) {
        auto *r = find(address & ~(Addr(block) * regionLines - 1), secure);
        if (!r) return;
        for (unsigned k = 0; k < im.size(); ++k)
            if (im[k].valid && im[k].source == source && im[k].pc == pc && im[k].context == context)
                r->instructions |= uint64_t(1) << k;
    }
    std::vector<Decision> observe(Addr address, unsigned block, bool secure) {
        std::vector<Decision> decisions;
        Addr base = address & ~(Addr(block) * regionLines - 1);
        auto *r = find(base, secure);
        if (!r) {
            auto victim = std::find_if(rm.begin(), rm.end(), [](const Region &x) { return !x.valid; });
            if (victim == rm.end()) {
                victim = std::min_element(rm.begin(), rm.end(),
                    [](const Region &a, const Region &b) { return a.last < b.last; });
                ++evictions;
                bool dense = unsigned(__builtin_popcountll(victim->lines)) > denseLines;
                denseEvictions += dense;
                uint64_t members = victim->instructions;
                for (unsigned k = 0; k < im.size(); ++k) {
                    auto &entry = im[k];
                    if (!(members & (uint64_t(1) << k)) || !entry.valid) continue;
                    ++entry.total; entry.dense += dense;
                    if (entry.total == trainingRegions) {
                        decisions.push_back({entry.source, uint64_t(4) * entry.dense > uint64_t(3) * entry.total, entry.pc, entry.context});
                        release(k);
                    }
                }
            }
            *victim = {}; victim->valid = true; victim->address = base; victim->secure = secure;
            r = &*victim; ++allocations;
        }
        r->last = ++sequence;
        r->lines |= uint64_t(1) << ((address - base) / block);
        return decisions;
    }
    unsigned invalidate(const InstructionSource &source) {
        unsigned released = 0;
        for (unsigned k = 0; k < im.size(); ++k)
            if (im[k].valid && im[k].source.sameLine(source)) { release(k); ++released; }
        return released;
    }
};
}
#endif
