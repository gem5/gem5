// Standalone tests of the production state encoding and finite C1 tables.
#include <cassert>
#include <iostream>
#include "mem/cache/prefetch/c1_tables.hh"
using namespace gem5;
using namespace gem5::prefetch;
static InstructionSource source(uint64_t generation = 1, unsigned offset = 7) {
    return {reinterpret_cast<const BaseCache *>(0x1000), 0x2000, generation, offset, false};
}
int main() {
    InstructionLineState state;
    state.initialize(1, 64);
    assert(state.matches(source()));
    assert(!state.update(64,3,-1,1) && !state.update(0,0,-1,1));
    for (int c = 1; c <= 3; ++c) assert(state.read(7, c) == 0);
    assert(state.update(7, 3, 0, 1));
    assert(!state.update(7, 3, 0, 3));
    assert(state.read(7, 1) == 0 && state.read(8, 3) == 0);
    assert(state.update(7, 1, 0, 2));
    assert(state.update(7, 2, 0, 3));
    assert(state.read(7, 3) == 1);
    assert(!state.matches(source(2)) && !state.matches(source(1,64)));
    auto moved = std::move(state);
    assert(moved.matches(source()) && moved.read(7,1) == 2);
    moved.initialize(2, 64); // Same physical address reloaded.
    assert(!moved.matches(source()) && moved.read(7,3) == 0);
    auto secure = source(); secure.secure = true;
    assert(!secure.sameLine(source()));

    // Four evicted samples; strict line threshold and strict 3/4 probability.
    for (unsigned denseSamples : {3u,4u}) {
        C1Tables t(16,1,16,6,4);
        t.allocate(0, source(), 0x400007, 0);
        std::vector<C1Tables::Decision> result;
        for (unsigned region = 0; region < 5; ++region) {
            unsigned lines = region < denseSamples ? 7 : 6;
            for (unsigned k = 0; k < lines; ++k) {
                auto ds = t.observe(region*1024+k*64,64,false);
                result.insert(result.end(),ds.begin(),ds.end());
                t.associate(source(),0x400007,0,region*1024+k*64,64,false);
                // Repeated accesses must not increase distinct-line density.
                assert(t.observe(region*1024+k*64,64,false).empty());
            }
        }
        assert(result.size()==1 && result[0].dense == (denseSamples==4));
        assert(t.freeSlot()==0);
        for(auto &r:t.rm) assert(r.instructions==0);
    }
    C1Tables t(2,2,16,6,4);
    t.allocate(0,source(),0x400007,0);
    t.allocate(1,source(2),0x400007,0);
    assert(t.freeSlot()==-1);
    t.observe(0,64,false); t.associate(source(),0x400007,0,0,64,false);
    assert(t.invalidate(source())==1 && t.im[1].valid);
    assert(t.rm[0].instructions==0 && t.rm[0].lines==1);
    t.allocate(0,source(3),0x400007,0);
    t.observe(1024,64,false); t.observe(0,64,false); t.observe(2048,64,false);
    assert(t.find(0,false) && !t.find(1024,false)); // True LRU.
    assert(t.im[0].total==0); // Cleared membership did not train the reused slot.
    std::cout << "state_tables: PASS\n";
}
