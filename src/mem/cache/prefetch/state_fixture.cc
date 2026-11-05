// Directed production-cache tests. This translation unit is opt-in only.
#include "mem/cache/prefetch/state_fixture.hh"
#include "params/TPCStateFixture.hh"
#include "params/TPCFixtureComponent.hh"
#include "sim/core.hh"
namespace gem5::prefetch {
FixtureComponent::FixtureComponent(const TPCFixtureComponentParams &p)
    : TPCComponent(p), recognized(p.recognize) {}
StateFixture::StateFixture(const TPCStateFixtureParams &p)
    : SimObject(p), cache(p.cache), controller(p.controller), component(p.component) {}
void StateFixture::run() {
    unsigned block = cache->getBlockSize();
    auto fill = [&](Addr address, bool secure = false) {
        auto req = std::make_shared<Request>(address, block,
            Request::INST_FETCH | (secure ? Request::SECURE : 0), Request::funcRequestorId);
        Packet packet(req, MemCmd::ReadResp);
        packet.allocate();
        std::memset(packet.getPtr<uint8_t>(), 0, block);
        PacketList writebacks;
        auto *blk = cache->handleFill(&packet, nullptr, writebacks, true);
        for (auto *pkt : writebacks) delete pkt;
        return InstructionSource{cache, address, blk->instructionState->generation, 7, secure};
    };
    auto original = fill(0x100000);
    auto read = [&](InstructionSource s, int c) { return cache->readInstructionState(s,c); };
    auto update = [&](InstructionSource s, int c, int from, int to) {
        return cache->updateInstructionState(s,c,from,to);
    };
    panic_if(read(original,3) != 0 || !update(original,3,0,1), "Initial state");
    panic_if(read(original,1) != 0 || read(original,2) != 0, "Namespace isolation");
    auto otherOffset = original; otherOffset.offset++;
    panic_if(read(otherOffset,3) != 0, "Offset isolation");
    // Hit response captures the old source before a later invalidation.
    auto fetchReq = std::make_shared<Request>(original.line + 7, 8,
        Request::INST_FETCH, Request::funcRequestorId);
    Packet fetchPacket(fetchReq, MemCmd::ReadReq); fetchPacket.allocate();
    auto *blk = cache->tags->findBlock(original.line,false);
    cache->satisfyRequest(&fetchPacket,blk);
    auto inFlight = instructionSource(fetchReq);
    panic_if(!(inFlight == original), "Fetch response provenance");
    auto copiedReq = std::make_shared<Request>(*fetchReq);
    panic_if(!(instructionSource(copiedReq) == original), "Copied/split request provenance");

    if (auto *c1 = dynamic_cast<C1Prefetcher *>(component)) {
        // Exercise cleanup through the real synchronous cache notification.
        c1->tables.allocate(0,original,0x400007,0);
        c1->tables.observe(0x200000,64,false);
        c1->tables.associate(original,0x400007,0,0x200000,64,false);
        cache->invalidateBlock(blk);
        panic_if(c1->tables.im[0].valid || c1->tables.rm[0].instructions,
                 "I-cache invalidation must clear IM/RM references");
    } else {
        cache->invalidateBlock(blk);
    }
    auto reloaded = fill(original.line);
    panic_if(reloaded.generation == original.generation || read(reloaded,3) != 0,
             "Reload must create unknown state in new generation");
    panic_if(cache->isSourceValid(inFlight) || update(original,3,0,3),
             "Old dynamic instruction or delayed response must not update reload");
    auto secured = fill(original.line,true);
    panic_if(!update(secured,3,0,2) || read(reloaded,3) != 0, "Security isolation");
    // Move the block with the production tag-store API; identity follows content.
    CacheBlk *destination = nullptr;
    cache->tags->forEachBlk([&](CacheBlk &candidate) {
        if (!candidate.isValid() && !destination && candidate.getSet() ==
            cache->tags->findBlock(reloaded.line,false)->getSet()) destination = &candidate;
        return false;
    });
    panic_if(!destination, "No empty destination for move test");
    cache->tags->moveBlock(cache->tags->findBlock(reloaded.line,false),destination);
    panic_if(!cache->isSourceValid(reloaded), "Block movement lost residency");
    // Explicit replacement follows the same evictBlock -> invalidateBlock path.
    PacketList writebacks;
    cache->evictBlock(cache->tags->findBlock(reloaded.line,false),writebacks);
    for(auto *pkt:writebacks) delete pkt;
    panic_if(cache->isSourceValid(reloaded), "Replacement must invalidate state");
    reloaded = fill(original.line);
    // A stale request is ignored, then a live one uses the actual controller.
    auto request = std::make_shared<Request>(0x200000,8,0,Request::funcRequestorId,0x400007,0);
    request->setPaddr(0x400000);
    setInstructionSource(request,original);
    AccessEvent event; event.request=request; event.level=1; event.address=0x400000; event.primaryMiss=true;
    controller->cacheAccess(event);
    panic_if(read(reloaded,3)!=0, "Stale access recreated classification");
    setInstructionSource(request,reloaded);
    controller->cacheAccess(event);
    if(auto *fake=dynamic_cast<FixtureComponent *>(component)) {
        panic_if(fake->source()!=1 || fake->trains!=1 || fake->predicts!=1,
                 "Ordered ownership must stop on recognition without candidates");
        panic_if(cache->accounting->value("trainingEvents",2) ||
                 cache->accounting->value("trainingEvents",3), "Lower sources trained after ownership");
        panic_if(fake->deferred()!=SubmissionResult::Transport ||
                 fake->deferred()!=SubmissionResult::Transport, "Live deferred transport failed");
        panic_if(cache->accounting->value("dropQueueDuplicate",1)!=1,
                 "Duplicate queued prediction was not counted");
        event.address=0x600000;
        controller->cacheAccess(event);
        panic_if(cache->accounting->value("dropDemandCancel",1)!=1,
                 "Demand cancellation was not counted");
        cache->invalidateBlock(cache->tags->findBlock(reloaded.line,false));
        fill(reloaded.line);
        panic_if(fake->deferred()!=SubmissionResult::ControllerRejected,
                 "Deferred prediction from stale residency was admitted");
    } else {
        panic_if(read(reloaded,3)!=1, "Live primary miss must start C1 monitoring");
        auto *c1=dynamic_cast<C1Prefetcher *>(component);
        // Real observe/train wrappers complete 4 dense evicted regions.
        for(unsigned region=0;region<24;++region) {
            for(unsigned k=0;k<7;++k) {
                event.address=0x400000+region*1024+k*64;
                controller->cacheAccess(event);
            }
        }
        panic_if(read(reloaded,3)!=3, "C1 dense training did not complete");
        panic_if(!c1 || c1->tables.freeSlot()!=0, "Completed IM was not released");
        // Re-fetch and retrain the same PC on a sparse stream.
        cache->invalidateBlock(cache->tags->findBlock(reloaded.line,false));
        reloaded = fill(original.line);
        setInstructionSource(request,reloaded);
        for(unsigned region=0;region<24;++region) {
            for(unsigned k=0;k<2;++k) {
                event.address=0x800000+region*1024+k*64;
                controller->cacheAccess(event);
            }
        }
        panic_if(read(reloaded,3)!=2, "Sparse retraining must classify non-dense");
        // beginPhase/reset must leave resident classification intact.
        cache->accounting->beginPhase("fixture-measurement");
        panic_if(read(reloaded,3)!=2, "Phase transition cleared classification");
    }
    inform("TPC_STATE_FIXTURE_PASS");
}
}
