#include "mem/cache/prefetch/c1.hh"
#include <algorithm>
#include <set>
#include "params/C1Prefetcher.hh"
#include "params/DivisionOfLaborController.hh"
namespace gem5::prefetch {
C1Prefetcher::Stats::Stats(statistics::Group *parent)
    : statistics::Group(parent, "c1"),
      ADD_STAT(rmAllocations, statistics::units::Count::get(), "Regions allocated"),
      ADD_STAT(rmEvictions, statistics::units::Count::get(), "Regions retired"),
      ADD_STAT(denseRegions, statistics::units::Count::get(), "Retired dense regions"),
      ADD_STAT(imAllocations, statistics::units::Count::get(), "Instruction training allocations"),
      ADD_STAT(imFull, statistics::units::Count::get(), "Training allocation blocked by full IM"),
      ADD_STAT(trainedDense, statistics::units::Count::get(), "Dense classifications"),
      ADD_STAT(trainedNonDense, statistics::units::Count::get(), "Non-dense classifications"),
      ADD_STAT(invalidatedTraining, statistics::units::Count::get(), "Training canceled by I-cache invalidation"),
      ADD_STAT(regionPredictions, statistics::units::Count::get(), "Whole-region prediction calls") {}
C1Prefetcher::C1Prefetcher(const C1PrefetcherParams &p)
    : TPCComponent(p), regionLines(p.region_lines),
      tables(p.im_entries, p.rm_entries, p.region_lines, p.dense_lines, p.training_regions), c1Stats(this)
{
    fatal_if(source() != 3 || !regionLines || regionLines > 64 ||
        (regionLines & (regionLines - 1)) || p.dense_lines >= regionLines ||
        !p.training_regions || !p.im_entries || p.im_entries > 64 || !p.rm_entries,
        "Invalid C1 parameters");
    fatal_if(p.observe_levels.size() != 1 || p.observe_levels[0] != 1,
             "C1 observes L1D demand accesses only");
}
void C1Prefetcher::onObserve(const AccessEvent &e) {
    if (e.level != 1 || e.request->isInstFetch()) return;
    auto allocations = tables.allocations, evictions = tables.evictions, dense = tables.denseEvictions;
    auto decisions = tables.observe(e.address, cacheView(1).blockSize(), e.secure);
    c1Stats.rmAllocations += tables.allocations - allocations;
    c1Stats.rmEvictions += tables.evictions - evictions;
    c1Stats.denseRegions += tables.denseEvictions - dense;
    for (const auto &d : decisions) {
        if (updateInstructionState(d.source, Monitoring, d.dense ? Dense : NonDense)) {
            if (d.dense) c1Stats.trainedDense++; else c1Stats.trainedNonDense++;
            inform("C1_CLASSIFICATION pc=%#x context=%d generation=%llu dense=%d",
                   d.pc, d.context, d.source.generation, d.dense);
        }
    }
}
void C1Prefetcher::onTrain(const AccessEvent &e) {
    auto source = instructionSource(e.request);
    int state = instructionState(source);
    if (state == Unknown && e.primaryMiss) {
        int slot = tables.freeSlot();
        if (slot < 0) { c1Stats.imFull++; return; }
        if (!updateInstructionState(source, Unknown, Monitoring)) return;
        tables.allocate(slot, source, e.request->getPC(), e.request->contextId());
        c1Stats.imAllocations++; state = Monitoring;
    }
    if (state == Monitoring)
        tables.associate(source, e.request->getPC(), e.request->contextId(),
                         e.address, cacheView(1).blockSize(), e.secure);
}
bool C1Prefetcher::canHandle(const AccessEvent &e) const {
    return instructionState(instructionSource(e.request)) == Dense;
}
void C1Prefetcher::onPredict(const AccessEvent &e, const SubmissionContext &context) {
    unsigned block = cacheView(2).blockSize();
    Addr start = e.address & ~(Addr(block) * regionLines - 1);
    std::vector<Candidate> candidates;
    for (unsigned k = 0; k < regionLines; ++k) {
        Addr address = start + k * block;
        if (address != (e.address & ~(Addr(block) - 1))) candidates.push_back({address, 2});
    }
    c1Stats.regionPredictions++;
    submitCandidates(context, candidates);
}
void C1Prefetcher::onInstructionInvalidated(const InstructionSource &source) {
    c1Stats.invalidatedTraining += tables.invalidate(source);
}
DivisionOfLaborController::DivisionOfLaborController(const DivisionOfLaborControllerParams &p)
    : TPCController(p), ordered(components) {
    std::set<int> expected;
    if (p.mode == "C") expected = {3};
    else if (p.mode == "T") expected = {1};
    else if (p.mode == "TP") expected = {1,2};
    else if (p.mode == "TPC") expected = {1,2,3};
    else fatal("Invalid TPC mode");
    std::set<int> actual;
    for (auto *c : ordered) actual.insert(c->source());
    fatal_if(actual != expected, "TPC mode does not match enabled sources");
    std::sort(ordered.begin(), ordered.end(), [](TPCComponent *a, TPCComponent *b) {
        return a->source() < b->source();
    });
}
void DivisionOfLaborController::onAccess(const AccessEvent &e) {
    if (e.level != 1 || !e.request->hasPC() || !e.request->hasContextId() ||
        e.request->isInstFetch() || !validSource(instructionSource(e.request))) return;
    for (auto *component : ordered) {
        component->train(e);
        if (component->recognizes(e)) { component->predict(e); break; }
    }
}
bool DivisionOfLaborController::allowCandidate(const TPCComponent &c, const Candidate &candidate) const {
    return candidate.target == (c.source() == 3 ? 2 : 1);
}
}
