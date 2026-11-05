// C1 spatial-region prefetcher and sequential division-of-labor policy.
#ifndef __MEM_CACHE_PREFETCH_C1_HH__
#define __MEM_CACHE_PREFETCH_C1_HH__
#include "mem/cache/prefetch/framework.hh"
#include "mem/cache/prefetch/c1_tables.hh"
namespace gem5 {
struct C1PrefetcherParams;
struct DivisionOfLaborControllerParams;
namespace prefetch {
class C1Prefetcher : public TPCComponent {
    friend class StateFixture;
    enum State { Unknown, Monitoring, NonDense, Dense };
    const unsigned regionLines;
    C1Tables tables;
    struct Stats : statistics::Group {
        statistics::Scalar rmAllocations, rmEvictions, denseRegions, imAllocations,
            imFull, trainedDense, trainedNonDense, invalidatedTraining, regionPredictions;
        Stats(statistics::Group *);
    } c1Stats;
  protected:
    void onObserve(const AccessEvent &) override;
    void onTrain(const AccessEvent &) override;
    bool canHandle(const AccessEvent &) const override;
    void onPredict(const AccessEvent &, const SubmissionContext &) override;
    void onInstructionInvalidated(const InstructionSource &) override;
  public:
    explicit C1Prefetcher(const C1PrefetcherParams &);
};
class DivisionOfLaborController : public TPCController {
    std::vector<TPCComponent *> ordered;
  protected:
    void onAccess(const AccessEvent &) override;
    bool allowCandidate(const TPCComponent &, const Candidate &) const override;
  public:
    explicit DivisionOfLaborController(const DivisionOfLaborControllerParams &);
};
}
}
#endif
