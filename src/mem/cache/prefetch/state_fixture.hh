#ifndef __MEM_CACHE_PREFETCH_STATE_FIXTURE_HH__
#define __MEM_CACHE_PREFETCH_STATE_FIXTURE_HH__
#include "mem/cache/prefetch/c1.hh"
#include <optional>
namespace gem5 {
struct TPCStateFixtureParams;
struct TPCFixtureComponentParams;
namespace prefetch {
class FixtureComponent : public TPCComponent {
    bool recognized;
    void onTrain(const AccessEvent &) override { ++trains; }
    bool canHandle(const AccessEvent &) const override { return recognized; }
    std::optional<SubmissionContext> saved;
    void onPredict(const AccessEvent &, const SubmissionContext &context) override {
        ++predicts; saved = context;
    }
  public:
    unsigned trains = 0, predicts = 0;
    SubmissionResult deferred() {
        return submitCandidates(*saved, {{0x600000, source() == 3 ? 2 : 1}}).front();
    }
    explicit FixtureComponent(const TPCFixtureComponentParams &);
};
class StateFixture : public SimObject {
    BaseCache *cache;
    TPCController *controller;
    TPCComponent *component;
  public:
    explicit StateFixture(const TPCStateFixtureParams &);
    void run();
};
}}
#endif
