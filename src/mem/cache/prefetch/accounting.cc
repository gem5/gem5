#include "mem/cache/prefetch/accounting.hh"
#include "mem/cache/cache_blk.hh"
#include "base/logging.hh"
#include "params/PrefetchAccounting.hh"
#include "sim/cur_tick.hh"
#include <cctype>

namespace gem5 {
void PrefetchAccounting::metric(const char *name, bool layered) {
    auto &m = metrics[name];
    m.size = layered ? 16 : 4;
    m.stat = std::make_unique<statistics::Vector>(this, name,
        "Project prefetch accounting; see README for stage/cohort definitions");
    m.stat->init(m.size);
    const char *sources[] = {"Multi", "T2", "P1", "C1"};
    const char *levels[] = {"L1I", "L1D", "L2", "L3"};
    for (int i = 0; i < m.size; ++i)
        m.stat->subname(i, layered ? std::string(sources[i / 4]) + "_" + levels[i % 4] : sources[i]);
}

PrefetchAccounting::PrefetchAccounting(const PrefetchAccountingParams &p)
    : SimObject(p), trace(p.trace) {
    for (auto n : {"generated", "enqueued", "dequeued", "accepted", "filled",
        "timelyUseful", "late", "unusedRemoved", "aborted", "unresolved",
        "inFlight", "residentUnused", "readyPending", "queuedPending", "carryInQueued",
        "carryInFilled", "carryInTimelyUseful", "carryInLate", "carryInUnusedRemoved",
        "carryInAborted", "carryInUnresolved", "fillEvents", "usefulEvents", "lateEvents",
        "unusedEvents", "accuracy", "latencySamples", "latencyTicks", "latencyMeanTicks",
        "leadSamples", "leadTicks", "leadMeanTicks", "lateWaitSamples", "lateWaitTicks",
        "lateWaitMeanTicks", "dropThrottle", "dropPage", "dropNoTranslation",
        "dropTranslation", "dropQueueDuplicate", "dropQueueFull", "dropDemandCancel",
        "dropSnoop", "dropCache", "dropMSHR", "dropWriteBuffer", "unusedReplacement",
        "unusedInvalidation", "unusedNoRetention"}) metric(n);
    for (auto n : {"requestEvents", "requestBytes", "responseEvents", "responseDataBytes", "hitEvents", "fillLayerEvents",
                   "timelyLayerEvents"}) metric(n, true);
    for (auto n : {"demandMshrMisses", "observedCoverage"}) {
        metric(n);
        const char *levels[] = {"L1I", "L1D", "L2", "L3"};
        for (int i = 0; i < 4; ++i) metrics.at(n).stat->subname(i, levels[i]);
    }
}

void PrefetchAccounting::count(const std::string &n, int source, double v) {
    if (source >= 0 && source < 4) metrics.at(n).values[source] += v;
}
void PrefetchAccounting::layer(const std::string &n, int source, int level, double v) {
    if (source >= 0 && source < 4 && level >= 0 && level < 4)
        metrics.at(n).values[source * 4 + level] += v;
}
double PrefetchAccounting::value(const std::string &n, int s) const {
    return metrics.at(n).values[s];
}
void PrefetchAccounting::beginPhase(const std::string &name) {
    ++epoch;
    if (trace) inform("Prefetch accounting phase %s epoch %llu", name, epoch);
}
void PrefetchAccounting::serialize(CheckpointOut &) const {
    fatal("Saving prefetch accounting state is unsupported; use architectural checkpoints from benchmarks/create_checkpoints.py");
}
void PrefetchAccounting::resetStats() {
    SimObject::resetStats();
    for (auto &m : metrics) m.second.values.fill(0);
    // Records and their original accepted epochs deliberately survive reset.
}
PrefetchAccounting::Ptr PrefetchAccounting::generate(int s, int t) {
    if (s < 0) return nullptr;
    fatal_if(s >= 4 || t < 0 || t >= 4, "Invalid prefetch source/target");
    auto r = std::make_shared<prefetch::AccountingRecord>();
    r->owner = this; r->id = ++nextId; r->source = s; r->target = t;
    r->generatedTick = curTick(); r->generatedEpoch = epoch;
    count("generated", s);
    return r;
}
void PrefetchAccounting::queue(const Ptr &r) {
    if (!r || r->queued) return;
    r->queued = true; records[r->id] = r;
    count("enqueued", r->source);
}
void PrefetchAccounting::dequeue(const Ptr &r) {
    if (!r) return;
    r->queued = false; count("dequeued", r->source);
}
void PrefetchAccounting::drop(const Ptr &r, const char *reason) {
    if (!r) return;
    r->queued = false; count(reason, r->source);
    if (trace) inform("Prefetch %llu source %d %s", r->id, r->source, reason);
}
void PrefetchAccounting::accept(const Ptr &r) {
    if (!r || r->accepted) return;
    r->queued = false; r->accepted = true; r->acceptedEpoch = epoch;
    r->acceptedTick = curTick(); records[r->id] = r;
    count("accepted", r->source);
}
void PrefetchAccounting::fill(const Ptr &r, int level, Tick ready) {
    if (!r) return;
    layer("fillLayerEvents", r->source, level);
    if (level != r->target || r->filled) return;
    r->filled = true; r->readyTick = ready;
    count("fillEvents", r->source);
    count(r->acceptedEpoch == epoch ? "filled" : "carryInFilled", r->source);
    count("latencySamples", r->source);
    count("latencyTicks", r->source, ready - r->acceptedTick);
    if (r->outcome == prefetch::Outcome::Late && r->demandSeen) {
        count("lateWaitSamples", r->source);
        count("lateWaitTicks", r->source, ready > r->demandTick ? ready - r->demandTick : 0);
    }
}
void PrefetchAccounting::finish(const Ptr &r, prefetch::Outcome outcome, const char *n) {
    if (!r || !r->accepted || r->outcome != prefetch::Outcome::Pending) return;
    r->outcome = outcome;
    count(r->acceptedEpoch == epoch ? n : "carryIn" + std::string(1, std::toupper(n[0])) + (n + 1), r->source);
    if (trace) inform("Prefetch %llu source %d %s", r->id, r->source, n);
}
void PrefetchAccounting::demand(const Ptr &r, int level, bool satisfied) {
    if (!r || level != r->target || !r->accepted || r->outcome != prefetch::Outcome::Pending) return;
    r->demandSeen = true; r->demandTick = curTick();
    if (satisfied && r->filled && curTick() >= r->readyTick) {
        count("usefulEvents", r->source); layer("timelyLayerEvents", r->source, level);
        count("leadSamples", r->source); count("leadTicks", r->source, curTick() - r->readyTick);
        finish(r, prefetch::Outcome::Useful, "timelyUseful");
    } else {
        count("lateEvents", r->source);
        finish(r, prefetch::Outcome::Late, "late");
        if (r->filled) {
            count("lateWaitSamples", r->source);
            count("lateWaitTicks", r->source, r->readyTick > curTick() ? r->readyTick - curTick() : 0);
        }
    }
}
void PrefetchAccounting::remove(const Ptr &r, int level, const char *reason) {
    if (!r || level != r->target || r->outcome != prefetch::Outcome::Pending) return;
    count("unusedEvents", r->source); count(reason, r->source);
    finish(r, prefetch::Outcome::Unused, "unusedRemoved");
}
void PrefetchAccounting::abort(const Ptr &r) { finish(r, prefetch::Outcome::Aborted, "aborted"); }
void PrefetchAccounting::demandMiss(int level) {
    if (level >= 0 && level < 4) metrics.at("demandMshrMisses").values[level]++;
}
void PrefetchAccounting::preDumpStats() {
    for (auto n : {"unresolved", "carryInUnresolved", "queuedPending", "carryInQueued", "inFlight", "residentUnused", "readyPending"})
        metrics.at(n).values.fill(0);
    for (auto it = records.begin(); it != records.end();) {
        auto r = it->second.lock();
        if (!r) { it = records.erase(it); continue; }
        ++it;
        if (r->queued) count(r->generatedEpoch != epoch ? "carryInQueued" : "queuedPending", r->source);
        if (!r->accepted) continue;
        if (r->acceptedEpoch == epoch && !r->filled) count("inFlight", r->source);
        if (r->outcome != prefetch::Outcome::Pending) continue;
        count(r->acceptedEpoch == epoch ? "unresolved" : "carryInUnresolved", r->source);
        if (r->acceptedEpoch == epoch && r->filled) {
            count("residentUnused", r->source);
            if (r->readyTick > curTick()) count("readyPending", r->source);
        }
    }
    for (int s = 0; s < 4; ++s) {
        auto ratio = [&](const char *out, const char *num, const char *den) {
            metrics.at(out).values[s] = value(den, s) ? value(num, s) / value(den, s) : 0;
        };
        ratio("accuracy", "timelyUseful", "accepted");
        ratio("latencyMeanTicks", "latencyTicks", "latencySamples");
        ratio("leadMeanTicks", "leadTicks", "leadSamples");
        ratio("lateWaitMeanTicks", "lateWaitTicks", "lateWaitSamples");
        fatal_if(value("accepted", s) != value("timelyUseful", s) + value("late", s) +
            value("unusedRemoved", s) + value("aborted", s) + value("unresolved", s),
            "Prefetch accepted/outcome accounting mismatch for source %d", s);
    }
    for (int l = 0; l < 4; ++l) {
        double useful = 0;
        for (int s = 0; s < 4; ++s) useful += metrics.at("timelyLayerEvents").values[s * 4 + l];
        double denominator = useful + metrics.at("demandMshrMisses").values[l];
        // Cache-wide coverage, once in the Multi slot; not per-component coverage.
        metrics.at("observedCoverage").values[l] = denominator ? useful / denominator : 0;
    }
    for (auto &item : metrics)
        for (int i = 0; i < item.second.size; ++i)
            (*item.second.stat)[i] = item.second.values[i];
    SimObject::preDumpStats();
}

void PrefetchAccounting::selfTest() {
    // Deterministic state-machine tests; no simulated requests are issued.
    beginPhase("test-warmup");
    auto carry = generate(0, 1); accept(carry); fill(carry, 1, curTick());
    resetStats(); beginPhase("test-measurement");
    demand(carry, 1, true);
    fatal_if(value("carryInTimelyUseful", 0) != 1 || value("timelyUseful", 0), "Carry-in counted in accuracy");
    std::array<Ptr, 4> active;
    for (int s = 0; s < 4; ++s) {
        active[s] = generate(s, 1); queue(active[s]); dequeue(active[s]);
        accept(active[s]); accept(active[s]); // retries cannot double count
    }
    auto request = std::make_shared<Request>(0, 64, 0, 0);
    request->setExtension(std::make_shared<PrefetchProvenance>(active[0]));
    Request copied(*request);
    fatal_if(copied.getExtension<PrefetchProvenance>()->record != active[0], "Request clone lost provenance");
    CacheBlk original, moved;
    original.insert(0, false);
    original.prefetchOrigin = active[0];
    moved = std::move(original);
    fatal_if(moved.prefetchOrigin != active[0] || original.prefetchOrigin,
        "Cache line move lost or duplicated ownership");
    moved.invalidate();
    fatal_if(moved.prefetchOrigin, "Cache line invalidation retained provenance");
    fill(active[0], 2, curTick());
    fatal_if(active[0]->filled, "Transit layer counted target fill");
    demand(active[0], 2, true);
    fatal_if(active[0]->outcome != prefetch::Outcome::Pending, "Transit demand counted target use");
    fill(active[0], 1, curTick()); demand(active[0], 1, true); demand(active[0], 1, true);
    demand(active[1], 1, false); fill(active[1], 1, curTick() + 10);
    fill(active[2], 1, curTick()); remove(active[2], 1, "unusedReplacement");
    remove(active[2], 1, "unusedInvalidation");
    abort(active[3]);
    for (int s = 0; s < 4; ++s) fatal_if(value("accepted", s) != 1, "Double accepted");
    fatal_if(value("timelyUseful", 0) != 1 || value("late", 1) != 1 ||
        value("unusedRemoved", 2) != 1 || value("aborted", 3) != 1,
        "Incorrect terminal accounting");
    for (auto n : {"dropThrottle", "dropPage", "dropNoTranslation", "dropTranslation",
        "dropQueueDuplicate", "dropQueueFull", "dropDemandCancel", "dropSnoop",
        "dropCache", "dropMSHR", "dropWriteBuffer"}) {
        auto r = generate(0, 1); queue(r); drop(r, n);
        fatal_if(value(n, 0) != 1 || r->queued, "Drop event failed");
    }
    auto future = generate(0, 1); accept(future); fill(future, 1, curTick() + 20);
    demand(future, 1, true);
    fatal_if(value("late", 0) != 1, "Hit before ready was counted timely");
    auto pending = generate(0, 1); accept(pending);
    auto queued = generate(0, 1); queue(queued);
    preDumpStats();
    fatal_if(value("unresolved", 0) != 1 || value("queuedPending", 0) != 1,
        "Pending gauges incorrect");
    resetStats(); records.clear(); beginPhase("test-finished");
    inform("PREFETCH_ACCOUNTING_SELFTEST_SUCCESS");
}
} // namespace gem5
