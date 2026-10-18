#include "mem/cache/retention/retention_manager.hh"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Cache.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/retention/bank_ports.hh"
#include "mem/cache/retention/reuse_tracker.hh"
#include "mem/cache/tags/base.hh"
#include "params/BaseCache.hh"

namespace gem5
{

RetentionManager::RetentionManager(BaseCache &_cache,
                                   const BaseCacheParams &p)
    : cache(_cache),
      topMRU(p.top_mru),
      refreshDirtyDaemon(p.refresh_dirty_daemon),
      strict(p.drt_strict),
      assoc(p.assoc),
      scanCycles(p.refresh_set_scan_cycles),
      lineCycles(p.refresh_line_cycles > 0 ? p.refresh_line_cycles :
                 Cycles(2 * p.data_latency)),
      banks(std::max(p.refresh_banks, 1u)),
      subarrays(std::max(p.refresh_subarrays, 1u)),
      groupOpen(false),
      groupStarted(false),
      groupStart(0),
      groupRefreshCounted(false),
      lineBudget(p.refresh_line_budget_cycles > 0 ?
                 p.refresh_line_budget_cycles : lineCycles),
      lineScan(p.refresh_scan == "line"),
      refreshMode(p.refresh_mode == "all_strict" ? RefreshMode::AllStrict :
                  p.refresh_mode == "all_opp" ? RefreshMode::AllOpp :
                  RefreshMode::Policy),
      raWindow(p.refresh_all_window_cycles),
      raFree(p.refresh_all_free),
      raEven(p.refresh_all_even),
      oppAge(p.refresh_opp_age),
      margin(p.retention_margin),
      guardSweeps(p.retention_guard_sweeps),
      activeInvalidate(p.active_invalidate),
      coldDirty(p.cold_dirty == "mru" ? ColdDirty::MRU :
                p.cold_dirty == "touched" ? ColdDirty::Touched :
                ColdDirty::Refresh),
      keepTouched(p.keep_touched),
      touchedClearAge(p.touched_clear_age),
      silentCleanDrop(p.silent_clean_drop),
      quantizedAge(p.quantized_age),
      epochLog2(p.age_epoch_log2),
      ageBits(p.age_bits),
      deadlineEpochsParam(p.deadline_epochs),
      oppAgeEpochsParam(p.opp_age_epochs),
      clearEpochsParam(p.touched_clear_epochs),
      deadlineEpochs(0),
      oppAgeEpochs(0),
      clearEpochs(0),
      inlineOnWrite(p.inline_refresh),
      inlineOnRead(p.inline_refresh_on_read),
      inlineCycles(p.inline_refresh_cycles > 0 ? p.inline_refresh_cycles :
                   Cycles(p.data_latency)),
      tSweep(0),
      tGuard(0),
      bankBusyUntil(banks, 0),
      bankLastAccess(banks, 0),
      sweepStart(0),
      refreshSetIdx(0),
      lastVisit(_cache.tags->getNumSets(), MaxTick),
      maxVisitInterval(0),
      refreshEvent([this] { processRefreshEvent(); },
                   _cache.name() + ".refreshDaemon"),
      maxStall(0),
      rowLastRefresh(_cache.tags->getNumSets() * assoc, MaxTick),
      maxRowGap(0),
      maxChargeAgeAtRefresh(0),
      maxChargeAgeAtDrop(0),
      raRows(0),
      raPeriod(0),
      raMaxRowInterval(0),
      raMaxChargeAge(0),
      stats(*this, _cache)
{
    fatal_if(!cache.tags->isForgetting(),
             "%s: drt > 0 but the tags do not support retention",
             cache.name());
    fatal_if(topMRU > assoc, "%s: top_mru (%u) > assoc (%u)",
             cache.name(), topMRU, assoc);
    fatal_if(scanCycles == 0, "%s: refresh_set_scan_cycles must be > 0",
             cache.name());
    fatal_if(p.refresh_banks == 0 ||
             p.refresh_banks > cache.tags->getNumSets(),
             "%s: refresh_banks (%u) must be in [1, numSets = %u]",
             cache.name(), p.refresh_banks, cache.tags->getNumSets());
    fatal_if(guardSweeps < 1.0, "%s: retention_guard_sweeps (%f) < 1",
             cache.name(), guardSweeps);
    // Only kept lines are refreshed, and an unkept dirty line would need
    // a writeback
    fatal_if(activeInvalidate && !refreshDirtyDaemon,
             "%s: active_invalidate needs refresh_dirty_daemon",
             cache.name());
    // The next visit within T_sweep needs the daemon to be the only user
    // of the banks
    fatal_if(p.cold_dirty != "refresh" && p.cold_dirty != "mru" &&
             p.cold_dirty != "touched",
             "%s: cold_dirty must be refresh, mru or touched",
             cache.name());
    fatal_if(coldDirty != ColdDirty::Refresh && !activeInvalidate,
             "%s: cold_dirty needs active_invalidate", cache.name());
    fatal_if(keepTouched && !activeInvalidate,
             "%s: keep_touched needs active_invalidate", cache.name());
    fatal_if(touchedClearAge < 0.0 || touchedClearAge >= 1.0 ||
             (touchedClearAge > 0.0 && !keepTouched),
             "%s: touched_clear_age must be in [0, 1) and needs "
             "keep_touched", cache.name());
    fatal_if(activeInvalidate && (inlineOnWrite || inlineOnRead),
             "%s: active_invalidate needs inline refresh off",
             cache.name());
    fatal_if(silentCleanDrop && !activeInvalidate,
             "%s: silent_clean_drop needs active_invalidate", cache.name());
    fatal_if(quantizedAge && (ageBits < 2 || ageBits > 16 ||
                              epochLog2 > 20),
             "%s: quantized_age needs age_bits in [2, 16] and "
             "age_epoch_log2 <= 20", cache.name());
    fatal_if(quantizedAge && inlineOnWrite,
             "%s: quantized_age needs inline refresh off", cache.name());
    fatal_if(p.refresh_scan != "set" && p.refresh_scan != "line",
             "%s: refresh_scan must be set or line", cache.name());
    fatal_if(p.refresh_mode != "policy" && p.refresh_mode != "all_strict" &&
             p.refresh_mode != "all_opp",
             "%s: refresh_mode must be policy, all_strict or all_opp",
             cache.name());
    fatal_if(raFree && refreshMode == RefreshMode::Policy,
             "%s: refresh_all_free needs a refresh-all refresh_mode",
             cache.name());

    if (refreshMode != RefreshMode::Policy) {
        // Refresh-all has no policy: no daemon, no inline refresh, no
        // stamps or reference bits
        fatal_if(topMRU > 0 || refreshDirtyDaemon || activeInvalidate ||
                 keepTouched || quantizedAge || silentCleanDrop ||
                 inlineOnWrite || inlineOnRead || lineScan,
                 "%s: refresh_mode=%s needs top_mru 0, no refresh_dirty, "
                 "active_invalidate, keep_touched, quantized_age, "
                 "silent_clean_drop or inline refresh, and "
                 "refresh_scan=set", cache.name(), p.refresh_mode);
        const unsigned num_sets = cache.tags->getNumSets();
        fatal_if(num_sets % banks != 0,
                 "%s: refresh_mode=%s needs numSets (%u) to be a multiple "
                 "of refresh_banks (%u)", cache.name(), p.refresh_mode,
                 num_sets, banks);
        raRows = num_sets / banks * assoc;
        raLastRefresh.assign(num_sets * assoc, MaxTick);
        raRow.assign(banks, raRows);
        raPeriodStart.assign(banks, 0);
        raForced.assign(banks, 0);
        for (unsigned b = 0; b < banks; b++) {
            const std::string sfx = banks == 1 ? "" : std::to_string(b);
            raEvents.push_back(std::make_unique<EventFunctionWrapper>(
                [this, b] { processRaStep(b, false); },
                cache.name() + ".refreshAll" + sfx));
            raOppEvents.push_back(std::make_unique<EventFunctionWrapper>(
                [this, b] { processRaStep(b, true); },
                cache.name() + ".refreshAllOpp" + sfx, false,
                Event::CPU_Tick_Pri + 1));
        }
    }

    if (subarrays > 1) {
        const unsigned num_sets = cache.tags->getNumSets();
        fatal_if(cache.bankPorts,
                 "%s: refresh_subarrays > 1 needs bank ports off",
                 cache.name());
        fatal_if(!lineScan && !(refreshMode == RefreshMode::AllStrict &&
                                raEven),
                 "%s: refresh_subarrays > 1 needs refresh_scan=line or "
                 "refresh-all strict with refresh_all_even", cache.name());
        fatal_if(num_sets % banks != 0 ||
                 (num_sets / banks * assoc) % subarrays != 0,
                 "%s: refresh_subarrays (%u) must divide the rows per bank "
                 "(numSets %u / banks %u x assoc %u)", cache.name(),
                 subarrays, num_sets, banks, assoc);
    }

    if (lineScan) {
        const unsigned num_sets = cache.tags->getNumSets();
        // The RTL's per-line rules: dirty, used, Shared; no MRU ranking
        fatal_if(!activeInvalidate,
                 "%s: refresh_scan=line needs active_invalidate",
                 cache.name());
        fatal_if(topMRU > 0, "%s: refresh_scan=line needs top_mru = 0",
                 cache.name());
        fatal_if(num_sets % banks != 0,
                 "%s: refresh_scan=line needs numSets (%u) to be a "
                 "multiple of refresh_banks (%u)", cache.name(), num_sets,
                 banks);

        lastVisit.assign(num_sets * assoc, MaxTick);
        scanRow.assign(banks, 0);
        slotFreeAt.assign(banks, 0);
        pendingOpp.assign(banks, PendingOpp());
        pendingOppGroup.assign(banks, std::vector<PendingOpp>());
        bankSweepStart.assign(banks, 0);
        for (unsigned b = 0; b < banks; b++) {
            lineEvents.push_back(std::make_unique<EventFunctionWrapper>(
                [this, b] { processLineStep(b); },
                cache.name() + ".refreshScanner" + std::to_string(b)));
        }
    }
}

void
RetentionManager::startup()
{
    const Tick drt = cache.tags->getDRT();
    const bool daemon = topMRU > 0 || refreshDirtyDaemon;

    if (refreshMode != RefreshMode::Policy) {
        // P = DRT - W from sweep start to sweep start; a sweep ends
        // within W, so a row waits at most P + W - rows * line < DRT
        const uint64_t drt_cyc = drt / cache.clockPeriod();
        // One bank operation refreshes one row of each subarray
        const unsigned ops = raRows / subarrays;
        const uint64_t sweep_cyc = uint64_t(ops) * lineCycles;
        // all_opp decides a cycle ahead: 2 cycles of slack
        const uint64_t slack =
            refreshMode == RefreshMode::AllOpp ? 2 : 0;
        fatal_if(sweep_cyc + slack > raWindow,
                 "%s: refresh-all sweep of %u rows x %u cycles = %lu "
                 "(+%lu) > W = %u cycles", cache.name(), raRows,
                 (unsigned)lineCycles, sweep_cyc, slack,
                 (unsigned)raWindow);
        fatal_if(raWindow >= drt_cyc,
                 "%s: refresh-all window W = %u cycles >= DRT = %lu cycles",
                 cache.name(), (unsigned)raWindow, drt_cyc);
        raPeriod = cache.cyclesToTicks(Cycles(drt_cyc - raWindow));
        tSweep = cache.cyclesToTicks(Cycles(sweep_cyc));
        const uint64_t bound_cyc = refreshMode == RefreshMode::AllStrict ?
            drt_cyc - raWindow : drt_cyc - sweep_cyc;
        inform("%s: retention DRT %lu ticks (%lu cycles), refresh-all %s%s, "
               "%u bank(s): %u rows per bank / %u subarray(s) = %u "
               "operations x %u cycles = sweep %lu cycles, "
               "window W %u, period P %lu cycles, max row interval <= %lu "
               "cycles (%.1f%% of DRT)", cache.name(), drt, drt_cyc,
               refreshMode == RefreshMode::AllStrict ? "strict" : "opp",
               raFree ? " (free: no bank time)" : "", banks, raRows,
               subarrays, ops, (unsigned)lineCycles, sweep_cyc,
               (unsigned)raWindow,
               drt_cyc - raWindow, bound_cyc,
               100.0 * double(bound_cyc) / double(drt_cyc));
        if (raEven) {
            // One row every P / rows cycles: a row is refreshed every P
            const uint64_t p_cyc = drt_cyc - raWindow;
            fatal_if(refreshMode != RefreshMode::AllStrict,
                     "%s: refresh_all_even needs refresh_mode=all_strict",
                     cache.name());
            fatal_if(p_cyc / ops < lineCycles,
                     "%s: refresh_all_even: P / operations = %lu / %u < %u "
                     "cycles per refresh", cache.name(), p_cyc, ops,
                     (unsigned)lineCycles);
            inform("%s: refresh-all even spacing: one operation (%u "
                   "row(s), one per subarray) of each bank every %.2f "
                   "cycles (P %lu / %u operations); row interval P (%.1f%% "
                   "of DRT)", cache.name(), subarrays, double(p_cyc) / ops,
                   p_cyc, ops, 100.0 * double(p_cyc) / double(drt_cyc));
        }
        for (unsigned b = 0; b < banks; b++) {
            raRow[b] = raEven ? 0 : raRows;
            raPeriodStart[b] = cache.clockEdge(Cycles(1));
            cache.schedule(*raEvents[b], raPeriodStart[b]);
        }
        return;
    }

    fatal_if(quantizedAge && !daemon,
             "%s: quantized_age needs the refresh daemon", cache.name());

    if (!daemon) {
        // No daemon to wait for: inline refreshes just before expiry
        tGuard = cache.cyclesToTicks(lineBudget) +
            Tick(margin * double(drt));
        inform("%s: retention DRT %lu ticks, no refresh daemon, "
               "T_guard %lu", cache.name(), drt, tGuard);
        return;
    }

    const unsigned num_sets = cache.tags->getNumSets();
    // Most bank operations one visit can start: a set, or one line
    const unsigned k_max = lineScan ? 1 :
        refreshDirtyDaemon ? assoc : topMRU;
    // Rounds of B sets: B scans + the previous round's refreshes
    const unsigned rounds = (num_sets + banks - 1) / banks;

    if (lineScan) {
        // Each bank's scanner: every row of the bank, each scanned and
        // at worst waiting for one operation
        // With subarrays a step covers a group of rows
        tSweep = Tick(num_sets / banks * assoc / subarrays) *
            cache.cyclesToTicks(Cycles(scanCycles + lineBudget));
        warn_if(guardSweeps != 1.0 || margin != 0.0,
                "%s: refresh_scan=line with guard_sweeps %f and margin %f: "
                "T_guard differs from the RTL's T_sweep + line",
                cache.name(), guardSweeps, margin);
    } else {
        tSweep = rounds * cache.cyclesToTicks(
            Cycles(banks * scanCycles + k_max * lineBudget));
    }
    tGuard = Tick(guardSweeps * double(tSweep)) +
        cache.cyclesToTicks(lineBudget) + Tick(margin * double(drt));

    // A demand access in the same cycle as a daemon step must see the
    // step's invalidations (it misses), so the daemon runs first
    fatal_if(activeInvalidate &&
             refreshEvent.priority() >= Event::CPU_Tick_Pri,
             "%s: the refresh daemon must run before the CPU tick",
             cache.name());

    fatal_if(tGuard >= drt,
             "%s: retention cannot be guaranteed: T_guard = %lu ticks >= "
             "DRT = %lu ticks (T_sweep = %u rounds x (%u banks x %u + "
             "%u x %u) cycles). Use a larger DRT or a faster refresh.",
             cache.name(), tGuard, drt, rounds, banks,
             (unsigned)scanCycles, k_max, (unsigned)lineBudget);

    if (quantizedAge) {
        deriveEpochThresholds(drt, k_max);
    }

    inform("%s: retention DRT %lu ticks, %u bank(s), %s scanner, T_sweep "
           "%lu, T_guard %lu (%s from %.1f%% of DRT)", cache.name(),
           drt, banks, lineScan ? "line" : "set", tSweep, tGuard,
           activeInvalidate ? "refresh or invalidate" : "force-refresh",
           100.0 * double(drt - tGuard) / double(drt));

    sweepStart = curTick();
    if (lineScan) {
        for (unsigned b = 0; b < banks; b++) {
            bankSweepStart[b] = curTick();
            cache.schedule(*lineEvents[b], cache.clockEdge(Cycles(1)));
        }
    } else {
        cache.schedule(refreshEvent, cache.clockEdge(Cycles(1)));
    }
}

Tick
RetentionManager::age(const CacheBlk *blk) const
{
    return curTick() - blk->getFullChargeTick();
}

void
RetentionManager::deriveEpochThresholds(Tick drt, unsigned k_max)
{
    const Tick period = cache.clockPeriod();
    const uint64_t epoch = uint64_t(1) << epochLog2;
    const uint64_t drt_cyc = drt / period;
    const uint64_t guard_cyc = divCeil(tGuard, period);
    const uint64_t sweep_cyc = divCeil(tSweep, period);

    // Every threshold rounds the safe way: act early. A visit that sees
    // age < F knows the exact age is below F epochs.
    deadlineEpochs = deadlineEpochsParam >= 0 ? deadlineEpochsParam :
        (drt_cyc - guard_cyc) / epoch;
    oppAgeEpochs = oppAgeEpochsParam >= 0 ? oppAgeEpochsParam :
        uint64_t(oppAge * double(drt_cyc)) / epoch;
    if (clearEpochsParam >= 0) {
        clearEpochs = clearEpochsParam;
    } else if (touchedClearAge > 0.0) {
        clearEpochs = std::max<uint64_t>(1, std::llround(
            touchedClearAge * double(drt_cyc) / double(epoch)));
    }

    // Not at the deadline at a visit: exact age < F epochs. The next
    // visit comes within T_sweep and its refreshes take k_max lines.
    const uint64_t worst = deadlineEpochs * epoch + sweep_cyc +
        k_max * uint64_t(lineBudget);
    fatal_if(deadlineEpochs == 0 || worst >= drt_cyc,
             "%s: quantized ages cannot guarantee retention: F = %u "
             "epochs x %lu + T_sweep %lu + %u x %u cycles >= DRT %lu "
             "cycles", cache.name(), deadlineEpochs, epoch, sweep_cyc,
             k_max, (unsigned)lineBudget, drt_cyc);
    // The RTL's stamp checks: F below the largest age, and a valid line
    // seen below F reaches at most F - 1 + ceil(T_sweep / epoch) at the
    // next visit, which must not wrap
    const uint64_t span = uint64_t(1) << ageBits;
    fatal_if(deadlineEpochs >= span - 1 ||
             sweep_cyc >= (span - deadlineEpochs) * epoch,
             "%s: a %u-bit stamp wraps before the deadline F = %u "
             "(T_sweep %lu cycles, epoch %lu)", cache.name(), ageBits,
             deadlineEpochs, sweep_cyc, epoch);
    fatal_if(clearEpochs > 0 && !keepTouched,
             "%s: touched_clear_epochs needs keep_touched", cache.name());

    inform("%s: quantized ages: epoch %lu cycles, %u-bit stamp, "
           "deadline F %u (acts at true age %.1f-%.1f%% of DRT), opp age "
           "%u, reference-bit clear %u epoch(s)", cache.name(), epoch,
           ageBits, deadlineEpochs,
           100.0 * double((deadlineEpochs - 1) * epoch) / double(drt_cyc),
           100.0 * double(deadlineEpochs * epoch + sweep_cyc) /
           double(drt_cyc), oppAgeEpochs, clearEpochs);
}

uint64_t
RetentionManager::epochOf(Tick t) const
{
    return (t / cache.clockPeriod()) >> epochLog2;
}

uint64_t
RetentionManager::nowEpoch() const
{
    // Speed only: the same value, computed once per tick
    if (nowEpochTick != curTick()) {
        nowEpochTick = curTick();
        nowEpochVal = epochOf(nowEpochTick);
    }
    return nowEpochVal;
}

uint64_t
RetentionManager::epochAgeExact(const CacheBlk *blk) const
{
    return nowEpoch() - epochOf(blk->getFullChargeTick());
}

unsigned
RetentionManager::epochAge(const CacheBlk *blk) const
{
    // The stamp holds the charge epoch mod 2^age_bits
    const uint64_t mask = (uint64_t(1) << ageBits) - 1;
    return ((nowEpoch() & mask) -
            (epochOf(blk->getFullChargeTick()) & mask)) & mask;
}

void
RetentionManager::checkQuantizedAges(unsigned set)
{
    for (CacheBlk *blk : cache.tags->getSetBlks(set)) {
        checkQuantizedAge(blk);
    }
}

void
RetentionManager::checkQuantizedAge(const CacheBlk *blk)
{
    const Tick drt = cache.tags->getDRT();
    const uint64_t wrap = (uint64_t(1) << ageBits) - 1;

    if (!blk->isValid()) {
        return;
    }
    if (age(blk) >= drt) {
        stats.quantExpiredAtVisit++;
        panic_if(strict, "%s: valid block %#lx reached DRT (age %lu "
                 ">= %lu) with quantized ages", cache.name(),
                 blk->getTag(), age(blk), drt);
    }
    if (epochAgeExact(blk) >= wrap) {
        stats.epochWrapRisk++;
        panic_if(strict, "%s: valid block %#lx epoch age %lu reached "
                 "the %u-bit stamp wrap", cache.name(), blk->getTag(),
                 epochAgeExact(blk), ageBits);
    }
}

void
RetentionManager::violation(const CacheBlk *blk, const char *where)
{
    stats.invariantViolations++;
    panic_if(strict, "%s: dirty block %#lx expired at %s "
             "(age %lu >= drt %lu)", cache.name(), blk->getTag(), where,
             age(blk), cache.tags->getDRT());
}

void
RetentionManager::expireOnAccess(PacketPtr pkt, PacketList &writebacks)
{
    // Only requests that use the stored data. Writebacks carry a full
    // line and replace it.
    if (!(pkt->isRead() || pkt->isWrite()) || pkt->isEviction() ||
        pkt->cmd == MemCmd::WriteClean) {
        return;
    }

    Addr blk_addr = pkt->getBlockAddr(cache.blkSize);

    // The access uses the bank of its set this cycle
    const unsigned bank = banks == 1 ? 0 :
        bankOf(cache.tags->getSetIndex({blk_addr, pkt->isSecure()}));
    bankLastAccess[bank] = curTick();
    CacheBlk *blk = cache.tags->findBlock({blk_addr, pkt->isSecure()});

    if (!blk) {
        // Stats only: a miss to a line the daemon invalidated; before
        // its expiry, the invalidation caused it
        if (activeInvalidate) {
            auto it = invalidatedAddrs.find(blk_addr);
            if (it != invalidatedAddrs.end()) {
                stats.activeInvalidateMisses++;
                if (curTick() < it->second.expiry) {
                    stats.activeInvalidateEarlyMisses++;
                    if (it->second.dirty) {
                        stats.coldDirtyEarlyMisses++;
                    }
                }
                invalidatedAddrs.erase(it);
            }
        }
        return;
    }

    // An inline refresh first reads the line; with less life left than
    // that read takes, the data is gone by the time it is read
    const Tick drt = cache.tags->getDRT();
    const bool expired = cache.tags->isExpired(blk);
    const bool too_late = !expired && inlineApplies(pkt) &&
        drt - age(blk) < cache.cyclesToTicks(inlineCycles);

    if (!expired && !too_late) {
        return;
    }
    if (too_late) {
        stats.inlineTooLate++;
    }

    const bool dirty = blk->isSet(CacheBlk::DirtyBit);

    // The block has a request in flight (O3): invalidating it would break
    // the MSHR, so it is served as usual (backdoor, counted)
    if (inFlight(blk_addr, pkt->isSecure())) {
        stats.expiredInflightHits++;
        if (dirty) {
            violation(blk, "in-flight access");
        }
        return;
    }

    stats.retentionMisses++;

    // Died recently: between 100% and 120% of DRT
    if (age(blk) < drt * 1.2) {
        stats.expiredHitPot++;
    }

    // Lazy writeback of a dirty block (fallback, counted), then
    // invalidate so the access misses
    if (dirty) {
        violation(blk, "access");
        if (pkt->isWrite()) {
            stats.lazyWritebacksWrite++;
        } else {
            stats.lazyWritebacksRead++;
        }
        PacketPtr wb_packet = cache.writebackBlk(blk);
        if (wb_packet) {
            writebacks.push_back(wb_packet);
        }
    }

    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Drop);
    cache.invalidateBlock(blk);
    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Other);
}

bool
RetentionManager::inFlight(Addr blk_addr, bool is_secure) const
{
    return cache.mshrQueue.findMatch(blk_addr, is_secure) ||
        cache.writeBuffer.findMatch(blk_addr, is_secure);
}

void
RetentionManager::onAccessHit(PacketPtr pkt, CacheBlk *blk)
{
    // A clean line leaving the cache above does not use this one
    if (pkt->cmd == MemCmd::CleanEvict) {
        return;
    }
    blk->setLastTouchTick(curTick());
    closeRefreshChain(blk, true);

    if (blk->getWasOppRefreshed()) {
        stats.refreshedBlockUtility++;
        blk->setWasOppRefreshed(false);
    }
}

bool
RetentionManager::inlineApplies(PacketPtr pkt) const
{
    return pkt->isWrite() ? inlineOnWrite : inlineOnRead;
}

Cycles
RetentionManager::hitDelay(PacketPtr pkt, CacheBlk *blk)
{
    Cycles delay(0);
    const unsigned bank = bankOf(blk->getSet());

    // A refresh occupies the block's bank: wait for it (with bank ports,
    // the hit already waited for its slot)
    if (!cache.bankPorts && bankBusyUntil[bank] > curTick()) {
        delay = cache.ticksToCycles(bankBusyUntil[bank] - curTick());
        stats.refreshStalledHits++;
        stats.refreshStallCycles += delay;
        stats.bankStalledHits[bank]++;
        stats.bankStallCycles[bank] += delay;
        maxStall = std::max(maxStall, delay);
    }

    if (!inlineApplies(pkt)) {
        return delay;
    }

    // Too little life left for the daemon to come back in time: this hit
    // recharges the line (read, merge a store, write the whole line)
    const Tick drt = cache.tags->getDRT();
    const Tick a = age(blk);
    if (drt - a >= tGuard) {
        return delay;
    }

    if (pkt->isWrite()) {
        stats.refreshInlineWrite++;
        if (!blk->isSet(CacheBlk::DirtyBit)) {
            stats.cleanToDirtyInDanger++;
        }
    } else {
        stats.refreshInlineRead++;
    }
    stats.ageAtInlineRefresh.sample(double(a) / double(drt));
    stats.inlineRefreshCycles += inlineCycles;

    DPRINTF(Cache, "Inline refresh: %s block %#lx age %lu\n",
            pkt->isWrite() ? "write" : "read", blk->getTag(), a);

    occupyBank(bank);
    blk->setFullChargeTick(curTick());

    return delay + inlineCycles;
}

Tick
RetentionManager::occupyBank(unsigned bank, bool refresh, bool force)
{
    // Full-line read + write; refreshes are serialized within a bank.
    // With bank ports a refresh blocks every slot of its bank: a forced
    // one from the next cycle, ahead of the waiting accesses; an
    // opportunistic one now (it is issued only with the bank idle).
    // Subarrays: later lines of an open group share its operation
    if (groupOpen && groupStarted) {
        if (refresh && !groupRefreshCounted) {
            stats.bankRefreshes[bank]++;
            groupRefreshCounted = true;
        }
        return groupStart;
    }
    Tick start = cache.bankPorts ?
        cache.bankPorts->block(bank, force ? cache.clockEdge(Cycles(1)) :
                               curTick(), lineCycles, force) :
        std::max(curTick(), bankBusyUntil[bank]);
    bankBusyUntil[bank] = start + cache.cyclesToTicks(lineCycles);
    stats.bankBusyCycles[bank] += lineCycles;
    if (refresh) {
        stats.bankRefreshes[bank]++;
    }
    if (groupOpen) {
        groupStarted = true;
        groupStart = start;
        groupRefreshCounted = refresh;
    }
    return start;
}

void
RetentionManager::onFullLineWrite(CacheBlk *blk)
{
    blk->setFullChargeTick(curTick());
    blk->setLastTouchTick(curTick());

    // Refilled without a demand miss (e.g. a prefetch): not a miss later
    if (activeInvalidate) {
        invalidatedAddrs.erase(cache.regenerateBlkAddr(blk));
    }
}

void
RetentionManager::onWritebackData(CacheBlk *blk)
{
    // A full-line write: charged now and used (the reference bit is set
    // at the charge, so it is recorded with it)
    blk->setFullChargeTick(curTick());
    blk->setLastTouchTick(curTick());
    blk->setWbChargeTick(curTick());

    if (activeInvalidate) {
        invalidatedAddrs.erase(cache.regenerateBlkAddr(blk));
    }
}

bool
RetentionManager::chargedByWriteback(const CacheBlk *blk) const
{
    return blk->getWbChargeTick() != MaxTick &&
        blk->getWbChargeTick() == blk->getFullChargeTick() &&
        blk->getRefClearTick() < blk->getFullChargeTick();
}

void
RetentionManager::onEvict(CacheBlk *blk)
{
    // The temp block is outside the tags and never charged
    if (blk == cache.tempBlock || !blk->isValid() ||
        !cache.tags->isExpired(blk)) {
        return;
    }

    if (blk->isSet(CacheBlk::DirtyBit)) {
        stats.expiredDirtyEvictions++;
        violation(blk, "eviction");
    } else {
        stats.expiredCleanEvictions++;
    }
}

void
RetentionManager::onFunctional(CacheBlk *blk)
{
    if (!blk->isValid() || !cache.tags->isExpired(blk)) {
        return;
    }

    stats.expiredFunctional++;
    if (blk->isSet(CacheBlk::DirtyBit)) {
        violation(blk, "functional access");
    }
}

bool
RetentionManager::expireForPrefetch(CacheBlk *blk)
{
    if (!blk->isValid() || !cache.tags->isExpired(blk) ||
        inFlight(cache.regenerateBlkAddr(blk), blk->isSecure())) {
        return false;
    }

    if (blk->isSet(CacheBlk::DirtyBit)) {
        // Lost dirty data: the demand access does the lazy writeback
        stats.expiredPrefetchDirty++;
        violation(blk, "prefetch");
        return false;
    }

    stats.expiredPrefetchRefills++;
    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Drop);
    cache.invalidateBlock(blk);
    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Other);
    return true;
}

void
RetentionManager::onInvalidate(CacheBlk *blk)
{
    if (blk != cache.tempBlock && blk->isValid()) {
        closeRefreshChain(blk, false);
    }
}

void
RetentionManager::closeRefreshChain(CacheBlk *blk, bool hit)
{
    CacheBlk::RefreshChain &chain = blk->getRefreshChain();
    if (!chain.open()) {
        return;
    }

    if (!hit) {
        stats.refreshDeadClean += chain.clean;
        stats.refreshDeadDirty += chain.dirty;
    } else if (curTick() >= chain.expiry) {
        stats.refreshNeededClean += chain.clean;
        stats.refreshNeededDirty += chain.dirty;
    } else {
        stats.refreshEarlyClean += chain.clean;
        stats.refreshEarlyDirty += chain.dirty;
    }
    chain = CacheBlk::RefreshChain();
}

double
RetentionManager::openChainRefreshes(bool dirty) const
{
    double n = 0;
    cache.tags->forEachBlk([&n, dirty](CacheBlk &blk) {
        if (blk.isValid() && blk.getRefreshChain().open()) {
            n += dirty ? blk.getRefreshChain().dirty :
                blk.getRefreshChain().clean;
        }
    });
    return n;
}

void
RetentionManager::onSnoop(CacheBlk *blk)
{
    if (!blk->isValid() || !cache.tags->isExpired(blk)) {
        return;
    }

    stats.expiredSnoops++;
    if (blk->isSet(CacheBlk::DirtyBit)) {
        violation(blk, "snoop");
    }
}

bool
RetentionManager::bankIdle(unsigned bank) const
{
    return bankFreeAt(bank) <= curTick() &&
        curTick() >= bankLastAccess[bank] + cache.clockPeriod();
}

Tick
RetentionManager::bankFreeAt(unsigned bank) const
{
    return cache.bankPorts ?
        std::max(bankBusyUntil[bank], cache.bankPorts->freeAt(bank)) :
        bankBusyUntil[bank];
}

void
RetentionManager::lateStartCheck(CacheBlk *blk, Tick start)
{
    const Tick now = curTick();
    if (start <= now || !blk->isValid() || cache.tags->isExpired(blk) ||
        age(blk) + (start - now) < cache.tags->getDRT()) {
        return;
    }
    stats.lateRefreshStarts++;
    if (blk->isSet(CacheBlk::DirtyBit)) {
        stats.lateRefreshStartsDirty++;
        violation(blk, "late refresh start");
    }
}

std::vector<CacheBlk *>
RetentionManager::topMRUBlks(unsigned set, unsigned n) const
{
    std::vector<CacheBlk *> blks;
    for (CacheBlk *blk : cache.tags->getSetBlks(set)) {
        if (blk->isValid()) {
            blks.push_back(blk);
        }
    }

    std::stable_sort(blks.begin(), blks.end(),
        [](const CacheBlk *a, const CacheBlk *b) {
            return a->getLastTouchTick() > b->getLastTouchTick();
        });

    if (blks.size() > n) {
        blks.resize(n);
    }
    return blks;
}

Tick
RetentionManager::refresh(CacheBlk *blk, bool force)
{
    const Tick drt = cache.tags->getDRT();

    DPRINTF(Cache, "RefreshDaemon: %s block %#lx age %lu\n",
            force ? "FORCE" : "OPP", blk->getTag(), age(blk));

    if (force) {
        stats.forceRefresh++;
        stats.ageAtForceRefresh.sample(double(age(blk)) / double(drt));
    } else {
        stats.oppRefresh++;
        stats.ageAtOppRefresh.sample(double(age(blk)) / double(drt));
    }
    stats.totalRefreshes++;

    // Stats only: the chain of refreshes until the next hit
    CacheBlk::RefreshChain &chain = blk->getRefreshChain();
    if (!chain.open()) {
        chain.expiry = blk->getFullChargeTick() + drt;
    }
    if (blk->isSet(CacheBlk::DirtyBit)) {
        stats.refreshDirty++;
        chain.dirty++;
    } else {
        stats.refreshClean++;
        chain.clean++;
    }

    const Tick start = occupyBank(bankOf(blk->getSet()), true, force);
    lateStartCheck(blk, start);
    noteRowRefresh(blk->getSet() * assoc + blk->getWay(), start,
                   age(blk) + (start - curTick()));

    blk->setFullChargeTick(curTick());
    blk->setWasOppRefreshed(true);
    return start;
}

void
RetentionManager::maybeRefresh(CacheBlk *blk)
{
    const Tick drt = cache.tags->getDRT();
    const Tick a = age(blk);

    // Would expire before the daemon comes back: must refresh now. With
    // quantized ages, the deadline and opp ages are in epochs.
    bool deadline, opp;
    if (quantizedAge) {
        const unsigned e = epochAge(blk);
        deadline = e >= deadlineEpochs;
        opp = e >= oppAgeEpochs;
    } else {
        deadline = drt - a < tGuard;
        opp = a >= oppAge * double(drt);
    }

    if (deadline) {
        if (quantizedAge) {
            stats.epochAgeAtDeadline.sample(epochAge(blk));
        }
        refresh(blk, true);
    } else if (opp && bankIdle(bankOf(blk->getSet()))) {
        refresh(blk, false);
    }
}

void
RetentionManager::flushVisitBatch()
{
    if (visitBatchCount) {
        stats.setVisitInterval.sample(double(visitBatchVal),
                                      visitBatchCount);
        visitBatchCount = 0;
    }
}

void
RetentionManager::recordVisit(unsigned idx)
{
    const Tick last = lastVisit[idx];
    lastVisit[idx] = curTick();
    if (last == MaxTick) {
        return;
    }

    // The guarantee assumes the daemon comes back within T_sweep
    const Tick interval = curTick() - last;
    const Cycles c = cache.ticksToCycles(interval);
    if (subarrays > 1) {
        // Speed only: equal consecutive samples go to the histogram as one
        // (bucket counts, samples, mean and stdev are the same; gmean can
        // differ in its last digits, its log-sum is added in another order)
        if (visitBatchCount &&
            (uint64_t(c) != visitBatchVal || visitBatchCount == 1 << 30)) {
            flushVisitBatch();
        }
        visitBatchVal = c;
        visitBatchCount++;
    } else {
        stats.setVisitInterval.sample(c);
    }
    maxVisitInterval = std::max(maxVisitInterval, interval);
    if (interval > tSweep) {
        stats.visitIntervalOverruns++;
        panic_if(strict && activeInvalidate, "%s: %s %u revisited after "
                 "%lu ticks > T_sweep = %lu", cache.name(),
                 lineScan ? "row" : "set", idx, interval, tSweep);
    }
}

bool
RetentionManager::touched(const CacheBlk *blk) const
{
    // A fill sets both ticks, a refresh only the charge, a hit only the
    // touch; the daemon may clear the bit in between
    return blk->getLastTouchTick() >
        std::max(blk->getFullChargeTick(), blk->getRefClearTick()) ||
        chargedByWriteback(blk);
}

bool
RetentionManager::isColdDirty(const CacheBlk *blk,
                              const std::vector<CacheBlk *> &mru) const
{
    if (coldDirty != ColdDirty::Refresh && keepTouched) {
        return !touched(blk) &&
            std::find(mru.begin(), mru.end(), blk) == mru.end();
    }

    switch (coldDirty) {
      case ColdDirty::MRU:
        return std::find(mru.begin(), mru.end(), blk) == mru.end();
      case ColdDirty::Touched:
        // Not hit since its last fill or refresh (a fill sets both)
        return blk->getLastTouchTick() <= blk->getFullChargeTick() &&
            !chargedByWriteback(blk);
      default:
        return false;
    }
}

void
RetentionManager::invalidateUnkept(unsigned set,
                                   const std::vector<CacheBlk *> &mru)
{
    const Tick drt = cache.tags->getDRT();

    for (CacheBlk *blk : cache.tags->getSetBlks(set)) {
        // The loops above refreshed every kept line inside T_guard, so a
        // line left inside it is clean and unkept, or cold and dirty
        const bool dirty = blk->isSet(CacheBlk::DirtyBit);
        if (!blk->isValid() || (dirty && !isColdDirty(blk, mru))) {
            continue;
        }
        if (quantizedAge) {
            if (epochAge(blk) < deadlineEpochs) {
                continue;
            }
            stats.epochAgeAtDeadline.sample(epochAge(blk));
        } else if (age(blk) + tGuard <= drt) {
            continue;
        }

        DropKeep keep;
        const LineOp op = planDrop(blk, keep);
        applyDrop(blk, op, keep);
    }
}

RetentionManager::LineOp
RetentionManager::planDrop(CacheBlk *blk, DropKeep &keep) const
{
    const bool dirty = blk->isSet(CacheBlk::DirtyBit);
    // A clean line dropped silently needs no write-buffer entry
    const bool silent = silentCleanDrop && !dirty;

    // A pending upgrade expects a Shared line to stay: keep it. The
    // coherence state is read with the other tag bits.
    if (!blk->isSet(CacheBlk::WritableBit)) {
        keep = DropKeep::Shared;
    } else if (inFlight(cache.regenerateBlkAddr(blk), blk->isSecure())) {
        // Simulator check: a writable line has no request in flight
        keep = DropKeep::Inflight;
    } else if (!silent && cache.writeBuffer.isFull()) {
        // No write-buffer entry for the eviction (the rest is reserved
        // for the writebacks of fills in flight): keep the line until
        // the next visit
        keep = DropKeep::WbFull;
    } else {
        keep = DropKeep::None;
    }

    if (keep != DropKeep::None) {
        return LineOp::Force;
    }
    return dirty ? LineOp::Wb : LineOp::Ce;
}

Tick
RetentionManager::applyDrop(CacheBlk *blk, LineOp op, DropKeep keep)
{
    const Tick drt = cache.tags->getDRT();
    const bool dirty = blk->isSet(CacheBlk::DirtyBit);
    const unsigned bank = bankOf(blk->getSet());

    if (op == LineOp::Force) {
        switch (keep) {
          case DropKeep::Shared:
            stats.activeKeptShared++;
            break;
          case DropKeep::Inflight:
            stats.activeInvalidateInflight++;
            panic_if(strict, "%s: writable block %#lx has a request in "
                     "flight", cache.name(), blk->getTag());
            break;
          case DropKeep::WbFull:
            stats.activeWbFullKept++;
            break;
          default:
            panic("%s: forced drop without a reason", cache.name());
        }
        return refresh(blk, true);
    }

    panic_if(op != LineOp::Wb && op != LineOp::Ce,
             "%s: applyDrop with no drop", cache.name());

    DPRINTF(Cache, "RefreshDaemon: %s block %#lx age %lu\n",
            dirty ? "WRITEBACK" : "INVALIDATE", blk->getTag(),
            age(blk));

    // A dirty drop reads the line from its bank for the writeback; a
    // clean drop (CleanEvict) asks nothing of the data array
    const Tick busy = bankBusyUntil[bank];
    Tick start = curTick();
    if (dirty) {
        stats.coldDirtyWritebacks++;
        // With bank ports the line read takes the next read slot,
        // ahead of the waiting reads (the eviction below books no other)
        if (cache.bankPorts) {
            start = cache.bankPorts->priorityRead(
                bank, cache.clockEdge(Cycles(1)));
            cache.bankPortsWbBooked = true;
        } else {
            start = occupyBank(bank, false);
        }
    } else {
        stats.activeInvalidations++;
    }
    if (cache.tags->isExpired(blk)) {
        stats.activeInvalidateExpired++;
        if (dirty) {
            violation(blk, "cold writeback");
        }
    }
    stats.ageAtActiveInvalidate.sample(double(age(blk)) / double(drt));
    maxChargeAgeAtDrop = std::max(maxChargeAgeAtDrop,
                                  age(blk) + (start - curTick()));
    invalidatedAddrs[cache.regenerateBlkAddr(blk)] =
        {blk->getFullChargeTick() + drt, dirty};

    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Drop);

    // Silent drop: invalidate without telling the level below (no
    // CleanEvict); it keeps a stale holder bit and answers later
    // snoops for the line with a miss
    if (silentCleanDrop && !dirty) {
        stats.silentCleanDrops++;
        cache.invalidateBlock(blk);
    } else {
        // Same message as any eviction (CleanEvict or WritebackDirty),
        // queued now so the next check sees the entry taken
        PacketList writebacks;
        cache.evictBlock(blk, writebacks);
        cache.doWritebacks(writebacks,
                           cache.clockEdge(cache.forwardLatency));
    }
    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Other);

    panic_if(!dirty && bankBusyUntil[bank] != busy,
             "%s: a clean drop occupied bank %u", cache.name(), bank);
    return start;
}

void
RetentionManager::processRefreshEvent()
{
    BaseTags *tags = cache.tags;

    DPRINTF(Cache, "RefreshDaemon: set %d\n", refreshSetIdx);

    recordVisit(refreshSetIdx);

    if (quantizedAge) {
        checkQuantizedAges(refreshSetIdx);
    }

    // Clear the reference bit of lines past touched_clear_age, once per
    // charge: from then on the bit means "hit since this point". With
    // quantized ages the clear age is in epochs.
    for (CacheBlk *blk : tags->getSetBlks(refreshSetIdx)) {
        clearRefBit(blk);
    }

    // Recency order of the set (unchanged by refreshes in this step)
    const std::vector<CacheBlk *> mru = topMRU > 0 ?
        topMRUBlks(refreshSetIdx, topMRU) : std::vector<CacheBlk *>();

    if (refreshDirtyDaemon) {
        for (CacheBlk *blk : tags->getSetBlks(refreshSetIdx)) {
            if (!blk->isValid() || !blk->isSet(CacheBlk::DirtyBit)) {
                continue;
            }

            if (tags->isExpired(blk)) {
                // Refreshing revives lost data (backdoor, counted)
                stats.daemonRevivedDirty++;
                violation(blk, "daemon visit");
                refresh(blk, true);
                continue;
            }

            // A cold line is written back at T_guard, never refreshed
            if (isColdDirty(blk, mru)) {
                continue;
            }

            maybeRefresh(blk);
        }
    }

    if (topMRU > 0) {
        for (CacheBlk *blk : mru) {
            // Refreshed by the dirty loop above in this same step
            if (blk->getFullChargeTick() == curTick()) {
                continue;
            }
            // Left by the dirty loop for the writeback
            if (blk->isSet(CacheBlk::DirtyBit) && isColdDirty(blk, mru)) {
                continue;
            }
            // A clean block that already expired has nothing to save
            if (tags->isExpired(blk)) {
                continue;
            }
            maybeRefresh(blk);
        }
    }

    // Clean lines kept only by their reference bit (dirty touched lines
    // are kept by the dirty loop)
    if (keepTouched) {
        for (CacheBlk *blk : tags->getSetBlks(refreshSetIdx)) {
            if (!blk->isValid() || blk->isSet(CacheBlk::DirtyBit) ||
                !touched(blk) || tags->isExpired(blk) ||
                std::find(mru.begin(), mru.end(), blk) != mru.end()) {
                continue;
            }
            const Tick before = blk->getFullChargeTick();
            maybeRefresh(blk);
            if (blk->getFullChargeTick() != before) {
                stats.refreshTouched++;
            }
        }
    }

    if (activeInvalidate) {
        invalidateUnkept(refreshSetIdx, mru);
    }

    refreshSetIdx = (refreshSetIdx + 1) % tags->getNumSets();
    if (refreshSetIdx == 0) {
        stats.daemonSweepCycles.sample(
            cache.ticksToCycles(curTick() - sweepStart));
        sweepStart = curTick();
    }

    // Next set after scanning this one, once the next set's bank has
    // finished its refreshes (with one bank: this set's refreshes)
    const Tick busy = bankBusyUntil[bankOf(refreshSetIdx)];
    Cycles wait = scanCycles;
    if (busy > curTick()) {
        wait = std::max(wait, cache.ticksToCycles(busy - curTick()));
    }
    cache.schedule(refreshEvent, cache.clockEdge(wait));
}

void
RetentionManager::clearRefBit(CacheBlk *blk)
{
    // Once per charge: from then on the bit means "hit since this point"
    if (!blk->isValid() ||
        blk->getRefClearTick() >= blk->getFullChargeTick()) {
        return;
    }

    bool at_clear_age;
    if (quantizedAge) {
        at_clear_age = clearEpochs > 0 && epochAge(blk) >= clearEpochs;
    } else {
        at_clear_age = touchedClearAge > 0.0 &&
            age(blk) >= Tick(touchedClearAge *
                             double(cache.tags->getDRT()));
    }
    if (at_clear_age) {
        blk->setRefClearTick(curTick());
        stats.refBitClears++;
    }
}

bool
RetentionManager::atDeadline(const CacheBlk *blk) const
{
    if (quantizedAge) {
        return epochAge(blk) >= deadlineEpochs;
    }
    // Also true for an expired line
    return age(blk) + tGuard > cache.tags->getDRT();
}

bool
RetentionManager::atOppAge(const CacheBlk *blk) const
{
    if (quantizedAge) {
        return epochAge(blk) >= oppAgeEpochs;
    }
    return age(blk) >= oppAge * double(cache.tags->getDRT());
}

RetentionManager::LineDecision
RetentionManager::decideLine(CacheBlk *blk) const
{
    LineDecision d;
    if (!blk->isValid()) {
        return d;
    }

    const bool dirty = blk->isSet(CacheBlk::DirtyBit);
    const bool expired = cache.tags->isExpired(blk);

    // Refreshing revives lost data (backdoor, counted)
    if (dirty && expired) {
        d.op = LineOp::Force;
        d.revived = true;
        return d;
    }

    // The same keep rules as the set scanner, for one line: dirty lines
    // unless cold, clean lines hit since their last charge (an expired
    // clean line has nothing to save)
    const bool kept = dirty ? !isColdDirty(blk, {}) :
        keepTouched && touched(blk) && !expired;
    const bool deadline = atDeadline(blk);

    if (kept) {
        d.op = deadline ? LineOp::Force :
            atOppAge(blk) ? LineOp::Opp : LineOp::None;
        return d;
    }
    if (!deadline) {
        return d;
    }

    d.drop = true;
    d.op = planDrop(blk, d.keep);
    return d;
}

void
RetentionManager::processLineStep(unsigned bank)
{
    if (subarrays > 1) {
        processGroupStep(bank);
        return;
    }

    const Tick now = curTick();
    const unsigned rows = cache.tags->getNumSets() / banks * assoc;

    // The slot's opportunistic refresh issues in a step with the bank
    // idle. The RTL restores whatever line the row holds; a row that
    // changed is not refreshed here.
    PendingOpp &opp = pendingOpp[bank];
    if (opp.blk && bankIdle(bank)) {
        if (opp.blk->isValid() &&
            cache.regenerateBlkAddr(opp.blk) == opp.addr) {
            if (keepTouched && !opp.blk->isSet(CacheBlk::DirtyBit)) {
                stats.refreshTouched++;
            }
            refresh(opp.blk, false);
        } else {
            stats.oppStale++;
        }
        opp = PendingOpp();
    }

    // Row: set_hi outer, way inner
    const unsigned row = scanRow[bank];
    const unsigned set = bank + banks * (row / assoc);
    const unsigned way = row % assoc;
    CacheBlk *blk = cache.tags->getSetBlks(set)[way];

    DPRINTF(Cache, "RefreshDaemon: bank %u set %u way %u\n", bank, set,
            way);

    clearRefBit(blk);
    const LineDecision d = decideLine(blk);

    // retire = !needs_bank || bank_ready: a row needing the bank waits
    // while the slot holds an unissued forced refresh or writeback read
    // (a slot opp is replaced); a CleanEvict needs no bank
    const bool needs_bank = d.op == LineOp::Opp ||
        d.op == LineOp::Force || d.op == LineOp::Wb;
    if (needs_bank && slotFreeAt[bank] > now) {
        stats.scanSlotWaits++;
        stats.scanSlotWaitCycles +=
            cache.ticksToCycles(slotFreeAt[bank] - now);
        cache.schedule(*lineEvents[bank], slotFreeAt[bank]);
        return;
    }

    recordVisit(set * assoc + way);
    if (quantizedAge) {
        checkQuantizedAge(blk);
    }

    if (needs_bank && opp.blk) {
        stats.oppReplaced++;
        opp = PendingOpp();
    }

    Tick start = now;
    if (d.revived) {
        stats.daemonRevivedDirty++;
        violation(blk, "daemon visit");
        start = refresh(blk, true);
    } else if (d.drop) {
        if (quantizedAge) {
            stats.epochAgeAtDeadline.sample(epochAge(blk));
        }
        start = applyDrop(blk, d.op, d.keep);
    } else if (d.op == LineOp::Force) {
        if (quantizedAge) {
            stats.epochAgeAtDeadline.sample(epochAge(blk));
        }
        if (keepTouched && !blk->isSet(CacheBlk::DirtyBit)) {
            stats.refreshTouched++;
        }
        start = refresh(blk, true);
    } else if (d.op == LineOp::Opp) {
        opp.blk = blk;
        opp.addr = cache.regenerateBlkAddr(blk);
    }

    // A forced refresh or writeback read holds the slot until it starts
    if (d.op == LineOp::Force || d.op == LineOp::Wb) {
        slotFreeAt[bank] = start;
    }

    scanRow[bank] = (row + 1) % rows;
    if (scanRow[bank] == 0) {
        stats.daemonSweepCycles.sample(
            cache.ticksToCycles(now - bankSweepStart[bank]));
        bankSweepStart[bank] = now;
    }
    cache.schedule(*lineEvents[bank], cache.clockEdge(scanCycles));
}

void
RetentionManager::processGroupStep(unsigned bank)
{
    const Tick now = curTick();
    const unsigned rows = cache.tags->getNumSets() / banks * assoc;

    // The slot's opportunistic group issues in a step with the bank idle:
    // one operation restores each of its lines whose row is unchanged
    std::vector<PendingOpp> &opp = pendingOppGroup[bank];
    if (!opp.empty() && bankIdle(bank)) {
        beginGroupOp();
        for (const PendingOpp &p : opp) {
            if (p.blk->isValid() &&
                cache.regenerateBlkAddr(p.blk) == p.addr) {
                if (keepTouched && !p.blk->isSet(CacheBlk::DirtyBit)) {
                    stats.refreshTouched++;
                }
                refresh(p.blk, false);
            } else {
                stats.oppStale++;
            }
        }
        endGroupOp();
        opp.clear();
    }

    // The group: subarrays consecutive rows of the bank (set_hi outer,
    // way inner), one in each subarray
    const unsigned row0 = scanRow[bank];
    std::vector<CacheBlk *> &blks = groupBlks;
    std::vector<LineDecision> &d = groupDecisions;
    blks.resize(subarrays);
    d.resize(subarrays);
    bool needs_bank = false;
    bool forced = false;
    for (unsigned g = 0; g < subarrays; g++) {
        const unsigned row = row0 + g;
        const int set = bank + banks * (row / assoc);
        // The block of (set, way) without copying the set (getSetBlks)
        blks[g] = static_cast<CacheBlk *>(
            cache.tags->findBlockBySetAndWay(set, row % assoc));
        clearRefBit(blks[g]);
        d[g] = decideLine(blks[g]);
        const bool f = d[g].op == LineOp::Force || d[g].op == LineOp::Wb;
        forced = forced || f;
        needs_bank = needs_bank || f || d[g].op == LineOp::Opp;
    }

    // As one line: a group needing the bank waits while the slot holds
    // an unissued forced operation
    if (needs_bank && slotFreeAt[bank] > now) {
        stats.scanSlotWaits++;
        stats.scanSlotWaitCycles +=
            cache.ticksToCycles(slotFreeAt[bank] - now);
        cache.schedule(*lineEvents[bank], slotFreeAt[bank]);
        return;
    }

    for (unsigned g = 0; g < subarrays; g++) {
        recordVisit(blks[g]->getSet() * assoc + blks[g]->getWay());
        if (quantizedAge) {
            checkQuantizedAge(blks[g]);
        }
    }

    if (needs_bank && !opp.empty()) {
        stats.oppReplaced += opp.size();
        opp.clear();
    }

    // A forced group: one operation serves its refreshes (opportunistic
    // ones too) and writeback reads; CleanEvicts need no bank
    if (forced) {
        beginGroupOp();
    }
    for (unsigned g = 0; g < subarrays; g++) {
        CacheBlk *blk = blks[g];
        if (d[g].revived) {
            stats.daemonRevivedDirty++;
            violation(blk, "daemon visit");
            refresh(blk, true);
        } else if (d[g].drop) {
            if (quantizedAge) {
                stats.epochAgeAtDeadline.sample(epochAge(blk));
            }
            applyDrop(blk, d[g].op, d[g].keep);
        } else if (d[g].op == LineOp::Force) {
            if (quantizedAge) {
                stats.epochAgeAtDeadline.sample(epochAge(blk));
            }
            if (keepTouched && !blk->isSet(CacheBlk::DirtyBit)) {
                stats.refreshTouched++;
            }
            refresh(blk, true);
        } else if (d[g].op == LineOp::Opp) {
            if (forced) {
                if (keepTouched && !blk->isSet(CacheBlk::DirtyBit)) {
                    stats.refreshTouched++;
                }
                refresh(blk, false);
            } else {
                opp.push_back({blk, cache.regenerateBlkAddr(blk)});
            }
        }
    }
    if (forced) {
        // The operation holds the slot until it starts
        slotFreeAt[bank] = groupStarted ? groupStart : now;
        endGroupOp();
    }

    scanRow[bank] = (row0 + subarrays) % rows;
    if (scanRow[bank] == 0) {
        stats.daemonSweepCycles.sample(
            cache.ticksToCycles(now - bankSweepStart[bank]));
        bankSweepStart[bank] = now;
    }
    cache.schedule(*lineEvents[bank], cache.clockEdge(scanCycles));
}

void
RetentionManager::processRaStep(unsigned bank, bool opp_step)
{
    if (raEven) {
        processRaEven(bank);
        return;
    }
    if (cache.bankPorts) {
        processRaPorts(bank, opp_step);
        return;
    }

    const Tick now = curTick();
    const Tick period = cache.clockPeriod();
    const Tick line = cache.cyclesToTicks(lineCycles);
    EventFunctionWrapper &ev = *raEvents[bank];
    EventFunctionWrapper &opp_ev = *raOppEvents[bank];

    if (raRow[bank] == raRows) {
        // Period start (the pre-CPU event at raPeriodStart)
        raRow[bank] = 0;
        raForced[bank] = refreshMode == RefreshMode::AllStrict;
        if (!raForced[bank]) {
            // all_opp: decide after this cycle's demand accesses
            cache.schedule(opp_ev, now);
            return;
        }
    }

    if (opp_step) {
        // After the CPU tick: the bank is idle this cycle if no demand
        // access used it and no refresh is in progress. Invariant: from
        // the next cycle, the remaining rows still fit before the end of
        // W back to back.
        const Tick end = raPeriodStart[bank] +
            cache.cyclesToTicks(raWindow);
        const Tick rest = Tick(raRows - raRow[bank]) * line;
        const bool idle = bankFreeAt(bank) <= now &&
            bankLastAccess[bank] != now;
        if (!idle) {
            if (now + 2 * period + rest > end) {
                // Waiting another cycle could miss W: the rest back to
                // back from the next cycle, preempting demand
                raForced[bank] = true;
                cache.schedule(ev, now + period);
            } else {
                cache.schedule(opp_ev, now + period);
            }
            return;
        }
    }

    // Row of the bank: set_hi outer, way inner (as the line scanner)
    const unsigned r = raRow[bank];
    const unsigned set = bank + banks * (r / assoc);
    const unsigned way = r % assoc;
    const unsigned row = set * assoc + way;
    CacheBlk *blk = cache.tags->getSetBlks(set)[way];

    if (raForced[bank]) {
        stats.raForcedRefreshes++;
    } else {
        stats.raOppRefreshes++;
    }
    stats.raRowRefreshes++;
    // Without bank ports nothing else uses the bank and the refresh
    // starts now; with them it starts after the accesses already waiting
    const Tick start = raFree ? now : occupyBank(bank);

    if (raLastRefresh[row] != MaxTick) {
        raMaxRowInterval = std::max(raMaxRowInterval,
                                    start - raLastRefresh[row]);
    }
    raLastRefresh[row] = start;

    if (blk->isValid()) {
        stats.raValidRefreshes++;
        // The charge is recorded now, before a late start: ages read
        // later are an upper bound
        const Tick drt = cache.tags->getDRT();
        if (cache.tags->isExpired(blk) ||
            (start > now && age(blk) + (start - now) >= drt)) {
            stats.raLate++;
            panic_if(strict, "%s: refresh-all reached row %u (block %#lx) "
                     "after it expired (age %lu >= drt %lu)", cache.name(),
                     row, blk->getTag(), age(blk), cache.tags->getDRT());
            if (blk->isSet(CacheBlk::DirtyBit)) {
                violation(blk, "refresh-all");
            }
        }
        raMaxChargeAge = std::max(raMaxChargeAge, age(blk) + (start - now));
        blk->setFullChargeTick(now);
    }

    if (++raRow[bank] < raRows) {
        cache.schedule(raForced[bank] ? ev : opp_ev, start + line);
        return;
    }

    // Sweep done: idle until the next period (a sweep the bank ports
    // stretched past it starts the next one late)
    stats.raSweeps++;
    stats.daemonSweepCycles.sample(
        cache.ticksToCycles(start + line - raPeriodStart[bank]));
    raPeriodStart[bank] += raPeriod;
    cache.schedule(ev, std::max(raPeriodStart[bank], start + line));
}

void
RetentionManager::noteRowRefresh(unsigned row, Tick start, Tick charge_age)
{
    if (rowLastRefresh[row] != MaxTick) {
        maxRowGap = std::max(maxRowGap, start - rowLastRefresh[row]);
    }
    rowLastRefresh[row] = start;
    maxChargeAgeAtRefresh = std::max(maxChargeAgeAtRefresh, charge_age);
}

Tick
RetentionManager::raBlock(unsigned bank, Tick earliest, bool forced)
{
    Tick start;
    if (raFree) {
        start = earliest;
    } else if (cache.bankPorts) {
        start = cache.bankPorts->block(bank, earliest, lineCycles, forced);
    } else {
        start = std::max(earliest, bankBusyUntil[bank]);
    }
    if (!raFree) {
        bankBusyUntil[bank] = start + cache.cyclesToTicks(lineCycles);
        stats.bankBusyCycles[bank] += lineCycles;
        stats.bankRefreshes[bank]++;
    }
    return start;
}

void
RetentionManager::raRefreshRow(unsigned bank, Tick start)
{
    const Tick now = curTick();
    const unsigned r = raRow[bank];
    const unsigned set = bank + banks * (r / assoc);
    const unsigned way = r % assoc;
    const unsigned row = set * assoc + way;
    CacheBlk *blk = cache.tags->getSetBlks(set)[way];

    stats.raRowRefreshes++;
    if (raLastRefresh[row] != MaxTick) {
        raMaxRowInterval = std::max(raMaxRowInterval,
                                    start - raLastRefresh[row]);
    }
    raLastRefresh[row] = start;

    if (blk->isValid()) {
        stats.raValidRefreshes++;
        // The charge is recorded now, before a later start: ages read
        // later are an upper bound
        const Tick drt = cache.tags->getDRT();
        const Tick age_at_start = age(blk) + (start - now);
        if (cache.tags->isExpired(blk) || age_at_start >= drt) {
            stats.raLate++;
            panic_if(strict, "%s: refresh-all reached row %u (block %#lx) "
                     "after it expired (age %lu >= drt %lu)", cache.name(),
                     row, blk->getTag(), age_at_start, drt);
            if (blk->isSet(CacheBlk::DirtyBit)) {
                violation(blk, "refresh-all");
            }
        }
        raMaxChargeAge = std::max(raMaxChargeAge, age_at_start);
        blk->setFullChargeTick(now);
    }
    raRow[bank]++;
}

Tick
RetentionManager::raEvenSlot(unsigned bank) const
{
    const uint64_t p_cyc = raPeriod / cache.clockPeriod();
    return raPeriodStart[bank] + cache.cyclesToTicks(
        Cycles(uint64_t(raRow[bank] / subarrays) * p_cyc /
               (raRows / subarrays)));
}

void
RetentionManager::processRaEven(unsigned bank)
{
    // The row's slot: the refresh starts on the next cycle, ahead of
    // the waiting accesses
    // One operation: a row of each subarray
    const Tick start = raBlock(bank, cache.clockEdge(Cycles(1)), true);
    for (unsigned g = 0; g < subarrays; g++) {
        stats.raForcedRefreshes++;
        raRefreshRow(bank, start);
    }

    if (raRow[bank] == raRows) {
        stats.raSweeps++;
        stats.daemonSweepCycles.sample(cache.ticksToCycles(
            start + cache.cyclesToTicks(lineCycles) - raPeriodStart[bank]));
        raRow[bank] = 0;
        raPeriodStart[bank] += raPeriod;
    }
    cache.schedule(*raEvents[bank], raEvenSlot(bank));
}

void
RetentionManager::processRaPorts(unsigned bank, bool opp_step)
{
    const Tick now = curTick();
    const Tick period = cache.clockPeriod();
    const Tick line = cache.cyclesToTicks(lineCycles);
    EventFunctionWrapper &ev = *raEvents[bank];
    EventFunctionWrapper &opp_ev = *raOppEvents[bank];

    if (raRow[bank] == raRows) {
        // Period start (the pre-CPU event at raPeriodStart)
        raRow[bank] = 0;
        raForced[bank] = refreshMode == RefreshMode::AllStrict;
        if (!raForced[bank]) {
            cache.schedule(opp_ev, now);
            return;
        }
    }

    if (opp_step) {
        // After the CPU tick: idle if no access waits or used the bank
        // this cycle and no refresh is booked. Invariant: from the next
        // cycle, the remaining rows still fit before the end of W.
        const Tick end = raPeriodStart[bank] +
            cache.cyclesToTicks(raWindow);
        const Tick rest = Tick(raRows - raRow[bank]) * line;
        const bool idle = bankFreeAt(bank) <= now &&
            bankLastAccess[bank] != now;
        if (!idle) {
            if (now + 2 * period + rest > end) {
                // Waiting another cycle could miss W: the rest back to
                // back from the next cycle, ahead of the waiting accesses
                raForced[bank] = true;
            } else {
                cache.schedule(opp_ev, now + period);
                return;
            }
        }
    }

    // Both kinds start on the next cycle: a forced row pre-empts the
    // waiting accesses, an opportunistic one found the bank idle
    const Tick start = raBlock(bank, now + period, raForced[bank]);
    if (raForced[bank]) {
        stats.raForcedRefreshes++;
    } else {
        stats.raOppRefreshes++;
    }
    raRefreshRow(bank, start);

    if (raRow[bank] < raRows) {
        // Decide the next row in the row's last cycle
        cache.schedule(raForced[bank] ? ev : opp_ev, start + line - period);
        return;
    }

    stats.raSweeps++;
    stats.daemonSweepCycles.sample(
        cache.ticksToCycles(start + line - raPeriodStart[bank]));
    raPeriodStart[bank] += raPeriod;
    cache.schedule(ev, std::max(raPeriodStart[bank], start + line));
}


RetentionManager::RetentionStats::RetentionStats(RetentionManager &_mgr,
                                                 BaseCache &cache)
    : statistics::Group(&cache, "retention"),
      mgr(_mgr),
      ADD_STAT(retentionMisses, statistics::units::Count::get(),
               "accesses that found their block expired and missed"),
      ADD_STAT(lazyWritebacksRead, statistics::units::Count::get(),
               "lazy writebacks of expired dirty blocks, by a read"),
      ADD_STAT(lazyWritebacksWrite, statistics::units::Count::get(),
               "lazy writebacks of expired dirty blocks, by a write"),
      ADD_STAT(lazyWritebacks, statistics::units::Count::get(),
               "all lazy writebacks of expired dirty blocks"),
      ADD_STAT(expiredInflightHits, statistics::units::Count::get(),
               "expired blocks served because a request was in flight"),
      ADD_STAT(expiredDirtyEvictions, statistics::units::Count::get(),
               "expired dirty blocks written back on eviction"),
      ADD_STAT(expiredCleanEvictions, statistics::units::Count::get(),
               "expired clean blocks evicted"),
      ADD_STAT(expiredFunctional, statistics::units::Count::get(),
               "functional accesses that used an expired block"),
      ADD_STAT(expiredSnoops, statistics::units::Count::get(),
               "snoops that used an expired block"),
      ADD_STAT(daemonRevivedDirty, statistics::units::Count::get(),
               "expired dirty blocks refreshed by the daemon"),
      ADD_STAT(invariantViolations, statistics::units::Count::get(),
               "dirty blocks found expired (data lost)"),
      ADD_STAT(oppRefresh, statistics::units::Count::get(),
               "daemon refreshes done while the data array was idle"),
      ADD_STAT(forceRefresh, statistics::units::Count::get(),
               "daemon refreshes forced by the retention guarantee"),
      ADD_STAT(totalRefreshes, statistics::units::Count::get(),
               "all daemon refreshes (one line read + one write each)"),
      ADD_STAT(ageAtForceRefresh, statistics::units::Ratio::get(),
               "block age at a forced refresh, as a fraction of DRT"),
      ADD_STAT(refreshStalledHits, statistics::units::Count::get(),
               "hits delayed because a refresh occupied the data array"),
      ADD_STAT(refreshStallCycles, statistics::units::Cycle::get(),
               "cycles hits waited for refreshes"),
      ADD_STAT(refreshInlineWrite, statistics::units::Count::get(),
               "inline refreshes triggered by a store hit"),
      ADD_STAT(refreshInlineRead, statistics::units::Count::get(),
               "inline refreshes triggered by a read hit"),
      ADD_STAT(ageAtInlineRefresh, statistics::units::Ratio::get(),
               "block age at an inline refresh, as a fraction of DRT"),
      ADD_STAT(cleanToDirtyInDanger, statistics::units::Count::get(),
               "stores that made a clean block dirty inside T_guard"),
      ADD_STAT(inlineRefreshCycles, statistics::units::Cycle::get(),
               "cycles added to hits by inline refreshes"),
      ADD_STAT(inlineTooLate, statistics::units::Count::get(),
               "hits with too little life left for an inline refresh"),
      ADD_STAT(daemonSweepCycles, statistics::units::Cycle::get(),
               "duration of each full daemon sweep"),
      ADD_STAT(tSweepCycles, statistics::units::Cycle::get(),
               "worst-case sweep time the retention guarantee assumes"),
      ADD_STAT(tGuardCycles, statistics::units::Cycle::get(),
               "force-refresh threshold on a block's remaining life"),
      ADD_STAT(expiredHitPot, statistics::units::Count::get(),
               "expired blocks accessed within 1.2 DRT of their charge"),
      ADD_STAT(refreshedBlockUtility, statistics::units::Count::get(),
               "hits on blocks the daemon refreshed"),
      ADD_STAT(refreshClean, statistics::units::Count::get(),
               "daemon refreshes of clean blocks"),
      ADD_STAT(refreshDirty, statistics::units::Count::get(),
               "daemon refreshes of dirty blocks"),
      ADD_STAT(refreshNeededClean, statistics::units::Count::get(),
               "clean refreshes whose next hit came after the would-be "
               "expiry"),
      ADD_STAT(refreshNeededDirty, statistics::units::Count::get(),
               "dirty refreshes whose next hit came after the would-be "
               "expiry"),
      ADD_STAT(refreshEarlyClean, statistics::units::Count::get(),
               "clean refreshes whose next hit came before the would-be "
               "expiry"),
      ADD_STAT(refreshEarlyDirty, statistics::units::Count::get(),
               "dirty refreshes whose next hit came before the would-be "
               "expiry"),
      ADD_STAT(refreshDeadClean, statistics::units::Count::get(),
               "clean refreshes of blocks invalidated before a hit"),
      ADD_STAT(refreshDeadDirty, statistics::units::Count::get(),
               "dirty refreshes of blocks invalidated before a hit"),
      ADD_STAT(refreshOpenClean, statistics::units::Count::get(),
               "clean refreshes of blocks with no hit yet (at dump)"),
      ADD_STAT(refreshOpenDirty, statistics::units::Count::get(),
               "dirty refreshes of blocks with no hit yet (at dump)"),
      ADD_STAT(expiredPrefetchRefills, statistics::units::Count::get(),
               "expired clean blocks invalidated so a prefetch refills "
               "them"),
      ADD_STAT(expiredPrefetchDirty, statistics::units::Count::get(),
               "prefetches that found an expired dirty block"),
      ADD_STAT(bankRefreshes, statistics::units::Count::get(),
               "line refreshes (daemon and inline) per bank"),
      ADD_STAT(bankStalledHits, statistics::units::Count::get(),
               "hits delayed by a refresh, per bank"),
      ADD_STAT(bankStallCycles, statistics::units::Cycle::get(),
               "cycles hits waited for refreshes, per bank"),
      ADD_STAT(setVisitInterval, statistics::units::Cycle::get(),
               "interval between two daemon visits of a set"),
      ADD_STAT(maxSetVisitCycles, statistics::units::Cycle::get(),
               "longest interval between two daemon visits of a set"),
      ADD_STAT(visitIntervalOverruns, statistics::units::Count::get(),
               "set visit intervals longer than T_sweep"),
      ADD_STAT(activeInvalidations, statistics::units::Count::get(),
               "unkept clean lines the daemon invalidated inside T_guard"),
      ADD_STAT(activeInvalidateExpired, statistics::units::Count::get(),
               "active invalidations of lines already expired"),
      ADD_STAT(ageAtActiveInvalidate, statistics::units::Ratio::get(),
               "block age at an active invalidation, as a fraction of DRT"),
      ADD_STAT(activeKeptShared, statistics::units::Count::get(),
               "Shared lines refreshed instead of invalidated"),
      ADD_STAT(activeInvalidateInflight, statistics::units::Count::get(),
               "writable lines inside T_guard with a request in flight"),
      ADD_STAT(activeInvalidateMisses, statistics::units::Count::get(),
               "misses to a line the daemon invalidated"),
      ADD_STAT(activeInvalidateEarlyMisses, statistics::units::Count::get(),
               "misses to an invalidated line before it would have "
               "expired"),
      ADD_STAT(activeWbFullKept, statistics::units::Count::get(),
               "lines refreshed instead of evicted: write buffer full"),
      ADD_STAT(refBitClears, statistics::units::Count::get(),
               "reference bits cleared at touched_clear_age"),
      ADD_STAT(refreshTouched, statistics::units::Count::get(),
               "daemon refreshes of clean lines kept by their reference "
               "bit"),
      ADD_STAT(coldDirtyWritebacks, statistics::units::Count::get(),
               "cold dirty lines written back and invalidated inside "
               "T_guard"),
      ADD_STAT(coldDirtyEarlyMisses, statistics::units::Count::get(),
               "misses to a written-back cold line before it would have "
               "expired"),
      ADD_STAT(silentCleanDrops, statistics::units::Count::get(),
               "clean lines dropped at their deadline without a "
               "CleanEvict"),
      ADD_STAT(ageAtOppRefresh, statistics::units::Ratio::get(),
               "block age at an opportunistic refresh, fraction of DRT"),
      ADD_STAT(epochCycles, statistics::units::Cycle::get(),
               "quantized ages: epoch length"),
      ADD_STAT(deadlineEpochs, statistics::units::Count::get(),
               "quantized ages: deadline age F in epochs"),
      ADD_STAT(oppAgeEpochs, statistics::units::Count::get(),
               "quantized ages: opportunistic refresh age in epochs"),
      ADD_STAT(touchedClearEpochs, statistics::units::Count::get(),
               "quantized ages: reference-bit clear age in epochs (0: off)"),
      ADD_STAT(epochAgeAtDeadline, statistics::units::Count::get(),
               "quantized ages: epoch age at each deadline action"),
      ADD_STAT(quantExpiredAtVisit, statistics::units::Count::get(),
               "quantized ages: valid lines at or past DRT at a visit"),
      ADD_STAT(epochWrapRisk, statistics::units::Count::get(),
               "quantized ages: valid lines at the stamp wrap at a visit"),
      ADD_STAT(oppReplaced, statistics::units::Count::get(),
               "line scanner: slot opp refreshes replaced before issuing"),
      ADD_STAT(oppStale, statistics::units::Count::get(),
               "line scanner: slot opp refreshes whose row changed"),
      ADD_STAT(scanSlotWaits, statistics::units::Count::get(),
               "line scanner: rows that waited for the bank's slot"),
      ADD_STAT(scanSlotWaitCycles, statistics::units::Cycle::get(),
               "line scanner: cycles the scanners waited for the slot"),
      ADD_STAT(bankBusyCycles, statistics::units::Cycle::get(),
               "cycles each bank is busy with refreshes and writeback "
               "reads"),
      ADD_STAT(maxRefreshStallCycles, statistics::units::Cycle::get(),
               "longest single hit stall behind a refresh"),
      ADD_STAT(raSweeps, statistics::units::Count::get(),
               "refresh-all: bank sweeps completed (banks per period)"),
      ADD_STAT(raRowRefreshes, statistics::units::Count::get(),
               "refresh-all: rows refreshed, valid or not"),
      ADD_STAT(raValidRefreshes, statistics::units::Count::get(),
               "refresh-all: rows refreshed while holding a valid line"),
      ADD_STAT(raOppRefreshes, statistics::units::Count::get(),
               "refresh-all: refreshes issued in an idle bank cycle"),
      ADD_STAT(raForcedRefreshes, statistics::units::Count::get(),
               "refresh-all: refreshes issued back to back"),
      ADD_STAT(raMaxRowIntervalCycles, statistics::units::Cycle::get(),
               "refresh-all: longest interval between two refreshes of a "
               "row"),
      ADD_STAT(raMaxChargeAgeCycles, statistics::units::Cycle::get(),
               "refresh-all: oldest valid line at its refresh"),
      ADD_STAT(raLate, statistics::units::Count::get(),
               "refresh-all: valid lines expired at their refresh"),
      ADD_STAT(lateRefreshStarts, statistics::units::Count::get(),
               "refreshes that started after their valid line expired"),
      ADD_STAT(lateRefreshStartsDirty, statistics::units::Count::get(),
               "of those, dirty lines (data lost)"),
      ADD_STAT(maxRowRefreshGapCycles, statistics::units::Cycle::get(),
               "longest gap between two refreshes of a row"),
      ADD_STAT(maxChargeAgeAtRefreshCycles, statistics::units::Cycle::get(),
               "oldest line at its refresh, since its last full write"),
      ADD_STAT(maxChargeAgeAtDropCycles, statistics::units::Cycle::get(),
               "oldest line at a deadline drop (writeback read start or "
               "CleanEvict), since its last full write"),
      ADD_STAT(raPeriodCycles, statistics::units::Cycle::get(),
               "refresh-all: period P"),
      ADD_STAT(raWindowCycles, statistics::units::Cycle::get(),
               "refresh-all: sweep window W")
{
}

void
RetentionManager::RetentionStats::regStats()
{
    statistics::Group::regStats();

    lazyWritebacks = lazyWritebacksRead + lazyWritebacksWrite;
    ageAtForceRefresh.init(0.0, 1.0, 0.02);
    ageAtInlineRefresh.init(0.0, 1.0, 0.02);
    daemonSweepCycles.init(20);
    setVisitInterval.init(20);
    ageAtActiveInvalidate.init(0.0, 1.0, 0.02);
    ageAtOppRefresh.init(0.0, 1.0, 0.02);
    epochAgeAtDeadline.init(0, 31, 1);
    epochCycles.functor([this] {
        return mgr.quantizedAge ? double(1u << mgr.epochLog2) : 0.0;
    });
    deadlineEpochs.functor([this] {
        return double(mgr.deadlineEpochs);
    });
    oppAgeEpochs.functor([this] { return double(mgr.oppAgeEpochs); });
    touchedClearEpochs.functor([this] {
        return double(mgr.clearEpochs);
    });
    maxSetVisitCycles.functor([this] {
        return double(mgr.maxVisitInterval) /
            double(mgr.cache.clockPeriod());
    });
    bankRefreshes.init(mgr.banks);
    bankStalledHits.init(mgr.banks);
    bankStallCycles.init(mgr.banks);
    bankBusyCycles.init(mgr.banks);
    maxRefreshStallCycles.functor([this] {
        return double(mgr.maxStall);
    });
    maxRowRefreshGapCycles.functor([this] {
        return double(std::max(mgr.maxRowGap, mgr.raMaxRowInterval)) /
            double(mgr.cache.clockPeriod());
    });
    maxChargeAgeAtRefreshCycles.functor([this] {
        return double(std::max(mgr.maxChargeAgeAtRefresh,
                               mgr.raMaxChargeAge)) /
            double(mgr.cache.clockPeriod());
    });
    maxChargeAgeAtDropCycles.functor([this] {
        return double(mgr.maxChargeAgeAtDrop) /
            double(mgr.cache.clockPeriod());
    });
    raMaxRowIntervalCycles.functor([this] {
        return double(mgr.raMaxRowInterval) /
            double(mgr.cache.clockPeriod());
    });
    raMaxChargeAgeCycles.functor([this] {
        return double(mgr.raMaxChargeAge) /
            double(mgr.cache.clockPeriod());
    });
    raPeriodCycles.functor([this] {
        return double(mgr.raPeriod) / double(mgr.cache.clockPeriod());
    });
    raWindowCycles.functor([this] {
        return mgr.refreshMode == RefreshMode::Policy ? 0.0 :
            double(mgr.raWindow);
    });
    tSweepCycles.functor([this] {
        return double(mgr.tSweep) / double(mgr.cache.clockPeriod());
    });
    tGuardCycles.functor([this] {
        return double(mgr.tGuard) / double(mgr.cache.clockPeriod());
    });
    refreshOpenClean.functor([this] {
        return mgr.openChainRefreshes(false);
    });
    refreshOpenDirty.functor([this] {
        return mgr.openChainRefreshes(true);
    });
}

void
RetentionManager::RetentionStats::preDumpStats()
{
    mgr.flushVisitBatch();
    statistics::Group::preDumpStats();
}

void
RetentionManager::RetentionStats::resetStats()
{
    // Samples not yet in the histogram are reset with it
    mgr.visitBatchCount = 0;
    statistics::Group::resetStats();

    mgr.maxVisitInterval = 0;
    mgr.maxStall = Cycles(0);
    mgr.raMaxRowInterval = 0;
    mgr.raMaxChargeAge = 0;
    mgr.maxRowGap = 0;
    mgr.maxChargeAgeAtRefresh = 0;
    mgr.maxChargeAgeAtDrop = 0;

    // Chains opened before the reset would be counted without their
    // refreshes: restart them all
    mgr.cache.tags->forEachBlk([](CacheBlk &blk) {
        blk.getRefreshChain() = CacheBlk::RefreshChain();
    });
}

} // namespace gem5
