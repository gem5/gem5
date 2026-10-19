#include "mem/cache/retention/retention_manager.hh"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "base/cprintf.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "debug/Cache.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/retention/bank_ports.hh"
#include "mem/cache/retention/reuse_tracker.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/base_set_assoc.hh"
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

    fatal_if(p.window_probation_frames > 0 && p.window_mode == "off",
             "%s: window_probation_frames needs window_mode", cache.name());
    fatal_if(p.window_mode != "off" && p.window_mode != "fixed" &&
             p.window_mode != "tcp" && p.window_mode != "amc",
             "%s: window_mode must be off, fixed, tcp or amc",
             cache.name());
    if (p.window_mode != "off") {
        const unsigned num_sets = cache.tags->getNumSets();
        // The window mode decides at the refresh-all slots, so it keeps
        // the even sweep and its bank occupancy
        fatal_if(refreshMode != RefreshMode::AllStrict || !raEven,
                 "%s: window_mode needs refresh_mode=all_strict and "
                 "refresh_all_even", cache.name());
        fatal_if(cache.bankPorts || cache.compressor ||
                 cache.partitionManager ||
                 !dynamic_cast<BaseSetAssoc *>(cache.tags),
                 "%s: window_mode needs BaseSetAssoc tags and no bank "
                 "ports, compressor or partition manager", cache.name());
        WindowController::Params wp;
        wp.mode = p.window_mode == "fixed" ? WindowController::Mode::Fixed :
            p.window_mode == "tcp" ? WindowController::Mode::Tcp :
            WindowController::Mode::Amc;
        wp.nFixed = p.window_n_fixed;
        wp.tcpNInit = p.window_tcp_n_init;
        wp.amcNInit = p.window_amc_n_init;
        wp.ssthreshInit = p.window_ssthresh_init;
        wp.nMax = p.window_n_max;
        wp.upShift = p.window_up_shift;
        wp.downShift = p.window_down_shift;
        wp.minAccesses = p.window_min_accesses;
        wp.streakLen = p.window_streak_len;
        wp.calmLen = p.window_calm_len;
        wp.amcTargetFrac = p.window_amc_target_frac;
        wp.amcHi = p.window_amc_hi;
        wp.amcLo = p.window_amc_lo;
        wp.groups = p.window_groups;
        wp.counterMax = p.window_group_counter_max;
        const std::string err = WindowController::check(wp);
        fatal_if(!err.empty(), "%s: window_mode: %s", cache.name(), err);
        fatal_if(num_sets % wp.groups != 0,
                 "%s: window_groups (%u) must divide numSets (%u)",
                 cache.name(), wp.groups, num_sets);
        fatal_if(wp.mode == WindowController::Mode::Amc &&
                 p.window_amc_interval_cycles == 0,
                 "%s: window_amc_interval_cycles must be > 0",
                 cache.name());

        window = std::make_unique<WindowController>(wp);
        winTimeN = window->nValue();
        winFillUAlways = p.window_fill_u_always;
        winCleanEvictUse = p.window_cleanevict_sets_u;
        // Probation: a 2-bit counter per line (0..M-1 in probation, 3 out)
        winProbM = p.window_probation_frames;
        fatal_if(winProbM > 3, "%s: window_probation_frames must be 0..3 "
                 "(a 2-bit counter)", cache.name());
        window->setListener([this](WindowController::Event e,
                                   unsigned old_val, unsigned new_val) {
            windowEvent(e, old_val, new_val);
        });
        const size_t rows = size_t(num_sets) * assoc;
        winUsed.assign(rows, 0);
        winPend.assign(rows, 0);
        winExpAddr.assign(rows, MaxAddr);
        winExpSecure.assign(rows, 0);
        winExpTaskId.assign(rows, 0);
        winExpInFilter.assign(rows, 0);
        winSetExpired.assign(num_sets, 0);
        winProbIn.assign(rows, 0);
        winProbCnt.assign(rows, 0);
        winExpProb.assign(rows, 0);
        winStats = std::make_unique<WindowStats>(*this, &stats);
        if (wp.mode == WindowController::Mode::Amc) {
            winAmcInterval = p.window_amc_interval_cycles;
            // After the refresh-all events of its tick, before the CPU
            winAmcEvent = std::make_unique<EventFunctionWrapper>(
                [this] { processWindowAmc(); }, cache.name() + ".windowAmc",
                false, Event::Default_Pri + 1);
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
        if (window) {
            const auto &wp = window->params();
            const uint64_t p_cyc = drt_cyc - raWindow;
            winTimeN = window->nValue();
            winTimeSince = curTick();
            winLog = simout.create(cache.name() + ".window_n.csv", false,
                                   true);
            ccprintf(*winLog->stream(),
                     "tick,cycle,event,N,ssthresh,R,A,pend\n");
            window->start();
            windowGranted();
            windowLog("init");
            if (winAmcEvent) {
                cache.schedule(*winAmcEvent,
                               cache.clockEdge(winAmcInterval));
            }
            inform("%s: window mode %s: %u groups (set mod %u), N %u (K "
                   "%u per frame), group counters saturate at %u%s; frame "
                   "= one sweep, P %lu cycles; a fill sets U %s; a "
                   "CleanEvict from above %s; %s", cache.name(),
                   wp.mode == WindowController::Mode::Fixed ? "fixed" :
                   wp.mode == WindowController::Mode::Tcp ? "tcp" : "amc",
                   wp.groups, wp.groups, window->nValue(), window->k(),
                   wp.counterMax,
                   window->nValue() > wp.counterMax + 1 ?
                   " (N above it: no group is ever due)" : "", p_cyc,
                   winFillUAlways ? "= 1" : "= decide of its group",
                   winCleanEvictUse ? "sets U" : "does not set U",
                   winProbM ? csprintf("fresh lines on probation, dropped "
                                       "unused at their %u%s slot after "
                                       "the fill", winProbM,
                                       winProbM == 1 ? "st" :
                                       winProbM == 2 ? "nd" : "rd").c_str()
                   : "no probation");
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
    // Before the read/write filter: it also sees InvalidateReq and
    // CleanEvict
    if (window)
        windowOnAccess(pkt);

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
    // Window mode: a writeback from above uses the line
    if (window && blk != cache.tempBlock)
        windowUse(blk->getSet() * assoc + blk->getWay());
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
        if (window) {
            const unsigned row = blk->getSet() * assoc + blk->getWay();
            winUsed[row] = 0;
            if (winPend[row]) {
                winPend[row] = 0;
                winPendLive--;
                // A drop of a deferred line counts in deferredDropped
                if (!winDropping)
                    winStats->deferredEvicted++;
            }
            if (winProbIn[row]) {
                winProbIn[row] = 0;
                winProbLive--;
                if (!winProbDropping)
                    winStats->probationRemovedOther++;
            }
        }
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
        if (window)
            windowRow(bank);
        raRefreshRow(bank, start);
    }

    if (raRow[bank] == raRows) {
        stats.raSweeps++;
        stats.daemonSweepCycles.sample(cache.ticksToCycles(
            start + cache.cyclesToTicks(lineCycles) - raPeriodStart[bank]));
        raRow[bank] = 0;
        raPeriodStart[bank] += raPeriod;
        // Every bank ends its sweep in this tick: the frame ends with
        // the last one
        if (window && ++winBanksDone == banks)
            windowFrameEnd();
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


void
RetentionManager::windowUse(unsigned row)
{
    winUsed[row] = 1;
    if (winPend[row]) {
        winPend[row] = 0;
        winPendLive--;
        winStats->deferredCleared++;
    }
    // Any use ends probation; the normal U rules apply from now on
    if (winProbIn[row]) {
        winProbIn[row] = 0;
        winProbLive--;
        winStats->probationEndedByUse++;
        winStats->probationUseAtP[winProbCnt[row]]++;
    }
}

void
RetentionManager::onDemandHit(PacketPtr pkt, CacheBlk *blk)
{
    if (!window || !pkt->isDemand())
        return;
    windowUse(blk->getSet() * assoc + blk->getWay());
}

void
RetentionManager::onAllocate(CacheBlk *blk, bool fill)
{
    if (!window || blk == cache.tempBlock)
        return;
    const unsigned set = blk->getSet();
    const unsigned row = set * assoc + blk->getWay();
    // windowVictim clears an expired victim's tag; these are backstops
    if (winExpAddr[row] != MaxAddr) {
        winStats->expiredOverwritten++;
        windowClearExpired(row);
    }
    if (winSetExpired[set]) {
        const int m = windowExpiredMatch(set, cache.regenerateBlkAddr(blk),
                                         blk->isSecure());
        if (m >= 0) {
            winStats->expiredSameAddrCleared++;
            windowClearExpired(m);
        }
    }
    // A fill is used if its group decides this frame (U = decide_g), or
    // always (window_fill_u_always)
    winUsed[row] = winFillUAlways ||
        window->decide(set % window->groups()) ? 1 : 0;
    // Probation: a demand fill (not a writeback that allocates, which is
    // a use) starts it, p = 0
    if (winProbM && fill) {
        panic_if(winProbIn[row], "%s: row %u already in probation",
                 cache.name(), row);
        winProbIn[row] = 1;
        winProbCnt[row] = 0;
        winProbLive++;
        winStats->probationStarted++;
    }
}

int
RetentionManager::windowExpiredMatch(unsigned set, Addr blk_addr,
                                     bool is_secure) const
{
    for (unsigned w = 0; w < assoc; w++) {
        const unsigned row = set * assoc + w;
        if (winExpAddr[row] == blk_addr &&
            bool(winExpSecure[row]) == is_secure) {
            return int(row);
        }
    }
    return -1;
}

void
RetentionManager::windowClearExpired(unsigned row)
{
    panic_if(winExpAddr[row] == MaxAddr, "%s: row %u has no expired tag",
             cache.name(), row);
    winExpAddr[row] = MaxAddr;
    winExpInFilter[row] = 0;
    winExpProb[row] = 0;
    winSetExpired[row / assoc]--;
    winExpLive--;
}

CacheBlk *
RetentionManager::windowVictim(Addr addr, bool is_secure,
                               std::vector<CacheBlk *> &evict_blks,
                               PacketList &writebacks)
{
    if (!window)
        return nullptr;
    const Addr blk_addr = addr & ~Addr(cache.blkSize - 1);
    const unsigned set = cache.tags->getSetIndex({blk_addr, is_secure});
    if (winSetExpired[set] == 0)
        return nullptr;

    // An expired way of this address (refill in place), then the first
    // truly invalid way, then the first expired way
    int row = windowExpiredMatch(set, blk_addr, is_secure);
    const bool same = row >= 0;
    if (row < 0) {
        for (unsigned w = 0; w < assoc && row < 0; w++) {
            const unsigned r = set * assoc + w;
            CacheBlk *b = static_cast<CacheBlk *>(
                cache.tags->findBlockBySetAndWay(set, w));
            if (!b->isValid() && winExpAddr[r] == MaxAddr)
                row = int(r);
        }
    }
    bool expired = same;
    if (row < 0) {
        for (unsigned w = 0; w < assoc && row < 0; w++) {
            if (winExpAddr[set * assoc + w] != MaxAddr)
                row = int(set * assoc + w);
        }
        expired = true;
    }
    panic_if(row < 0, "%s: set %u counts expired tags but has none",
             cache.name(), set);

    CacheBlk *victim = static_cast<CacheBlk *>(
        cache.tags->findBlockBySetAndWay(set, unsigned(row) % assoc));
    panic_if(victim->isValid() && expired, "%s: expired row %d holds a "
             "valid block", cache.name(), row);
    WindowStats &ws = *winStats;
    if (!expired) {
        ws.victimsInvalid++;
    } else {
        ws.victimsExpired++;
        if (same) {
            ws.expiredWbCleared++;
        } else {
            ws.expiredOverwritten++;
            // The level below still lists a clean-dropped line: send the
            // CleanEvict an SRAM cache sends when it evicts the line
            if (winExpInFilter[row]) {
                RequestPtr req = std::make_shared<Request>(
                    winExpAddr[row], cache.blkSize, 0,
                    Request::wbRequestorId);
                if (winExpSecure[row])
                    req->setFlags(Request::SECURE);
                req->taskId(winExpTaskId[row]);
                PacketPtr pkt = new Packet(req, MemCmd::CleanEvict);
                pkt->allocate();
                writebacks.push_back(pkt);
                ws.expiredCleanEvicts++;
            }
        }
        windowClearExpired(unsigned(row));
    }
    evict_blks.push_back(victim);
    return victim;
}

void
RetentionManager::windowOnAccess(PacketPtr pkt)
{
    const bool demand = pkt->isDemand();
    // An invalidating cache maintenance request (clflush) invalidates
    // an SRAM copy too, and takes the line off the level below's list
    const bool inval = pkt->cmd == MemCmd::InvalidateReq ||
        (pkt->req->isCacheMaintenance() && pkt->isInvalidate());
    const bool clean_evict = pkt->cmd == MemCmd::CleanEvict;
    // A prefetch refills the line (no prefetcher in the window configs)
    const bool prefetch = pkt->cmd.isPrefetch();
    if (!demand && !inval && !clean_evict && !prefetch)
        return;

    const Addr blk_addr = pkt->getBlockAddr(cache.blkSize);
    const bool secure = pkt->isSecure();
    CacheBlk *blk = cache.tags->findBlock({blk_addr, secure});
    if (demand) {
        // The hit test of BaseCache::access
        const bool hit = blk && (pkt->needsWritable() ?
            blk->isSet(CacheBlk::WritableBit) :
            blk->isSet(CacheBlk::ReadableBit));
        window->demandAccess(hit);
        winStats->demandAccesses++;
        if (hit)
            winStats->demandHits++;
    }
    if (blk) {
        // window_cleanevict_sets_u: a clean line leaving the cache above
        // uses this one
        if (clean_evict && winCleanEvictUse) {
            windowUse(blk->getSet() * assoc + blk->getWay());
            winStats->cleanEvictUses++;
        }
        return;
    }

    const unsigned set = cache.tags->getSetIndex({blk_addr, secure});
    if (winSetExpired[set] == 0)
        return;
    const int row = windowExpiredMatch(set, blk_addr, secure);
    if (row < 0)
        return;
    if (demand && winExpProb[row]) {
        // A miss on a line probation dropped: counted apart, not in R
        winStats->probationRetentionMisses++;
        winStats->expiredMissed++;
        windowClearExpired(unsigned(row));
    } else if (demand) {
        // A retention miss, counted once: the tag goes now
        winStats->retentionMisses++;
        winStats->expiredMissed++;
        window->retentionMiss();
        windowClearExpired(unsigned(row));
    } else if (inval) {
        // An SRAM cache would invalidate the line too
        winStats->expiredInvalidated++;
        windowClearExpired(unsigned(row));
    } else if (prefetch) {
        winStats->expiredSameAddrCleared++;
        windowClearExpired(unsigned(row));
    } else if (winExpInFilter[row] && !inFlight(blk_addr, secure)) {
        // The CleanEvict goes on below (no block, nothing in flight that
        // would sink it) and removes the line from the level below
        winExpInFilter[row] = 0;
        winStats->expiredFilterCleared++;
    }
}

void
RetentionManager::windowRow(unsigned bank)
{
    const unsigned r = raRow[bank];
    const unsigned set = bank + banks * (r / assoc);
    const unsigned way = r % assoc;
    const unsigned row = set * assoc + way;
    CacheBlk *blk = static_cast<CacheBlk *>(
        cache.tags->findBlockBySetAndWay(set, way));
    WindowStats &ws = *winStats;

    if (!blk->isValid()) {
        ws.slotsSkipped++;
        if (winExpAddr[row] != MaxAddr)
            ws.slotsSkippedExpired++;
        return;
    }

    const bool decide = window->decide(set % window->groups());
    const bool pend = winPend[row];
    if (pend)
        ws.deferredFrames++;
    if (winProbIn[row]) {
        // Probation: exempt from the group's decision (U untouched);
        // unused at its M-th slot after the fill, the line is dropped
        if (++winProbCnt[row] < winProbM) {
            ws.linesRefreshed++;
            return;
        }
        const bool dirty = blk->isSet(CacheBlk::DirtyBit);
        const bool inflight = inFlight(cache.regenerateBlkAddr(blk),
                                       blk->isSecure());
        const bool wb_full = dirty && cache.writeBuffer.isFull();
        if (!inflight && !wb_full) {
            windowDrop(blk, row, dirty, true);
            return;
        }
        // Kept (the only copy) and retried at the next slot: p stays
        // M - 1 (2 bits)
        winProbCnt[row] = winProbM - 1;
        if (!pend) {
            winPend[row] = 1;
            winPendLive++;
            winPendPeak = std::max(winPendPeak, winPendLive);
            ws.probationDeferred++;
        }
        ws.linesRefreshed++;
        return;
    }
    if (!winUsed[row] && (decide || pend)) {
        const bool dirty = blk->isSet(CacheBlk::DirtyBit);
        const bool inflight = inFlight(cache.regenerateBlkAddr(blk),
                                       blk->isSecure());
        // A clean drop needs no write-buffer entry; a dirty one does
        const bool wb_full = dirty && cache.writeBuffer.isFull();
        if (!inflight && !wb_full) {
            windowDrop(blk, row, dirty);
            return;
        }
        // Kept for a later frame: refreshed now (the only copy)
        if (!pend) {
            winPend[row] = 1;
            winPendLive++;
            winPendPeak = std::max(winPendPeak, winPendLive);
            if (!dirty) {
                ws.deferredCleanInflight++;
            } else {
                ws.dirtyDropsDeferred++;
                if (inflight)
                    ws.deferredInflight++;
                else
                    ws.deferredWbFull++;
            }
        }
        ws.linesRefreshed++;
        return;
    }
    if (decide && winUsed[row]) {
        winUsed[row] = 0;
        ws.usedRefreshes++;
    }
    ws.linesRefreshed++;
}

void
RetentionManager::windowDrop(CacheBlk *blk, unsigned row, bool dirty,
                             bool probation)
{
    // Read before the invalidation, which clears the tag and the bits
    const Addr blk_addr = cache.regenerateBlkAddr(blk);
    const bool secure = blk->isSecure();
    const uint32_t task_id = blk->getTaskId();
    const unsigned set = blk->getSet();
    const bool was_pend = winPend[row];
    const uint8_t wb_flag = 1 << BaseCache::Blocked_NoWBBuffers;
    const bool blocked_before = cache.blocked & wb_flag;
    WindowStats &ws = *winStats;

    maxChargeAgeAtDrop = std::max(maxChargeAgeAtDrop, age(blk));

    ws.ageAtDrop.sample(double(age(blk)) /
                        double(cache.tags->getDRT()));

    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Drop);
    winDropping = true;
    winProbDropping = probation;
    if (dirty) {
        // The existing deadline-writeback path (as applyDrop); the line
        // read is part of the refresh-all operation already booked
        PacketList writebacks;
        cache.evictBlock(blk, writebacks);
        if (cache.system->isTimingMode()) {
            cache.doWritebacks(writebacks,
                               cache.clockEdge(cache.forwardLatency));
        } else {
            cache.doWritebacksAtomic(writebacks);
        }
    } else {
        // No write-buffer entry: the level below keeps listing the line
        // until its expired tag is replaced (windowVictim)
        cache.invalidateBlock(blk);
    }
    winDropping = false;
    winProbDropping = false;
    if (cache.reuse)
        cache.reuse->setReason(ReuseTracker::Reason::Other);
    if (probation) {
        if (dirty)
            ws.probationDropsDirty++;
        else
            ws.probationDropsClean++;
    }

    if (dirty) {
        ws.dropsDirty++;
        ws.deadlineWritebacks++;
        if (!blocked_before && (cache.blocked & wb_flag) && !winBlockOpen) {
            winBlockOpen = true;
            winBlockSince = curTick();
            ws.dropWbBlockEpisodes++;
        }
    } else {
        ws.dropsClean++;
    }
    if (was_pend)
        ws.deferredDropped++;
    winFrameDrops++;

    panic_if(winExpAddr[row] != MaxAddr, "%s: row %u already expired",
             cache.name(), row);
    winExpAddr[row] = blk_addr;
    winExpSecure[row] = secure;
    winExpTaskId[row] = task_id;
    // A dirty drop's WritebackDirty already took the line off the level
    // below's list
    winExpInFilter[row] = !dirty;
    winExpProb[row] = probation;
    winSetExpired[set]++;
    winExpLive++;
    ws.expiredCreated++;
}

void
RetentionManager::windowGranted()
{
    WindowStats &ws = *winStats;
    ws.grantsPerFrame.sample(window->lastPicked());
    ws.groupDecisions += window->lastPicked();
    ws.dueWaitedPerFrame.sample(window->lastWaited());
    ws.dueWaitedTotal += window->lastWaited();
}

void
RetentionManager::windowFrameEnd()
{
    WindowStats &ws = *winStats;
    winBanksDone = 0;
    ws.frames++;
    ws.dropsPerFrame.sample(winFrameDrops);
    winFrameDrops = 0;
    window->frameEnd();
    if (window->lastCheck())
        ws.checks++;
    windowGranted();
}

void
RetentionManager::processWindowAmc()
{
    window->amcInterval();
    cache.schedule(*winAmcEvent, curTick() +
                   cache.cyclesToTicks(winAmcInterval));
}

void
RetentionManager::windowAccountN()
{
    const double cycles = double(curTick() - winTimeSince) /
        double(cache.clockPeriod());
    WindowStats &ws = *winStats;
    ws.nCycles[std::min(winTimeN, 65u) - 1] += cycles;
    ws.nCyclesTotal += cycles;
    ws.nCyclesTimesN += double(winTimeN) * cycles;
    winTimeSince = curTick();
}

void
RetentionManager::windowEvent(WindowController::Event e, unsigned old_val,
                              unsigned new_val)
{
    using E = WindowController::Event;
    WindowStats &ws = *winStats;
    const char *name = "";
    switch (e) {
      case E::Grow: name = "grow"; break;
      case E::GrowCapped: name = "grow_capped"; ws.nGrowCapped++; break;
      case E::Dec: name = "dec"; break;
      case E::Halve: name = "halve"; break;
      case E::Ssthresh: name = "ssthresh"; break;
      case E::AmcDecide: name = "amc_decide"; ws.amcIntervals++; break;
      case E::AmcGrow: name = "amc_grow"; break;
      case E::AmcShrink: name = "amc_shrink"; break;
    }
    const bool n_event = e == E::Grow || e == E::Dec || e == E::Halve ||
        e == E::AmcGrow || e == E::AmcShrink;
    if (n_event && old_val != new_val) {
        // The time at the old N ends here
        windowAccountN();
        winTimeN = new_val;
        ws.nChanges++;
        if (new_val > old_val)
            ws.nGrows++;
        else
            ws.nShrinks++;
    }
    windowLog(name);
}

void
RetentionManager::windowLog(const char *event)
{
    if (!winLog)
        return;
    std::ostream &os = *winLog->stream();
    ccprintf(os, "%d,%d,%s,%d,%d,%d,%d,%d\n", curTick(),
             curTick() / cache.clockPeriod(), event, window->nValue(),
             window->ssthresh(), window->rCount(), window->aCount(),
             window->pending());
    os.flush();
}

void
RetentionManager::onClearBlocked(int cause)
{
    if (!window || !winBlockOpen || cause != BaseCache::Blocked_NoWBBuffers)
        return;
    winStats->dropWbBlockedCycles += double(curTick() - winBlockSince) /
        double(cache.clockPeriod());
    winBlockOpen = false;
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

RetentionManager::WindowStats::WindowStats(RetentionManager &_mgr,
                                           statistics::Group *parent)
    : statistics::Group(parent, "window"),
      mgr(_mgr),
      ADD_STAT(retentionMisses, statistics::units::Count::get(),
               "demand accesses that matched an expired tag"),
      ADD_STAT(dropsClean, statistics::units::Count::get(),
               "unused clean lines dropped at their slot (no write buffer)"),
      ADD_STAT(dropsDirty, statistics::units::Count::get(),
               "unused dirty lines written back and dropped at their slot"),
      ADD_STAT(deadlineWritebacks, statistics::units::Count::get(),
               "WritebackDirty packets queued by drops"),
      ADD_STAT(linesRefreshed, statistics::units::Count::get(),
               "valid lines refreshed at their slot (kept ones included)"),
      ADD_STAT(usedRefreshes, statistics::units::Count::get(),
               "lines kept by their used bit at a decision (bit cleared)"),
      ADD_STAT(slotsSkipped, statistics::units::Count::get(),
               "slots with no valid line (no refresh needed)"),
      ADD_STAT(slotsSkippedExpired, statistics::units::Count::get(),
               "of those, slots holding an expired tag"),
      ADD_STAT(nCycles, statistics::units::Cycle::get(),
               "cycles spent at each N (1..64, gt64)"),
      ADD_STAT(nCyclesTotal, statistics::units::Cycle::get(),
               "cycles accounted in nCycles"),
      ADD_STAT(nCyclesTimesN, statistics::units::Cycle::get(),
               "sum of N x cycles at N"),
      ADD_STAT(avgN, statistics::units::Count::get(),
               "time-weighted average N"),
      ADD_STAT(dropsPerFrame, statistics::units::Count::get(),
               "drops (clean + dirty) per frame"),
      ADD_STAT(dirtyDropsDeferred, statistics::units::Count::get(),
               "dirty lines whose drop was deferred (D set)"),
      ADD_STAT(deferredWbFull, statistics::units::Count::get(),
               "dirty lines that got D because the write buffer was full"),
      ADD_STAT(deferredInflight, statistics::units::Count::get(),
               "dirty lines that got D because a request was in flight"),
      ADD_STAT(deferredCleanInflight, statistics::units::Count::get(),
               "clean lines that got D because a request was in flight"),
      ADD_STAT(deferredFrames, statistics::units::Count::get(),
               "frames spent deferred: slot visits that found D = 1 (the "
               "visit that drops the line included)"),
      ADD_STAT(deferredPeak, statistics::units::Count::get(),
               "peak number of lines with D = 1"),
      ADD_STAT(deferredLive, statistics::units::Count::get(),
               "lines with D = 1 now"),
      ADD_STAT(deferredDropped, statistics::units::Count::get(),
               "deferred lines dropped on a retry"),
      ADD_STAT(deferredCleared, statistics::units::Count::get(),
               "deferred lines whose D a use cleared"),
      ADD_STAT(deferredEvicted, statistics::units::Count::get(),
               "deferred lines evicted or invalidated otherwise"),
      ADD_STAT(ageAtDrop, statistics::units::Ratio::get(),
               "line age at a drop, as a fraction of DRT"),
      ADD_STAT(dueWaitedPerFrame, statistics::units::Count::get(),
               "due groups that waited, per frame"),
      ADD_STAT(dueWaitedTotal, statistics::units::Count::get(),
               "due groups that waited, summed over frames"),
      ADD_STAT(grantsPerFrame, statistics::units::Count::get(),
               "groups granted a decision, per frame"),
      ADD_STAT(groupDecisions, statistics::units::Count::get(),
               "group decisions granted"),
      ADD_STAT(frames, statistics::units::Count::get(),
               "frames (sweeps of every bank) completed"),
      ADD_STAT(checks, statistics::units::Count::get(),
               "global checks (every N frames)"),
      ADD_STAT(nChanges, statistics::units::Count::get(),
               "changes of N"),
      ADD_STAT(nGrows, statistics::units::Count::get(),
               "increases of N"),
      ADD_STAT(nGrowCapped, statistics::units::Count::get(),
               "tcp growth tests that fired at N_max"),
      ADD_STAT(nShrinks, statistics::units::Count::get(),
               "decreases of N"),
      ADD_STAT(amcIntervals, statistics::units::Count::get(),
               "amc decisions"),
      ADD_STAT(finalN, statistics::units::Count::get(), "N now"),
      ADD_STAT(finalSsthresh, statistics::units::Count::get(),
               "ssthresh now (tcp)"),
      ADD_STAT(victimsInvalid, statistics::units::Count::get(),
               "victims in a set with expired tags: truly invalid ways"),
      ADD_STAT(victimsExpired, statistics::units::Count::get(),
               "victims in a set with expired tags: expired ways"),
      ADD_STAT(expiredCreated, statistics::units::Count::get(),
               "expired tags made by drops"),
      ADD_STAT(expiredMissed, statistics::units::Count::get(),
               "expired tags cleared by a retention miss"),
      ADD_STAT(expiredOverwritten, statistics::units::Count::get(),
               "expired tags replaced by another address"),
      ADD_STAT(expiredSameAddrCleared, statistics::units::Count::get(),
               "expired tags cleared because their address was allocated "
               "in another way (should be 0)"),
      ADD_STAT(expiredInvalidated, statistics::units::Count::get(),
               "expired tags cleared by an InvalidateReq"),
      ADD_STAT(expiredWbCleared, statistics::units::Count::get(),
               "expired tags refilled in place by an allocation of their "
               "address (a writeback from above)"),
      ADD_STAT(expiredCleanEvicts, statistics::units::Count::get(),
               "CleanEvicts sent when a clean-dropped expired tag was "
               "replaced"),
      ADD_STAT(expiredFilterCleared, statistics::units::Count::get(),
               "clean-dropped expired tags whose line a CleanEvict from "
               "above removed from the level below"),
      ADD_STAT(expiredLive, statistics::units::Count::get(),
               "expired tags now"),
      ADD_STAT(expiredLiveAtReset, statistics::units::Count::get(),
               "expired tags at the last stats reset"),
      ADD_STAT(expiredBalance, statistics::units::Count::get(),
               "created + liveAtReset - every clear - live (must be 0)"),
      ADD_STAT(demandAccesses, statistics::units::Count::get(),
               "demand accesses (A), as gem5's demandAccesses"),
      ADD_STAT(demandHits, statistics::units::Count::get(),
               "demand hits, as gem5's demandHits"),
      ADD_STAT(dropWbBlockedCycles, statistics::units::Cycle::get(),
               "cycles the cache was blocked on a full write buffer that "
               "a drop writeback filled"),
      ADD_STAT(dropWbBlockEpisodes, statistics::units::Count::get(),
               "drop writebacks that filled the write buffer"),
      ADD_STAT(cleanEvictUses, statistics::units::Count::get(),
               "CleanEvicts from above that used a valid line "
               "(window_cleanevict_sets_u)"),
      ADD_STAT(probationStarted, statistics::units::Count::get(),
               "demand fills that started probation "
               "(window_probation_frames)"),
      ADD_STAT(probationEndedByUse, statistics::units::Count::get(),
               "lines whose probation a use ended"),
      ADD_STAT(probationUseAtP, statistics::units::Count::get(),
               "those, by their frame counter p at the use"),
      ADD_STAT(probationDropsClean, statistics::units::Count::get(),
               "clean lines dropped unused at the end of probation (also "
               "in dropsClean)"),
      ADD_STAT(probationDropsDirty, statistics::units::Count::get(),
               "dirty lines dropped unused at the end of probation (also "
               "in dropsDirty)"),
      ADD_STAT(probationDeferred, statistics::units::Count::get(),
               "probation drops deferred (write buffer full or in "
               "flight; D = 1, retried at the next slot)"),
      ADD_STAT(probationRemovedOther, statistics::units::Count::get(),
               "lines evicted or invalidated in probation, not by a "
               "probation drop"),
      ADD_STAT(probationRetentionMisses, statistics::units::Count::get(),
               "demand misses to an expired tag a probation drop left (not "
               "in retentionMisses; not fed to the controller)"),
      ADD_STAT(probationLive, statistics::units::Count::get(),
               "lines in probation, now"),
      ADD_STAT(probationLiveAtReset, statistics::units::Count::get(),
               "lines in probation at the last stats reset"),
      ADD_STAT(probationBalance, statistics::units::Count::get(),
               "started + live at reset - ended by use - drops - removed "
               "other - live (0)")
{
}

void
RetentionManager::WindowStats::regStats()
{
    statistics::Group::regStats();

    nCycles.init(65);
    for (unsigned i = 0; i < 64; i++)
        nCycles.subname(i, std::to_string(i + 1));
    nCycles.subname(64, "gt64");
    avgN = nCyclesTimesN / nCyclesTotal;
    dropsPerFrame.init(20);
    ageAtDrop.init(0.0, 1.0, 0.02);
    dueWaitedPerFrame.init(0, 64, 1);
    grantsPerFrame.init(0, 64, 1);
    deferredPeak.functor([this] { return double(mgr.winPendPeak); });
    deferredLive.functor([this] { return double(mgr.winPendLive); });
    finalN.functor([this] { return double(mgr.window->nValue()); });
    finalSsthresh.functor([this] {
        return double(mgr.window->ssthresh());
    });
    expiredLive.functor([this] { return double(mgr.winExpLive); });
    probationUseAtP.init(4);
    probationLive.functor([this] { return double(mgr.winProbLive); });
    probationBalance = probationStarted + probationLiveAtReset -
        probationEndedByUse - probationDropsClean - probationDropsDirty -
        probationRemovedOther - probationLive;
    expiredBalance = expiredCreated + expiredLiveAtReset - expiredMissed -
        expiredOverwritten - expiredSameAddrCleared - expiredInvalidated -
        expiredWbCleared - expiredLive;
}

void
RetentionManager::WindowStats::preDumpStats()
{
    mgr.windowAccountN();
    if (mgr.winBlockOpen) {
        dropWbBlockedCycles += double(curTick() - mgr.winBlockSince) /
            double(mgr.cache.clockPeriod());
        mgr.winBlockSince = curTick();
    }
    mgr.windowLog("dump");
    statistics::Group::preDumpStats();
}

void
RetentionManager::WindowStats::resetStats()
{
    statistics::Group::resetStats();
    mgr.winTimeSince = curTick();
    mgr.winPendPeak = mgr.winPendLive;
    if (mgr.winBlockOpen)
        mgr.winBlockSince = curTick();
    expiredLiveAtReset = double(mgr.winExpLive);
    probationLiveAtReset = double(mgr.winProbLive);
    mgr.windowLog("reset");
}

} // namespace gem5
