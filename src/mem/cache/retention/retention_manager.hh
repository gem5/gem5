/**
 * Selective Retention Cache (SRC) support for the classic cache.
 *
 * A GC-eDRAM data array loses a block's contents DRT ticks after the
 * block was last fully written (filled or refreshed). All SRC state,
 * logic, the refresh daemon and the SRC statistics live in this class.
 *
 * BaseCache owns an instance only when drt > 0. With drt == 0 no
 * instance exists, every hook in BaseCache is skipped, and the cache
 * behaves exactly like stock gem5.
 *
 * Data-array banks. The data array has B banks (refresh_banks); set s
 * lives in bank s mod B. A refresh occupies only its bank, a hit waits
 * only for its own bank, and banks refresh in parallel. One daemon (a
 * single scanner, as with B = 1) visits the sets in order and hands
 * each set's refreshes to that set's bank; it waits only when the bank
 * of the next set is still busy. B = 1 is a single unbanked array.
 *
 * Retention guarantee. The refresh daemon visits one set per step; a
 * step scans the set's tags (refresh_set_scan_cycles) and refreshes the
 * chosen blocks one after the other in the set's bank, each occupying
 * that bank for refresh_line_cycles. Take rounds of B consecutive sets:
 * a round scans B sets and waits only for each bank's refreshes from the
 * previous round, at most k_max * line. So the daemon comes back to a
 * set within
 *
 *   T_sweep = ceil(numSets / B) * (B * scan + k_max * line) cycles,
 *
 * which is numSets * (scan + k_max * line) for B = 1, k_max being the
 * most blocks a step can refresh. The daemon therefore
 * force-refreshes a block whose remaining life is below
 *
 *   T_guard = T_sweep + line + margin * DRT,
 *
 * since it would expire before the next visit. Startup fails if
 * T_guard >= DRT: no refresh schedule can then keep dirty data alive.
 *
 * The same bound, seen from the write side: a store hit that leaves a
 * block dirty with less than T_guard of life left may have arrived just
 * after the daemon passed its set, so the store itself recharges the
 * line (inline refresh: read the line, merge the store, write the whole
 * line back), costing inline_refresh_cycles of extra latency. A store
 * that finds less life left than that read needs is too late and is
 * handled like an expired block.
 *
 * Active invalidation (active_invalidate) replaces the inline refresh.
 * A daemon step refreshes every kept line (dirty, top-MRU) inside
 * T_guard as above, and invalidates every other valid line inside it,
 * which is clean. A Shared line is refreshed instead: a pending upgrade
 * expects it to stay, and only a non-writable line can have one. With
 * T_guard >= T_sweep + line (retention_guard_sweeps = 2 gives a full
 * sweep of slack), every valid line has more than T_sweep of life left
 * after each visit of its set, so a store that dirties a clean line is
 * seen by the next visit before the line expires, and no valid line is
 * ever expired. This needs the daemon to be the only user of the banks
 * (inline refresh off); each interval between two visits of a set is
 * checked against T_sweep (a panic under drt_strict). The daemon runs
 * before the CPU tick, so an access in the cycle of an invalidation
 * misses.
 *
 * RTL alignment (both off by default). silent_clean_drop: a clean line
 * dropped at its deadline is invalidated without a CleanEvict and takes
 * no write-buffer slot. quantized_age: the daemon reads a line's age as
 * an age_bits-bit epoch stamp, epochs of 2^age_epoch_log2 cycles, and
 * acts at age >= F = floor((DRT - T_guard) / epoch): a visit that sees
 * age < F knows the exact age is below F epochs, so the line has more
 * than T_guard left. The opp and reference-bit clear ages are in epochs
 * too. The stamp is the epoch of the last full charge, so it needs no
 * state beyond the charge tick.
 *
 * Line scanner (refresh_scan = line), as the RTL controller has it: one
 * scanner per bank walks the bank's rows (ways inner, sets outer), one
 * line per step, and decides that line alone. Each bank has a single
 * pending-operation slot. A row whose decision needs the bank (a
 * refresh or a writeback read) retires only when the slot holds no
 * unissued forced refresh or writeback read; otherwise the scanner
 * waits and decides the row again. A row needing nothing, or only a
 * CleanEvict, retires at once. A forced refresh or writeback read starts
 * as soon as the bank is free; an opportunistic refresh waits in the
 * slot for a step with the bank idle, and the next operation for the
 * bank replaces it. So a row is revisited within
 *
 *   T_sweep = (numSets / B) * assoc * (scan + line_budget) cycles,
 *
 * and the RTL's T_guard = T_sweep + line_budget needs guard_sweeps 1 and
 * margin 0. line_budget (refresh_line_budget_cycles) can exceed the
 * bank occupancy (refresh_line_cycles).
 *
 * Refresh-all (refresh_mode = all_strict or all_opp): no policy at all.
 * Every period of P = DRT - W cycles (W = refresh_all_window_cycles)
 * each bank's sweep refreshes its rows in order (sets bank, bank + B,
 * ..., way inner, as the line scanner), banks in parallel, each row
 * occupying its bank for
 * refresh_line_cycles, whether the row holds a valid line or not; then
 * the controller idles until the next period. all_strict issues the
 * rows back to back from the period start. all_opp issues each row in
 * the next idle bank cycle (no demand access to the bank in that cycle,
 * decided after the CPU tick), and once waiting another cycle could
 * miss W, issues the rest back to back. Back-to-back refreshes run
 * before the CPU tick and win a same-cycle conflict; a demand hit waits
 * for the refresh in progress. A row is refreshed again within P + W - rows *
 * line cycles < DRT (rows/bank), so no valid line ever expires; there are no
 * stamps, reference bits, invalidations or deadline writebacks.
 *
 * Subarrays (refresh_subarrays G, line scanner or refresh-all even
 * only): each bank has G subarrays refreshed in parallel, so one bank
 * operation (refresh_line_cycles) covers G consecutive rows of the bank,
 * one per subarray. The line scanner reads the G rows' metadata in one
 * step and serves every refresh and writeback read of the group in one
 * operation (an opportunistic-only group waits in the slot as one); the
 * rows per sweep become rows / G operations, so T_sweep = (numSets / B)
 * * assoc / G * (scan + line_budget). Refresh-all even issues one
 * operation of G rows every P / (rows / G) cycles.
 *
 * Writebacks from above (a cache below another): a writeback writes the
 * whole line, so it recharges the line, and it counts as a use (the
 * reference bit); a CleanEvict is not a use.
 *
 * Window mode (window_mode, on top of refresh-all strict with even
 * spacing; the sweep and the bank occupancy are unchanged): per line a
 * used bit U (set by demand hits and writebacks from above) and a
 * pending-drop bit D. The cache is split into groups (set mod groups)
 * that the WindowController grants a decision at most once every N
 * frames (one frame = one sweep). At a line's slot in a deciding group a
 * used line is kept (U cleared) and an unused one is dropped: a clean
 * line is invalidated at once; a dirty one is written back first, and if
 * the write buffer is full (or a request for it is in flight) it gets
 * D = 1, stays refreshed, and is retried at its slot in every later frame
 * while U = 0. A dropped line leaves an expired tag (a gem5-invalid block
 * plus a remembered address): a demand access that matches it is a
 * retention miss; victims are an expired way matching the address, then
 * truly invalid ways, then expired ways, then the replacement policy. A
 * clean drop sends no CleanEvict, so the level below still lists the
 * line until the expired tag is replaced, which sends the CleanEvict an
 * SRAM cache would send when the line is evicted.
 *
 * Window options: window_fill_u_always (a fill sets U = 1) and
 * window_cleanevict_sets_u (a CleanEvict from above uses the line).
 * Probation (window_probation_frames = M > 0): a line filled by a demand
 * miss is on probation, exempt from its group's decisions; a 2-bit
 * counter p counts its slots after the fill, and if it is still unused
 * at the M-th one it is dropped like any line (clean: invalidated; dirty:
 * written back, or D = 1 and retried at the next slot). Any use (as U)
 * ends probation. Its expired tag is marked: a demand access matching it
 * counts in probationRetentionMisses and not in R (the controller does
 * not see it).
 */

#ifndef __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__
#define __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__

#include <memory>
#include <unordered_map>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/retention/window_controller.hh"
#include "mem/packet.hh"
#include "sim/eventq.hh"

namespace gem5
{

class BaseCache;
class CacheBlk;
class OutputStream;
struct BaseCacheParams;

class RetentionManager
{
  public:
    RetentionManager(BaseCache &cache, const BaseCacheParams &p);

    /**
     * Compute the retention guarantee and start the refresh daemon.
     * Called from BaseCache::startup().
     */
    void startup();

    /**
     * Expiry check for the block addressed by a read or write request,
     * done before the tag access. An expired block is written back if
     * dirty (the lazy writeback fallback) and invalidated, so the tag
     * access that follows misses.
     *
     * An expired block with a request already in flight (MSHR or write
     * buffer) is left in place and served as usual; this is counted.
     *
     * @param pkt The request being serviced.
     * @param writebacks List to append the lazy writeback to.
     */
    void expireOnAccess(PacketPtr pkt, PacketList &writebacks);

    /**
     * Bookkeeping for a block returned by the tag access. A CleanEvict
     * from above does not use the line.
     */
    void onAccessHit(PacketPtr pkt, CacheBlk *blk);

    /**
     * A writeback from above wrote the whole line of blk (hit or
     * allocation): it is fully charged again and used.
     */
    void onWritebackData(CacheBlk *blk);

    /**
     * Extra hit latency: waiting for a refresh that occupies the data
     * array, plus an inline refresh if this hit triggers one.
     */
    Cycles hitDelay(PacketPtr pkt, CacheBlk *blk);

    /**
     * The whole line of blk was written (allocation, fill): it is fully
     * charged again.
     */
    void onFullLineWrite(CacheBlk *blk);

    /**
     * A block is being evicted. An expired dirty block is still written
     * back (fallback), and counted.
     */
    void onEvict(CacheBlk *blk);

    /** A functional access found blk. Expired data is used; counted. */
    void onFunctional(CacheBlk *blk);

    /** A snoop found blk. Expired data is used; counted. */
    void onSnoop(CacheBlk *blk);

    /**
     * A hardware prefetch targets blk, which is in the tags. An expired
     * clean block holds no data: it is invalidated so the prefetch can
     * refill it. An expired dirty block (a violation) or one with a
     * request in flight is left for the demand path.
     *
     * @return Whether blk was invalidated.
     */
    bool expireForPrefetch(CacheBlk *blk);

    /** blk is being invalidated (evicted, expired, snooped away). */
    void onInvalidate(CacheBlk *blk);

    /** Window mode: a demand hit on blk uses the line. */
    void onDemandHit(PacketPtr pkt, CacheBlk *blk);

    /** Window mode: blk was allocated (its U follows its group); fill:
     *  by a fill response (starts probation), not a writeback. */
    void onAllocate(CacheBlk *blk, bool fill = false);

    /**
     * Window mode: the victim for an allocation of addr when its set
     * holds an expired tag (see the window mode above), pushed into
     * evict_blks; replacing an expired tag the level below still lists
     * appends its CleanEvict to writebacks. nullptr: no expired tag in
     * the set, use the tags' victim.
     */
    CacheBlk *windowVictim(Addr addr, bool is_secure,
                           std::vector<CacheBlk *> &evict_blks,
                           PacketList &writebacks);

    /** Stats only: the cache unblocked for cause. */
    void onClearBlocked(int cause);

  private:
    /** One daemon step: visit set refreshSetIdx, then the next set. */
    void processRefreshEvent();

    /**
     * Refresh-all: one step of bank's sweep (see refresh_mode). opp_step:
     * an all_opp step, run after the CPU tick so it sees this cycle's
     * demand accesses; otherwise a period start or a forced step, run
     * before the CPU tick so the refresh wins a same-cycle conflict.
     */
    void processRaStep(unsigned bank, bool opp_step);

    /**
     * Refresh-all strict with even spacing (refresh_all_even): one row of
     * bank every P / rows cycles; the row starts on the next cycle, ahead
     * of the waiting accesses.
     */
    void processRaEven(unsigned bank);

    /** Tick of the current row's slot (refresh_all_even). */
    Tick raEvenSlot(unsigned bank) const;

    /**
     * Refresh-all with bank ports (bursts): a forced row pre-empts the
     * waiting accesses from the next cycle; an all_opp row waits for an
     * idle bank, then starts on the next cycle.
     */
    void processRaPorts(unsigned bank, bool opp_step);

    /**
     * Book bank for one refresh-all row from earliest (forced: ahead of
     * the waiting accesses). @return The tick it starts at.
     */
    Tick raBlock(unsigned bank, Tick earliest, bool forced);

    /** Refresh-all: the current row of bank, starting at start. */
    void raRefreshRow(unsigned bank, Tick start);

    /**
     * Stats: a refresh of row (set * assoc + way) starting at start, the
     * line charge_age ticks old then.
     */
    void noteRowRefresh(unsigned row, Tick start, Tick charge_age);

    /**
     * Stats and check: the interval since the last visit of a set (set
     * scanner) or a row, set * assoc + way (line scanner).
     */
    void recordVisit(unsigned idx);

    /**
     * Active invalidation: after the refresh loops, invalidate every
     * valid clean line of set inside T_guard that was not kept. A Shared
     * line is refreshed instead (a pending upgrade expects it).
     */
    void invalidateUnkept(unsigned set,
                          const std::vector<CacheBlk *> &mru);

    /** Hit since its last full charge (the reference bit). */
    bool touched(const CacheBlk *blk) const;

    /** Dirty line the daemon writes back instead of refreshing. */
    bool isColdDirty(const CacheBlk *blk,
                     const std::vector<CacheBlk *> &mru) const;

    /** Daemon policy for one valid, unexpired block. */
    void maybeRefresh(CacheBlk *blk);

    /** What the daemon does with one line. */
    enum class LineOp
    {
        None,
        /** Opportunistic refresh (line scanner: into the slot). */
        Opp,
        /** Forced refresh. */
        Force,
        /** Dirty drop: writeback read of the line, then invalidate. */
        Wb,
        /** Clean drop: CleanEvict, no bank time. */
        Ce
    };

    /** Why a line at its deadline is refreshed instead of dropped. */
    enum class DropKeep { None, Shared, Inflight, WbFull };

    /** Line scanner: the decision for one line. */
    struct LineDecision
    {
        LineOp op = LineOp::None;
        /** Not kept and at its deadline: carried out by applyDrop. */
        bool drop = false;
        /** With drop and op Force: why the line is refreshed. */
        DropKeep keep = DropKeep::None;
        /** An expired dirty line, refreshed (backdoor, counted). */
        bool revived = false;
    };

    /**
     * Drop decision for a line at its deadline that is not kept: Force
     * (with the reason in keep), Wb or Ce. No side effects.
     */
    LineOp planDrop(CacheBlk *blk, DropKeep &keep) const;

    /**
     * Carry out planDrop's decision, with its stats.
     *
     * @return The tick the bank operation starts at (curTick() for a
     *         CleanEvict, which takes no bank time).
     */
    Tick applyDrop(CacheBlk *blk, LineOp op, DropKeep keep);

    /** Line scanner: one step of the scanner of bank. */
    void processLineStep(unsigned bank);

    /**
     * Line scanner with subarrays: one step decides a group of G rows
     * and serves it with at most one bank operation.
     */
    void processGroupStep(unsigned bank);

    /**
     * Subarrays: the bank operations booked between beginGroupOp and
     * endGroupOp are one operation (the first books the bank, the rest
     * share its start).
     */
    void beginGroupOp() { groupOpen = true; groupStarted = false; }
    void endGroupOp() { groupOpen = false; }

    /** The current charge of blk came from a writeback from above. */
    bool chargedByWriteback(const CacheBlk *blk) const;

    /** Line scanner: the decision for one line, no side effects. */
    LineDecision decideLine(CacheBlk *blk) const;

    /** Clear blk's reference bit at the clear age (once per charge). */
    void clearRefBit(CacheBlk *blk);

    /** quantized_age checks for one block (see checkQuantizedAges). */
    void checkQuantizedAge(const CacheBlk *blk);

    /** blk is at its deadline (remaining life below T_guard). */
    bool atDeadline(const CacheBlk *blk) const;

    /** blk is at the opportunistic refresh age. */
    bool atOppAge(const CacheBlk *blk) const;

    /**
     * Recharge blk with a full-line read and write.
     *
     * @return The tick the bank operation starts at.
     */
    Tick refresh(CacheBlk *blk, bool force);

    /** Data-array bank of a set. */
    unsigned bankOf(unsigned set) const { return set % banks; }

    /**
     * Occupy a bank with one full-line operation: a refresh, or the
     * line read of a writeback (refresh = false). Returns the tick the
     * operation starts at.
     */
    Tick occupyBank(unsigned bank, bool refresh = true, bool force = true);

    /** Whether an access of this kind triggers inline refreshes. */
    bool inlineApplies(PacketPtr pkt) const;

    /** Valid blocks of a set, most recently touched first, at most n. */
    std::vector<CacheBlk *> topMRUBlks(unsigned set, unsigned n) const;

    /** No demand access to the bank this cycle and no refresh in it. */
    bool bankIdle(unsigned bank) const;

    /**
     * The tick bank is free from: its refreshes are done and, with bank
     * ports, so are the accesses waiting for it.
     */
    Tick bankFreeAt(unsigned bank) const;

    /**
     * Stats and check: a refresh of blk that starts at start, after the
     * bank ports delayed it, finds the line expired.
     */
    void lateStartCheck(CacheBlk *blk, Tick start);

    /** Ticks since blk was last fully written. */
    Tick age(const CacheBlk *blk) const;

    /**
     * quantized_age: set the deadline, opp and clear ages in epochs
     * (unless given) and check that they keep every line alive.
     */
    void deriveEpochThresholds(Tick drt, unsigned k_max);

    /** quantized_age: epoch of tick t (epochs since tick 0). */
    uint64_t epochOf(Tick t) const;

    /** quantized_age: epochs since blk's last full charge, unwrapped. */
    uint64_t epochAgeExact(const CacheBlk *blk) const;

    /** quantized_age: the age the RTL reads, the stamp difference mod
     *  2^age_bits. */
    unsigned epochAge(const CacheBlk *blk) const;

    /**
     * quantized_age checks at a daemon visit: no valid line of set has
     * reached DRT or the stamp's wrap age (panic under drt_strict).
     */
    void checkQuantizedAges(unsigned set);

    /** A request for the block is in the MSHRs or the write buffer. */
    bool inFlight(Addr blk_addr, bool is_secure) const;

    /**
     * Close blk's refresh chain (stats only): hit says whether a hit
     * closes it (else the block is invalidated).
     */
    void closeRefreshChain(CacheBlk *blk, bool hit);

    /** Refreshes in the open chains of clean or dirty refreshes. */
    double openChainRefreshes(bool dirty) const;

    /**
     * Record a retention invariant violation: a dirty block was found
     * expired, so its data is lost. Panics if drt_strict is set.
     */
    void violation(const CacheBlk *blk, const char *where);

    /** Window mode: the decision for row raRow[bank] (before its
     *  refresh-all refresh). */
    void windowRow(unsigned bank);

    /** Window mode: every bank finished its sweep (a frame ended). */
    void windowFrameEnd();

    /** Window mode: bookkeeping for a request, before the tag access. */
    void windowOnAccess(PacketPtr pkt);

    /** Window mode: drop blk (dirty: write back first). */
    void windowDrop(CacheBlk *blk, unsigned row, bool dirty,
                    bool probation = false);

    /** Window mode: clear the expired tag of row. */
    void windowClearExpired(unsigned row);

    /** Window mode: the expired row of set matching addr, or -1. */
    int windowExpiredMatch(unsigned set, Addr blk_addr,
                           bool is_secure) const;

    /** Window mode: add the cycles at the current N up to now. */
    void windowAccountN();

    /** Window mode: a controller event (N change, decision, ...). */
    void windowEvent(WindowController::Event e, unsigned old_val,
                     unsigned new_val);

    /** Window mode: one row of the N log. */
    void windowLog(const char *event);

    /** Window mode amc: one decision interval. */
    void processWindowAmc();

    /** Window mode: stats of the grant just made for the next frame. */
    void windowGranted();

    /** Window mode: row was used (U = 1, D cleared). */
    void windowUse(unsigned row);

    /** window_mode=wb: the slot of valid row (instead of the group
     *  decision): reload the count if used, then refresh, write back
     *  (count 0, dirty) or drop (count 0, clean). */
    void wbRow(CacheBlk *blk, unsigned row);

    /** window_mode=wb: write dirty blk back (WriteClean); it stays valid
     *  and clean, and keeps its write permission. */
    void wbCountWriteback(CacheBlk *blk);

    BaseCache &cache;

    /** Number of top MRU blocks per set the daemon keeps alive. */
    const unsigned topMRU;

    /** Whether the daemon keeps dirty blocks alive. */
    const bool refreshDirtyDaemon;

    /** Panic on a retention invariant violation instead of counting. */
    const bool strict;

    /** Associativity, the most blocks one daemon step can refresh. */
    const unsigned assoc;

    /** Cycles for the daemon to scan one set's tags. */
    const Cycles scanCycles;

    /** Data-array cycles of one full-line refresh (read + write). */
    const Cycles lineCycles;

    /** Number of data-array banks. */
    const unsigned banks;

    /** Subarrays per bank refreshed by one bank operation. */
    const unsigned subarrays;

    /** Subarrays: a group operation is open / has booked the bank, at
     *  groupStart. */
    bool groupOpen;
    bool groupStarted;
    Tick groupStart;
    /** Subarrays: the open group's operation counted as a refresh. */
    bool groupRefreshCounted;

    /** Cycles one bank operation is budgeted for in T_sweep / T_guard. */
    const Cycles lineBudget;

    /** One scanner per bank, one line per step (refresh_scan = line). */
    const bool lineScan;

    /** Daemon policy, or refresh every row once per period. */
    enum class RefreshMode { Policy, AllStrict, AllOpp };
    const RefreshMode refreshMode;

    /** Refresh-all: sweep completion window W. */
    const Cycles raWindow;

    /** Refresh-all, test only: refreshes take no bank time. */
    const bool raFree;

    /** Refresh-all strict: rows evenly spaced over P. */
    const bool raEven;

    /** Age (fraction of DRT) from which idle-time refreshes start. */
    const double oppAge;

    /** Safety margin of the guarantee, as a fraction of DRT. */
    const double margin;

    /** Worst-case daemon sweeps counted in T_guard. */
    const double guardSweeps;

    /** The daemon invalidates unkept clean lines inside T_guard. */
    const bool activeInvalidate;

    /** What the daemon does with dirty lines (cold_dirty). */
    enum class ColdDirty { Refresh, MRU, Touched };
    const ColdDirty coldDirty;

    /** Lines hit since their last full charge are kept (keep_touched). */
    const bool keepTouched;

    /** Age (fraction of DRT) at which the daemon clears the bit. */
    const double touchedClearAge;

    /** Clean deadline drops send no CleanEvict (silent_clean_drop). */
    const bool silentCleanDrop;

    /** Ages in epochs from a stamp (quantized_age). */
    const bool quantizedAge;

    /** log2 of the epoch length in cycles. */
    const unsigned epochLog2;

    /** Bits of the epoch stamp. */
    const unsigned ageBits;

    /** Epoch thresholds as given (-1: derive at startup). */
    const int deadlineEpochsParam;
    const int oppAgeEpochsParam;
    const int clearEpochsParam;

    /** Epoch thresholds in use: deadline F, opp age, reference-bit clear
     *  age (0: no clear). */
    unsigned deadlineEpochs;
    unsigned oppAgeEpochs;
    unsigned clearEpochs;

    /** Store hits near expiry recharge the line. */
    const bool inlineOnWrite;

    /** Read hits near expiry recharge the line. */
    const bool inlineOnRead;

    /** Extra latency of an inline refresh (reading the line first). */
    const Cycles inlineCycles;

    /** Worst-case time for the daemon to come back to a set. */
    Tick tSweep;

    /** Force-refresh a block whose remaining life is below this. */
    Tick tGuard;

    /** Each bank is busy with refreshes until this tick. */
    std::vector<Tick> bankBusyUntil;

    /** Tick of the last demand access to each bank. */
    std::vector<Tick> bankLastAccess;

    /** Tick the current daemon sweep started at set 0. */
    Tick sweepStart;

    /** Set the daemon visits next. */
    unsigned refreshSetIdx;

    /**
     * Stats only: tick of each set's (line scanner: each row's) last
     * daemon visit (MaxTick: none).
     */
    std::vector<Tick> lastVisit;

    /** Stats only: longest interval between two visits of a set. */
    Tick maxVisitInterval;

    /**
     * Stats only: lines the daemon invalidated, until they miss or are
     * refilled, with the tick they would have expired at.
     */
    struct Invalidated
    {
        /** When the line would have expired. */
        Tick expiry;
        /** It was a dirty line written back. */
        bool dirty;
    };
    std::unordered_map<Addr, Invalidated> invalidatedAddrs;

    EventFunctionWrapper refreshEvent;

    /** Line scanner, per bank: next row (set_hi * assoc + way). */
    std::vector<unsigned> scanRow;

    /**
     * Line scanner, per bank: the slot holds an unissued forced refresh
     * or writeback read until this tick.
     */
    std::vector<Tick> slotFreeAt;

    /** Line scanner, per bank: an opportunistic refresh in the slot. */
    struct PendingOpp
    {
        CacheBlk *blk = nullptr;
        Addr addr = 0;
    };
    std::vector<PendingOpp> pendingOpp;

    /** Line scanner with subarrays, per bank: an opportunistic group
     *  in the slot. */
    std::vector<std::vector<PendingOpp>> pendingOppGroup;

    /** Line scanner with subarrays: scratch for one group step. */
    std::vector<CacheBlk *> groupBlks;
    std::vector<LineDecision> groupDecisions;

    /** Subarrays, speed only: equal consecutive visit intervals not yet
     *  sampled into setVisitInterval (flushVisitBatch). */
    uint64_t visitBatchVal = 0;
    int visitBatchCount = 0;
    void flushVisitBatch();

    /** Speed only: epochOf(curTick()), cached for the current tick. */
    uint64_t nowEpoch() const;
    mutable Tick nowEpochTick = MaxTick;
    mutable uint64_t nowEpochVal = 0;

    /** Line scanner, per bank: tick the current sweep started at. */
    std::vector<Tick> bankSweepStart;

    /** Line scanner: one event per bank. */
    std::vector<std::unique_ptr<EventFunctionWrapper>> lineEvents;

    /** Stats only: longest single hit stall behind a refresh. */
    Cycles maxStall;

    /** Stats only (SRC refreshes): tick of each row's last refresh, the
     *  longest gap between two, and the oldest line at its refresh. */
    std::vector<Tick> rowLastRefresh;
    Tick maxRowGap;
    Tick maxChargeAgeAtRefresh;

    /** Stats only: the oldest line at an SRC drop (bank operation start
     *  for a writeback read). */
    Tick maxChargeAgeAtDrop;

    /** Refresh-all: rows per bank (sets * assoc / banks) and the
     *  period P in ticks. */
    unsigned raRows;
    Tick raPeriod;

    /** Refresh-all, per bank: next row of its sweep (raRows: done). */
    std::vector<unsigned> raRow;

    /** Refresh-all, per bank: tick its current period started at. */
    std::vector<Tick> raPeriodStart;

    /** Refresh-all, per bank: the rest of its sweep issues back to
     *  back. */
    std::vector<char> raForced;

    /** Refresh-all, stats only: tick of each row's last refresh. */
    std::vector<Tick> raLastRefresh;

    /** Refresh-all, stats only: longest interval between two refreshes
     *  of a row, and the oldest valid line at its refresh, in ticks. */
    Tick raMaxRowInterval;
    Tick raMaxChargeAge;

    /** Refresh-all, per bank: period starts and forced steps (before
     *  the CPU tick), and all_opp steps (after it). */
    std::vector<std::unique_ptr<EventFunctionWrapper>> raEvents;
    std::vector<std::unique_ptr<EventFunctionWrapper>> raOppEvents;

    /** Window mode: the controller (nullptr: window mode off). */
    std::unique_ptr<WindowController> window;

    /** Window mode, per row: U, D, and the expired tag (MaxAddr:
     *  none) with its secure bit, task id, and whether the level below
     *  still lists the line (simulator only). */
    std::vector<uint8_t> winUsed;
    std::vector<uint8_t> winPend;
    std::vector<Addr> winExpAddr;
    std::vector<uint8_t> winExpSecure;
    std::vector<uint32_t> winExpTaskId;
    std::vector<uint8_t> winExpInFilter;

    /** Window mode, per set: expired tags. */
    std::vector<uint16_t> winSetExpired;

    /** Window mode: expired tags and lines with D = 1, now. */
    uint64_t winExpLive = 0;
    uint64_t winPendLive = 0;
    uint64_t winPendPeak = 0;

    /** Window mode: a drop is invalidating its line. */
    bool winDropping = false;

    /** Window mode probation (window_probation_frames = M, 0 = off), per
     *  row: in probation, its frame counter p, and whether the expired
     *  tag came from a probation drop; lines in probation now; a
     *  probation drop is invalidating its line. */
    unsigned winProbM = 0;
    std::vector<uint8_t> winProbIn;
    std::vector<uint8_t> winProbCnt;
    std::vector<uint8_t> winExpProb;
    uint64_t winProbLive = 0;
    bool winProbDropping = false;

    /** Window mode: a fill sets U = 1; a CleanEvict from above uses a
     *  valid line. */
    bool winFillUAlways = false;
    bool winCleanEvictUse = false;

    /** window_mode=wb (Refrint WB(n,m)): on, n and m (-1 = infinity),
     *  and per row the countdown (-1 = infinity) and the frames since
     *  the last use (stats only). */
    bool wbMode = false;
    int wbN = -1;
    int wbM = -1;
    std::vector<int16_t> wbCount;
    std::vector<uint32_t> wbIdle;

    /** Window mode: banks done with the current sweep, drops in the
     *  current frame. */
    unsigned winBanksDone = 0;
    uint64_t winFrameDrops = 0;

    /** Window mode: N whose time is being accounted, since this tick. */
    unsigned winTimeN = 0;
    Tick winTimeSince = 0;

    /** Window mode: an episode of the write buffer full because of a
     *  drop writeback is open since this tick. */
    bool winBlockOpen = false;
    Tick winBlockSince = 0;

    /** Window mode amc: the decision event and its interval. */
    std::unique_ptr<EventFunctionWrapper> winAmcEvent;
    Cycles winAmcInterval = Cycles(0);

    /** Window mode: the N log (nullptr: none). */
    OutputStream *winLog = nullptr;

    struct WindowStats : public statistics::Group
    {
        WindowStats(RetentionManager &mgr, statistics::Group *parent);

        void regStats() override;

        /** Adds the time at the current N (and an open blocking
         *  episode) up to the dump. */
        void preDumpStats() override;

        /** Restarts the time accounting and the peaks. */
        void resetStats() override;

        RetentionManager &mgr;

        statistics::Scalar retentionMisses;
        statistics::Scalar dropsClean;
        statistics::Scalar dropsDirty;
        statistics::Scalar deadlineWritebacks;
        statistics::Scalar linesRefreshed;
        statistics::Scalar usedRefreshes;
        statistics::Scalar slotsSkipped;
        statistics::Scalar slotsSkippedExpired;
        statistics::Vector nCycles;
        statistics::Scalar nCyclesTotal;
        statistics::Scalar nCyclesTimesN;
        statistics::Formula avgN;
        statistics::Histogram dropsPerFrame;
        statistics::Scalar dirtyDropsDeferred;
        statistics::Scalar deferredWbFull;
        statistics::Scalar deferredInflight;
        statistics::Scalar deferredCleanInflight;
        statistics::Scalar deferredFrames;
        statistics::Value deferredPeak;
        statistics::Value deferredLive;
        statistics::Scalar deferredDropped;
        statistics::Scalar deferredCleared;
        statistics::Scalar deferredEvicted;
        statistics::Distribution ageAtDrop;
        statistics::Distribution dueWaitedPerFrame;
        statistics::Scalar dueWaitedTotal;
        statistics::Distribution grantsPerFrame;
        statistics::Scalar groupDecisions;
        statistics::Scalar frames;
        statistics::Scalar checks;
        statistics::Scalar nChanges;
        statistics::Scalar nGrows;
        statistics::Scalar nGrowCapped;
        statistics::Scalar nShrinks;
        statistics::Scalar amcIntervals;
        statistics::Value finalN;
        statistics::Value finalSsthresh;
        statistics::Scalar victimsInvalid;
        statistics::Scalar victimsExpired;
        statistics::Scalar expiredCreated;
        statistics::Scalar expiredMissed;
        statistics::Scalar expiredOverwritten;
        statistics::Scalar expiredSameAddrCleared;
        statistics::Scalar expiredInvalidated;
        statistics::Scalar expiredWbCleared;
        statistics::Scalar expiredCleanEvicts;
        statistics::Scalar expiredFilterCleared;
        statistics::Value expiredLive;
        statistics::Scalar expiredLiveAtReset;
        statistics::Formula expiredBalance;
        statistics::Scalar demandAccesses;
        statistics::Scalar demandHits;
        statistics::Scalar dropWbBlockedCycles;
        statistics::Scalar dropWbBlockEpisodes;
        statistics::Scalar cleanEvictUses;
        statistics::Scalar probationStarted;
        statistics::Scalar probationEndedByUse;
        statistics::Vector probationUseAtP;
        statistics::Scalar probationDropsClean;
        statistics::Scalar probationDropsDirty;
        statistics::Scalar probationDeferred;
        statistics::Scalar probationRemovedOther;
        statistics::Scalar probationRetentionMisses;
        statistics::Value probationLive;
        statistics::Scalar probationLiveAtReset;
        statistics::Formula probationBalance;
    };
    std::unique_ptr<WindowStats> winStats;

    struct WbStats : public statistics::Group
    {
        WbStats(RetentionManager &mgr, statistics::Group *parent);

        void regStats() override;

        RetentionManager &mgr;

        statistics::Scalar linesRefreshed;
        statistics::Scalar slotsSkipped;
        statistics::Scalar countWritebacks;
        statistics::Scalar wbDeferred;
        statistics::Scalar cleanDeferred;
        statistics::Scalar dropsClean;
        statistics::Scalar retentionMisses;
        statistics::Scalar usesCountReload;
        statistics::Scalar slotReloads;
        statistics::Distribution idleFramesAtDrop;
        statistics::Formula avgValidLines;
        statistics::Formula slotBalance;
    };
    std::unique_ptr<WbStats> wbStats;

    struct RetentionStats : public statistics::Group
    {
        RetentionStats(RetentionManager &mgr, BaseCache &cache);

        void regStats() override;

        /** Flushes the batched visit-interval samples. */
        void preDumpStats() override;

        /** Also restarts every refresh chain (stats only). */
        void resetStats() override;

        RetentionManager &mgr;

        /** Accesses that found their block expired and missed. */
        statistics::Scalar retentionMisses;
        /** Lazy writebacks of expired dirty blocks, by a read access. */
        statistics::Scalar lazyWritebacksRead;
        /** Lazy writebacks of expired dirty blocks, by a write access. */
        statistics::Scalar lazyWritebacksWrite;
        /** All lazy writebacks. */
        statistics::Formula lazyWritebacks;
        /** Expired blocks served anyway: a request was in flight. */
        statistics::Scalar expiredInflightHits;
        /** Expired dirty blocks written back on eviction. */
        statistics::Scalar expiredDirtyEvictions;
        /** Expired clean blocks evicted (dead, no data loss). */
        statistics::Scalar expiredCleanEvictions;
        /** Functional accesses that used an expired block. */
        statistics::Scalar expiredFunctional;
        /** Snoops that used an expired block. */
        statistics::Scalar expiredSnoops;
        /** Expired dirty blocks the daemon refreshed (revived). */
        statistics::Scalar daemonRevivedDirty;
        /** Dirty blocks found expired, anywhere (should be 0). */
        statistics::Scalar invariantViolations;
        /** Daemon refreshes done while the data array was idle. */
        statistics::Scalar oppRefresh;
        /** Daemon refreshes forced by the retention guarantee. */
        statistics::Scalar forceRefresh;
        /** All daemon refreshes (each is one line read + one write). */
        statistics::Scalar totalRefreshes;
        /** Block age at a forced refresh, as a fraction of DRT. */
        statistics::Distribution ageAtForceRefresh;
        /** Hits delayed because a refresh occupied the data array. */
        statistics::Scalar refreshStalledHits;
        /** Cycles hits waited for refreshes. */
        statistics::Scalar refreshStallCycles;
        /** Inline refreshes triggered by a store hit. */
        statistics::Scalar refreshInlineWrite;
        /** Inline refreshes triggered by a read hit. */
        statistics::Scalar refreshInlineRead;
        /** Block age at an inline refresh, as a fraction of DRT. */
        statistics::Distribution ageAtInlineRefresh;
        /** Stores that made a clean block dirty inside T_guard. */
        statistics::Scalar cleanToDirtyInDanger;
        /** Cycles added to hits by inline refreshes. */
        statistics::Scalar inlineRefreshCycles;
        /** Hits with too little life left for an inline refresh. */
        statistics::Scalar inlineTooLate;
        /** Duration of each full daemon sweep, in cycles. */
        statistics::Histogram daemonSweepCycles;
        /** Worst-case sweep time the guarantee assumes, in cycles. */
        statistics::Value tSweepCycles;
        /** Force-refresh threshold on remaining life, in cycles. */
        statistics::Value tGuardCycles;
        /** Expired blocks accessed within 1.2 DRT of their charge. */
        statistics::Scalar expiredHitPot;
        /** Hits on blocks the daemon refreshed. */
        statistics::Scalar refreshedBlockUtility;
        /** Daemon refreshes of clean / dirty blocks. */
        statistics::Scalar refreshClean;
        statistics::Scalar refreshDirty;
        /**
         * Daemon refreshes by outcome, for clean / dirty blocks: the
         * next hit came after the block would have expired (needed),
         * before it (early), the block was invalidated first (dead), or
         * the chain is still open at the stats dump (open). The four add
         * up to refreshClean / refreshDirty.
         */
        statistics::Scalar refreshNeededClean;
        statistics::Scalar refreshNeededDirty;
        statistics::Scalar refreshEarlyClean;
        statistics::Scalar refreshEarlyDirty;
        statistics::Scalar refreshDeadClean;
        statistics::Scalar refreshDeadDirty;
        statistics::Value refreshOpenClean;
        statistics::Value refreshOpenDirty;
        /** Expired clean blocks invalidated so a prefetch refills them. */
        statistics::Scalar expiredPrefetchRefills;
        /** Prefetches that found an expired dirty block. */
        statistics::Scalar expiredPrefetchDirty;
        /** Line refreshes (daemon and inline) per bank. */
        statistics::Vector bankRefreshes;
        /** Hits delayed by a refresh, per bank. */
        statistics::Vector bankStalledHits;
        /** Cycles hits waited for refreshes, per bank. */
        statistics::Vector bankStallCycles;
        /** Interval between two daemon visits of a set, in cycles. */
        statistics::Histogram setVisitInterval;
        /** Longest such interval, in cycles. */
        statistics::Value maxSetVisitCycles;
        /** Visit intervals longer than T_sweep (should be 0). */
        statistics::Scalar visitIntervalOverruns;
        /** Unkept clean lines the daemon invalidated inside T_guard. */
        statistics::Scalar activeInvalidations;
        /** Of those, lines already expired (should be 0). */
        statistics::Scalar activeInvalidateExpired;
        /** Block age at an active invalidation, as a fraction of DRT. */
        statistics::Distribution ageAtActiveInvalidate;
        /** Shared lines refreshed instead of invalidated. */
        statistics::Scalar activeKeptShared;
        /** Writable lines with a request in flight (should be 0). */
        statistics::Scalar activeInvalidateInflight;
        /** Misses to a line the daemon invalidated (stats only). */
        statistics::Scalar activeInvalidateMisses;
        /** Of those, misses before the line would have expired. */
        statistics::Scalar activeInvalidateEarlyMisses;
        /** Lines refreshed instead of evicted: write buffer full. */
        statistics::Scalar activeWbFullKept;
        /** Reference bits the daemon cleared at touched_clear_age. */
        statistics::Scalar refBitClears;
        /** Daemon refreshes of lines kept only by their reference bit. */
        statistics::Scalar refreshTouched;
        /** Cold dirty lines written back and invalidated inside T_guard. */
        statistics::Scalar coldDirtyWritebacks;
        /** Misses to a written-back cold line before it would expire. */
        statistics::Scalar coldDirtyEarlyMisses;
        /** Clean deadline drops without a CleanEvict (silent_clean_drop). */
        statistics::Scalar silentCleanDrops;
        /** Block age at an opportunistic refresh, as a fraction of DRT. */
        statistics::Distribution ageAtOppRefresh;
        /** quantized_age: epoch length in cycles. */
        statistics::Value epochCycles;
        /** quantized_age: thresholds in use, in epochs. */
        statistics::Value deadlineEpochs;
        statistics::Value oppAgeEpochs;
        statistics::Value touchedClearEpochs;
        /** quantized_age: epoch age at each deadline action (all F). */
        statistics::Distribution epochAgeAtDeadline;
        /** quantized_age: valid lines found at or past DRT at a visit
         *  (should be 0). */
        statistics::Scalar quantExpiredAtVisit;
        /** quantized_age: valid lines whose epoch age reached the stamp's
         *  wrap (2^age_bits - 1) at a visit (should be 0). */
        statistics::Scalar epochWrapRisk;
        /** Line scanner: slot opportunistic refreshes replaced by the
         *  next operation for the bank before they issued. */
        statistics::Scalar oppReplaced;
        /** Line scanner: slot opportunistic refreshes dropped because
         *  their row changed before they issued. */
        statistics::Scalar oppStale;
        /** Line scanner: steps in which a row needing the bank waited
         *  for the slot. */
        statistics::Scalar scanSlotWaits;
        /** Line scanner: cycles the scanners waited for the slot. */
        statistics::Scalar scanSlotWaitCycles;
        /** Cycles each bank is busy with refreshes and writeback reads. */
        statistics::Vector bankBusyCycles;
        /** Longest single hit stall behind a refresh, in cycles. */
        statistics::Value maxRefreshStallCycles;
        /** Refresh-all: sweeps completed. */
        statistics::Scalar raSweeps;
        /** Refresh-all: rows refreshed, valid or not. */
        statistics::Scalar raRowRefreshes;
        /** Refresh-all: rows refreshed while holding a valid line. */
        statistics::Scalar raValidRefreshes;
        /** Refresh-all: refreshes issued in an idle bank cycle. */
        statistics::Scalar raOppRefreshes;
        /** Refresh-all: refreshes issued back to back (forced). */
        statistics::Scalar raForcedRefreshes;
        /** Refresh-all: longest interval between two refreshes of a
         *  row, in cycles (must stay below DRT). */
        statistics::Value raMaxRowIntervalCycles;
        /** Refresh-all: oldest valid line at its refresh, in cycles. */
        statistics::Value raMaxChargeAgeCycles;
        /** Refresh-all: valid lines found expired at their refresh
         *  (should be 0). */
        statistics::Scalar raLate;
        /** Refreshes that started after their valid line expired (the
         *  bank ports delayed them; should be 0), and of those, dirty
         *  lines (data lost). */
        statistics::Scalar lateRefreshStarts;
        statistics::Scalar lateRefreshStartsDirty;
        /** Longest gap between two refreshes of a row, and the oldest
         *  line at its refresh, in cycles (every refresh mode). */
        statistics::Value maxRowRefreshGapCycles;
        statistics::Value maxChargeAgeAtRefreshCycles;
        /** Oldest line at a deadline drop (writeback or CleanEvict). */
        statistics::Value maxChargeAgeAtDropCycles;
        /** Refresh-all: the period P and the window W, in cycles. */
        statistics::Value raPeriodCycles;
        statistics::Value raWindowCycles;
    } stats;
};

} // namespace gem5

#endif // __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__
