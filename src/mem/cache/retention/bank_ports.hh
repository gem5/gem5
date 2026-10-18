/**
 * Data-array bank ports (data_read_ports, data_write_ports).
 *
 * Each data-array bank (bank = set mod refresh_banks) serves at most R
 * reads and W writes per cycle. Reads are load hits and the line read of
 * a writeback (an eviction or an SRC deadline drop); writes are store
 * hits and fills. A refresh (SRC daemon or refresh-all) blocks its bank
 * completely, no reads and no writes, for all its cycles.
 *
 * Each bank keeps a read queue and a write queue. An access is served at
 * once if its queue is empty, the bank is not blocked and a slot of its
 * kind is left this cycle; otherwise it waits, and an arbiter serves the
 * queues in arrival order, R reads and W writes per unblocked cycle,
 * before the accesses that arrive in that cycle. An access is released
 * when served: a hit's response leaves that much later, a fill is
 * processed (block written, MSHR targets answered) only then.
 *
 * Refreshes are blocks booked by the retention manager. A forced block
 * (refresh-all strict row, SRC deadline refresh, refresh-all forced
 * tail) is booked from the next cycle and takes priority: the accesses
 * already waiting wait for it. An opportunistic block is only booked
 * when the bank is idle (no queued access, no block). An SRC deadline
 * writeback takes the next read slot ahead of the queued reads.
 *
 * Each wait is split by cause: the cycles of the wait during which the
 * bank was blocked by a refresh, and the rest (the slot was taken by
 * other accesses: the port limit).
 *
 * BaseCache owns an instance only when both port counts are non-zero;
 * otherwise no instance exists and the data array is unlimited, as in
 * stock gem5.
 */

#ifndef __MEM_CACHE_RETENTION_BANK_PORTS_HH__
#define __MEM_CACHE_RETENTION_BANK_PORTS_HH__

#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "sim/eventq.hh"

namespace gem5
{

class BaseCache;

class BankPorts
{
  public:
    /** What an access does in the data array. */
    enum Kind
    {
        LoadHit,
        StoreHit,
        Fill,
        WbRead,
        NumKinds
    };

    /** Called with the tick an access is served at. */
    using Done = std::function<void(Tick)>;

    BankPorts(BaseCache &cache, unsigned banks, unsigned read_ports,
              unsigned write_ports);

    /**
     * An access of kind to set's bank arrives now; done (may be empty)
     * runs when it is served, possibly before this returns.
     */
    void request(unsigned set, Kind kind, Done done);

    /**
     * Book a refresh block of bank for cycles, from earliest or the end
     * of the blocks already booked. forced: a priority refresh (counted
     * when it pre-empts a waiting access).
     *
     * @return The tick the block starts at.
     */
    Tick block(unsigned bank, Tick earliest, Cycles cycles, bool forced);

    /**
     * Book a read slot of bank at earliest or the first unblocked cycle
     * after it, ahead of the queued reads (SRC deadline writeback).
     *
     * @return The tick the read is expected at.
     */
    Tick priorityRead(unsigned bank, Tick earliest);

    /**
     * The tick bank is free from: the end of its booked blocks if no
     * access is waiting, MaxTick otherwise.
     */
    Tick freeAt(unsigned bank) const;

  private:
    struct Entry
    {
        Kind kind;
        Tick arrival;
        Done done;
    };

    struct Block
    {
        Tick start;
        Tick end;
        bool forced;
        /** Whether the pre-emption check has been done. */
        bool checked;
    };

    struct Bank
    {
        std::deque<Entry> readQ;
        std::deque<Entry> writeQ;
        /** Priority reads: earliest tick of each. */
        std::deque<Tick> prioReads;
        /** Refresh blocks, in order; kept while a wait may overlap. */
        std::deque<Block> blocks;
        /** Slots used in cycle usedCycle. */
        Tick usedCycle = MaxTick;
        unsigned readsUsed = 0;
        unsigned writesUsed = 0;
        std::unique_ptr<EventFunctionWrapper> event;
    };

    /** Arbiter: serve bank's queues in this cycle. */
    void serve(unsigned bank_idx);

    /** Make the arbiter of bank run at t (or earlier). */
    void wake(Bank &bank, Tick t);

    /** The block covering cycle t, or nullptr. */
    const Block *blockAt(const Bank &bank, Tick t) const;

    /** Slots left for kind in cycle t. */
    bool slotLeft(Bank &bank, bool read, Tick t);

    /** Take a slot in cycle t. */
    void take(Bank &bank, bool read, Tick t);

    /** Stats of an access served at t; then run its done. */
    void served(Bank &bank, Entry &e, Tick t);

    /** Ticks of [from, to) during which bank is blocked by a refresh. */
    Tick blockedIn(const Bank &bank, Tick from, Tick to) const;

    /** Drop blocks no wait can overlap any more. */
    void prune(Bank &bank, Tick now);

    BaseCache &cache;
    const unsigned readPorts;
    const unsigned writePorts;
    std::vector<Bank> banks;

    struct BankPortsStats : public statistics::Group
    {
        BankPortsStats(BankPorts &owner, BaseCache &cache);

        void regStats() override;

        /** Also restarts the longest wait. */
        void resetStats() override;

        BankPorts &owner;

        /** Accesses served, by kind. */
        statistics::Vector accesses;
        /** Accesses that waited at all, by kind. */
        statistics::Vector delayedAccesses;
        /** Accesses that waited for a refresh, by kind. */
        statistics::Vector refreshDelayedAccesses;
        /** Accesses that waited for the port limit, by kind. */
        statistics::Vector portDelayedAccesses;
        /** Cycles waited while the bank was blocked by a refresh. */
        statistics::Vector refreshDelayCycles;
        /** Cycles waited for a slot taken by other accesses. */
        statistics::Vector portDelayCycles;
        /** Refresh blocks and the bank cycles they block. */
        statistics::Scalar refreshBlocks;
        statistics::Scalar refreshBlockedCycles;
        /** Forced (priority) blocks, and those that started while an
         *  access was waiting (pre-empting it). */
        statistics::Scalar forcedBlocks;
        statistics::Scalar forcedOverQueued;
        /** Priority reads (SRC deadline writebacks), and those served
         *  while a read was waiting. */
        statistics::Scalar priorityReads;
        statistics::Scalar priorityReadsOverQueued;
        /** Longest single wait, in cycles. */
        statistics::Value maxDelayCycles;
    } stats;

    Cycles maxDelay;
};

} // namespace gem5

#endif // __MEM_CACHE_RETENTION_BANK_PORTS_HH__
