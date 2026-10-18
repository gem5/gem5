#include "mem/cache/retention/bank_ports.hh"

#include <algorithm>
#include <string>
#include <utility>

#include "base/logging.hh"
#include "mem/cache/base.hh"

namespace gem5
{

BankPorts::BankPorts(BaseCache &_cache, unsigned num_banks,
                     unsigned read_ports, unsigned write_ports)
    : cache(_cache),
      readPorts(read_ports),
      writePorts(write_ports),
      banks(num_banks),
      stats(*this, _cache),
      maxDelay(0)
{
    fatal_if(num_banks == 0 || read_ports == 0 || write_ports == 0,
             "%s: bank ports need at least one bank, one read port and "
             "one write port", cache.name());
    for (unsigned b = 0; b < num_banks; b++) {
        // Before the other events of the cycle: the waiting accesses
        // take the cycle's slots first
        banks[b].event = std::make_unique<EventFunctionWrapper>(
            [this, b] { serve(b); },
            cache.name() + ".bankPorts" + std::to_string(b), false,
            Event::Default_Pri - 1);
    }
}

const BankPorts::Block *
BankPorts::blockAt(const Bank &bank, Tick t) const
{
    for (const Block &b : bank.blocks) {
        if (b.start > t) {
            break;
        }
        if (t < b.end) {
            return &b;
        }
    }
    return nullptr;
}

bool
BankPorts::slotLeft(Bank &bank, bool read, Tick t)
{
    if (bank.usedCycle != t) {
        return true;
    }
    return read ? bank.readsUsed < readPorts : bank.writesUsed < writePorts;
}

void
BankPorts::take(Bank &bank, bool read, Tick t)
{
    if (bank.usedCycle != t) {
        bank.usedCycle = t;
        bank.readsUsed = bank.writesUsed = 0;
    }
    if (read) {
        bank.readsUsed++;
    } else {
        bank.writesUsed++;
    }
}

Tick
BankPorts::blockedIn(const Bank &bank, Tick from, Tick to) const
{
    Tick t = 0;
    for (const Block &b : bank.blocks) {
        if (b.start >= to) {
            break;
        }
        if (b.end > from) {
            t += std::min(b.end, to) - std::max(b.start, from);
        }
    }
    return t;
}

void
BankPorts::prune(Bank &bank, Tick now)
{
    // A block can still overlap the wait of the oldest queued access
    Tick keep = now;
    if (!bank.readQ.empty()) {
        keep = std::min(keep, bank.readQ.front().arrival);
    }
    if (!bank.writeQ.empty()) {
        keep = std::min(keep, bank.writeQ.front().arrival);
    }
    while (!bank.blocks.empty() && bank.blocks.front().end <= keep) {
        bank.blocks.pop_front();
    }
}

void
BankPorts::wake(Bank &bank, Tick t)
{
    if (!bank.event->scheduled()) {
        cache.schedule(*bank.event, t);
    } else if (bank.event->when() > t) {
        cache.reschedule(*bank.event, t);
    }
}

void
BankPorts::served(Bank &bank, Entry &e, Tick t)
{
    stats.accesses[e.kind]++;
    if (t > e.arrival) {
        const Cycles wait = cache.ticksToCycles(t - e.arrival);
        const Cycles refresh =
            cache.ticksToCycles(blockedIn(bank, e.arrival, t));
        const Cycles port = wait - refresh;
        stats.delayedAccesses[e.kind]++;
        if (refresh > 0) {
            stats.refreshDelayedAccesses[e.kind]++;
            stats.refreshDelayCycles[e.kind] += refresh;
        }
        if (port > 0) {
            stats.portDelayedAccesses[e.kind]++;
            stats.portDelayCycles[e.kind] += port;
        }
        maxDelay = std::max(maxDelay, wait);
    }
    if (e.done) {
        e.done(t);
    }
}

void
BankPorts::request(unsigned set, Kind kind, Done done)
{
    Bank &bank = banks[set % banks.size()];
    const Tick now = cache.clockEdge();
    const bool read = kind == LoadHit || kind == WbRead;
    std::deque<Entry> &q = read ? bank.readQ : bank.writeQ;
    prune(bank, now);

    const bool prio_due = read && !bank.prioReads.empty() &&
        bank.prioReads.front() <= now;
    if (q.empty() && !prio_due && !blockAt(bank, now) &&
        slotLeft(bank, read, now)) {
        take(bank, read, now);
        Entry e{kind, now, std::move(done)};
        served(bank, e, now);
        return;
    }
    q.push_back({kind, now, std::move(done)});
    wake(bank, cache.clockEdge(Cycles(1)));
}

void
BankPorts::serve(unsigned bank_idx)
{
    Bank &bank = banks[bank_idx];
    const Tick c = curTick();
    prune(bank, c);

    // A forced block that starts while an access waits pre-empts it
    for (Block &b : bank.blocks) {
        if (b.start > c) {
            break;
        }
        if (b.checked) {
            continue;
        }
        b.checked = true;
        if (b.forced &&
            ((!bank.readQ.empty() && bank.readQ.front().arrival < b.start) ||
             (!bank.writeQ.empty() &&
              bank.writeQ.front().arrival < b.start))) {
            stats.forcedOverQueued++;
        }
    }

    // Blocked: come back when the (back-to-back) blocks end
    if (const Block *b = blockAt(bank, c)) {
        Tick t = b->end;
        while (const Block *n = blockAt(bank, t)) {
            t = n->end;
        }
        wake(bank, t);
        return;
    }

    while (!bank.prioReads.empty() && bank.prioReads.front() <= c &&
           slotLeft(bank, true, c)) {
        if (!bank.readQ.empty() && bank.readQ.front().arrival < c) {
            stats.priorityReadsOverQueued++;
        }
        stats.priorityReads++;
        take(bank, true, c);
        bank.prioReads.pop_front();
    }
    // Pop before done: it may queue more (a fill's writeback read)
    while (!bank.readQ.empty() && slotLeft(bank, true, c)) {
        Entry e = std::move(bank.readQ.front());
        bank.readQ.pop_front();
        take(bank, true, c);
        served(bank, e, c);
    }
    while (!bank.writeQ.empty() && slotLeft(bank, false, c)) {
        Entry e = std::move(bank.writeQ.front());
        bank.writeQ.pop_front();
        take(bank, false, c);
        served(bank, e, c);
    }

    if (!bank.readQ.empty() || !bank.writeQ.empty()) {
        wake(bank, cache.clockEdge(Cycles(1)));
    } else if (!bank.prioReads.empty()) {
        wake(bank, std::max(cache.clockEdge(Cycles(1)),
                            bank.prioReads.front()));
    }
}

Tick
BankPorts::block(unsigned bank_idx, Tick earliest, Cycles cycles,
                 bool forced)
{
    Bank &bank = banks[bank_idx];
    prune(bank, curTick());
    Tick start = std::max(earliest, curTick());
    if (!bank.blocks.empty()) {
        start = std::max(start, bank.blocks.back().end);
    }
    bank.blocks.push_back(
        {start, start + cache.cyclesToTicks(cycles), forced, false});
    stats.refreshBlocks++;
    stats.refreshBlockedCycles += cycles;
    if (forced) {
        stats.forcedBlocks++;
    }
    return start;
}

Tick
BankPorts::priorityRead(unsigned bank_idx, Tick earliest)
{
    Bank &bank = banks[bank_idx];
    Tick t = earliest;
    while (const Block *b = blockAt(bank, t)) {
        t = b->end;
    }
    bank.prioReads.push_back(earliest);
    wake(bank, earliest);
    return t;
}

Tick
BankPorts::freeAt(unsigned bank_idx) const
{
    const Bank &bank = banks[bank_idx];
    if (!bank.readQ.empty() || !bank.writeQ.empty() ||
        !bank.prioReads.empty()) {
        return MaxTick;
    }
    return bank.blocks.empty() ? 0 : bank.blocks.back().end;
}

BankPorts::BankPortsStats::BankPortsStats(BankPorts &_owner,
                                          BaseCache &cache)
    : statistics::Group(&cache, "bankPorts"),
      owner(_owner),
      ADD_STAT(accesses, statistics::units::Count::get(),
               "data-array accesses served, by kind"),
      ADD_STAT(delayedAccesses, statistics::units::Count::get(),
               "accesses that waited for their bank, by kind"),
      ADD_STAT(refreshDelayedAccesses, statistics::units::Count::get(),
               "accesses that waited while a refresh blocked the bank"),
      ADD_STAT(portDelayedAccesses, statistics::units::Count::get(),
               "accesses that waited for a slot taken by other accesses"),
      ADD_STAT(refreshDelayCycles, statistics::units::Cycle::get(),
               "cycles waited while a refresh blocked the bank"),
      ADD_STAT(portDelayCycles, statistics::units::Cycle::get(),
               "cycles waited for a slot taken by other accesses"),
      ADD_STAT(refreshBlocks, statistics::units::Count::get(),
               "refreshes that blocked a bank"),
      ADD_STAT(refreshBlockedCycles, statistics::units::Cycle::get(),
               "bank cycles blocked by refreshes"),
      ADD_STAT(forcedBlocks, statistics::units::Count::get(),
               "forced (priority) refreshes"),
      ADD_STAT(forcedOverQueued, statistics::units::Count::get(),
               "forced refreshes that started while an access waited"),
      ADD_STAT(priorityReads, statistics::units::Count::get(),
               "priority reads (SRC deadline writebacks)"),
      ADD_STAT(priorityReadsOverQueued, statistics::units::Count::get(),
               "priority reads served while a read waited"),
      ADD_STAT(maxDelayCycles, statistics::units::Cycle::get(),
               "longest single wait for a bank")
{
}

void
BankPorts::BankPortsStats::regStats()
{
    statistics::Group::regStats();

    for (statistics::Vector *v : {&accesses, &delayedAccesses,
                                  &refreshDelayedAccesses,
                                  &portDelayedAccesses, &refreshDelayCycles,
                                  &portDelayCycles}) {
        v->init(NumKinds);
        v->subname(LoadHit, "loadHit");
        v->subname(StoreHit, "storeHit");
        v->subname(Fill, "fill");
        v->subname(WbRead, "wbRead");
    }
    maxDelayCycles.functor([this] { return double(owner.maxDelay); });
}

void
BankPorts::BankPortsStats::resetStats()
{
    statistics::Group::resetStats();
    owner.maxDelay = Cycles(0);
}

} // namespace gem5
