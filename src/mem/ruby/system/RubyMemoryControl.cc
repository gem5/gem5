/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2012 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Description:  This module simulates a basic DDR-style memory controller
 * (and can easily be extended to do FB-DIMM as well).
 *
 * This module models a single channel, connected to any number of
 * DIMMs with any number of ranks of DRAMs each.  If you want multiple
 * address/data channels, you need to instantiate multiple copies of
 * this module.
 *
 * Each memory request is placed in a queue associated with a specific
 * memory bank.  This queue is of finite size; if the queue is full
 * the request will back up in an (infinite) common queue and will
 * effectively throttle the whole system.  This sort of behavior is
 * intended to be closer to real system behavior than if we had an
 * infinite queue on each bank.  If you want the latter, just make
 * the bank queues unreasonably large.
 *
 * The head item on a bank queue is issued when all of the
 * following are true:
 *   the bank is available
 *   the address path to the DIMM is available
 *   the data path to or from the DIMM is available
 *
 * Note that we are not concerned about fixed offsets in time.  The bank
 * will not be used at the same moment as the address path, but since
 * there is no queue in the DIMM or the DRAM it will be used at a constant
 * number of cycles later, so it is treated as if it is used at the same
 * time.
 *
 * We are assuming closed bank policy; that is, we automatically close
 * each bank after a single read or write.  Adding an option for open
 * bank policy is for future work.
 *
 * We are assuming "posted CAS"; that is, we send the READ or WRITE
 * immediately after the ACTIVATE.  This makes scheduling the address
 * bus trivial; we always schedule a fixed set of cycles.  For DDR-400,
 * this is a set of two cycles; for some configurations such as
 * DDR-800 the parameter tRRD forces this to be set to three cycles.
 *
 * We assume a four-bit-time transfer on the data wires.  This is
 * the minimum burst length for DDR-2.  This would correspond
 * to (for example) a memory where each DIMM is 72 bits wide
 * and DIMMs are ganged in pairs to deliver 64 bytes at a shot.
 * This gives us the same occupancy on the data wires as on the
 * address wires (for the two-address-cycle case).
 *
 * The only non-trivial scheduling problem is the data wires.
 * A write will use the wires earlier in the operation than a read
 * will; typically one cycle earlier as seen at the DRAM, but earlier
 * by a worst-case round-trip wire delay when seen at the memory controller.
 * So, while reads from one rank can be scheduled back-to-back
 * every two cycles, and writes (to any rank) scheduled every two cycles,
 * when a read is followed by a write we need to insert a bubble.
 * Furthermore, consecutive reads from two different ranks may need
 * to insert a bubble due to skew between when one DRAM stops driving the
 * wires and when the other one starts.  (These bubbles are parameters.)
 *
 * This means that when some number of reads and writes are at the
 * heads of their queues, reads could starve writes, and/or reads
 * to the same rank could starve out other requests, since the others
 * would never see the data bus ready.
 * For this reason, we have implemented an anti-starvation feature.
 * A group of requests is marked "old", and a counter is incremented
 * each cycle as long as any request from that batch has not issued.
 * if the counter reaches twice the bank busy time, we hold off any
 * newer requests until all of the "old" requests have issued.
 *
 * We also model tFAW.  This is an obscure DRAM parameter that says
 * that no more than four activate requests can happen within a window
 * of a certain size.  For most configurations this does not come into play,
 * or has very little effect, but it could be used to throttle the power
 * consumption of the DRAM.  In this implementation (unlike in a DRAM
 * data sheet) TFAW is measured in memory bus cycles; i.e. if TFAW = 16
 * then no more than four activates may happen within any 16 cycle window.
 * Refreshes are included in the activates.
 *
 */

#include "base/cast.hh"
#include "base/cprintf.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/slicc_interface/NetworkMessage.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/system/RubyMemoryControl.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

class Consumer;

// Value to reset watchdog timer to.
// If we're idle for this many memory control cycles,
// shut down our clock (our rescheduling of ourselves).
// Refresh shuts down as well.
// When we restart, we'll be in a different phase
// with respect to ruby cycles, so this introduces
// a slight inaccuracy.  But it is necessary or the
// ruby tester never terminates because the event
// queue is never empty.
#define IDLECOUNT_MAX_VALUE 1000

// Output operator definition

ostream&
operator<<(ostream& out, const RubyMemoryControl& obj)
{
    obj.print(out);
    out << flush;
    return out;
}


// ****************************************************************

// CONSTRUCTOR
RubyMemoryControl::RubyMemoryControl(const Params *p)
    : MemoryControl(p)
{
    m_banks_per_rank = p->banks_per_rank;
    m_ranks_per_dimm = p->ranks_per_dimm;
    m_dimms_per_channel = p->dimms_per_channel;
    m_bank_bit_0 = p->bank_bit_0;
    m_rank_bit_0 = p->rank_bit_0;
    m_dimm_bit_0 = p->dimm_bit_0;
    m_bank_queue_size = p->bank_queue_size;
    m_bank_busy_time = p->bank_busy_time;
    m_rank_rank_delay = p->rank_rank_delay;
    m_read_write_delay = p->read_write_delay;
    m_basic_bus_busy_time = p->basic_bus_busy_time;
    m_mem_ctl_latency = p->mem_ctl_latency;
    m_refresh_period = p->refresh_period;
    m_tFaw = p->tFaw;
    m_mem_random_arbitrate = p->mem_random_arbitrate;
    m_mem_fixed_delay = p->mem_fixed_delay;

    m_profiler_ptr = new MemCntrlProfiler(name(),
                                          m_banks_per_rank,
                                          m_ranks_per_dimm,
                                          m_dimms_per_channel);
}

void
RubyMemoryControl::init()
{
    m_msg_counter = 0;

    assert(m_tFaw <= 62); // must fit in a uint64 shift register

    m_total_banks = m_banks_per_rank * m_ranks_per_dimm * m_dimms_per_channel;
    m_total_ranks = m_ranks_per_dimm * m_dimms_per_channel;
    m_refresh_period_system = m_refresh_period / m_total_banks;

    m_bankQueues = new list<MemoryNode *> [m_total_banks];
    assert(m_bankQueues);

    m_bankBusyCounter = new int [m_total_banks];
    assert(m_bankBusyCounter);

    m_oldRequest = new int [m_total_banks];
    assert(m_oldRequest);

    for (int i = 0; i < m_total_banks; i++) {
        m_bankBusyCounter[i] = 0;
        m_oldRequest[i] = 0;
    }

    m_busBusyCounter_Basic = 0;
    m_busBusyCounter_Write = 0;
    m_busBusyCounter_ReadNewRank = 0;
    m_busBusy_WhichRank = 0;

    m_roundRobin = 0;
    m_refresh_count = 1;
    m_need_refresh = 0;
    m_refresh_bank = 0;
    m_idleCount = 0;
    m_ageCounter = 0;

    // Each tfaw shift register keeps a moving bit pattern
    // which shows when recent activates have occurred.
    // m_tfaw_count keeps track of how many 1 bits are set
    // in each shift register.  When m_tfaw_count is >= 4,
    // new activates are not allowed.
    m_tfaw_shift = new uint64[m_total_ranks];
    m_tfaw_count = new int[m_total_ranks];
    for (int i = 0; i < m_total_ranks; i++) {
        m_tfaw_shift[i] = 0;
        m_tfaw_count[i] = 0;
    }
}

void
RubyMemoryControl::reset()
{
    m_msg_counter = 0;

    assert(m_tFaw <= 62); // must fit in a uint64 shift register

    m_total_banks = m_banks_per_rank * m_ranks_per_dimm * m_dimms_per_channel;
    m_total_ranks = m_ranks_per_dimm * m_dimms_per_channel;
    m_refresh_period_system = m_refresh_period / m_total_banks;

    assert(m_bankQueues);

    assert(m_bankBusyCounter);

    assert(m_oldRequest);

    for (int i = 0; i < m_total_banks; i++) {
        m_bankBusyCounter[i] = 0;
        m_oldRequest[i] = 0;
    }

    m_busBusyCounter_Basic = 0;
    m_busBusyCounter_Write = 0;
    m_busBusyCounter_ReadNewRank = 0;
    m_busBusy_WhichRank = 0;

    m_roundRobin = 0;
    m_refresh_count = 1;
    m_need_refresh = 0;
    m_refresh_bank = 0;
    m_idleCount = 0;
    m_ageCounter = 0;

    // Each tfaw shift register keeps a moving bit pattern
    // which shows when recent activates have occurred.
    // m_tfaw_count keeps track of how many 1 bits are set
    // in each shift register.  When m_tfaw_count is >= 4,
    // new activates are not allowed.
    for (int i = 0; i < m_total_ranks; i++) {
        m_tfaw_shift[i] = 0;
        m_tfaw_count[i] = 0;
    }
}

RubyMemoryControl::~RubyMemoryControl()
{
    delete [] m_bankQueues;
    delete [] m_bankBusyCounter;
    delete [] m_oldRequest;
    delete m_profiler_ptr;
}

// enqueue new request from directory
void
RubyMemoryControl::enqueue(const MsgPtr& message, Cycles latency)
{
    Cycles arrival_time = curCycle() + latency;
    const MemoryMsg* memMess = safe_cast<const MemoryMsg*>(message.get());
    physical_address_t addr = memMess->getAddr().getAddress();
    MemoryRequestType type = memMess->getType();
    bool is_mem_read = (type == MemoryRequestType_MEMORY_READ);
    MemoryNode *thisReq = new MemoryNode(arrival_time, message, addr,
                                         is_mem_read, !is_mem_read);
    enqueueMemRef(thisReq);
}

// Alternate entry point used when we already have a MemoryNode
// structure built.
void
RubyMemoryControl::enqueueMemRef(MemoryNode *memRef)
{
    m_msg_counter++;
    memRef->m_msg_counter = m_msg_counter;
    physical_address_t addr = memRef->m_addr;
    int bank = getBank(addr);

    DPRINTF(RubyMemory,
            "New memory request%7d: %#08x %c arrived at %10d bank = %3x sched %c\n",
            m_msg_counter, addr, memRef->m_is_mem_read ? 'R':'W',
            memRef->m_time * g_system_ptr->clockPeriod(),
            bank, m_event.scheduled() ? 'Y':'N');

    m_profiler_ptr->profileMemReq(bank);
    m_input_queue.push_back(memRef);

    if (!m_event.scheduled()) {
        schedule(m_event, clockEdge());
    }
}

// dequeue, peek, and isReady are used to transfer completed requests
// back to the directory
void
RubyMemoryControl::dequeue()
{
    assert(isReady());
    MemoryNode *req = m_response_queue.front();
    m_response_queue.pop_front();
    delete req;
}

const Message*
RubyMemoryControl::peek()
{
    MemoryNode *node = peekNode();
    Message* msg_ptr = node->m_msgptr.get();
    assert(msg_ptr != NULL);
    return msg_ptr;
}

MemoryNode *
RubyMemoryControl::peekNode()
{
    assert(isReady());
    MemoryNode *req = m_response_queue.front();
    DPRINTF(RubyMemory, "Peek: memory request%7d: %#08x %c sched %c\n",
            req->m_msg_counter, req->m_addr, req->m_is_mem_read ? 'R':'W',
            m_event.scheduled() ? 'Y':'N');

    return req;
}

bool
RubyMemoryControl::isReady()
{
    return ((!m_response_queue.empty()) &&
            (m_response_queue.front()->m_time <= g_system_ptr->curCycle()));
}

void
RubyMemoryControl::setConsumer(Consumer* consumer_ptr)
{
    m_consumer_ptr = consumer_ptr;
}

void
RubyMemoryControl::print(ostream& out) const
{
}

// Queue up a completed request to send back to directory
void
RubyMemoryControl::enqueueToDirectory(MemoryNode *req, Cycles latency)
{
    Tick arrival_time = clockEdge(latency);
    Cycles ruby_arrival_time = g_system_ptr->ticksToCycles(arrival_time);
    req->m_time = ruby_arrival_time;
    m_response_queue.push_back(req);

    DPRINTF(RubyMemory, "Enqueueing msg %#08x %c back to directory at %15d\n",
            req->m_addr, req->m_is_mem_read ? 'R':'W', arrival_time);

    // schedule the wake up
    m_consumer_ptr->scheduleEventAbsolute(arrival_time);
}

// getBank returns an integer that is unique for each
// bank across this memory controller.
const int
RubyMemoryControl::getBank(const physical_address_t addr) const
{
    int dimm = (addr >> m_dimm_bit_0) & (m_dimms_per_channel - 1);
    int rank = (addr >> m_rank_bit_0) & (m_ranks_per_dimm - 1);
    int bank = (addr >> m_bank_bit_0) & (m_banks_per_rank - 1);
    return (dimm * m_ranks_per_dimm * m_banks_per_rank)
        + (rank * m_banks_per_rank)
        + bank;
}

const int
RubyMemoryControl::getRank(const physical_address_t addr) const
{
    int bank = getBank(addr);
    int rank = (bank / m_banks_per_rank);
    assert (rank < (m_ranks_per_dimm * m_dimms_per_channel));
    return rank;
}

// getRank returns an integer that is unique for each rank
// and independent of individual bank.
const int
RubyMemoryControl::getRank(int bank) const
{
    int rank = (bank / m_banks_per_rank);
    assert (rank < (m_ranks_per_dimm * m_dimms_per_channel));
    return rank;
}

// Not used!
const int
RubyMemoryControl::getChannel(const physical_address_t addr) const
{
    assert(false);
    return -1;
}

// Not used!
const int
RubyMemoryControl::getRow(const physical_address_t addr) const
{
    assert(false);
    return -1;
}

// queueReady determines if the head item in a bank queue
// can be issued this cycle
bool
RubyMemoryControl::queueReady(int bank)
{
    if ((m_bankBusyCounter[bank] > 0) && !m_mem_fixed_delay) {
        m_profiler_ptr->profileMemBankBusy();

        DPRINTF(RubyMemory, "bank %x busy %d\n", bank, m_bankBusyCounter[bank]);
        return false;
    }

    if (m_mem_random_arbitrate >= 2) {
        if ((random() % 100) < m_mem_random_arbitrate) {
            m_profiler_ptr->profileMemRandBusy();
            return false;
        }
    }

    if (m_mem_fixed_delay)
        return true;

    if ((m_ageCounter > (2 * m_bank_busy_time)) && !m_oldRequest[bank]) {
        m_profiler_ptr->profileMemNotOld();
        return false;
    }

    if (m_busBusyCounter_Basic == m_basic_bus_busy_time) {
        // Another bank must have issued this same cycle.  For
        // profiling, we count this as an arb wait rather than a bus
        // wait.  This is a little inaccurate since it MIGHT have also
        // been blocked waiting for a read-write or a read-read
        // instead, but it's pretty close.
        m_profiler_ptr->profileMemArbWait(1);
        return false;
    }

    if (m_busBusyCounter_Basic > 0) {
        m_profiler_ptr->profileMemBusBusy();
        return false;
    }

    int rank = getRank(bank);
    if (m_tfaw_count[rank] >= ACTIVATE_PER_TFAW) {
        m_profiler_ptr->profileMemTfawBusy();
        return false;
    }

    bool write = !m_bankQueues[bank].front()->m_is_mem_read;
    if (write && (m_busBusyCounter_Write > 0)) {
        m_profiler_ptr->profileMemReadWriteBusy();
        return false;
    }

    if (!write && (rank != m_busBusy_WhichRank)
        && (m_busBusyCounter_ReadNewRank > 0)) {
        m_profiler_ptr->profileMemDataBusBusy();
        return false;
    }

    return true;
}

// issueRefresh checks to see if this bank has a refresh scheduled
// and, if so, does the refresh and returns true
bool
RubyMemoryControl::issueRefresh(int bank)
{
    if (!m_need_refresh || (m_refresh_bank != bank))
        return false;
    if (m_bankBusyCounter[bank] > 0)
        return false;
    // Note that m_busBusyCounter will prevent multiple issues during
    // the same cycle, as well as on different but close cycles:
    if (m_busBusyCounter_Basic > 0)
        return false;
    int rank = getRank(bank);
    if (m_tfaw_count[rank] >= ACTIVATE_PER_TFAW)
        return false;

    // Issue it:
    DPRINTF(RubyMemory, "Refresh bank %3x\n", bank);

    m_profiler_ptr->profileMemRefresh();
    m_need_refresh--;
    m_refresh_bank++;
    if (m_refresh_bank >= m_total_banks)
        m_refresh_bank = 0;
    m_bankBusyCounter[bank] = m_bank_busy_time;
    m_busBusyCounter_Basic = m_basic_bus_busy_time;
    m_busBusyCounter_Write = m_basic_bus_busy_time;
    m_busBusyCounter_ReadNewRank = m_basic_bus_busy_time;
    markTfaw(rank);
    return true;
}

// Mark the activate in the tFaw shift register
void
RubyMemoryControl::markTfaw(int rank)
{
    if (m_tFaw) {
        m_tfaw_shift[rank] |= (1 << (m_tFaw-1));
        m_tfaw_count[rank]++;
    }
}

// Issue a memory request: Activate the bank, reserve the address and
// data buses, and queue the request for return to the requesting
// processor after a fixed latency.
void
RubyMemoryControl::issueRequest(int bank)
{
    int rank = getRank(bank);
    MemoryNode *req = m_bankQueues[bank].front();
    m_bankQueues[bank].pop_front();

    DPRINTF(RubyMemory, "Mem issue request%7d: %#08x %c "
            "bank=%3x sched %c\n", req->m_msg_counter, req->m_addr,
            req->m_is_mem_read? 'R':'W',
            bank, m_event.scheduled() ? 'Y':'N');

    if (req->m_msgptr) {  // don't enqueue L3 writebacks
        enqueueToDirectory(req, Cycles(m_mem_ctl_latency + m_mem_fixed_delay));
    }
    m_oldRequest[bank] = 0;
    markTfaw(rank);
    m_bankBusyCounter[bank] = m_bank_busy_time;
    m_busBusy_WhichRank = rank;
    if (req->m_is_mem_read) {
        m_profiler_ptr->profileMemRead();
        m_busBusyCounter_Basic = m_basic_bus_busy_time;
        m_busBusyCounter_Write = m_basic_bus_busy_time + m_read_write_delay;
        m_busBusyCounter_ReadNewRank =
            m_basic_bus_busy_time + m_rank_rank_delay;
    } else {
        m_profiler_ptr->profileMemWrite();
        m_busBusyCounter_Basic = m_basic_bus_busy_time;
        m_busBusyCounter_Write = m_basic_bus_busy_time;
        m_busBusyCounter_ReadNewRank = m_basic_bus_busy_time;
    }
}

// executeCycle:  This function is called once per memory clock cycle
// to simulate all the periodic hardware.
void
RubyMemoryControl::executeCycle()
{
    // Keep track of time by counting down the busy counters:
    for (int bank=0; bank < m_total_banks; bank++) {
        if (m_bankBusyCounter[bank] > 0) m_bankBusyCounter[bank]--;
    }
    if (m_busBusyCounter_Write > 0)
        m_busBusyCounter_Write--;
    if (m_busBusyCounter_ReadNewRank > 0)
        m_busBusyCounter_ReadNewRank--;
    if (m_busBusyCounter_Basic > 0)
        m_busBusyCounter_Basic--;

    // Count down the tFAW shift registers:
    for (int rank=0; rank < m_total_ranks; rank++) {
        if (m_tfaw_shift[rank] & 1) m_tfaw_count[rank]--;
        m_tfaw_shift[rank] >>= 1;
    }

    // After time period expires, latch an indication that we need a refresh.
    // Disable refresh if in mem_fixed_delay mode.
    if (!m_mem_fixed_delay) m_refresh_count--;
    if (m_refresh_count == 0) {
        m_refresh_count = m_refresh_period_system;

        // Are we overrunning our ability to refresh?
        assert(m_need_refresh < 10);
        m_need_refresh++;
    }

    // If this batch of requests is all done, make a new batch:
    m_ageCounter++;
    int anyOld = 0;
    for (int bank=0; bank < m_total_banks; bank++) {
        anyOld |= m_oldRequest[bank];
    }
    if (!anyOld) {
        for (int bank=0; bank < m_total_banks; bank++) {
            if (!m_bankQueues[bank].empty()) m_oldRequest[bank] = 1;
        }
        m_ageCounter = 0;
    }

    // If randomness desired, re-randomize round-robin position each cycle
    if (m_mem_random_arbitrate) {
        m_roundRobin = random() % m_total_banks;
    }

    // For each channel, scan round-robin, and pick an old, ready
    // request and issue it.  Treat a refresh request as if it were at
    // the head of its bank queue.  After we issue something, keep
    // scanning the queues just to gather statistics about how many
    // are waiting.  If in mem_fixed_delay mode, we can issue more
    // than one request per cycle.
    int queueHeads = 0;
    int banksIssued = 0;
    for (int i = 0; i < m_total_banks; i++) {
        m_roundRobin++;
        if (m_roundRobin >= m_total_banks) m_roundRobin = 0;
        issueRefresh(m_roundRobin);
        int qs = m_bankQueues[m_roundRobin].size();
        if (qs > 1) {
            m_profiler_ptr->profileMemBankQ(qs-1);
        }
        if (qs > 0) {
            // we're not idle if anything is queued
            m_idleCount = IDLECOUNT_MAX_VALUE;
            queueHeads++;
            if (queueReady(m_roundRobin)) {
                issueRequest(m_roundRobin);
                banksIssued++;
                if (m_mem_fixed_delay) {
                    m_profiler_ptr->profileMemWaitCycles(m_mem_fixed_delay);
                }
            }
        }
    }

    // memWaitCycles is a redundant catch-all for the specific
    // counters in queueReady
    m_profiler_ptr->profileMemWaitCycles(queueHeads - banksIssued);

    // Check input queue and move anything to bank queues if not full.
    // Since this is done here at the end of the cycle, there will
    // always be at least one cycle of latency in the bank queue.  We
    // deliberately move at most one request per cycle (to simulate
    // typical hardware).  Note that if one bank queue fills up, other
    // requests can get stuck behind it here.
    if (!m_input_queue.empty()) {
        // we're not idle if anything is pending
        m_idleCount = IDLECOUNT_MAX_VALUE;
        MemoryNode *req = m_input_queue.front();
        int bank = getBank(req->m_addr);
        if (m_bankQueues[bank].size() < m_bank_queue_size) {
            m_input_queue.pop_front();
            m_bankQueues[bank].push_back(req);
        }
        m_profiler_ptr->profileMemInputQ(m_input_queue.size());
    }
}

unsigned int
RubyMemoryControl::drain(DrainManager *dm)
{
    DPRINTF(RubyMemory, "MemoryController drain\n");
    if(m_event.scheduled()) {
        deschedule(m_event);
    }
    return 0;
}

// wakeup:  This function is called once per memory controller clock cycle.
void
RubyMemoryControl::wakeup()
{
    DPRINTF(RubyMemory, "MemoryController wakeup\n");
    // execute everything
    executeCycle();

    m_idleCount--;
    if (m_idleCount > 0) {
        assert(!m_event.scheduled());
        schedule(m_event, clockEdge(Cycles(1)));
    }
}

/**
 * This function reads the different buffers that exist in the Ruby Memory
 * Controller, and figures out if any of the buffers hold a message that
 * contains the data for the address provided in the packet. True is returned
 * if any of the messages was read, otherwise false is returned.
 *
 * I think we should move these buffers to being message buffers, instead of
 * being lists.
 */
bool
RubyMemoryControl::functionalReadBuffers(Packet *pkt)
{
    for (std::list<MemoryNode *>::iterator it = m_input_queue.begin();
         it != m_input_queue.end(); ++it) {
        Message* msg_ptr = (*it)->m_msgptr.get();
        if (msg_ptr->functionalRead(pkt)) {
            return true;
        }
    }

    for (std::list<MemoryNode *>::iterator it = m_response_queue.begin();
         it != m_response_queue.end(); ++it) {
        Message* msg_ptr = (*it)->m_msgptr.get();
        if (msg_ptr->functionalRead(pkt)) {
            return true;
        }
    }

    for (uint32_t bank = 0; bank < m_total_banks; ++bank) {
        for (std::list<MemoryNode *>::iterator it = m_bankQueues[bank].begin();
             it != m_bankQueues[bank].end(); ++it) {
            Message* msg_ptr = (*it)->m_msgptr.get();
            if (msg_ptr->functionalRead(pkt)) {
                return true;
            }
        }
    }

    return false;
}

/**
 * This function reads the different buffers that exist in the Ruby Memory
 * Controller, and figures out if any of the buffers hold a message that
 * needs to functionally written with the data in the packet.
 *
 * The number of messages written is returned at the end. This is required
 * for debugging purposes.
 */
uint32_t
RubyMemoryControl::functionalWriteBuffers(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (std::list<MemoryNode *>::iterator it = m_input_queue.begin();
         it != m_input_queue.end(); ++it) {
        Message* msg_ptr = (*it)->m_msgptr.get();
        if (msg_ptr->functionalWrite(pkt)) {
            num_functional_writes++;
        }
    }

    for (std::list<MemoryNode *>::iterator it = m_response_queue.begin();
         it != m_response_queue.end(); ++it) {
        Message* msg_ptr = (*it)->m_msgptr.get();
        if (msg_ptr->functionalWrite(pkt)) {
            num_functional_writes++;
        }
    }

    for (uint32_t bank = 0; bank < m_total_banks; ++bank) {
        for (std::list<MemoryNode *>::iterator it = m_bankQueues[bank].begin();
             it != m_bankQueues[bank].end(); ++it) {
            Message* msg_ptr = (*it)->m_msgptr.get();
            if (msg_ptr->functionalWrite(pkt)) {
                num_functional_writes++;
            }
        }
    }

    return num_functional_writes;
}

void
RubyMemoryControl::regStats()
{
    m_profiler_ptr->regStats();
}

RubyMemoryControl *
RubyMemoryControlParams::create()
{
    return new RubyMemoryControl(this);
}
