/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 *
 * Authors: Andreas Hansson
 *          Ani Udipi
 */

/**
 * @file
 * SimpleDRAM declaration
 */

#ifndef __MEM_SIMPLE_DRAM_HH__
#define __MEM_SIMPLE_DRAM_HH__

#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "enums/MemSched.hh"
#include "enums/PageManage.hh"
#include "mem/abstract_mem.hh"
#include "mem/qport.hh"
#include "params/SimpleDRAM.hh"
#include "sim/eventq.hh"

/**
 * The simple DRAM is a basic single-channel memory controller aiming
 * to mimic a high-level DRAM controller and the most important timing
 * constraints associated with the DRAM. The focus is really on
 * modelling the impact on the system rather than the DRAM itself,
 * hence the focus is on the controller model and not on the
 * memory. By adhering to the correct timing constraints, ultimately
 * there is no need for a memory model in addition to the controller
 * model.
 *
 * As a basic design principle, this controller is not cycle callable,
 * but instead uses events to decide when new decisions can be made,
 * when resources become available, when things are to be considered
 * done, and when to send things back. Through these simple
 * principles, we achieve a performant model that is not
 * cycle-accurate, but enables us to evaluate the system impact of a
 * wide range of memory technologies, and also collect statistics
 * about the use of the memory.
 */
class SimpleDRAM : public AbstractMemory
{

  private:

    // For now, make use of a queued slave port to avoid dealing with
    // flow control for the responses being sent back
    class MemoryPort : public QueuedSlavePort
    {

        SlavePacketQueue queue;
        SimpleDRAM& memory;

      public:

        MemoryPort(const std::string& name, SimpleDRAM& _memory);

      protected:

        Tick recvAtomic(PacketPtr pkt);

        void recvFunctional(PacketPtr pkt);

        bool recvTimingReq(PacketPtr);

        virtual AddrRangeList getAddrRanges() const;

    };

    /**
     * Our incoming port, for a multi-ported controller add a crossbar
     * in front of it
     */
    MemoryPort port;

    /**
     * Remember if we have to retry a request when available.
     */
    bool retryRdReq;
    bool retryWrReq;

    /**
     * Remember that a row buffer hit occured
     */
    bool rowHitFlag;

    /**
     * Use this flag to shutoff reads, i.e. do not schedule any reads
     * beyond those already done so that we can turn the bus around
     * and do a few writes, or refresh, or whatever
     */
    bool stopReads;

    /**
     * A basic class to track the bank state indirectly via
     * times "freeAt" and "tRASDoneAt" and what page is currently open
     */
    class Bank
    {

      public:

        static const uint32_t INVALID_ROW = -1;

        uint32_t openRow;

        Tick freeAt;
        Tick tRASDoneAt;

        Bank() : openRow(INVALID_ROW), freeAt(0), tRASDoneAt(0)
        { }
    };

    /**
     * A DRAM packet stores packets along with the timestamp of when
     * the packet entered the queue, and also the decoded address.
     */
    class DRAMPacket {

      public:

        /** When did request enter the controller */
        const Tick entryTime;

        /** When will request leave the controller */
        Tick readyTime;

        /** This comes from the outside world */
        const PacketPtr pkt;

        /** Will be populated by address decoder */
        const uint8_t rank;
        const uint16_t bank;
        const uint16_t row;
        const Addr addr;
        Bank& bank_ref;

        DRAMPacket(PacketPtr _pkt, uint8_t _rank,
                   uint16_t _bank, uint16_t _row, Addr _addr, Bank& _bank_ref)
            : entryTime(curTick()), readyTime(curTick()),
              pkt(_pkt), rank(_rank), bank(_bank), row(_row), addr(_addr),
              bank_ref(_bank_ref)
        { }

    };

    /**
     * Bunch of things requires to setup "events" in gem5
     * When event "writeEvent" occurs for example, the method
     * processWriteEvent is called; no parameters are allowed
     * in these methods
     */
    void processWriteEvent();
    EventWrapper<SimpleDRAM, &SimpleDRAM::processWriteEvent> writeEvent;

    void processRespondEvent();
    EventWrapper<SimpleDRAM, &SimpleDRAM::processRespondEvent> respondEvent;

    void processRefreshEvent();
    EventWrapper<SimpleDRAM, &SimpleDRAM::processRefreshEvent> refreshEvent;

    void processNextReqEvent();
    EventWrapper<SimpleDRAM,&SimpleDRAM::processNextReqEvent> nextReqEvent;


    /**
     * Check if the read queue has room for more entries
     *
     * @return true if read queue is full, false otherwise
     */
    bool readQueueFull() const;

    /**
     * Check if the write queue has room for more entries
     *
     * @return true if write queue is full, false otherwise
     */
    bool writeQueueFull() const;

    /**
     * When a new read comes in, first check if the write q has a
     * pending request to the same address.\ If not, decode the
     * address to populate rank/bank/row, create a "dram_pkt", and
     * push it to the back of the read queue.\ If this is the only
     * read request in the system, schedule an event to start
     * servicing it.
     *
     * @param pkt The request packet from the outside world
     */
    void addToReadQueue(PacketPtr pkt);

    /**
     * Decode the incoming pkt, create a dram_pkt and push to the
     * back of the write queue. \If the write q length is more than
     * the threshold specified by the user, ie the queue is beginning
     * to get full, stop reads, and start draining writes.
     *
     * @param pkt The request packet from the outside world
     */
    void addToWriteQueue(PacketPtr pkt);

    /**
     * Actually do the DRAM access - figure out the latency it
     * will take to service the req based on bank state, channel state etc
     * and then update those states to account for this request.\ Based
     * on this, update the packet's "readyTime" and move it to the
     * response q from where it will eventually go back to the outside
     * world.
     *
     * @param pkt The DRAM packet created from the outside world pkt
     */
    void doDRAMAccess(DRAMPacket* dram_pkt);

    /**
     * Check when the channel is free to turnaround, add turnaround
     * delay and schedule a whole bunch of writes.
     */
    void triggerWrites();

    /**
     * When a packet reaches its "readyTime" in the response Q,
     * use the "access()" method in AbstractMemory to actually
     * create the response packet, and send it back to the outside
     * world requestor.
     *
     * @param pkt The packet from the outside world
     */
    void accessAndRespond(PacketPtr pkt);

    /**
     * Address decoder to figure out physical mapping onto ranks,
     * banks, and rows.
     *
     * @param pkt The packet from the outside world
     * @return A DRAMPacket pointer with the decoded information
     */
    DRAMPacket* decodeAddr(PacketPtr pkt);

    /**
     * The memory schduler/arbiter - picks which request needs to
     * go next, based on the specified policy such as fcfs or frfcfs
     * and moves it to the head of the read queue
     *
     * @return True if a request was chosen, False if Q is empty
     */
    bool chooseNextReq();

    /**
     * Calls chooseNextReq() to pick the right request, then calls
     * doDRAMAccess on that request in order to actually service
     * that request
     */
    void scheduleNextReq();

    /**
     *Looks at the state of the banks, channels, row buffer hits etc
     * to estimate how long a request will take to complete.
     *
     * @param dram_pkt The request for which we want to estimate latency
     * @param inTime The tick at which you want to probe the memory
     *
     * @return A pair of ticks, one indicating how many ticks *after*
     *         inTime the request require, and the other indicating how
     *         much of that was just the bank access time, ignoring the
     *         ticks spent simply waiting for resources to become free
     */
    std::pair<Tick, Tick> estimateLatency(DRAMPacket* dram_pkt, Tick inTime);

    /**
     * Move the request at the head of the read queue to the response
     * queue, sorting by readyTime.\ If it is the only packet in the
     * response queue, schedule a respond event to send it back to the
     * outside world
     */
    void moveToRespQ();

    /**
     * Scheduling policy within the write Q
     */
    void chooseNextWrite();

    /**
     * Looking at all banks, determine the moment in time when they
     * are all free.
     *
     * @return The tick when all banks are free
     */
    Tick maxBankFreeAt() const;

    void printParams() const;
    void printQs() const;

    /**
     * The controller's main read and write queues
     */
    std::list<DRAMPacket*> dramReadQueue;
    std::list<DRAMPacket*> dramWriteQueue;

    /**
     * Response queue where read packets wait after we're done working
     * with them, but it's not time to send the response yet.\ It is
     * seperate mostly to keep the code clean and help with gem5 events,
     * but for all logical purposes such as sizing the read queue, this
     * and the main read queue need to be added together.
     */
    std::list<DRAMPacket*> dramRespQueue;

    /** If we need to drain, keep the drain manager around until we're done
     * here.
     */
    DrainManager *drainManager;

    /**
     * Multi-dimensional vector of banks, first dimension is ranks,
     * second is bank
     */
    std::vector<std::vector<Bank> > banks;

    /**
     * The following are basic design parameters of the memory
     * controller, and are initialized based on parameter values. The
     * bytesPerCacheLine is based on the neighbouring port and thus
     * determined outside the constructor. Similarly, the rowsPerBank
     * is determined based on the capacity, number of ranks and banks,
     * the cache line size, and the row buffer size.
     */
    uint32_t bytesPerCacheLine;
    const uint32_t linesPerRowBuffer;
    const uint32_t ranksPerChannel;
    const uint32_t banksPerRank;
    uint32_t rowsPerBank;
    const uint32_t readBufferSize;
    const uint32_t writeBufferSize;
    const double writeThresholdPerc;
    uint32_t writeThreshold;

    /**
     * Basic memory timing parameters initialized based on parameter
     * values.
     */
    const Tick tWTR;
    const Tick tBURST;
    const Tick tRCD;
    const Tick tCL;
    const Tick tRP;
    const Tick tRFC;
    const Tick tREFI;

    /**
     * Memory controller configuration initialized based on parameter
     * values.
     */
    Enums::MemSched memSchedPolicy;
    Enums::AddrMap addrMapping;
    Enums::PageManage pageMgmt;

    /**
     * Till when has the main data bus been spoken for already?
     */
    Tick busBusyUntil;

    Tick prevdramaccess;
    Tick writeStartTime;
    Tick prevArrival;
    int numReqs;

    // All statistics that the model needs to capture
    Stats::Scalar readReqs;
    Stats::Scalar writeReqs;
    Stats::Scalar cpuReqs;
    Stats::Scalar bytesRead;
    Stats::Scalar bytesWritten;
    Stats::Scalar bytesConsumedRd;
    Stats::Scalar bytesConsumedWr;
    Stats::Scalar servicedByWrQ;
    Stats::Scalar neitherReadNorWrite;
    Stats::Vector perBankRdReqs;
    Stats::Vector perBankWrReqs;
    Stats::Scalar numRdRetry;
    Stats::Scalar numWrRetry;
    Stats::Scalar totGap;
    Stats::Vector readPktSize;
    Stats::Vector writePktSize;
    Stats::Vector neitherPktSize;
    Stats::Vector rdQLenPdf;
    Stats::Vector wrQLenPdf;


    // Latencies summed over all requests
    Stats::Scalar totQLat;
    Stats::Scalar totMemAccLat;
    Stats::Scalar totBusLat;
    Stats::Scalar totBankLat;

    // Average latencies per request
    Stats::Formula avgQLat;
    Stats::Formula avgBankLat;
    Stats::Formula avgBusLat;
    Stats::Formula avgMemAccLat;

    // Average bandwidth
    Stats::Formula avgRdBW;
    Stats::Formula avgWrBW;
    Stats::Formula avgConsumedRdBW;
    Stats::Formula avgConsumedWrBW;
    Stats::Formula peakBW;
    Stats::Formula busUtil;

    // Average queue lengths
    Stats::Average avgRdQLen;
    Stats::Average avgWrQLen;

    // Row hit count and rate
    Stats::Scalar readRowHits;
    Stats::Scalar writeRowHits;
    Stats::Formula readRowHitRate;
    Stats::Formula writeRowHitRate;
    Stats::Formula avgGap;

    /** @todo this is a temporary workaround until the 4-phase code is
     * committed. upstream caches needs this packet until true is returned, so
     * hold onto it for deletion until a subsequent call
     */
    std::vector<PacketPtr> pendingDelete;

  public:

    void regStats();

    SimpleDRAM(const SimpleDRAMParams* p);

    unsigned int drain(DrainManager* dm);

    virtual BaseSlavePort& getSlavePort(const std::string& if_name,
                                        PortID idx = InvalidPortID);

    virtual void init();
    virtual void startup();

  protected:

    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);

};

#endif //__MEM_SIMPLE_DRAM_HH__
