/*
 * Copyright (c) 2012-2020 ARM Limited
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
 * Copyright (c) 2013 Amin Farmahini-Farahani
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

/**
 * @file
 * MemCtrl declaration
 */

#ifndef __MEM_CTRL_HH__
#define __MEM_CTRL_HH__

#include <deque>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "enums/MemSched.hh"
#include "mem/qos/mem_ctrl.hh"
#include "mem/qport.hh"
#include "params/MemCtrl.hh"
#include "sim/eventq.hh"

namespace gem5
{

namespace memory
{

class MemInterface;
class DRAMInterface;
class NVMInterface;

/**
 * A burst helper helps organize and manage a packet that is larger than
 * the memory burst size. A system packet that is larger than the burst size
 * is split into multiple packets and all those packets point to
 * a single burst helper such that we know when the whole packet is served.
 */
class BurstHelper
{
  public:
    /** Number of bursts requred for a system packet **/
    const unsigned int burstCount;

    /** Number of bursts serviced so far for a system packet **/
    unsigned int burstsServiced;

    BurstHelper(unsigned int _burstCount)
        : burstCount(_burstCount), burstsServiced(0)
    {}
};

/**
 * A memory packet stores packets along with the timestamp of when
 * the packet entered the queue, and also the decoded address.
 */
class MemPacket
{
  public:
    /** When did request enter the controller */
    const Tick entryTime;

    /** When will request leave the controller */
    Tick readyTime;

    /** This comes from the outside world */
    const PacketPtr pkt;

    /** RequestorID associated with the packet */
    const RequestorID _requestorId;

    const bool read;

    /** Does this packet access DRAM?*/
    const bool dram;

    /** pseudo channel num*/
    const uint8_t pseudoChannel;

    /** Will be populated by address decoder */
    const uint8_t rank;
    const uint8_t bank;
    const uint32_t row;

    /**
     * Bank id is calculated considering banks in all the ranks
     * eg: 2 ranks each with 8 banks, then bankId = 0 --> rank0, bank0 and
     * bankId = 8 --> rank1, bank0
     */
    const uint16_t bankId;

    /**
     * The starting address of the packet.
     * This address could be unaligned to burst size boundaries. The
     * reason is to keep the address offset so we can accurately check
     * incoming read packets with packets in the write queue.
     */
    Addr addr;

    /**
     * The size of this dram packet in bytes
     * It is always equal or smaller than the burst size
     */
    unsigned int size;

    /**
     * A pointer to the BurstHelper if this MemPacket is a split packet
     * If not a split packet (common case), this is set to NULL
     */
    BurstHelper *burstHelper;

    /**
     * QoS value of the encapsulated packet read at queuing time
     */
    uint8_t _qosValue;

    /**
     * Set the packet QoS value
     * (interface compatibility with Packet)
     */
    inline void
    qosValue(const uint8_t qv)
    {
        _qosValue = qv;
    }

    /**
     * Get the packet QoS value
     * (interface compatibility with Packet)
     */
    inline uint8_t
    qosValue() const
    {
        return _qosValue;
    }

    /**
     * Get the packet RequestorID
     * (interface compatibility with Packet)
     */
    inline RequestorID
    requestorId() const
    {
        return _requestorId;
    }

    /**
     * Get the packet size
     * (interface compatibility with Packet)
     */
    inline unsigned int
    getSize() const
    {
        return size;
    }

    /**
     * Get the packet address
     * (interface compatibility with Packet)
     */
    inline Addr
    getAddr() const
    {
        return addr;
    }

    /**
     * Return true if its a read packet
     * (interface compatibility with Packet)
     */
    inline bool
    isRead() const
    {
        return read;
    }

    /**
     * Return true if its a write packet
     * (interface compatibility with Packet)
     */
    inline bool
    isWrite() const
    {
        return !read;
    }

    /**
     * Return true if its a DRAM access
     */
    inline bool
    isDram() const
    {
        return dram;
    }

    MemPacket(PacketPtr _pkt, bool is_read, bool is_dram, uint8_t _channel,
              uint8_t _rank, uint8_t _bank, uint32_t _row, uint16_t bank_id,
              Addr _addr, unsigned int _size)
        : entryTime(curTick()),
          readyTime(curTick()),
          pkt(_pkt),
          _requestorId(pkt->requestorId()),
          read(is_read),
          dram(is_dram),
          pseudoChannel(_channel),
          rank(_rank),
          bank(_bank),
          row(_row),
          bankId(bank_id),
          addr(_addr),
          size(_size),
          burstHelper(NULL),
          _qosValue(_pkt->qosValue())
    {}
};

// The memory packets are store in a multiple dequeue structure,
// based on their QoS priority
typedef std::deque<MemPacket *> MemPacketQueue;

/**
 * The memory controller is a single-channel memory controller capturing
 * the most important timing constraints associated with a
 * contemporary controller. For multi-channel memory systems, the controller
 * is combined with a crossbar model, with the channel address
 * interleaving taking part in the crossbar.
 *
 * As a basic design principle, this controller
 * model is not cycle callable, but instead uses events to: 1) decide
 * when new decisions can be made, 2) when resources become available,
 * 3) when things are to be considered done, and 4) when to send
 * things back. The controller interfaces to media specific interfaces
 * to enable flexible topoloties.
 * Through these simple principles, the model delivers
 * high performance, and lots of flexibility, allowing users to
 * evaluate the system impact of a wide range of memory technologies.
 *
 * For more details, please see Hansson et al, "Simulating DRAM
 * controllers for future system architecture exploration",
 * Proc. ISPASS, 2014. If you use this model as part of your research
 * please cite the paper.
 *
 */
class MemCtrl : public qos::MemCtrl
{
  protected:
    // For now, make use of a queued response port to avoid dealing with
    // flow control for the responses being sent back
    class MemoryPort : public QueuedResponsePort
    {
        RespPacketQueue queue;
        MemCtrl &ctrl;

      public:
        MemoryPort(const std::string &name, MemCtrl &_ctrl);
        void disableSanityCheck();

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        Tick recvAtomicBackdoor(PacketPtr pkt,
                                MemBackdoorPtr &backdoor) override;

        void recvFunctional(PacketPtr pkt) override;
        void recvMemBackdoorReq(const MemBackdoorReq &req,
                                MemBackdoorPtr &backdoor) override;

        bool recvTimingReq(PacketPtr) override;

        AddrRangeList getAddrRanges() const override;
    };

    /**
     * Our incoming port, for a multi-ported controller add a crossbar
     * in front of it
     */
    MemoryPort port;

    /**
     * Remember if the memory system is in timing mode
     */
    bool isTimingMode;

    /**
     * Remember if we have to retry a request when available.
     */
    bool retryRdReq;
    bool retryWrReq;

    /**
     * Bunch of things requires to setup "events" in gem5
     * When event "respondEvent" occurs for example, the method
     * processRespondEvent is called; no parameters are allowed
     * in these methods
     */
    virtual void processNextReqEvent(MemInterface *mem_intr,
                                     MemPacketQueue &resp_queue,
                                     EventFunctionWrapper &resp_event,
                                     EventFunctionWrapper &next_req_event,
                                     bool &retry_wr_req);
    EventFunctionWrapper nextReqEvent;

    virtual void processRespondEvent(MemInterface *mem_intr,
                                     MemPacketQueue &queue,
                                     EventFunctionWrapper &resp_event,
                                     bool &retry_rd_req);
    EventFunctionWrapper respondEvent;

    /**
     * Check if the read queue has room for more entries
     *
     * @param pkt_count The number of entries needed in the read queue
     * @return true if read queue is full, false otherwise
     */
    bool readQueueFull(unsigned int pkt_count) const;

    /**
     * Check if the write queue has room for more entries
     *
     * @param pkt_count The number of entries needed in the write queue
     * @return true if write queue is full, false otherwise
     */
    bool writeQueueFull(unsigned int pkt_count) const;

    /**
     * When a new read comes in, first check if the write q has a
     * pending request to the same address.\ If not, decode the
     * address to populate rank/bank/row, create one or mutliple
     * "mem_pkt", and push them to the back of the read queue.\
     * If this is the only
     * read request in the system, schedule an event to start
     * servicing it.
     *
     * @param pkt The request packet from the outside world
     * @param pkt_count The number of memory bursts the pkt
     * @param mem_intr The memory interface this pkt will
     * eventually go to
     * @return if all the read pkts are already serviced by wrQ
     */
    bool addToReadQueue(PacketPtr pkt, unsigned int pkt_count,
                        MemInterface *mem_intr);

    /**
     * Decode the incoming pkt, create a mem_pkt and push to the
     * back of the write queue. \If the write q length is more than
     * the threshold specified by the user, ie the queue is beginning
     * to get full, stop reads, and start draining writes.
     *
     * @param pkt The request packet from the outside world
     * @param pkt_count The number of memory bursts the pkt
     * @param mem_intr The memory interface this pkt will
     * eventually go to
     */
    void addToWriteQueue(PacketPtr pkt, unsigned int pkt_count,
                         MemInterface *mem_intr);

    /**
     * Actually do the burst based on media specific access function.
     * Update bus statistics when complete.
     *
     * @param mem_pkt The memory packet created from the outside world pkt
     * @param mem_intr The memory interface to access
     * @return Time when the command was issued
     *
     */
    virtual Tick doBurstAccess(MemPacket *mem_pkt, MemInterface *mem_intr);

    /**
     * When a packet reaches its "readyTime" in the response Q,
     * use the "access()" method in AbstractMemory to actually
     * create the response packet, and send it back to the outside
     * world requestor.
     *
     * @param pkt The packet from the outside world
     * @param static_latency Static latency to add before sending the packet
     * @param mem_intr the memory interface to access
     */
    virtual void accessAndRespond(PacketPtr pkt, Tick static_latency,
                                  MemInterface *mem_intr);

    /**
     * Determine if there is a packet that can issue.
     *
     * @param pkt The packet to evaluate
     */
    virtual bool packetReady(MemPacket *pkt, MemInterface *mem_intr);

    /**
     * Calculate the minimum delay used when scheduling a read-to-write
     * transision.
     * @param return minimum delay
     */
    virtual Tick minReadToWriteDataGap();

    /**
     * Calculate the minimum delay used when scheduling a write-to-read
     * transision.
     * @param return minimum delay
     */
    virtual Tick minWriteToReadDataGap();

    /**
     * The memory schduler/arbiter - picks which request needs to
     * go next, based on the specified policy such as FCFS or FR-FCFS
     * and moves it to the head of the queue.
     * Prioritizes accesses to the same rank as previous burst unless
     * controller is switching command type.
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @param mem_intr the memory interface to choose from
     * @return an iterator to the selected packet, else queue.end()
     */
    virtual MemPacketQueue::iterator chooseNext(MemPacketQueue &queue,
                                                Tick extra_col_delay,
                                                MemInterface *mem_intr);

    /**
     * For FR-FCFS policy reorder the read/write queue depending on row buffer
     * hits and earliest bursts available in memory
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @return an iterator to the selected packet, else queue.end()
     */
    virtual std::pair<MemPacketQueue::iterator, Tick>
    chooseNextFRFCFS(MemPacketQueue &queue, Tick extra_col_delay,
                     MemInterface *mem_intr);

    /**
     * Calculate burst window aligned tick
     *
     * @param cmd_tick Initial tick of command
     * @return burst window aligned tick
     */
    Tick getBurstWindow(Tick cmd_tick);

    /**
     * Used for debugging to observe the contents of the queues.
     */
    void printQs() const;

    /**
     * Burst-align an address.
     *
     * @param addr The potentially unaligned address
     * @param mem_intr The DRAM interface this pkt belongs to
     *
     * @return An address aligned to a memory burst
     */
    virtual Addr burstAlign(Addr addr, MemInterface *mem_intr) const;

    /**
     * Check if mem pkt's size is sane
     *
     * @param mem_pkt memory packet
     * @param mem_intr memory interface
     * @return An address aligned to a memory burst
     */
    virtual bool pktSizeCheck(MemPacket *mem_pkt,
                              MemInterface *mem_intr) const;

    /**
     * The controller's main read and write queues,
     * with support for QoS reordering
     */
    std::vector<MemPacketQueue> readQueue;
    std::vector<MemPacketQueue> writeQueue;

    /**
     * To avoid iterating over the write queue to check for
     * overlapping transactions, maintain a set of burst addresses
     * that are currently queued. Since we merge writes to the same
     * location we never have more than one address to the same burst
     * address.
     */
    std::unordered_set<Addr> isInWriteQueue;

    /**
     * Response queue where read packets wait after we're done working
     * with them, but it's not time to send the response yet. The
     * responses are stored separately mostly to keep the code clean
     * and help with events scheduling. For all logical purposes such
     * as sizing the read queue, this and the main read queue need to
     * be added together.
     */
    std::deque<MemPacket *> respQueue;

    /**
     * Holds count of commands issued in burst window starting at
     * defined Tick. This is used to ensure that the command bandwidth
     * does not exceed the allowable media constraints.
     */
    std::unordered_multiset<Tick> burstTicks;

    /**
+    * Create pointer to interface of the actual memory media when connected
+    */
    MemInterface *dram;

    virtual AddrRangeList getAddrRanges();

    /**
     * The following are basic design parameters of the memory
     * controller, and are initialized based on parameter values.
     * The rowsPerBank is determined based on the capacity, number of
     * ranks and banks, the burst size, and the row buffer size.
     */
    uint32_t readBufferSize;
    uint32_t writeBufferSize;
    uint32_t writeHighThreshold;
    uint32_t writeLowThreshold;
    const uint32_t minWritesPerSwitch;
    const uint32_t minReadsPerSwitch;

    /**
     * Memory controller configuration initialized based on parameter
     * values.
     */
    enums::MemSched memSchedPolicy;

    /**
     * Pipeline latency of the controller frontend. The frontend
     * contribution is added to writes (that complete when they are in
     * the write buffer) and reads that are serviced the write buffer.
     */
    const Tick frontendLatency;

    /**
     * Pipeline latency of the backend and PHY. Along with the
     * frontend contribution, this latency is added to reads serviced
     * by the memory.
     */
    const Tick backendLatency;

    /**
     * Length of a command window, used to check
     * command bandwidth
     */
    const Tick commandWindow;

    /**
     * Till when must we wait before issuing next RD/WR burst?
     */
    Tick nextBurstAt;

    Tick prevArrival;

    /**
     * The soonest you have to start thinking about the next request
     * is the longest access time that can occur before
     * nextBurstAt. Assuming you need to precharge, open a new row,
     * and access, it is tRP + tRCD + tCL.
     */
    Tick nextReqTime;

    struct CtrlStats : public statistics::Group
    {
        CtrlStats(MemCtrl &ctrl);

        void regStats() override;

        MemCtrl &ctrl;

        // All statistics that the model needs to capture
        statistics::Scalar readReqs;
        statistics::Scalar writeReqs;
        statistics::Scalar readBursts;
        statistics::Scalar writeBursts;
        statistics::Scalar servicedByWrQ;
        statistics::Scalar mergedWrBursts;
        statistics::Scalar neitherReadNorWriteReqs;
        // Average queue lengths
        statistics::Average avgRdQLen;
        statistics::Average avgWrQLen;

        statistics::Scalar numRdRetry;
        statistics::Scalar numWrRetry;
        statistics::Vector readPktSize;
        statistics::Vector writePktSize;
        statistics::Vector rdQLenPdf;
        statistics::Vector wrQLenPdf;
        statistics::Histogram rdPerTurnAround;
        statistics::Histogram wrPerTurnAround;

        statistics::Scalar bytesReadWrQ;
        statistics::Scalar bytesReadSys;
        statistics::Scalar bytesWrittenSys;
        // Average bandwidth
        statistics::Formula avgRdBWSys;
        statistics::Formula avgWrBWSys;

        statistics::Scalar totGap;
        statistics::Formula avgGap;

        // per-requestor bytes read and written to memory
        statistics::Vector requestorReadBytes;
        statistics::Vector requestorWriteBytes;

        // per-requestor bytes read and written to memory rate
        statistics::Formula requestorReadRate;
        statistics::Formula requestorWriteRate;

        // per-requestor read and write serviced memory accesses
        statistics::Vector requestorReadAccesses;
        statistics::Vector requestorWriteAccesses;

        // per-requestor read and write total memory access latency
        statistics::Vector requestorReadTotalLat;
        statistics::Vector requestorWriteTotalLat;

        // per-requestor raed and write average memory access latency
        statistics::Formula requestorReadAvgLat;
        statistics::Formula requestorWriteAvgLat;
    };

    CtrlStats stats;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

    /**
     * Select either the read or write queue
     *
     * @param is_read The current burst is a read, select read queue
     * @return a reference to the appropriate queue
     */
    std::vector<MemPacketQueue> &
    selQueue(bool is_read)
    {
        return (is_read ? readQueue : writeQueue);
    };

    virtual bool
    respQEmpty()
    {
        return respQueue.empty();
    }

    /**
     * Checks if the memory interface is already busy
     *
     * @param mem_intr memory interface to check
     * @return a boolean indicating if memory is busy
     */
    virtual bool memBusy(MemInterface *mem_intr);

    /**
     * Will access memory interface and select non-deterministic
     * reads to issue
     * @param mem_intr memory interface to use
     */
    virtual void nonDetermReads(MemInterface *mem_intr);

    /**
     * Will check if all writes are for nvm interface
     * and nvm's write resp queue is full. The generic mem_intr is
     * used as the same function can be called for a dram interface,
     * in which case dram functions will eventually return false
     * @param mem_intr memory interface to use
     * @return a boolean showing if nvm is blocked with writes
     */
    virtual bool nvmWriteBlock(MemInterface *mem_intr);

    /**
     * Remove commands that have already issued from burstTicks
     */
    virtual void pruneBurstTick();

  public:
    MemCtrl(const MemCtrlParams &p);

    /**
     * Ensure that all interfaced have drained commands
     *
     * @return bool flag, set once drain complete
     */
    virtual bool allIntfDrained() const;

    DrainState drain() override;

    /**
     * Check for command bus contention for single cycle command.
     * If there is contention, shift command to next burst.
     * Check verifies that the commands issued per burst is less
     * than a defined max number, maxCommandsPerWindow.
     * Therefore, contention per cycle is not verified and instead
     * is done based on a burst window.
     *
     * @param cmd_tick Initial tick of command, to be verified
     * @param max_cmds_per_burst Number of commands that can issue
     *                           in a burst window
     * @return tick for command issue without contention
     */
    virtual Tick verifySingleCmd(Tick cmd_tick, Tick max_cmds_per_burst,
                                 bool row_cmd);

    /**
     * Check for command bus contention for multi-cycle (2 currently)
     * command. If there is contention, shift command(s) to next burst.
     * Check verifies that the commands issued per burst is less
     * than a defined max number, maxCommandsPerWindow.
     * Therefore, contention per cycle is not verified and instead
     * is done based on a burst window.
     *
     * @param cmd_tick Initial tick of command, to be verified
     * @param max_multi_cmd_split Maximum delay between commands
     * @param max_cmds_per_burst Number of commands that can issue
     *                           in a burst window
     * @return tick for command issue without contention
     */
    virtual Tick verifyMultiCmd(Tick cmd_tick, Tick max_cmds_per_burst,
                                Tick max_multi_cmd_split = 0);

    /**
     * Is there a respondEvent scheduled?
     *
     * @return true if event is scheduled
     */
    virtual bool
    respondEventScheduled(uint8_t pseudo_channel = 0) const
    {
        assert(pseudo_channel == 0);
        return respondEvent.scheduled();
    }

    /**
     * Is there a read/write burst Event scheduled?
     *
     * @return true if event is scheduled
     */
    virtual bool
    requestEventScheduled(uint8_t pseudo_channel = 0) const
    {
        assert(pseudo_channel == 0);
        return nextReqEvent.scheduled();
    }

    /**
     * restart the controller
     * This can be used by interfaces to restart the
     * scheduler after maintainence commands complete
     * @param Tick to schedule next event
     * @param pseudo_channel pseudo channel number for which scheduler
     * needs to restart, will always be 0 for controllers which control
     * only a single channel
     */
    virtual void
    restartScheduler(Tick tick, uint8_t pseudo_channel = 0)
    {
        assert(pseudo_channel == 0);
        schedule(nextReqEvent, tick);
    }

    /**
     * Check the current direction of the memory channel
     *
     * @param next_state Check either the current or next bus state
     * @return True when bus is currently in a read state
     */
    bool inReadBusState(bool next_state, const MemInterface *mem_intr) const;

    /**
     * Check the current direction of the memory channel
     *
     * @param next_state Check either the current or next bus state
     * @return True when bus is currently in a write state
     */
    bool inWriteBusState(bool next_state, const MemInterface *mem_intr) const;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    virtual void init() override;
    virtual void startup() override;
    virtual void drainResume() override;

  protected:
    virtual Tick recvAtomic(PacketPtr pkt);
    virtual Tick recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor);
    virtual void recvFunctional(PacketPtr pkt);
    virtual void recvMemBackdoorReq(const MemBackdoorReq &req,
                                    MemBackdoorPtr &backdoor);
    virtual bool recvTimingReq(PacketPtr pkt);

    bool recvFunctionalLogic(PacketPtr pkt, MemInterface *mem_intr);
    Tick recvAtomicLogic(PacketPtr pkt, MemInterface *mem_intr);
};

} // namespace memory
} // namespace gem5

#endif //__MEM_CTRL_HH__
