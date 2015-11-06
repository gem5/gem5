/*
 * Copyright (c) 2012-2015 ARM Limited
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
 *
 * Authors: Andreas Hansson
 *          Ani Udipi
 *          Neha Agarwal
 *          Omar Naji
 */

/**
 * @file
 * DRAMCtrl declaration
 */

#ifndef __MEM_DRAM_CTRL_HH__
#define __MEM_DRAM_CTRL_HH__

#include <deque>
#include <string>
#include <unordered_set>

#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "enums/MemSched.hh"
#include "enums/PageManage.hh"
#include "mem/abstract_mem.hh"
#include "mem/qport.hh"
#include "params/DRAMCtrl.hh"
#include "sim/eventq.hh"
#include "mem/drampower.hh"

/**
 * The DRAM controller is a single-channel memory controller capturing
 * the most important timing constraints associated with a
 * contemporary DRAM. For multi-channel memory systems, the controller
 * is combined with a crossbar model, with the channel address
 * interleaving taking part in the crossbar.
 *
 * As a basic design principle, this controller
 * model is not cycle callable, but instead uses events to: 1) decide
 * when new decisions can be made, 2) when resources become available,
 * 3) when things are to be considered done, and 4) when to send
 * things back. Through these simple principles, the model delivers
 * high performance, and lots of flexibility, allowing users to
 * evaluate the system impact of a wide range of memory technologies,
 * such as DDR3/4, LPDDR2/3/4, WideIO1/2, HBM and HMC.
 *
 * For more details, please see Hansson et al, "Simulating DRAM
 * controllers for future system architecture exploration",
 * Proc. ISPASS, 2014. If you use this model as part of your research
 * please cite the paper.
 */
class DRAMCtrl : public AbstractMemory
{

  private:

    // For now, make use of a queued slave port to avoid dealing with
    // flow control for the responses being sent back
    class MemoryPort : public QueuedSlavePort
    {

        RespPacketQueue queue;
        DRAMCtrl& memory;

      public:

        MemoryPort(const std::string& name, DRAMCtrl& _memory);

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
     * Remeber if the memory system is in timing mode
     */
    bool isTimingMode;

    /**
     * Remember if we have to retry a request when available.
     */
    bool retryRdReq;
    bool retryWrReq;

    /**
     * Bus state used to control the read/write switching and drive
     * the scheduling of the next request.
     */
    enum BusState {
        READ = 0,
        READ_TO_WRITE,
        WRITE,
        WRITE_TO_READ
    };

    BusState busState;

    /**
     * A basic class to track the bank state, i.e. what row is
     * currently open (if any), when is the bank free to accept a new
     * column (read/write) command, when can it be precharged, and
     * when can it be activated.
     *
     * The bank also keeps track of how many bytes have been accessed
     * in the open row since it was opened.
     */
    class Bank
    {

      public:

        static const uint32_t NO_ROW = -1;

        uint32_t openRow;
        uint8_t bank;
        uint8_t bankgr;

        Tick colAllowedAt;
        Tick preAllowedAt;
        Tick actAllowedAt;

        uint32_t rowAccesses;
        uint32_t bytesAccessed;

        Bank() :
            openRow(NO_ROW), bank(0), bankgr(0),
            colAllowedAt(0), preAllowedAt(0), actAllowedAt(0),
            rowAccesses(0), bytesAccessed(0)
        { }
    };


    /**
     * Rank class includes a vector of banks. Refresh and Power state
     * machines are defined per rank. Events required to change the
     * state of the refresh and power state machine are scheduled per
     * rank. This class allows the implementation of rank-wise refresh
     * and rank-wise power-down.
     */
    class Rank : public EventManager
    {

      private:

        /**
         * The power state captures the different operational states of
         * the DRAM and interacts with the bus read/write state machine,
         * and the refresh state machine. In the idle state all banks are
         * precharged. From there we either go to an auto refresh (as
         * determined by the refresh state machine), or to a precharge
         * power down mode. From idle the memory can also go to the active
         * state (with one or more banks active), and in turn from there
         * to active power down. At the moment we do not capture the deep
         * power down and self-refresh state.
         */
        enum PowerState {
            PWR_IDLE = 0,
            PWR_REF,
            PWR_PRE_PDN,
            PWR_ACT,
            PWR_ACT_PDN
        };

        /**
         * The refresh state is used to control the progress of the
         * refresh scheduling. When normal operation is in progress the
         * refresh state is idle. From there, it progresses to the refresh
         * drain state once tREFI has passed. The refresh drain state
         * captures the DRAM row active state, as it will stay there until
         * all ongoing accesses complete. Thereafter all banks are
         * precharged, and lastly, the DRAM is refreshed.
         */
        enum RefreshState {
            REF_IDLE = 0,
            REF_DRAIN,
            REF_PRE,
            REF_RUN
        };

        /**
         * A reference to the parent DRAMCtrl instance
         */
        DRAMCtrl& memory;

        /**
         * Since we are taking decisions out of order, we need to keep
         * track of what power transition is happening at what time, such
         * that we can go back in time and change history. For example, if
         * we precharge all banks and schedule going to the idle state, we
         * might at a later point decide to activate a bank before the
         * transition to idle would have taken place.
         */
        PowerState pwrStateTrans;

        /**
         * Current power state.
         */
        PowerState pwrState;

        /**
         * Track when we transitioned to the current power state
         */
        Tick pwrStateTick;

        /**
         * current refresh state
         */
        RefreshState refreshState;

        /**
         * Keep track of when a refresh is due.
         */
        Tick refreshDueAt;

        /*
         * Command energies
         */
        Stats::Scalar actEnergy;
        Stats::Scalar preEnergy;
        Stats::Scalar readEnergy;
        Stats::Scalar writeEnergy;
        Stats::Scalar refreshEnergy;

        /*
         * Active Background Energy
         */
        Stats::Scalar actBackEnergy;

        /*
         * Precharge Background Energy
         */
        Stats::Scalar preBackEnergy;

        Stats::Scalar totalEnergy;
        Stats::Scalar averagePower;

        /**
         * Track time spent in each power state.
         */
        Stats::Vector pwrStateTime;

        /**
         * Function to update Power Stats
         */
        void updatePowerStats();

        /**
         * Schedule a power state transition in the future, and
         * potentially override an already scheduled transition.
         *
         * @param pwr_state Power state to transition to
         * @param tick Tick when transition should take place
         */
        void schedulePowerEvent(PowerState pwr_state, Tick tick);

      public:

        /**
         * Current Rank index
         */
        uint8_t rank;

        /**
         * One DRAMPower instance per rank
         */
        DRAMPower power;

        /**
         * Vector of Banks. Each rank is made of several devices which in
         * term are made from several banks.
         */
        std::vector<Bank> banks;

        /**
         *  To track number of banks which are currently active for
         *  this rank.
         */
        unsigned int numBanksActive;

        /** List to keep track of activate ticks */
        std::deque<Tick> actTicks;

        Rank(DRAMCtrl& _memory, const DRAMCtrlParams* _p);

        const std::string name() const
        {
            return csprintf("%s_%d", memory.name(), rank);
        }

        /**
         * Kick off accounting for power and refresh states and
         * schedule initial refresh.
         *
         * @param ref_tick Tick for first refresh
         */
        void startup(Tick ref_tick);

        /**
         * Stop the refresh events.
         */
        void suspend();

        /**
         * Check if the current rank is available for scheduling.
         *
         * @param Return true if the rank is idle from a refresh point of view
         */
        bool isAvailable() const { return refreshState == REF_IDLE; }

        /**
         * Let the rank check if it was waiting for requests to drain
         * to allow it to transition states.
         */
        void checkDrainDone();

        /*
         * Function to register Stats
         */
        void regStats();

        void processActivateEvent();
        EventWrapper<Rank, &Rank::processActivateEvent>
        activateEvent;

        void processPrechargeEvent();
        EventWrapper<Rank, &Rank::processPrechargeEvent>
        prechargeEvent;

        void processRefreshEvent();
        EventWrapper<Rank, &Rank::processRefreshEvent>
        refreshEvent;

        void processPowerEvent();
        EventWrapper<Rank, &Rank::processPowerEvent>
        powerEvent;

    };

    /**
     * A burst helper helps organize and manage a packet that is larger than
     * the DRAM burst size. A system packet that is larger than the burst size
     * is split into multiple DRAM packets and all those DRAM packets point to
     * a single burst helper such that we know when the whole packet is served.
     */
    class BurstHelper {

      public:

        /** Number of DRAM bursts requred for a system packet **/
        const unsigned int burstCount;

        /** Number of DRAM bursts serviced so far for a system packet **/
        unsigned int burstsServiced;

        BurstHelper(unsigned int _burstCount)
            : burstCount(_burstCount), burstsServiced(0)
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

        const bool isRead;

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
         * The starting address of the DRAM packet.
         * This address could be unaligned to burst size boundaries. The
         * reason is to keep the address offset so we can accurately check
         * incoming read packets with packets in the write queue.
         */
        Addr addr;

        /**
         * The size of this dram packet in bytes
         * It is always equal or smaller than DRAM burst size
         */
        unsigned int size;

        /**
         * A pointer to the BurstHelper if this DRAMPacket is a split packet
         * If not a split packet (common case), this is set to NULL
         */
        BurstHelper* burstHelper;
        Bank& bankRef;
        Rank& rankRef;

        DRAMPacket(PacketPtr _pkt, bool is_read, uint8_t _rank, uint8_t _bank,
                   uint32_t _row, uint16_t bank_id, Addr _addr,
                   unsigned int _size, Bank& bank_ref, Rank& rank_ref)
            : entryTime(curTick()), readyTime(curTick()),
              pkt(_pkt), isRead(is_read), rank(_rank), bank(_bank), row(_row),
              bankId(bank_id), addr(_addr), size(_size), burstHelper(NULL),
              bankRef(bank_ref), rankRef(rank_ref)
        { }

    };

    /**
     * Bunch of things requires to setup "events" in gem5
     * When event "respondEvent" occurs for example, the method
     * processRespondEvent is called; no parameters are allowed
     * in these methods
     */
    void processNextReqEvent();
    EventWrapper<DRAMCtrl,&DRAMCtrl::processNextReqEvent> nextReqEvent;

    void processRespondEvent();
    EventWrapper<DRAMCtrl, &DRAMCtrl::processRespondEvent> respondEvent;

    /**
     * Check if the read queue has room for more entries
     *
     * @param pktCount The number of entries needed in the read queue
     * @return true if read queue is full, false otherwise
     */
    bool readQueueFull(unsigned int pktCount) const;

    /**
     * Check if the write queue has room for more entries
     *
     * @param pktCount The number of entries needed in the write queue
     * @return true if write queue is full, false otherwise
     */
    bool writeQueueFull(unsigned int pktCount) const;

    /**
     * When a new read comes in, first check if the write q has a
     * pending request to the same address.\ If not, decode the
     * address to populate rank/bank/row, create one or mutliple
     * "dram_pkt", and push them to the back of the read queue.\
     * If this is the only
     * read request in the system, schedule an event to start
     * servicing it.
     *
     * @param pkt The request packet from the outside world
     * @param pktCount The number of DRAM bursts the pkt
     * translate to. If pkt size is larger then one full burst,
     * then pktCount is greater than one.
     */
    void addToReadQueue(PacketPtr pkt, unsigned int pktCount);

    /**
     * Decode the incoming pkt, create a dram_pkt and push to the
     * back of the write queue. \If the write q length is more than
     * the threshold specified by the user, ie the queue is beginning
     * to get full, stop reads, and start draining writes.
     *
     * @param pkt The request packet from the outside world
     * @param pktCount The number of DRAM bursts the pkt
     * translate to. If pkt size is larger then one full burst,
     * then pktCount is greater than one.
     */
    void addToWriteQueue(PacketPtr pkt, unsigned int pktCount);

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
     * When a packet reaches its "readyTime" in the response Q,
     * use the "access()" method in AbstractMemory to actually
     * create the response packet, and send it back to the outside
     * world requestor.
     *
     * @param pkt The packet from the outside world
     * @param static_latency Static latency to add before sending the packet
     */
    void accessAndRespond(PacketPtr pkt, Tick static_latency);

    /**
     * Address decoder to figure out physical mapping onto ranks,
     * banks, and rows. This function is called multiple times on the same
     * system packet if the pakcet is larger than burst of the memory. The
     * dramPktAddr is used for the offset within the packet.
     *
     * @param pkt The packet from the outside world
     * @param dramPktAddr The starting address of the DRAM packet
     * @param size The size of the DRAM packet in bytes
     * @param isRead Is the request for a read or a write to DRAM
     * @return A DRAMPacket pointer with the decoded information
     */
    DRAMPacket* decodeAddr(PacketPtr pkt, Addr dramPktAddr, unsigned int size,
                           bool isRead);

    /**
     * The memory schduler/arbiter - picks which request needs to
     * go next, based on the specified policy such as FCFS or FR-FCFS
     * and moves it to the head of the queue.
     * Prioritizes accesses to the same rank as previous burst unless
     * controller is switching command type.
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @return true if a packet is scheduled to a rank which is available else
     * false
     */
    bool chooseNext(std::deque<DRAMPacket*>& queue, Tick extra_col_delay);

    /**
     * For FR-FCFS policy reorder the read/write queue depending on row buffer
     * hits and earliest bursts available in DRAM
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @return true if a packet is scheduled to a rank which is available else
     * false
     */
    bool reorderQueue(std::deque<DRAMPacket*>& queue, Tick extra_col_delay);

    /**
     * Find which are the earliest banks ready to issue an activate
     * for the enqueued requests. Assumes maximum of 64 banks per DIMM
     * Also checks if the bank is already prepped.
     *
     * @param queue Queued requests to consider
     * @param time of seamless burst command
     * @return One-hot encoded mask of bank indices
     * @return boolean indicating burst can issue seamlessly, with no gaps
     */
    std::pair<uint64_t, bool> minBankPrep(const std::deque<DRAMPacket*>& queue,
                                          Tick min_col_at) const;

    /**
     * Keep track of when row activations happen, in order to enforce
     * the maximum number of activations in the activation window. The
     * method updates the time that the banks become available based
     * on the current limits.
     *
     * @param rank_ref Reference to the rank
     * @param bank_ref Reference to the bank
     * @param act_tick Time when the activation takes place
     * @param row Index of the row
     */
    void activateBank(Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row);

    /**
     * Precharge a given bank and also update when the precharge is
     * done. This will also deal with any stats related to the
     * accesses to the open page.
     *
     * @param rank_ref The rank to precharge
     * @param bank_ref The bank to precharge
     * @param pre_at Time when the precharge takes place
     * @param trace Is this an auto precharge then do not add to trace
     */
    void prechargeBank(Rank& rank_ref, Bank& bank_ref,
                       Tick pre_at, bool trace = true);

    /**
     * Used for debugging to observe the contents of the queues.
     */
    void printQs() const;

    /**
     * Burst-align an address.
     *
     * @param addr The potentially unaligned address
     *
     * @return An address aligned to a DRAM burst
     */
    Addr burstAlign(Addr addr) const { return (addr & ~(Addr(burstSize - 1))); }

    /**
     * The controller's main read and write queues
     */
    std::deque<DRAMPacket*> readQueue;
    std::deque<DRAMPacket*> writeQueue;

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
     * responses are stored seperately mostly to keep the code clean
     * and help with events scheduling. For all logical purposes such
     * as sizing the read queue, this and the main read queue need to
     * be added together.
     */
    std::deque<DRAMPacket*> respQueue;

    /**
     * Vector of ranks
     */
    std::vector<Rank*> ranks;

    /**
     * The following are basic design parameters of the memory
     * controller, and are initialized based on parameter values.
     * The rowsPerBank is determined based on the capacity, number of
     * ranks and banks, the burst size, and the row buffer size.
     */
    const uint32_t deviceSize;
    const uint32_t deviceBusWidth;
    const uint32_t burstLength;
    const uint32_t deviceRowBufferSize;
    const uint32_t devicesPerRank;
    const uint32_t burstSize;
    const uint32_t rowBufferSize;
    const uint32_t columnsPerRowBuffer;
    const uint32_t columnsPerStripe;
    const uint32_t ranksPerChannel;
    const uint32_t bankGroupsPerRank;
    const bool bankGroupArch;
    const uint32_t banksPerRank;
    const uint32_t channels;
    uint32_t rowsPerBank;
    const uint32_t readBufferSize;
    const uint32_t writeBufferSize;
    const uint32_t writeHighThreshold;
    const uint32_t writeLowThreshold;
    const uint32_t minWritesPerSwitch;
    uint32_t writesThisTime;
    uint32_t readsThisTime;

    /**
     * Basic memory timing parameters initialized based on parameter
     * values.
     */
    const Tick M5_CLASS_VAR_USED tCK;
    const Tick tWTR;
    const Tick tRTW;
    const Tick tCS;
    const Tick tBURST;
    const Tick tCCD_L;
    const Tick tRCD;
    const Tick tCL;
    const Tick tRP;
    const Tick tRAS;
    const Tick tWR;
    const Tick tRTP;
    const Tick tRFC;
    const Tick tREFI;
    const Tick tRRD;
    const Tick tRRD_L;
    const Tick tXAW;
    const uint32_t activationLimit;

    /**
     * Memory controller configuration initialized based on parameter
     * values.
     */
    Enums::MemSched memSchedPolicy;
    Enums::AddrMap addrMapping;
    Enums::PageManage pageMgmt;

    /**
     * Max column accesses (read and write) per row, before forefully
     * closing it.
     */
    const uint32_t maxAccessesPerRow;

    /**
     * Pipeline latency of the controller frontend. The frontend
     * contribution is added to writes (that complete when they are in
     * the write buffer) and reads that are serviced the write buffer.
     */
    const Tick frontendLatency;

    /**
     * Pipeline latency of the backend and PHY. Along with the
     * frontend contribution, this latency is added to reads serviced
     * by the DRAM.
     */
    const Tick backendLatency;

    /**
     * Till when has the main data bus been spoken for already?
     */
    Tick busBusyUntil;

    Tick prevArrival;

    /**
     * The soonest you have to start thinking about the next request
     * is the longest access time that can occur before
     * busBusyUntil. Assuming you need to precharge, open a new row,
     * and access, it is tRP + tRCD + tCL.
     */
    Tick nextReqTime;

    // All statistics that the model needs to capture
    Stats::Scalar readReqs;
    Stats::Scalar writeReqs;
    Stats::Scalar readBursts;
    Stats::Scalar writeBursts;
    Stats::Scalar bytesReadDRAM;
    Stats::Scalar bytesReadWrQ;
    Stats::Scalar bytesWritten;
    Stats::Scalar bytesReadSys;
    Stats::Scalar bytesWrittenSys;
    Stats::Scalar servicedByWrQ;
    Stats::Scalar mergedWrBursts;
    Stats::Scalar neitherReadNorWrite;
    Stats::Vector perBankRdBursts;
    Stats::Vector perBankWrBursts;
    Stats::Scalar numRdRetry;
    Stats::Scalar numWrRetry;
    Stats::Scalar totGap;
    Stats::Vector readPktSize;
    Stats::Vector writePktSize;
    Stats::Vector rdQLenPdf;
    Stats::Vector wrQLenPdf;
    Stats::Histogram bytesPerActivate;
    Stats::Histogram rdPerTurnAround;
    Stats::Histogram wrPerTurnAround;

    // Latencies summed over all requests
    Stats::Scalar totQLat;
    Stats::Scalar totMemAccLat;
    Stats::Scalar totBusLat;

    // Average latencies per request
    Stats::Formula avgQLat;
    Stats::Formula avgBusLat;
    Stats::Formula avgMemAccLat;

    // Average bandwidth
    Stats::Formula avgRdBW;
    Stats::Formula avgWrBW;
    Stats::Formula avgRdBWSys;
    Stats::Formula avgWrBWSys;
    Stats::Formula peakBW;
    Stats::Formula busUtil;
    Stats::Formula busUtilRead;
    Stats::Formula busUtilWrite;

    // Average queue lengths
    Stats::Average avgRdQLen;
    Stats::Average avgWrQLen;

    // Row hit count and rate
    Stats::Scalar readRowHits;
    Stats::Scalar writeRowHits;
    Stats::Formula readRowHitRate;
    Stats::Formula writeRowHitRate;
    Stats::Formula avgGap;

    // DRAM Power Calculation
    Stats::Formula pageHitRate;

    // Holds the value of the rank of burst issued
    uint8_t activeRank;

    // timestamp offset
    uint64_t timeStampOffset;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

    /**
     * This function increments the energy when called. If stats are
     * dumped periodically, note accumulated energy values will
     * appear in the stats (even if the stats are reset). This is a
     * result of the energy values coming from DRAMPower, and there
     * is currently no support for resetting the state.
     *
     * @param rank Currrent rank
     */
    void updatePowerStats(Rank& rank_ref);

    /**
     * Function for sorting commands in the command list of DRAMPower.
     *
     * @param a Memory Command in command list of DRAMPower library
     * @param next Memory Command in command list of DRAMPower
     * @return true if timestamp of Command 1 < timestamp of Command 2
     */
    static bool sortTime(const Data::MemCommand& m1,
                         const Data::MemCommand& m2) {
        return m1.getTime() < m2.getTime();
    };


  public:

    void regStats() override;

    DRAMCtrl(const DRAMCtrlParams* p);

    DrainState drain() override;

    virtual BaseSlavePort& getSlavePort(const std::string& if_name,
                                        PortID idx = InvalidPortID) override;

    virtual void init() override;
    virtual void startup() override;
    virtual void drainResume() override;

  protected:

    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);

};

#endif //__MEM_DRAM_CTRL_HH__
