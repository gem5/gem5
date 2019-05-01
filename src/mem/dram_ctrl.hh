/*
 * Copyright (c) 2012-2018 ARM Limited
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
 *          Matthias Jung
 *          Wendy Elsasser
 *          Radhika Jagtap
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
#include <vector>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "enums/MemSched.hh"
#include "enums/PageManage.hh"
#include "mem/drampower.hh"
#include "mem/qos/mem_ctrl.hh"
#include "mem/qport.hh"
#include "params/DRAMCtrl.hh"
#include "sim/eventq.hh"

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
 *
 * The low-power functionality implements a staggered powerdown
 * similar to that described in "Optimized Active and Power-Down Mode
 * Refresh Control in 3D-DRAMs" by Jung et al, VLSI-SoC, 2014.
 */
class DRAMCtrl : public QoS::MemCtrl
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
     * Remember if the memory system is in timing mode
     */
    bool isTimingMode;

    /**
     * Remember if we have to retry a request when available.
     */
    bool retryRdReq;
    bool retryWrReq;

    /**/

    /**
     * Simple structure to hold the values needed to keep track of
     * commands for DRAMPower
     */
    struct Command {
       Data::MemCommand::cmds type;
       uint8_t bank;
       Tick timeStamp;

       constexpr Command(Data::MemCommand::cmds _type, uint8_t _bank,
                         Tick time_stamp)
            : type(_type), bank(_bank), timeStamp(time_stamp)
        { }
    };

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

        Tick rdAllowedAt;
        Tick wrAllowedAt;
        Tick preAllowedAt;
        Tick actAllowedAt;

        uint32_t rowAccesses;
        uint32_t bytesAccessed;

        Bank() :
            openRow(NO_ROW), bank(0), bankgr(0),
            rdAllowedAt(0), wrAllowedAt(0), preAllowedAt(0), actAllowedAt(0),
            rowAccesses(0), bytesAccessed(0)
        { }
    };


    /**
     * The power state captures the different operational states of
     * the DRAM and interacts with the bus read/write state machine,
     * and the refresh state machine.
     *
     * PWR_IDLE      : The idle state in which all banks are closed
     *                 From here can transition to:  PWR_REF, PWR_ACT,
     *                 PWR_PRE_PDN
     *
     * PWR_REF       : Auto-refresh state.  Will transition when refresh is
     *                 complete based on power state prior to PWR_REF
     *                 From here can transition to:  PWR_IDLE, PWR_PRE_PDN,
     *                 PWR_SREF
     *
     * PWR_SREF      : Self-refresh state.  Entered after refresh if
     *                 previous state was PWR_PRE_PDN
     *                 From here can transition to:  PWR_IDLE
     *
     * PWR_PRE_PDN   : Precharge power down state
     *                 From here can transition to:  PWR_REF, PWR_IDLE
     *
     * PWR_ACT       : Activate state in which one or more banks are open
     *                 From here can transition to:  PWR_IDLE, PWR_ACT_PDN
     *
     * PWR_ACT_PDN   : Activate power down state
     *                 From here can transition to:  PWR_ACT
     */
     enum PowerState {
         PWR_IDLE = 0,
         PWR_REF,
         PWR_SREF,
         PWR_PRE_PDN,
         PWR_ACT,
         PWR_ACT_PDN
     };

    /**
     * The refresh state is used to control the progress of the
     * refresh scheduling. When normal operation is in progress the
     * refresh state is idle. Once tREFI has elasped, a refresh event
     * is triggered to start the following STM transitions which are
     * used to issue a refresh and return back to normal operation
     *
     * REF_IDLE      : IDLE state used during normal operation
     *                 From here can transition to:  REF_DRAIN
     *
     * REF_SREF_EXIT : Exiting a self-refresh; refresh event scheduled
     *                 after self-refresh exit completes
     *                 From here can transition to:  REF_DRAIN
     *
     * REF_DRAIN     : Drain state in which on going accesses complete.
     *                 From here can transition to:  REF_PD_EXIT
     *
     * REF_PD_EXIT   : Evaluate pwrState and issue wakeup if needed
     *                 Next state dependent on whether banks are open
     *                 From here can transition to:  REF_PRE, REF_START
     *
     * REF_PRE       : Close (precharge) all open banks
     *                 From here can transition to:  REF_START
     *
     * REF_START     : Issue refresh command and update DRAMPower stats
     *                 From here can transition to:  REF_RUN
     *
     * REF_RUN       : Refresh running, waiting for tRFC to expire
     *                 From here can transition to:  REF_IDLE, REF_SREF_EXIT
     */
     enum RefreshState {
         REF_IDLE = 0,
         REF_DRAIN,
         REF_PD_EXIT,
         REF_SREF_EXIT,
         REF_PRE,
         REF_START,
         REF_RUN
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
         * A reference to the parent DRAMCtrl instance
         */
        DRAMCtrl& memory;

        /**
         * Since we are taking decisions out of order, we need to keep
         * track of what power transition is happening at what time
         */
        PowerState pwrStateTrans;

        /**
         * Previous low-power state, which will be re-entered after refresh.
         */
        PowerState pwrStatePostRefresh;

        /**
         * Track when we transitioned to the current power state
         */
        Tick pwrStateTick;

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

        /*
         * Active Power-Down Energy
         */
        Stats::Scalar actPowerDownEnergy;

        /*
         * Precharge Power-Down Energy
         */
        Stats::Scalar prePowerDownEnergy;

        /*
         * self Refresh Energy
         */
        Stats::Scalar selfRefreshEnergy;

        Stats::Scalar totalEnergy;
        Stats::Scalar averagePower;

        /**
         * Stat to track total DRAM idle time
         *
         */
        Stats::Scalar totalIdleTime;

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
         * Current power state.
         */
        PowerState pwrState;

       /**
         * current refresh state
         */
        RefreshState refreshState;

        /**
         * rank is in or transitioning to power-down or self-refresh
         */
        bool inLowPowerState;

        /**
         * Current Rank index
         */
        uint8_t rank;

       /**
         * Track number of packets in read queue going to this rank
         */
        uint32_t readEntries;

       /**
         * Track number of packets in write queue going to this rank
         */
        uint32_t writeEntries;

        /**
         * Number of ACT, RD, and WR events currently scheduled
         * Incremented when a refresh event is started as well
         * Used to determine when a low-power state can be entered
         */
        uint8_t outstandingEvents;

        /**
         * delay power-down and self-refresh exit until this requirement is met
         */
        Tick wakeUpAllowedAt;

        /**
         * One DRAMPower instance per rank
         */
        DRAMPower power;

        /**
         * List of commands issued, to be sent to DRAMPpower at refresh
         * and stats dump.  Keep commands here since commands to different
         * banks are added out of order.  Will only pass commands up to
         * curTick() to DRAMPower after sorting.
         */
        std::vector<Command> cmdList;

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

        Rank(DRAMCtrl& _memory, const DRAMCtrlParams* _p, int rank);

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
         * Check if there is no refresh and no preparation of refresh ongoing
         * i.e. the refresh state machine is in idle
         *
         * @param Return true if the rank is idle from a refresh point of view
         */
        bool inRefIdleState() const { return refreshState == REF_IDLE; }

        /**
         * Check if the current rank has all banks closed and is not
         * in a low power state
         *
         * @param Return true if the rank is idle from a bank
         *        and power point of view
         */
        bool inPwrIdleState() const { return pwrState == PWR_IDLE; }

        /**
         * Trigger a self-refresh exit if there are entries enqueued
         * Exit if there are any read entries regardless of the bus state.
         * If we are currently issuing write commands, exit if we have any
         * write commands enqueued as well.
         * Could expand this in the future to analyze state of entire queue
         * if needed.
         *
         * @return boolean indicating self-refresh exit should be scheduled
         */
        bool forceSelfRefreshExit() const {
            return (readEntries != 0) ||
                   ((memory.busStateNext == WRITE) && (writeEntries != 0));
        }

        /**
         * Check if the command queue of current rank is idle
         *
         * @param Return true if the there are no commands in Q.
         *                    Bus direction determines queue checked.
         */
        bool isQueueEmpty() const;

        /**
         * Let the rank check if it was waiting for requests to drain
         * to allow it to transition states.
         */
        void checkDrainDone();

        /**
         * Push command out of cmdList queue that are scheduled at
         * or before curTick() to DRAMPower library
         * All commands before curTick are guaranteed to be complete
         * and can safely be flushed.
         */
        void flushCmdList();

        /*
         * Function to register Stats
         */
        void regStats();

        /**
         * Computes stats just prior to dump event
         */
        void computeStats();

        /**
         * Reset stats on a stats event
         */
        void resetStats();

        /**
         * Schedule a transition to power-down (sleep)
         *
         * @param pwr_state Power state to transition to
         * @param tick Absolute tick when transition should take place
         */
        void powerDownSleep(PowerState pwr_state, Tick tick);

       /**
         * schedule and event to wake-up from power-down or self-refresh
         * and update bank timing parameters
         *
         * @param exit_delay Relative tick defining the delay required between
         *                   low-power exit and the next command
         */
        void scheduleWakeUpEvent(Tick exit_delay);

        void processWriteDoneEvent();
        EventFunctionWrapper writeDoneEvent;

        void processActivateEvent();
        EventFunctionWrapper activateEvent;

        void processPrechargeEvent();
        EventFunctionWrapper prechargeEvent;

        void processRefreshEvent();
        EventFunctionWrapper refreshEvent;

        void processPowerEvent();
        EventFunctionWrapper powerEvent;

        void processWakeUpEvent();
        EventFunctionWrapper wakeUpEvent;

    };

    /**
     * Define the process to compute stats on a stats dump event, e.g. on
     * simulation exit or intermediate stats dump. This is defined per rank
     * as the per rank stats are based on state transition and periodically
     * updated, requiring re-sync at exit.
     */
    class RankDumpCallback : public Callback
    {
        Rank *ranks;
      public:
        RankDumpCallback(Rank *r) : ranks(r) {}
        virtual void process() { ranks->computeStats(); };
    };

    /** Define a process to clear power lib counters on a stats reset */
    class RankResetCallback : public Callback
    {
      private:
        /** Pointer to the rank, thus we instantiate per rank */
        Rank *rank;

      public:
        RankResetCallback(Rank *r) : rank(r) {}
        virtual void process() { rank->resetStats(); };
    };

    /** Define a process to store the time on a stats reset */
    class MemResetCallback : public Callback
    {
      private:
        /** A reference to the DRAMCtrl instance */
        DRAMCtrl *mem;

      public:
        MemResetCallback(DRAMCtrl *_mem) : mem(_mem) {}
        virtual void process() { mem->lastStatsResetTick = curTick(); };
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

        /** MasterID associated with the packet */
        const MasterID _masterId;

        const bool read;

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

        /**
         * QoS value of the encapsulated packet read at queuing time
         */
        uint8_t _qosValue;

        /**
         * Set the packet QoS value
         * (interface compatibility with Packet)
         */
        inline void qosValue(const uint8_t qv) { _qosValue = qv; }

        /**
         * Get the packet QoS value
         * (interface compatibility with Packet)
         */
        inline uint8_t qosValue() const { return _qosValue; }

        /**
         * Get the packet MasterID
         * (interface compatibility with Packet)
         */
        inline MasterID masterId() const { return _masterId; }

        /**
         * Get the packet size
         * (interface compatibility with Packet)
         */
        inline unsigned int getSize() const { return size; }

        /**
         * Get the packet address
         * (interface compatibility with Packet)
         */
        inline Addr getAddr() const { return addr; }

        /**
         * Return true if its a read packet
         * (interface compatibility with Packet)
         */
        inline bool isRead() const { return read; }

        /**
         * Return true if its a write packet
         * (interface compatibility with Packet)
         */
        inline bool isWrite() const { return !read; }


        DRAMPacket(PacketPtr _pkt, bool is_read, uint8_t _rank, uint8_t _bank,
                   uint32_t _row, uint16_t bank_id, Addr _addr,
                   unsigned int _size, Bank& bank_ref, Rank& rank_ref)
            : entryTime(curTick()), readyTime(curTick()), pkt(_pkt),
              _masterId(pkt->masterId()),
              read(is_read), rank(_rank), bank(_bank), row(_row),
              bankId(bank_id), addr(_addr), size(_size), burstHelper(NULL),
              bankRef(bank_ref), rankRef(rank_ref), _qosValue(_pkt->qosValue())
        { }

    };

    // The DRAM packets are store in a multiple dequeue structure,
    // based on their QoS priority
    typedef std::deque<DRAMPacket*> DRAMPacketQueue;

    /**
     * Bunch of things requires to setup "events" in gem5
     * When event "respondEvent" occurs for example, the method
     * processRespondEvent is called; no parameters are allowed
     * in these methods
     */
    void processNextReqEvent();
    EventFunctionWrapper nextReqEvent;

    void processRespondEvent();
    EventFunctionWrapper respondEvent;

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
    DRAMPacket* decodeAddr(const PacketPtr pkt, Addr dramPktAddr,
                           unsigned int size, bool isRead) const;

    /**
     * The memory schduler/arbiter - picks which request needs to
     * go next, based on the specified policy such as FCFS or FR-FCFS
     * and moves it to the head of the queue.
     * Prioritizes accesses to the same rank as previous burst unless
     * controller is switching command type.
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @return an iterator to the selected packet, else queue.end()
     */
    DRAMPacketQueue::iterator chooseNext(DRAMPacketQueue& queue,
        Tick extra_col_delay);

    /**
     * For FR-FCFS policy reorder the read/write queue depending on row buffer
     * hits and earliest bursts available in DRAM
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @return an iterator to the selected packet, else queue.end()
     */
    DRAMPacketQueue::iterator chooseNextFRFCFS(DRAMPacketQueue& queue,
            Tick extra_col_delay);

    /**
     * Find which are the earliest banks ready to issue an activate
     * for the enqueued requests. Assumes maximum of 32 banks per rank
     * Also checks if the bank is already prepped.
     *
     * @param queue Queued requests to consider
     * @param min_col_at time of seamless burst command
     * @return One-hot encoded mask of bank indices
     * @return boolean indicating burst can issue seamlessly, with no gaps
     */
    std::pair<std::vector<uint32_t>, bool>
    minBankPrep(const DRAMPacketQueue& queue, Tick min_col_at) const;

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
     * The controller's main read and write queues, with support for QoS reordering
     */
    std::vector<DRAMPacketQueue> readQueue;
    std::vector<DRAMPacketQueue> writeQueue;

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
    const Tick tRTW;
    const Tick tCS;
    const Tick tBURST;
    const Tick tCCD_L_WR;
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
    const Tick tXP;
    const Tick tXS;
    const uint32_t activationLimit;
    const Tick rankToRankDly;
    const Tick wrToRdDly;
    const Tick rdToWrDly;

    /**
     * Memory controller configuration initialized based on parameter
     * values.
     */
    Enums::MemSched memSchedPolicy;
    Enums::AddrMap addrMapping;
    Enums::PageManage pageMgmt;

    /**
     * Max column accesses (read and write) per row, before forcefully
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

    // per-master bytes read and written to memory
    Stats::Vector masterReadBytes;
    Stats::Vector masterWriteBytes;

    // per-master bytes read and written to memory rate
    Stats::Formula masterReadRate;
    Stats::Formula masterWriteRate;

    // per-master read and write serviced memory accesses
    Stats::Vector masterReadAccesses;
    Stats::Vector masterWriteAccesses;

    // per-master read and write total memory access latency
    Stats::Vector masterReadTotalLat;
    Stats::Vector masterWriteTotalLat;

    // per-master raed and write average memory access latency
    Stats::Formula masterReadAvgLat;
    Stats::Formula masterWriteAvgLat;

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

    /** The time when stats were last reset used to calculate average power */
    Tick lastStatsResetTick;

    /** Enable or disable DRAM powerdown states. */
    bool enableDRAMPowerdown;

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
     * @param rank Current rank
     */
    void updatePowerStats(Rank& rank_ref);

    /**
     * Function for sorting Command structures based on timeStamp
     *
     * @param a Memory Command
     * @param next Memory Command
     * @return true if timeStamp of Command 1 < timeStamp of Command 2
     */
    static bool sortTime(const Command& cmd, const Command& cmd_next) {
        return cmd.timeStamp < cmd_next.timeStamp;
    };

  public:

    void regStats() override;

    DRAMCtrl(const DRAMCtrlParams* p);

    DrainState drain() override;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    virtual void init() override;
    virtual void startup() override;
    virtual void drainResume() override;

    /**
     * Return true once refresh is complete for all ranks and there are no
     * additional commands enqueued.  (only evaluated when draining)
     * This will ensure that all banks are closed, power state is IDLE, and
     * power stats have been updated
     *
     * @return true if all ranks have refreshed, with no commands enqueued
     *
     */
    bool allRanksDrained() const;

  protected:

    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);

};

#endif //__MEM_DRAM_CTRL_HH__
