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
 * MemInterface declaration
 */

#ifndef __MEM_INTERFACE_HH__
#define __MEM_INTERFACE_HH__

#include <deque>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "enums/PageManage.hh"
#include "mem/abstract_mem.hh"
#include "mem/drampower.hh"
#include "mem/mem_ctrl.hh"
#include "params/DRAMInterface.hh"
#include "params/MemInterface.hh"
#include "params/NVMInterface.hh"
#include "sim/eventq.hh"

/**
 * General interface to memory device
 * Includes functions and parameters shared across media types
 */
class MemInterface : public AbstractMemory
{
  protected:
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
     * A pointer to the parent MemCtrl instance
     */
    MemCtrl* ctrl;

    /**
     * Number of commands that can issue in the defined controller
     * command window, used to verify command bandwidth
     */
    unsigned int maxCommandsPerWindow;

    /**
     * Memory controller configuration initialized based on parameter
     * values.
     */
    Enums::AddrMap addrMapping;

    /**
     * General device and channel characteristics
     * The rowsPerBank is determined based on the capacity, number of
     * ranks and banks, the burst size, and the row buffer size.
     */
    const uint32_t burstSize;
    const uint32_t deviceSize;
    const uint32_t deviceRowBufferSize;
    const uint32_t devicesPerRank;
    const uint32_t rowBufferSize;
    const uint32_t burstsPerRowBuffer;
    const uint32_t burstsPerStripe;
    const uint32_t ranksPerChannel;
    const uint32_t banksPerRank;
    uint32_t rowsPerBank;

    /**
     * General timing requirements
     */
    const Tick M5_CLASS_VAR_USED tCK;
    const Tick tCS;
    const Tick tBURST;
    const Tick tRTW;
    const Tick tWTR;

    /*
     * @return delay between write and read commands
     */
    virtual Tick writeToReadDelay() const { return tBURST + tWTR; }

    /*
     * @return delay between write and read commands
     */
    Tick readToWriteDelay() const { return tBURST + tRTW; }

    /*
     * @return delay between accesses to different ranks
     */
    Tick rankToRankDelay() const { return tBURST + tCS; }


  public:

    /**
      * Buffer sizes for read and write queues in the controller
      * These are passed to the controller on instantiation
      * Defining them here allows for buffers to be resized based
      * on memory type / configuration.
      */
    const uint32_t readBufferSize;
    const uint32_t writeBufferSize;

    /** Set a pointer to the controller and initialize
     * interface based on controller parameters
     * @param _ctrl pointer to the parent controller
     * @param command_window size of command window used to
     *                       check command bandwidth
     */
    void setCtrl(MemCtrl* _ctrl, unsigned int command_window);

    /**
     * Get an address in a dense range which starts from 0. The input
     * address is the physical address of the request in an address
     * space that contains other SimObjects apart from this
     * controller.
     *
     * @param addr The intput address which should be in the addrRange
     * @return An address in the continues range [0, max)
     */
    Addr getCtrlAddr(Addr addr) { return range.getOffset(addr); }

    /**
     * Setup the rank based on packet received
     *
     * @param integer value of rank to be setup. used to index ranks vector
     * @param are we setting up rank for read or write packet?
     */
    virtual void setupRank(const uint8_t rank, const bool is_read) = 0;

    /**
     * Check drain state of interface
     *
     * @return true if all ranks are drained and idle
     *
     */
    virtual bool allRanksDrained() const = 0;

    /**
     * For FR-FCFS policy, find first command that can issue
     * Function will be overriden by interface to select based
     * on media characteristics, used to determine when read
     * or write can issue.
     *
     * @param queue Queued requests to consider
     * @param min_col_at Minimum tick for 'seamless' issue
     * @return an iterator to the selected packet, else queue.end()
     * @return the tick when the packet selected will issue
     */
    virtual std::pair<MemPacketQueue::iterator, Tick>
    chooseNextFRFCFS(MemPacketQueue& queue, Tick min_col_at) const = 0;

    /*
     * Function to calulate unloaded latency
     */
    virtual Tick accessLatency() const = 0;

    /**
     * @return number of bytes in a burst for this interface
     */
    uint32_t bytesPerBurst() const { return burstSize; }

    /*
     * @return time to offset next command
     */
    virtual Tick commandOffset() const = 0;

    /**
     * Check if a burst operation can be issued to the interface
     *
     * @param Return true if RD/WR can issue
     */
    virtual bool burstReady(MemPacket* pkt) const = 0;

    /**
     * Determine the required delay for an access to a different rank
     *
     * @return required rank to rank delay
     */
    Tick rankDelay() const { return tCS; }

    /**
     *
     * @return minimum additional bus turnaround required for read-to-write
     */
    Tick minReadToWriteDataGap() const { return std::min(tRTW, tCS); }

    /**
     *
     * @return minimum additional bus turnaround required for write-to-read
     */
    Tick minWriteToReadDataGap() const { return std::min(tWTR, tCS); }

    /**
     * Address decoder to figure out physical mapping onto ranks,
     * banks, and rows. This function is called multiple times on the same
     * system packet if the pakcet is larger than burst of the memory. The
     * pkt_addr is used for the offset within the packet.
     *
     * @param pkt The packet from the outside world
     * @param pkt_addr The starting address of the packet
     * @param size The size of the packet in bytes
     * @param is_read Is the request for a read or a write to memory
     * @param is_dram Is the request to a DRAM interface
     * @return A MemPacket pointer with the decoded information
     */
    MemPacket* decodePacket(const PacketPtr pkt, Addr pkt_addr,
                           unsigned int size, bool is_read, bool is_dram);

    /**
     *  Add rank to rank delay to bus timing to all banks in all ranks
     *  when access to an alternate interface is issued
     *
     *  param cmd_at Time of current command used as starting point for
     *               addition of rank-to-rank delay
     */
    virtual void addRankToRankDelay(Tick cmd_at) = 0;

    typedef MemInterfaceParams Params;
    MemInterface(const Params* _p);
};

/**
 * Interface to DRAM devices with media specific parameters,
 * statistics, and functions.
 * The DRAMInterface includes a class for individual ranks
 * and per rank functions.
 */
class DRAMInterface : public MemInterface
{
  private:
    /**
     * Simple structure to hold the values needed to keep track of
     * commands for DRAMPower
     */
    struct Command
    {
       Data::MemCommand::cmds type;
       uint8_t bank;
       Tick timeStamp;

       constexpr Command(Data::MemCommand::cmds _type, uint8_t _bank,
                         Tick time_stamp)
            : type(_type), bank(_bank), timeStamp(time_stamp)
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
    enum PowerState
    {
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
    enum RefreshState
    {
        REF_IDLE = 0,
        REF_DRAIN,
        REF_PD_EXIT,
        REF_SREF_EXIT,
        REF_PRE,
        REF_START,
        REF_RUN
    };

    class Rank;
    struct RankStats : public Stats::Group
    {
        RankStats(DRAMInterface &dram, Rank &rank);

        void regStats() override;
        void resetStats() override;
        void preDumpStats() override;

        Rank &rank;

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
         * A reference to the parent DRAMInterface instance
         */
        DRAMInterface& dram;

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
         * delay low-power exit until this requirement is met
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

        /**
         * Track when we issued the last read/write burst
         */
        Tick lastBurstTick;

        Rank(const DRAMInterfaceParams* _p, int _rank,
             DRAMInterface& _dram);

        const std::string name() const { return csprintf("%d", rank); }

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
        bool forceSelfRefreshExit() const;

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

      protected:
        RankStats stats;
    };

    /**
     * Function for sorting Command structures based on timeStamp
     *
     * @param a Memory Command
     * @param next Memory Command
     * @return true if timeStamp of Command 1 < timeStamp of Command 2
     */
    static bool
    sortTime(const Command& cmd, const Command& cmd_next)
    {
        return cmd.timeStamp < cmd_next.timeStamp;
    }

    /**
     * DRAM specific device characteristics
     */
    const uint32_t bankGroupsPerRank;
    const bool bankGroupArch;

    /**
     * DRAM specific timing requirements
     */
    const Tick tCL;
    const Tick tBURST_MIN;
    const Tick tBURST_MAX;
    const Tick tCCD_L_WR;
    const Tick tCCD_L;
    const Tick tRCD;
    const Tick tRP;
    const Tick tRAS;
    const Tick tWR;
    const Tick tRTP;
    const Tick tRFC;
    const Tick tREFI;
    const Tick tRRD;
    const Tick tRRD_L;
    const Tick tPPD;
    const Tick tAAD;
    const Tick tXAW;
    const Tick tXP;
    const Tick tXS;
    const Tick clkResyncDelay;
    const bool dataClockSync;
    const bool burstInterleave;
    const uint8_t twoCycleActivate;
    const uint32_t activationLimit;
    const Tick wrToRdDlySameBG;
    const Tick rdToWrDlySameBG;


    Enums::PageManage pageMgmt;
    /**
     * Max column accesses (read and write) per row, before forefully
     * closing it.
     */
    const uint32_t maxAccessesPerRow;

    // timestamp offset
    uint64_t timeStampOffset;

    // Holds the value of the DRAM rank of burst issued
    uint8_t activeRank;

    /** Enable or disable DRAM powerdown states. */
    bool enableDRAMPowerdown;

    /** The time when stats were last reset used to calculate average power */
    Tick lastStatsResetTick;

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
     * @param pre_tick Time when the precharge takes place
     * @param auto_or_preall Is this an auto-precharge or precharge all command
     * @param trace Is this an auto precharge then do not add to trace
     */
    void prechargeBank(Rank& rank_ref, Bank& bank_ref,
                       Tick pre_tick, bool auto_or_preall = false,
                       bool trace = true);

    struct DRAMStats : public Stats::Group
    {
        DRAMStats(DRAMInterface &dram);

        void regStats() override;
        void resetStats() override;

        DRAMInterface &dram;

        /** total number of DRAM bursts serviced */
        Stats::Scalar readBursts;
        Stats::Scalar writeBursts;

        /** DRAM per bank stats */
        Stats::Vector perBankRdBursts;
        Stats::Vector perBankWrBursts;

        // Latencies summed over all requests
        Stats::Scalar totQLat;
        Stats::Scalar totBusLat;
        Stats::Scalar totMemAccLat;

        // Average latencies per request
        Stats::Formula avgQLat;
        Stats::Formula avgBusLat;
        Stats::Formula avgMemAccLat;

        // Row hit count and rate
        Stats::Scalar readRowHits;
        Stats::Scalar writeRowHits;
        Stats::Formula readRowHitRate;
        Stats::Formula writeRowHitRate;
        Stats::Histogram bytesPerActivate;
        // Number of bytes transferred to/from DRAM
        Stats::Scalar bytesRead;
        Stats::Scalar bytesWritten;

        // Average bandwidth
        Stats::Formula avgRdBW;
        Stats::Formula avgWrBW;
        Stats::Formula peakBW;
        // bus utilization
        Stats::Formula busUtil;
        Stats::Formula busUtilRead;
        Stats::Formula busUtilWrite;
        Stats::Formula pageHitRate;
    };

    DRAMStats stats;

    /**
      * Vector of dram ranks
      */
    std::vector<Rank*> ranks;

    /*
     * @return delay between write and read commands
     */
    Tick writeToReadDelay() const override { return tBURST + tWTR + tCL; }

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
    minBankPrep(const MemPacketQueue& queue, Tick min_col_at) const;

    /*
     * @return time to send a burst of data without gaps
     */
    Tick
    burstDelay() const
    {
        return (burstInterleave ? tBURST_MAX / 2 : tBURST);
    }

  public:
    /**
     * Initialize the DRAM interface and verify parameters
     */
    void init() override;

    /**
     * Iterate through dram ranks and instantiate per rank startup routine
     */
    void startup() override;

    /**
     * Setup the rank based on packet received
     *
     * @param integer value of rank to be setup. used to index ranks vector
     * @param are we setting up rank for read or write packet?
     */
    void setupRank(const uint8_t rank, const bool is_read) override;

    /**
     * Iterate through dram ranks to exit self-refresh in order to drain
     */
    void drainRanks();

    /**
     * Return true once refresh is complete for all ranks and there are no
     * additional commands enqueued.  (only evaluated when draining)
     * This will ensure that all banks are closed, power state is IDLE, and
     * power stats have been updated
     *
     * @return true if all ranks have refreshed, with no commands enqueued
     *
     */
    bool allRanksDrained() const override;

    /**
     * Iterate through DRAM ranks and suspend them
     */
    void suspend();

    /*
     * @return time to offset next command
     */
    Tick commandOffset() const override { return (tRP + tRCD); }

    /*
     * Function to calulate unloaded, closed bank access latency
     */
    Tick accessLatency() const override { return (tRP + tRCD + tCL); }

    /**
     * For FR-FCFS policy, find first DRAM command that can issue
     *
     * @param queue Queued requests to consider
     * @param min_col_at Minimum tick for 'seamless' issue
     * @return an iterator to the selected packet, else queue.end()
     * @return the tick when the packet selected will issue
     */
    std::pair<MemPacketQueue::iterator, Tick>
    chooseNextFRFCFS(MemPacketQueue& queue, Tick min_col_at) const override;

    /**
     * Actually do the burst - figure out the latency it
     * will take to service the req based on bank state, channel state etc
     * and then update those states to account for this request. Based
     * on this, update the packet's "readyTime" and move it to the
     * response q from where it will eventually go back to the outside
     * world.
     *
     * @param mem_pkt The packet created from the outside world pkt
     * @param next_burst_at Minimum bus timing requirement from controller
     * @param queue Reference to the read or write queue with the packet
     * @return pair, tick when current burst is issued and
     *               tick when next burst can issue
     */
    std::pair<Tick, Tick>
    doBurstAccess(MemPacket* mem_pkt, Tick next_burst_at,
                  const std::vector<MemPacketQueue>& queue);

    /**
     * Check if a burst operation can be issued to the DRAM
     *
     * @param Return true if RD/WR can issue
     *                    This requires the DRAM to be in the
     *                    REF IDLE state
     */
    bool
    burstReady(MemPacket* pkt) const override
    {
        return ranks[pkt->rank]->inRefIdleState();
    }

    /**
     * This function checks if ranks are actively refreshing and
     * therefore busy. The function also checks if ranks are in
     * the self-refresh state, in which case, a self-refresh exit
     * is initiated.
     *
     * return boolean if all ranks are in refresh and therefore busy
     */
    bool isBusy();

    /**
     *  Add rank to rank delay to bus timing to all DRAM banks in alli ranks
     *  when access to an alternate interface is issued
     *
     *  param cmd_at Time of current command used as starting point for
     *               addition of rank-to-rank delay
     */
    void addRankToRankDelay(Tick cmd_at) override;

    /**
     * Complete response process for DRAM when read burst is complete
     * This will update the counters and check if a power down state
     * can be entered.
     *
     * @param rank Specifies rank associated with read burst
     */
    void respondEvent(uint8_t rank);

    /**
     * Check the refresh state to determine if refresh needs
     * to be kicked back into action after a read response
     *
     * @param rank Specifies rank associated with read burst
     */
    void checkRefreshState(uint8_t rank);

    DRAMInterface(const DRAMInterfaceParams* _p);
};

/**
 * Interface to NVM devices with media specific parameters,
 * statistics, and functions.
 * The NVMInterface includes a class for individual ranks
 * and per rank functions.
 */
class NVMInterface : public MemInterface
{
  private:
    /**
     * NVM rank class simply includes a vector of banks.
     */
    class Rank : public EventManager
    {
      public:

        /**
         * Current Rank index
         */
        uint8_t rank;

        /**
         * Vector of NVM banks. Each rank is made of several banks
         * that can be accessed in parallel.
         */
        std::vector<Bank> banks;

        Rank(const NVMInterfaceParams* _p, int _rank,
             NVMInterface& _nvm);
    };

    /**
     * NVM specific device and channel characteristics
     */
    const uint32_t maxPendingWrites;
    const uint32_t maxPendingReads;
    const bool twoCycleRdWr;

    /**
     * NVM specific timing requirements
     */
    const Tick tREAD;
    const Tick tWRITE;
    const Tick tSEND;

    struct NVMStats : public Stats::Group
    {
        NVMStats(NVMInterface &nvm);

        void regStats() override;

        NVMInterface &nvm;

        /** NVM stats */
        Stats::Scalar readBursts;
        Stats::Scalar writeBursts;

        Stats::Vector perBankRdBursts;
        Stats::Vector perBankWrBursts;

        // Latencies summed over all requests
        Stats::Scalar totQLat;
        Stats::Scalar totBusLat;
        Stats::Scalar totMemAccLat;

        // Average latencies per request
        Stats::Formula avgQLat;
        Stats::Formula avgBusLat;
        Stats::Formula avgMemAccLat;

        Stats::Scalar bytesRead;
        Stats::Scalar bytesWritten;

        // Average bandwidth
        Stats::Formula avgRdBW;
        Stats::Formula avgWrBW;
        Stats::Formula peakBW;
        Stats::Formula busUtil;
        Stats::Formula busUtilRead;
        Stats::Formula busUtilWrite;

        /** NVM stats */
        Stats::Histogram pendingReads;
        Stats::Histogram pendingWrites;
        Stats::Histogram bytesPerBank;
    };
    NVMStats stats;

    void processWriteRespondEvent();
    EventFunctionWrapper writeRespondEvent;

    void processReadReadyEvent();
    EventFunctionWrapper readReadyEvent;

    /**
      * Vector of nvm ranks
      */
    std::vector<Rank*> ranks;

    /**
     * Holding queue for non-deterministic write commands, which
     * maintains writes that have been issued but have not completed
     * Stored seperately mostly to keep the code clean and help with
     * events scheduling.
     * This mimics a buffer on the media controller and therefore is
     * not added to the main write queue for sizing
     */
    std::list<Tick> writeRespQueue;

    std::deque<Tick> readReadyQueue;

    /**
     * Check if the write response queue is empty
     *
     * @param Return true if empty
     */
    bool writeRespQueueEmpty() const { return writeRespQueue.empty(); }

    /**
     * Till when must we wait before issuing next read command?
     */
    Tick nextReadAt;

    // keep track of reads that have issued for which data is either
    // not yet ready or has not yet been transferred to the ctrl
    uint16_t numPendingReads;
    uint16_t numReadDataReady;

  public:
    // keep track of the number of reads that have yet to be issued
    uint16_t numReadsToIssue;

    // number of writes in the writeQueue for the NVM interface
    uint32_t numWritesQueued;

    /**
     * Initialize the NVM interface and verify parameters
     */
    void init() override;

    /**
     * Setup the rank based on packet received
     *
     * @param integer value of rank to be setup. used to index ranks vector
     * @param are we setting up rank for read or write packet?
     */
    void setupRank(const uint8_t rank, const bool is_read) override;

    /**
     * Check drain state of NVM interface
     *
     * @return true if write response queue is empty
     *
     */
    bool allRanksDrained() const override { return writeRespQueueEmpty(); }

    /*
     * @return time to offset next command
     */
    Tick commandOffset() const override { return tBURST; }

    /**
     * Check if a burst operation can be issued to the NVM
     *
     * @param Return true if RD/WR can issue
     *                    for reads, also verfy that ready count
     *                    has been updated to a non-zero value to
     *                    account for race conditions between events
     */
    bool burstReady(MemPacket* pkt) const override;

    /**
     * This function checks if ranks are busy.
     * This state is true when either:
     * 1) There is no command with read data ready to transmit or
     * 2) The NVM inteface has reached the maximum number of outstanding
     *    writes commands.
     * @param read_queue_empty There are no read queued
     * @param all_writes_nvm   All writes in queue are for NVM interface
     * @return true of NVM is busy
     *
     */
    bool isBusy(bool read_queue_empty, bool all_writes_nvm);
    /**
     * For FR-FCFS policy, find first NVM command that can issue
     * default to first command to prepped region
     *
     * @param queue Queued requests to consider
     * @param min_col_at Minimum tick for 'seamless' issue
     * @return an iterator to the selected packet, else queue.end()
     * @return the tick when the packet selected will issue
     */
    std::pair<MemPacketQueue::iterator, Tick>
    chooseNextFRFCFS(MemPacketQueue& queue, Tick min_col_at) const override;

    /**
     *  Add rank to rank delay to bus timing to all NVM banks in alli ranks
     *  when access to an alternate interface is issued
     *
     *  param cmd_at Time of current command used as starting point for
     *               addition of rank-to-rank delay
     */
    void addRankToRankDelay(Tick cmd_at) override;


    /**
     * Select read command to issue asynchronously
     */
    void chooseRead(MemPacketQueue& queue);

    /*
     * Function to calulate unloaded access latency
     */
    Tick accessLatency() const override { return (tREAD + tSEND); }

    /**
     * Check if the write response queue has reached defined threshold
     *
     * @param Return true if full
     */
    bool
    writeRespQueueFull() const
    {
        return writeRespQueue.size() == maxPendingWrites;
    }

    bool
    readsWaitingToIssue() const
    {
        return ((numReadsToIssue != 0) &&
                (numPendingReads < maxPendingReads));
    }

    /**
     * Actually do the burst and update stats.
     *
     * @param pkt The packet created from the outside world pkt
     * @param next_burst_at Minimum bus timing requirement from controller
     * @return pair, tick when current burst is issued and
     *               tick when next burst can issue
     */
    std::pair<Tick, Tick>
    doBurstAccess(MemPacket* pkt, Tick next_burst_at);

    NVMInterface(const NVMInterfaceParams* _p);
};

#endif //__MEM_INTERFACE_HH__
