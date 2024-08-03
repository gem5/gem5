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
 * DRAMInterface declaration
 */

#ifndef __DRAM_INTERFACE_HH__
#define __DRAM_INTERFACE_HH__

#include "mem/drampower.hh"
#include "mem/mem_interface.hh"
#include "params/DRAMInterface.hh"

namespace gem5
{

namespace memory
{

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
    struct RankStats : public statistics::Group
    {
        RankStats(DRAMInterface &dram, Rank &rank);

        void regStats() override;
        void resetStats() override;
        void preDumpStats() override;

        Rank &rank;

        /*
         * Command energies
         */
        statistics::Scalar actEnergy;
        statistics::Scalar preEnergy;
        statistics::Scalar readEnergy;
        statistics::Scalar writeEnergy;
        statistics::Scalar refreshEnergy;

        /*
         * Active Background Energy
         */
        statistics::Scalar actBackEnergy;

        /*
         * Precharge Background Energy
         */
        statistics::Scalar preBackEnergy;

        /*
         * Active Power-Down Energy
         */
        statistics::Scalar actPowerDownEnergy;

        /*
         * Precharge Power-Down Energy
         */
        statistics::Scalar prePowerDownEnergy;

        /*
         * self Refresh Energy
         */
        statistics::Scalar selfRefreshEnergy;

        statistics::Scalar totalEnergy;
        statistics::Scalar averagePower;

        /**
         * Stat to track total DRAM idle time
         *
         */
        statistics::Scalar totalIdleTime;

        /**
         * Track time spent in each power state.
         */
        statistics::Vector pwrStateTime;
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

        Rank(const DRAMInterfaceParams &_p, int _rank,
             DRAMInterface& _dram);

        const std::string
        name() const
        {
            return csprintf("%s.rank%d", dram.name(), rank);
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
        bool
        inPwrIdleState() const
        {
            // If powerdown is not enabled, then the ranks never go to idle
            // states. In that case return true here to prevent checkpointing
            // from getting stuck waiting for DRAM to be idle.
            if (!dram.enableDRAMPowerdown) {
                return true;
            }

            return pwrState == PWR_IDLE;
        }

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
    const Tick tRL;
    const Tick tWL;
    const Tick tBURST_MIN;
    const Tick tBURST_MAX;
    const Tick tCCD_L_WR;
    const Tick tCCD_L;
    const Tick tRCD_RD;
    const Tick tRCD_WR;
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


    enums::PageManage pageMgmt;
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

    struct DRAMStats : public statistics::Group
    {
        DRAMStats(DRAMInterface &dram);

        void regStats() override;
        void resetStats() override;

        DRAMInterface &dram;

        /** total number of DRAM bursts serviced */
        statistics::Scalar readBursts;
        statistics::Scalar writeBursts;

        /** DRAM per bank stats */
        statistics::Vector perBankRdBursts;
        statistics::Vector perBankWrBursts;

        // Latencies summed over all requests
        statistics::Scalar totQLat;
        statistics::Scalar totBusLat;
        statistics::Scalar totMemAccLat;

        // Average latencies per request
        statistics::Formula avgQLat;
        statistics::Formula avgBusLat;
        statistics::Formula avgMemAccLat;

        // Row hit count and rate
        statistics::Scalar readRowHits;
        statistics::Scalar writeRowHits;
        statistics::Formula readRowHitRate;
        statistics::Formula writeRowHitRate;
        statistics::Histogram bytesPerActivate;
        // Number of bytes transferred to/from DRAM
        statistics::Scalar dramBytesRead;
        statistics::Scalar dramBytesWritten;

        // Average bandwidth
        statistics::Formula avgRdBW;
        statistics::Formula avgWrBW;
        statistics::Formula peakBW;
        // bus utilization
        statistics::Formula busUtil;
        statistics::Formula busUtilRead;
        statistics::Formula busUtilWrite;
        statistics::Formula pageHitRate;
    };

    DRAMStats stats;

    /**
      * Vector of dram ranks
      */
    std::vector<Rank*> ranks;

    /*
     * @return delay between write and read commands
     */
    Tick writeToReadDelay() const override { return tBURST + tWTR + tWL; }

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

    MemPacket* decodePacket(const PacketPtr pkt, Addr pkt_addr,
                           unsigned int size, bool is_read,
                           uint8_t pseudo_channel = 0) override;

    /**
     * Iterate through dram ranks to exit self-refresh in order to drain
     */
    void drainRanks() override;

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
    void suspend() override;

    /*
     * @return time to offset next command
     */
    Tick commandOffset() const override
    {
        return (tRP + std::max(tRCD_RD, tRCD_WR));
    }

    /*
     * Function to calulate unloaded, closed bank access latency
     */
    Tick accessLatency() const override { return (tRP + tRCD_RD + tRL); }

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
                  const std::vector<MemPacketQueue>& queue) override;

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
     * The arguments are NVM-specific and will be ignored by DRAM.
     * return boolean if all ranks are in refresh and therefore busy
     */
    bool isBusy(bool read_queue_empty, bool all_writes_nvm) override;

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
    void respondEvent(uint8_t rank) override;

    /**
     * Check the refresh state to determine if refresh needs
     * to be kicked back into action after a read response
     *
     * @param rank Specifies rank associated with read burst
     */
    void checkRefreshState(uint8_t rank) override;

    /**
     * The next three functions are NVM-specific and will be ignored by DRAM.
     */
    bool readsWaitingToIssue() const override { return false;}
    void chooseRead(MemPacketQueue& queue) override { }
    bool writeRespQueueFull() const override { return false;}

    DRAMInterface(const DRAMInterfaceParams &_p);
};

} // namespace memory
} // namespace gem5

#endif //__DRAM_INTERFACE_HH__
