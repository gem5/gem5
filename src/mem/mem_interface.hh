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

#include "base/compiler.hh"
#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "enums/PageManage.hh"
#include "mem/abstract_mem.hh"
#include "mem/mem_ctrl.hh"
#include "params/MemInterface.hh"
#include "sim/eventq.hh"

namespace gem5
{

namespace memory
{

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

        Bank()
            : openRow(NO_ROW),
              bank(0),
              bankgr(0),
              rdAllowedAt(0),
              wrAllowedAt(0),
              preAllowedAt(0),
              actAllowedAt(0),
              rowAccesses(0),
              bytesAccessed(0)
        {}
    };

    /**
     * A pointer to the parent memory controller instance
     */
    MemCtrl *ctrl;

    /**
     * Number of commands that can issue in the defined controller
     * command window, used to verify command bandwidth
     */
    unsigned int maxCommandsPerWindow;

    /**
     * Memory controller configuration initialized based on parameter
     * values.
     */
    enums::AddrMap addrMapping;

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
    GEM5_CLASS_VAR_USED const Tick tCK;
    const Tick tCS;
    const Tick tBURST;
    const Tick tRTW;
    const Tick tWTR;

    /*
     * @return delay between write and read commands
     */
    virtual Tick
    writeToReadDelay() const
    {
        return tBURST + tWTR;
    }

    /*
     * @return delay between write and read commands
     */
    Tick
    readToWriteDelay() const
    {
        return tBURST + tRTW;
    }

    /*
     * @return delay between accesses to different ranks
     */
    Tick
    rankToRankDelay() const
    {
        return tBURST + tCS;
    }

  public:
    /**
     * Buffer sizes for read and write queues in the controller
     * These are passed to the controller on instantiation
     * Defining them here allows for buffers to be resized based
     * on memory type / configuration.
     */
    const uint32_t readBufferSize;
    const uint32_t writeBufferSize;

    /**
     * NVM specific variable, but declaring it here allows
     * treating different interfaces in a more genral way
     * at the memory controller's end
     */
    uint32_t numWritesQueued;

    /**
     * Till when the controller must wait before issuing next RD/WR burst?
     */
    Tick nextBurstAt = 0;
    Tick nextReqTime = 0;

    /**
     * Reads/writes performed by the controller for this interface before
     * bus direction is switched
     */
    uint32_t readsThisTime = 0;
    uint32_t writesThisTime = 0;

    /**
     * Read/write packets in the read/write queue for this interface
     * qos/mem_ctrl.hh has similar counters, but they track all packets
     * in the controller for all memory interfaces connected to the
     * controller.
     */
    uint32_t readQueueSize = 0;
    uint32_t writeQueueSize = 0;

    MemCtrl::BusState busState = MemCtrl::READ;

    /** bus state for next request event triggered */
    MemCtrl::BusState busStateNext = MemCtrl::READ;

    /**
     * pseudo channel number used for HBM modeling
     */
    uint8_t pseudoChannel;

    /** Set a pointer to the controller and initialize
     * interface based on controller parameters
     * @param _ctrl pointer to the parent controller
     * @param command_window size of command window used to
     *                       check command bandwidth
     *  @param pseudo_channel pseudo channel number
     */
    void setCtrl(MemCtrl *_ctrl, unsigned int command_window,
                 uint8_t pseudo_channel = 0);

    /**
     * Get an address in a dense range which starts from 0. The input
     * address is the physical address of the request in an address
     * space that contains other SimObjects apart from this
     * controller.
     *
     * @param addr The intput address which should be in the addrRange
     * @return An address in the continues range [0, max)
     */
    Addr
    getCtrlAddr(Addr addr)
    {
        return range.getOffset(addr);
    }

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
    chooseNextFRFCFS(MemPacketQueue &queue, Tick min_col_at) const = 0;

    /*
     * Function to calulate unloaded latency
     */
    virtual Tick accessLatency() const = 0;

    /**
     * @return number of bytes in a burst for this interface
     */
    uint32_t
    bytesPerBurst() const
    {
        return burstSize;
    }

    /*
     * @return time to offset next command
     */
    virtual Tick commandOffset() const = 0;

    /**
     * Check if a burst operation can be issued to the interface
     *
     * @param Return true if RD/WR can issue
     */
    virtual bool burstReady(MemPacket *pkt) const = 0;

    /**
     * Determine the required delay for an access to a different rank
     *
     * @return required rank to rank delay
     */
    Tick
    rankDelay() const
    {
        return tCS;
    }

    /**
     *
     * @return minimum additional bus turnaround required for read-to-write
     */
    Tick
    minReadToWriteDataGap() const
    {
        return std::min(tRTW, tCS);
    }

    /**
     *
     * @return minimum additional bus turnaround required for write-to-read
     */
    Tick
    minWriteToReadDataGap() const
    {
        return std::min(tWTR, tCS);
    }

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
     * @param pseudo_channel pseudo channel number of the packet
     * @return A MemPacket pointer with the decoded information
     */
    virtual MemPacket *
    decodePacket(const PacketPtr pkt, Addr pkt_addr, unsigned int size,
                 bool is_read, uint8_t pseudo_channel = 0)
    {
        panic("MemInterface decodePacket should not be executed from here.\n");
        return nullptr;
    }

    /**
     *  Add rank to rank delay to bus timing to all banks in all ranks
     *  when access to an alternate interface is issued
     *
     *  param cmd_at Time of current command used as starting point for
     *               addition of rank-to-rank delay
     */
    virtual void addRankToRankDelay(Tick cmd_at) = 0;

    /**
     * This function checks if ranks are busy.
     */
    virtual bool isBusy(bool read_queue_empty, bool all_writes_nvm) = 0;

    /**
     * This function performs the burst and update stats.
     */
    virtual std::pair<Tick, Tick>
    doBurstAccess(MemPacket *mem_pkt, Tick next_burst_at,
                  const std::vector<MemPacketQueue> &queue) = 0;

    /**
     * This function is DRAM specific.
     */
    virtual void
    respondEvent(uint8_t rank)
    {
        panic("MemInterface respondEvent should not be executed from here.\n");
    };

    /**
     * This function is DRAM specific.
     */
    virtual void
    checkRefreshState(uint8_t rank)
    {
        panic("MemInterface checkRefreshState (DRAM) should "
              "not be executed from here.\n");
    };

    /**
     * This function is DRAM specific.
     */
    virtual void
    drainRanks()
    {
        panic("MemInterface drainRanks (DRAM) should "
              "not be executed from here.\n");
    }

    /**
     * This function is DRAM specific.
     */
    virtual void
    suspend()
    {
        panic("MemInterface suspend (DRAM) should "
              "not be executed from here.\n");
    }

    /**
     * This function is NVM specific.
     */
    virtual bool
    readsWaitingToIssue() const
    {
        panic("MemInterface readsWaitingToIssue (NVM) "
              "should not be executed from here.\n");
    };

    /**
     * This function is NVM specific.
     */
    virtual void
    chooseRead(MemPacketQueue &queue)
    {
        panic("MemInterface chooseRead (NVM) should "
              "not be executed from here.\n");
    };

    /**
     * This function is NVM specific.
     */
    virtual bool
    writeRespQueueFull() const
    {
        panic("MemInterface writeRespQueueFull (NVM) "
              "should not be executed from here.\n");
    }

    typedef MemInterfaceParams Params;
    MemInterface(const Params &_p);
};

} // namespace memory
} // namespace gem5

#endif //__MEM_INTERFACE_HH__
