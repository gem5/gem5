/*
 * Copyright (c) 2013-2015 ARM Limited
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
 */
#ifndef __DEV_ARM_FLASH_DEVICE_HH__
#define __DEV_ARM_FLASH_DEVICE_HH__

#include <deque>

#include "base/statistics.hh"
#include "debug/FlashDevice.hh"
#include "dev/arm/abstract_nvm.hh"
#include "enums/DataDistribution.hh"
#include "params/FlashDevice.hh"
#include "sim/serialize.hh"

/**
 * Flash Device model
 * The Flash Device model is a timing model for a NAND flash device.
 * It doesn't tranfer any data
 */
class FlashDevice : public AbstractNVM
{
  public:

    /** Initialize functions*/
    FlashDevice(const FlashDeviceParams*);
    ~FlashDevice();

    /** Checkpoint functions*/
    DrainState drain() override;
    void checkDrain();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    /** Defines the possible actions to the flash*/
    enum Actions {
        ActionRead,
        ActionWrite,
        ActionErase,
        /**
         * A copy involves taking all the used pages from a block and store
         *  it in another
         */
        ActionCopy
    };

    /** Every logical address maps to a physical block and a physical page*/
    struct PageMapEntry {
        uint32_t page;
        uint32_t block;
    };

    struct CallBackEntry {
        Tick time;
        Callback *function;
    };

    struct FlashDeviceStats {
        /** Amount of GC activations*/
        Stats::Scalar totalGCActivations;

        /** Histogram of address accesses*/
        Stats::Histogram writeAccess;
        Stats::Histogram readAccess;
        Stats::Histogram fileSystemAccess;

        /** Histogram of access latencies*/
        Stats::Histogram writeLatency;
        Stats::Histogram readLatency;
    };

    /** Device access functions Inherrited from AbstractNVM*/
    void initializeMemory(uint64_t disk_size, uint32_t sector_size) override
    {
        initializeFlash(disk_size, sector_size);
    }

    void readMemory(uint64_t address, uint32_t amount,
                    Callback *event) override
    {
        accessDevice(address, amount, event, ActionRead);
    }

    void writeMemory(uint64_t address, uint32_t amount,
                     Callback *event) override
    {
        accessDevice(address, amount, event, ActionWrite);
    }

    /**Initialization function; called when all disk specifics are known*/
    void initializeFlash(uint64_t disk_size, uint32_t sector_size);

    /**Flash action function*/
    void accessDevice(uint64_t address, uint32_t amount, Callback *event,
                      Actions action);

    /** Event rescheduler*/
    void actionComplete();

    /** FTL functionality */
    Tick remap(uint64_t logic_page_addr);

    /** Access time calculator*/
    Tick accessTimes(uint64_t address, Actions accesstype);

    /** Function to indicate that a page is known*/
    void clearUnknownPages(uint32_t index);

    /** Function to test if a page is known*/
    bool getUnknownPages(uint32_t index);

    /**Stats register function*/
    void regStats() override;

    /** Disk sizes in bytes */
    uint64_t diskSize;
    const uint32_t blockSize;
    const uint32_t pageSize;

    /** Garbage collection algorithm emulator */
    const uint32_t GCActivePercentage;

    /** Access latencies */
    const Tick readLatency;
    const Tick writeLatency;
    const Tick eraseLatency;

    /** Flash organization */
    const Enums::DataDistribution dataDistribution;
    const uint32_t numPlanes;

    /** RequestHandler stats */
    struct FlashDeviceStats stats;

    /** Disk dimensions in pages and blocks */
    uint32_t pagesPerBlock;
    uint32_t pagesPerDisk;
    uint32_t blocksPerDisk;

    uint32_t planeMask;

    /**
     * when the disk is first started we are unsure of the number of
     * used pages, this variable will help determining what we do know.
     */
    std::vector<uint32_t> unknownPages;
    /** address to logic place has a block and a page field*/
    std::vector<struct PageMapEntry> locationTable;
    /** number of valid entries per block*/
    std::vector<uint32_t> blockValidEntries;
    /** number of empty entries*/
    std::vector<uint32_t> blockEmptyEntries;

    /**This vector of queues keeps track of all the callbacks per plane*/
    std::vector<std::deque<struct CallBackEntry> > planeEventQueue;

    /** Completion event */
    EventFunctionWrapper planeEvent;
};
#endif //__DEV_ARM_FLASH_DEVICE_HH__
