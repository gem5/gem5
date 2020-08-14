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

/** @file
 * This simplistic flash model is designed to model managed SLC NAND flash.
 * This device will need an interface module (such as NVMe or UFS); Note that
 * this model only calculates the delay and does not perform the actual
 * transaction.
 *
 * To access the memory, use either readMemory or writeMemory. This will
 * schedule an event at the tick where the action will finish. If a callback
 * has been given as argument then that function will be called on completion
 * of that event. Note that this does not guarantee that there are no other
 * actions pending in the flash device.
 *
 * IMPORTANT: number of planes should be a power of 2.
 */

#include "dev/arm/flash_device.hh"

#include "base/trace.hh"
#include "debug/Drain.hh"

/**
 * Create this device
 */

FlashDevice*
FlashDeviceParams::create()
{
    return new FlashDevice(this);
}


/**
 * Flash Device constructor and destructor
 */

FlashDevice::FlashDevice(const FlashDeviceParams* p):
    AbstractNVM(p),
    diskSize(0),
    blockSize(p->blk_size),
    pageSize(p->page_size),
    GCActivePercentage(p->GC_active),
    readLatency(p->read_lat),
    writeLatency(p->write_lat),
    eraseLatency(p->erase_lat),
    dataDistribution(p->data_distribution),
    numPlanes(p->num_planes),
    pagesPerBlock(0),
    pagesPerDisk(0),
    blocksPerDisk(0),
    planeMask(numPlanes - 1),
    planeEventQueue(numPlanes),
    planeEvent([this]{ actionComplete(); }, name())
{

    /*
     * Let 'a' be a power of two of n bits, written such that a-n is the msb
     * and a-0 is the lsb. Since it is a power of two, only one bit (a-x,
     * with 0 <= x <= n) is set. If we subtract one from this number the bits
     * a-(x-1) to a-0 are set and all the other bits are cleared. Hence a
     * bitwise AND with those two numbers results in an integer with all bits
     * cleared.
     */
    if (numPlanes & planeMask)
        fatal("Number of planes is not a power of 2 in flash device.\n");
}

/**
 * Initiates all the flash functions: initializes the lookup tables, age of
 * the device, etc. This can only be done once the disk image is known.
 * Thats why it can't be done in the constructor.
 */
void
FlashDevice::initializeFlash(uint64_t disk_size, uint32_t sector_size)
{
    diskSize = disk_size * sector_size;
    pagesPerBlock = blockSize / pageSize;
    pagesPerDisk = diskSize / pageSize;
    blocksPerDisk = diskSize / blockSize;

    /** Sanity information: check flash configuration */
    DPRINTF(FlashDevice, "diskSize: %d Bytes; %d pages per block, %d pages "
            "per disk\n", diskSize, pagesPerBlock, pagesPerDisk);

    locationTable.resize(pagesPerDisk);

    /**Garbage collection related*/
    blockValidEntries.resize(blocksPerDisk, 0);
    blockEmptyEntries.resize(blocksPerDisk, pagesPerBlock);

    /**
     * This is a bitmap. Every bit is a page
     * unknownPages is a vector of 32 bit integers. If every page was an
     * integer, the total size would be pagesPerDisk; since we can map one
     * page per bit we need ceil(pagesPerDisk/32) entries. 32 = 1 << 5 hence
     * it will do to just shift pagesPerDisk five positions and add one. This
     * will allocate one integer to many for this data structure in the worst
     * case.
     */
    unknownPages.resize((pagesPerDisk >> 5) + 1, 0xFFFFFFFF);

    for (uint32_t count = 0; count < pagesPerDisk; count++) {
        //setup lookup table + physical aspects

        if (dataDistribution == Enums::stripe) {
            locationTable[count].page = count / blocksPerDisk;
            locationTable[count].block = count % blocksPerDisk;

        } else {
            locationTable[count].page = count % pagesPerBlock;
            locationTable[count].block = count / pagesPerBlock;
        }
    }
}

FlashDevice::~FlashDevice()
{
    DPRINTF(FlashDevice, "Remove FlashDevice\n");
}

/**
 * Handles the accesses to the device.
 * The function determines when certain actions are scheduled and schedules
 * an event that uses the callback function on completion of the action.
 */
void
FlashDevice::accessDevice(uint64_t address, uint32_t amount,
                          const std::function<void()> &event, Actions action)
{
    DPRINTF(FlashDevice, "Flash calculation for %d bytes in %d pages\n"
            , amount, pageSize);

    std::vector<Tick> time(numPlanes, 0);
    uint64_t logic_page_addr = address / pageSize;
    uint32_t plane_address = 0;

    /**
     * The access will be broken up in a number of page accesses. The number
     * of page accesses depends on the amount that needs to be transfered.
     * The assumption here is that the interface is completely ignorant of
     * the page size and that this model has to figure out all of the
     * transaction characteristics.
     */
    for (uint32_t count = 0; amount > (count * pageSize); count++) {
        uint32_t index = (locationTable[logic_page_addr].block *
                          pagesPerBlock) + (logic_page_addr % pagesPerBlock);

        DPRINTF(FlashDevice, "Index 0x%8x, Block 0x%8x, pages/block %d,"
                " logic address 0x%8x\n", index,
                locationTable[logic_page_addr].block, pagesPerBlock,
                logic_page_addr);
        DPRINTF(FlashDevice, "Page %d; %d bytes up to this point\n", count,
                (count * pageSize));

        plane_address = locationTable[logic_page_addr].block & planeMask;

        if (action == ActionRead) {
            //lookup
            //call accessTimes
            time[plane_address] += accessTimes(locationTable[logic_page_addr]
                                               .block, ActionRead);

            /*stats*/
            stats.readAccess.sample(logic_page_addr);
            stats.readLatency.sample(time[plane_address]);
        } else { //write
            //lookup
            //call accessTimes if appropriate, page may be unknown, so lets
            //give it the benefit of the doubt

            if (getUnknownPages(index))
                time[plane_address] += accessTimes
                    (locationTable[logic_page_addr].block, ActionWrite);

            else //A remap is needed
                time[plane_address] += remap(logic_page_addr);

            /*stats*/
            stats.writeAccess.sample(logic_page_addr);
            stats.writeLatency.sample(time[plane_address]);
        }

        /**
         * Check if the page is known and used. unknownPages is a bitmap of
         * all the pages. It tracks wether we can be sure that the
         * information of this page is taken into acount in the model (is it
         * considered in blockValidEntries and blockEmptyEntries?). If it has
         * been used in the past, then it is known.
         */
        if (getUnknownPages(index)) {
            clearUnknownPages(index);
            --blockEmptyEntries[locationTable[logic_page_addr].block];
            ++blockValidEntries[locationTable[logic_page_addr].block];
        }

        stats.fileSystemAccess.sample(address);
        ++logic_page_addr;
    }

    /**
     * previous part of the function found the times spend in different
     * planes, now lets find the maximum to know when to callback the disk
     */
    for (uint32_t count = 0; count < numPlanes; count++){
        plane_address = (time[plane_address] > time[count]) ? plane_address
            : count;

        DPRINTF(FlashDevice, "Plane %d is busy for %d ticks\n", count,
                time[count]);

        if (time[count] != 0) {

            struct CallBackEntry cbe;
            /**
             * If there are no events for this plane, then add the current
             * time to the occupation time; otherwise, plan it after the
             * last event. If by chance that event is handled in this tick,
             * then we would still end up with the same result.
             */
            if (planeEventQueue[count].empty())
                cbe.time = time[count] + curTick();
            else
                cbe.time = time[count] +
                           planeEventQueue[count].back().time;
            planeEventQueue[count].push_back(cbe);

            DPRINTF(FlashDevice, "scheduled at: %ld\n", cbe.time);

            if (!planeEvent.scheduled())
                schedule(planeEvent, planeEventQueue[count].back().time);
            else if (planeEventQueue[count].back().time < planeEvent.when())
                reschedule(planeEvent,
                    planeEventQueue[plane_address].back().time, true);
        }
    }

    //worst case two plane finish at the same time, each triggers an event
    //and this callback will be called once. Maybe before the other plane
    //could execute its event, but in the same tick.
    planeEventQueue[plane_address].back().function = event;
    DPRINTF(FlashDevice, "Callback queued for plane %d; %d in queue\n",
            plane_address, planeEventQueue[plane_address].size());
    DPRINTF(FlashDevice, "first event @ %d\n", planeEvent.when());
}

/**
 * When a plane completes its action, this event is triggered. When a
 * callback function was associated with that event, it will be called.
 */

void
FlashDevice::actionComplete()
{
    DPRINTF(FlashDevice, "Plane action completed\n");
    uint8_t plane_address = 0;

    uint8_t next_event = 0;

    /**Search for a callback that is supposed to happen in this Tick*/
    for (plane_address = 0; plane_address < numPlanes; plane_address++) {
        if (!planeEventQueue[plane_address].empty()) {
            /**
             * Invariant: All queued events are scheduled in the present
             *  or future.
             */
            assert(planeEventQueue[plane_address].front().time >= curTick());

            if (planeEventQueue[plane_address].front().time == curTick()) {
                /**
                 * To ensure that the follow-up action is executed correctly,
                 * the callback entry first need to be cleared before it can
                 * be called.
                 */
                auto temp = planeEventQueue[plane_address].front().function;
                planeEventQueue[plane_address].pop_front();

                /**Found a callback, lets make it happen*/
                if (temp) {
                    DPRINTF(FlashDevice, "Callback, %d\n", plane_address);
                    temp();
                }
            }
        }
    }

    /** Find when to schedule the planeEvent next */
    for (plane_address = 0; plane_address < numPlanes; plane_address++) {
        if (!planeEventQueue[plane_address].empty())
            if (planeEventQueue[next_event].empty() ||
                    (planeEventQueue[plane_address].front().time <
                     planeEventQueue[next_event].front().time))
                next_event = plane_address;
    }

    /**Schedule the next plane that will be ready (if any)*/
    if (!planeEventQueue[next_event].empty()) {
        DPRINTF(FlashDevice, "Schedule plane: %d\n", plane_address);
        reschedule(planeEvent, planeEventQueue[next_event].front().time, true);
    }

    checkDrain();

    DPRINTF(FlashDevice, "returing from flash event\n");
    DPRINTF(FlashDevice, "first event @ %d\n", planeEvent.when());
}

/**
 * Handles the remapping of the pages. It is a (I hope) sensible statistic
 * approach. asumption: garbage collection happens when a clean is needed
 * (may become stochastic function).
 */
Tick
FlashDevice::remap(uint64_t logic_page_addr)
{
    /**
     * Are there any empty left in this Block, or do we need to do an erase
     */
    if (blockEmptyEntries[locationTable[logic_page_addr].block] > 0) {
        //just a remap
        //update tables
        --blockEmptyEntries[locationTable[logic_page_addr].block];
        //access to this table won't be sequential anymore
        locationTable[logic_page_addr].page = pagesPerBlock + 2;
        //access new block
        Tick time = accessTimes(locationTable[logic_page_addr].block,
                                ActionWrite);

        DPRINTF(FlashDevice, "Remap returns %d ticks\n", time);
        return time;

    } else {
        //calculate how much time GC would have taken
        uint32_t block = locationTable[logic_page_addr].block;
        Tick time = ((GCActivePercentage *
                       (accessTimes(block, ActionCopy) +
                        accessTimes(block, ActionErase)))
                     / 100);

        //use block as the logical start address of the block
        block = locationTable[logic_page_addr].block * pagesPerBlock;

        //assumption: clean will improve locality
        for (uint32_t count = 0; count < pagesPerBlock; count++) {
            assert(block + count < pagesPerDisk);
            locationTable[block + count].page = (block + count) %
                pagesPerBlock;
        }

        blockEmptyEntries[locationTable[logic_page_addr].block] =
            pagesPerBlock;
        /*stats*/
        ++stats.totalGCActivations;

        DPRINTF(FlashDevice, "Remap with erase action returns %d ticks\n",
                time);

        return time;
    }

}

/**
 * Calculates the accesstime per operation needed
 */
Tick
FlashDevice::accessTimes(uint64_t block, Actions action)
{
    Tick time = 0;

    switch(action) {
      case ActionRead: {
          /**Just read the page*/
          time = readLatency;
      } break;

      case ActionWrite: {
          /**Write the page, and read the result*/
          time = writeLatency + readLatency;
      } break;

      case ActionErase: {
          /**Erase and check wether it was successfull*/
          time = eraseLatency + readLatency;
      } break;

      case ActionCopy: {
          /**Copy every valid page*/
          uint32_t validpages = blockValidEntries[block];
          time = validpages * (readLatency + writeLatency);
      } break;

      default: break;
    }

    //Used to determine sequential action.
    DPRINTF(FlashDevice, "Access returns %d ticks\n", time);
    return time;
}

/**
 * clearUnknownPages. defines that a page is known and used
 * unknownPages is a bitmap of all the pages. It tracks wether we can be sure
 * that the information of this page is taken into acount in the model (is it
 * considered in blockValidEntries and blockEmptyEntries?). If it has been
 * used in the past, then it is known. But it needs to be tracked to make
 * decisions about write accesses, and indirectly about copy actions. one
 * unknownPage entry is a 32 bit integer. So if we have a page index, then
 * that means that we need entry floor(index/32) (index >> 5) and we need to
 * select the bit which number is equal to the remainder of index/32
 * (index%32). The bit is cleared to make sure that we see it as considered
 * in the future.
 */

inline
void
FlashDevice::clearUnknownPages(uint32_t index)
{
    unknownPages[index >> 5] &= ~(0x01 << (index % 32));
}

/**
 * getUnknownPages. Verify wether a page is known
 */

inline
bool
FlashDevice::getUnknownPages(uint32_t index)
{
    return unknownPages[index >> 5] & (0x01 << (index % 32));
}

void
FlashDevice::regStats()
{
    AbstractNVM::regStats();

    using namespace Stats;

    std::string fd_name = name() + ".FlashDevice";

    // Register the stats
    /** Amount of GC activations*/
    stats.totalGCActivations
        .name(fd_name + ".totalGCActivations")
        .desc("Number of Garbage collector activations")
        .flags(none);

    /** Histogram of address accesses*/
    stats.writeAccess
        .init(2)
        .name(fd_name + ".writeAccessHist")
        .desc("Histogram of write addresses")
        .flags(pdf);
    stats.readAccess
        .init(2)
        .name(fd_name + ".readAccessHist")
        .desc("Histogram of read addresses")
        .flags(pdf);
    stats.fileSystemAccess
        .init(100)
        .name(fd_name + ".fileSystemAccessHist")
        .desc("Histogram of file system accesses")
        .flags(pdf);

    /** Histogram of access latencies*/
    stats.writeLatency
        .init(100)
        .name(fd_name + ".writeLatencyHist")
        .desc("Histogram of write latency")
        .flags(pdf);
    stats.readLatency
        .init(100)
        .name(fd_name + ".readLatencyHist")
        .desc("Histogram of read latency")
        .flags(pdf);
}

/**
 * Serialize; needed to create checkpoints
 */

void
FlashDevice::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(planeMask);

    SERIALIZE_CONTAINER(unknownPages);
    SERIALIZE_CONTAINER(blockValidEntries);
    SERIALIZE_CONTAINER(blockEmptyEntries);

    int location_table_size = locationTable.size();
    SERIALIZE_SCALAR(location_table_size);
    for (uint32_t count = 0; count < location_table_size; count++) {
        paramOut(cp, csprintf("locationTable[%d].page", count),
                 locationTable[count].page);
        paramOut(cp, csprintf("locationTable[%d].block", count),
                 locationTable[count].block);
    }
};

/**
 * Unserialize; needed to restore from checkpoints
 */

void
FlashDevice::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(planeMask);

    UNSERIALIZE_CONTAINER(unknownPages);
    UNSERIALIZE_CONTAINER(blockValidEntries);
    UNSERIALIZE_CONTAINER(blockEmptyEntries);

    int location_table_size;
    UNSERIALIZE_SCALAR(location_table_size);
    locationTable.resize(location_table_size);
    for (uint32_t count = 0; count < location_table_size; count++) {
        paramIn(cp, csprintf("locationTable[%d].page", count),
                locationTable[count].page);
        paramIn(cp, csprintf("locationTable[%d].block", count),
                locationTable[count].block);
    }
};

/**
 * Drain; needed to enable checkpoints
 */

DrainState
FlashDevice::drain()
{
    if (planeEvent.scheduled()) {
        DPRINTF(Drain, "Flash device is draining...\n");
        return DrainState::Draining;
    } else {
        DPRINTF(Drain, "Flash device in drained state\n");
        return DrainState::Drained;
    }
}

/**
 * Checkdrain; needed to enable checkpoints
 */

void
FlashDevice::checkDrain()
{
    if (drainState() != DrainState::Draining)
        return;

    if (planeEvent.when() > curTick()) {
        DPRINTF(Drain, "Flash device is still draining\n");
    } else {
        DPRINTF(Drain, "Flash device is done draining\n");
        signalDrainDone();
    }
}
