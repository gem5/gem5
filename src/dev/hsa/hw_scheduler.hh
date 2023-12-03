/*
 * Copyright (c) 2016-2017 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DEV_HSA_HW_SCHEDULER_HH__
#define __DEV_HSA_HW_SCHEDULER_HH__

#include <cstdint>
#include <map>

#include "base/types.hh"
#include "dev/hsa/hsa_packet_processor.hh"
#include "enums/GfxVersion.hh"
#include "sim/eventq.hh"

// We allocate one PIO page for doorbells and each
// address is 8 bytes
#define MAX_ACTIVE_QUEUES (PAGE_SIZE / 8)

namespace gem5
{

class HWScheduler
{
  public:
    HWScheduler(HSAPacketProcessor *hsa_pp, Tick wakeup_delay)
        : hsaPP(hsa_pp),
          nextALId(0),
          nextRLId(0),
          wakeupDelay(wakeup_delay),
          schedWakeupEvent(this)
    {}

    void write(Addr db_addr, uint64_t doorbell_reg);
    void registerNewQueue(uint64_t hostReadIndexPointer, uint64_t basePointer,
                          uint64_t queue_id, uint32_t size, int doorbellSize,
                          GfxVersion gfxVersion, Addr offset = 0,
                          uint64_t rd_idx = 0);
    void unregisterQueue(uint64_t queue_id, int doorbellSize);
    void wakeup();
    void schedWakeup();

    class SchedulerWakeupEvent : public Event
    {
      private:
        HWScheduler *hwSchdlr;

      public:
        SchedulerWakeupEvent(HWScheduler *hw_schdlr) : hwSchdlr(hw_schdlr) {}

        virtual void process();
        virtual const char *description() const;
    };

    bool isRLQIdle(uint32_t rl_idx);
    bool findNextActiveALQ();
    bool findNextIdleRLQ();
    bool unmapQFromRQ();
    bool contextSwitchQ();
    bool findEmptyHWQ();
    bool mapQIfSlotAvlbl(uint32_t al_idx, AQLRingBuffer *aql_buf,
                         HSAQueueDescriptor *q_desc);
    void addQCntxt(uint32_t al_idx, AQLRingBuffer *aql_buf,
                   HSAQueueDescriptor *q_desc);
    void removeQCntxt();
    void scheduleAndWakeupMappedQ();
    void updateRRVars(uint32_t al_idx, uint32_t rl_idx);

  private:
    // Active list keeps track of all queues created
    std::map<uint32_t, QCntxt> activeList;
    // TODO: Modify this to support multi-process in the future.
    //  doorbell map, maps doorbell offsets to queue ID
    std::map<Addr, uint32_t> dbMap;
    // Reverse of doorbell map, maps queue ID to doorbell offset
    std::map<uint64_t, Addr> qidMap;
    // regdListMap keeps track of the mapping of queues to
    // registered list. regdListMap is indexed with active
    // list index (which is same as queue ID)
    std::map<uint32_t, uint32_t> regdListMap;
    HSAPacketProcessor *hsaPP;

    // Scheduling information.
    // For now, this is simple round robin but
    // this will be changed to a sophisticated logic
    // in the future. So, in the future, we will
    // move these variables into a scheduler class
    uint32_t nextALId;
    uint32_t nextRLId;
    const Tick wakeupDelay;
    SchedulerWakeupEvent schedWakeupEvent;
};

} // namespace gem5

#endif // __DEV_HSA_HW_SCHEDULER_HH__
