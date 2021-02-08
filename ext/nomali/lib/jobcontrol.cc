/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Andreas Sandberg
 */

#include "jobcontrol.hh"

#include "gpu.hh"
#include "regutils.hh"

namespace NoMali {

JobControl::JobControl(GPU &_gpu)
    : GPUBlockInt(_gpu,
                  RegAddr(JOB_IRQ_RAWSTAT),
                  RegAddr(JOB_IRQ_CLEAR),
                  RegAddr(JOB_IRQ_MASK),
                  RegAddr(JOB_IRQ_STATUS))
{
    slots.reserve(16);
    for (int i = 0; i < 16; ++i)
        slots.emplace_back(_gpu, *this, i);

}

JobControl::~JobControl()
{
}

void
JobControl::reset()
{
    GPUBlockInt::reset();

    for (auto &js : slots)
        js.reset();
}

uint32_t
JobControl::readReg(RegAddr addr)
{
    if (addr >= RegAddr(JOB_SLOT0)) {
        return slots[getJobSlotNo(addr)].readReg(getJobSlotAddr(addr));
    } else {
        return GPUBlockInt::readReg(addr);
    }
}

void
JobControl::writeReg(RegAddr addr, uint32_t value)
{
    switch(addr.value) {
      case JOB_IRQ_CLEAR:
        // Update JS state for all jobs that were affected by the IRQ
        // clear
        updateJsState((value & 0xFFFF) | ((value & 0xFFFF0000) >> 16));

        // FALLTHROUGH - IRQ handling in base class
      case JOB_IRQ_RAWSTAT:
      case JOB_IRQ_MASK:
      case JOB_IRQ_STATUS:
        GPUBlockInt::writeReg(addr, value);
        break;

      default:
        if (addr >= RegAddr(JOB_SLOT0))
            slots[getJobSlotNo(addr)].writeReg(getJobSlotAddr(addr), value);
        break;
    }
}

uint32_t
JobControl::readRegRaw(RegAddr addr)
{
    if (addr >= RegAddr(JOB_SLOT0)) {
        return slots[getJobSlotNo(addr)].readRegRaw(getJobSlotAddr(addr));
    } else {
        return GPUBlockInt::readRegRaw(addr);
    }
}


void
JobControl::writeRegRaw(RegAddr addr, uint32_t value)
{
    if (addr >= RegAddr(JOB_SLOT0)) {
        slots[getJobSlotNo(addr)].writeRegRaw(getJobSlotAddr(addr), value);
    } else {
        GPUBlockInt::writeRegRaw(addr, value);
    }
}

void
JobControl::jobDone(uint8_t slot)
{
    assert(slot <= 15);
    raiseInterrupt(1 << slot);
}

void
JobControl::jobFailed(uint8_t slot)
{
    assert(slot <= 15);
    raiseInterrupt(0x10000 << slot);
}

void
JobControl::updateJsState(uint16_t jobs)
{
    // The JS_STATE register contains two bits per job slot; one bit
    // representing an active job and one bit representing the queued
    // job. We need to mask out bits of the jobs affected by this update.
    const uint32_t job_mask(jobs | (jobs << 16));
    uint16_t js_state(regs[RegAddr(JOB_IRQ_JS_STATE)] & ~job_mask);

    // Find if there is an active or active next job for all jobs in
    // the job mask.
    for (int i = 0; i < 16; ++i) {
        const JobSlot &slot(slots[i]);
        if (jobs & (1 << i)) {
            js_state |= (slot.active() ? (1 << i) : 0) |
                (slot.activeNext() ? (0x10000 << i) : 0);
        }
    }
    regs[RegAddr(JOB_IRQ_JS_STATE)] = js_state;
}

void
JobControl::onInterrupt(int set)
{
    gpu.intJob(set);
}

}
