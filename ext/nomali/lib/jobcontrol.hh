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

#ifndef _LIBNOMALIMODEL_JOBCONTROL_HH
#define _LIBNOMALIMODEL_JOBCONTROL_HH

#include <vector>

#include "gpublock.hh"
#include "jobslot.hh"
#include "types.hh"

namespace NoMali {

class GPU;

/**
 * Minimal GPU job control implementation.
 *
 * This class implements the job control block of a Midgard style
 * GPU. The job control block mainly coordinates interrupt delivery
 * and register mappings for the different job slots within the
 * block. The actual job slots are implemented by the JobSlot class.
 *
 * @see JobSlot
 */
class JobControl
    : public GPUBlockInt
{
  public:
    JobControl(GPU &_gpu);
    virtual ~JobControl();

    void reset() override;

    uint32_t readReg(RegAddr idx)  override;
    void writeReg(RegAddr idx, uint32_t value) override;

    uint32_t readRegRaw(RegAddr idx)  override;
    void writeRegRaw(RegAddr idx, uint32_t value) override;

    /**
     * Signal job done.
     *
     * Calling this method raises the job done interrupt for a
     * specific job slot. This is typically called from the job slot
     * running the job chain.
     *
     * @param slot Job slot number.
     */
    void jobDone(uint8_t slot);
    /**
     * Signal job failed.
     *
     * Calling this method raises the job failed interrupt for a
     * specific job slot. This is typically called from the job slot
     * running the job chain.
     *
     * @param slot Job slot number.
     */
    void jobFailed(uint8_t slot);

  protected:
    /**
     * Update the state of the job slot state snapshot register.
     *
     * @param jobs Bit mask representing which job slots to update.
     */
    void updateJsState(uint16_t jobs);

    void onInterrupt(int set) override;

    /** Job slots belonging to this job control block */
    std::vector<JobSlot> slots;
};

}

#endif // _LIBNOMALIMODEL_JOBCONTROL_HH
