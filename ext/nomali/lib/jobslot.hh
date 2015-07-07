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

#ifndef _LIBNOMALIMODEL_JOBSLOT_HH
#define _LIBNOMALIMODEL_JOBSLOT_HH

#include <vector>

#include "gpublock.hh"
#include "types.hh"

namespace NoMali {

class GPU;

class JobControl;

/**
 * Midgard job slot implementation.
 *
 * A job slot is a part of a JobControl block that controls the state
 * of one out of 16 active jobs. Each slot can contain one running job
 * and a pending job.
 */
class JobSlot
    : public GPUBlock
{
  public:
    JobSlot(GPU &_gpu, JobControl &_jc, uint8_t slot_id);
    JobSlot(JobSlot &&rhs);
    virtual ~JobSlot();

    void writeReg(RegAddr idx, uint32_t value) override;

    /** Is there an active job in this job slot? */
    bool active() const;
    /** Is there a pending next job in this job slot? */
    bool activeNext() const;

  protected:
    /**
     * @{
     * @name Job Control
     */

    /**
     * Try to start the next job in the slot.
     *
     * Start the next job if the following conditions are true:
     * <ul>
     *   <li>There is no currently running job.
     *   <li>The pending command in the JSn_COMMAND_NEXT register is
     *       JSn_COMMAND_START.
     * </ul>
     *
     * When the job is started, the registers describing the next job
     * chain are moved (resetting them to zero) into the register
     * block describing the currently running job. The job is then run
     * by a call to runJob().
     */
    void tryStart();

    /**
     * Execute the job in described by the current job registers.
     */
    void runJob();

    /**
     * Report the exit status of an exiting job.
     *
     * @note The exit status must be of the class
     * Status::CLASS_NOFAULT or Status::CLASS_JOB.
     *
     * @note The fault address isn't always a fault address, it is
     * sometimes used to represent a TSC value. See the Midgard
     * architecture specification for details.
     *
     * @param status Job exit status.
     * @param fault_address Fault address to write into descriptor.
     */
    void exitJob(Status status, uint64_t fault_address);

    /** @} */

    /**
     * @{
     * @name Job slot commands
     */

    /**
     * Control command dispatcher.
     *
     * This method is called whenever there is a write to the
     * JSn_COMMAND register. The method uses a lookup table to call
     * the right command handling method.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void jobCommand(uint32_t cmd);

    /**
     * Command handler for No-ops.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdNop(uint32_t cmd);
    /**
     * Command handler for job start commands.
     *
     * @note This should <i>NEVER</i> be called as the start command
     * should never be written to the JSn_COMMAND register. Jobs are
     * normally started by tryStart() whenever the state of the
     * currently running job changes or JSn_COMMAND_START is written
     * to the JSn_COMMAND_NEXT register.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdStart(uint32_t cmd);
    /**
     * Gently stop the currently running job chain.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdSoftStop(uint32_t cmd);
    /**
     * Force a stop of the currently running job chain.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdHardStop(uint32_t cmd);
    /**
     * Soft stop the current job chain if the JOB_CHAIN_FLAG <i>IS
     * NOT</i> set.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdSoftStop0(uint32_t cmd);
    /**
     * Hard stop the current job chain if the JOB_CHAIN_FLAG <i>IS
     * NOT</i> set.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdHardStop0(uint32_t cmd);
    /**
     * Soft stop the current job chain if the JOB_CHAIN_FLAG <i>IS</i>
     * set.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdSoftStop1(uint32_t cmd);
    /**
     * Hard stop the current job chain if the JOB_CHAIN_FLAG <i>IS</i>
     * set.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdHardStop1(uint32_t cmd);

    /** @} */

    /** Job slot ID */
    const uint8_t id;

    /** Parent JobControl block */
    JobControl &jc;

  private:
    typedef void (JobSlot::*cmd_t)(uint32_t);

    /**
     * Mapping between command IDs and command handling methods.
     *
     * @note The order of this vector <i>MUST</i> correspond to the
     * job control command IDs in the Midgard architecture
     * specification.
     */
    static const std::vector<cmd_t> cmds;
};

}

#endif // _LIBNOMALIMODEL_JOBSLOT_HH
