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

#ifndef _LIBNOMALIMODEL_GPUCONTROL_HH
#define _LIBNOMALIMODEL_GPUCONTROL_HH

#include <vector>

#include "types.hh"
#include "gpublock.hh"

namespace NoMali {

class GPU;

/**
 * Limited GPU control block implementation.
 *
 * This is a minimal implementation of the Midgard GPU control
 * block. It contains the stuff necessary to do command decoding and
 * dispatch, interrupt handling, and GPU block ready handling.
 *
 * An actual GPU implementation should specialize this class to setup
 * the following registers from the reset() method:
 * <ul>
 *   <li>GPU_ID
 *   <li>Feature registers (XX_FEATURES)
 *   <li>Present registers (XX_PRESENT)
 *   <li>Thread discovery (THREAD_XX)
 *   <li>Present registers (XX_PRESENT)
 * </ul>
 */
class GPUControl
    : public GPUBlockInt
{
  public:
    GPUControl(GPU &_gpu);
    virtual ~GPUControl();

    virtual void reset() override = 0;

    void writeReg(RegAddr idx, uint32_t value) override;

  protected:
    void onInterrupt(int set) override;

    /**
     * @{
     * @name GPU control block commands
     */

    /**
     * Control command dispatcher.
     *
     * This method is called whenever there is a write to the
     * GPU_COMMAND register. The method uses a lookup table to call
     * the right command handling method.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void gpuCommand(uint32_t cmd);
    /**
     * Command handler for No-ops.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdNop(uint32_t cmd);
    /**
     * Command handler for GPU-wide hard resets
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdHardReset(uint32_t cmd);
    /**
     * Command handler for GPU-wide soft resets
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdSoftReset(uint32_t cmd);
    /**
     * Command handler for performance counter clear operations.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdPerfCntClear(uint32_t cmd);
    /**
     * Command handler for performance counter sample operations.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdPerfCntSample(uint32_t cmd);
    /**
     * Command handler for cycle counter start operations.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdCycleCountStart(uint32_t cmd);
    /**
     * Command handler for cycle counter stop operations.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdCycleCountStop(uint32_t cmd);
    /**
     * Command handler for cache cleaning operations.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdCleanCaches(uint32_t cmd);
    /**
     * Command handler for cache clean and invalidate operations.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    virtual void cmdCleanInvCaches(uint32_t cmd);

    /** @} */

  protected:
    typedef void (GPUControl::*cmd_t)(uint32_t);
    /**
     * Mapping between command IDs and command handling methods.
     *
     * @note The order of this vector <i>MUST</i> correspond to the
     * GPU control command IDs in the Midgard architecture
     * specification.
     */
    static const std::vector<cmd_t> cmds;
};

}

#endif // _LIBNOMALIMODEL_GPUCONTROL_HH
