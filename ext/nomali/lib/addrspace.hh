/*
 * Copyright (c) 2016 ARM Limited
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

#ifndef _LIBNOMALIMODEL_ADDRSPACE_HH
#define _LIBNOMALIMODEL_ADDRSPACE_HH

#include <vector>

#include "gpublock.hh"
#include "types.hh"

namespace NoMali {

class GPU;

class MMU;

/**
 * Midgard job slot implementation.
 *
 * A job slot is a part of a JobControl block that controls the state
 * of one out of 16 active jobs. Each slot can contain one running job
 * and a pending job.
 */
class AddrSpace
    : public GPUBlock
{
  public:
    AddrSpace(GPU &_gpu, MMU &_mmu, uint8_t slot_id);
    AddrSpace(AddrSpace &&rhs);
    virtual ~AddrSpace();

    void writeReg(RegAddr idx, uint32_t value) override;

  protected:
    /**
     * @{
     * @name Address Space Control
     */


    /** @} */

    /**
     * @{
     * @name Job slot commands
     */

    /**
     * Address space command dispatcher.
     *
     * This method is called whenever there is a write to the
     * ASn_COMMAND register. The method uses a lookup table to call
     * the right command handling method.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void asCommand(uint32_t cmd);

    /**
     * Command handler for No-ops.
     *
     * @param cmd Command number (see the Midgard architecture
     * specification)
     */
    void cmdNop(uint32_t cmd);

    void cmdUpdate(uint32_t cmd);
    void cmdLock(uint32_t cmd);
    void cmdUnlock(uint32_t cmd);
    void cmdFlushPT(uint32_t cmd);
    void cmdFlushMem(uint32_t cmd);

    /** @} */

    /** Address space ID */
    const uint8_t id;

    /** Parent MMU block */
    MMU &mmu;

  private:
    typedef void (AddrSpace::*cmd_t)(uint32_t);

    /**
     * Mapping between command IDs and command handling methods.
     *
     * @note The order of this vector <i>MUST</i> correspond to the
     * address space command IDs in the Midgard architecture
     * specification.
     */
    static const std::vector<cmd_t> cmds;
};

}

#endif // _LIBNOMALIMODEL_ADDRSPACE_HH
