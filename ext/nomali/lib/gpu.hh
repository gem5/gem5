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

#ifndef _LIBNOMALIMODEL_GPU_HH
#define _LIBNOMALIMODEL_GPU_HH

#include <vector>

#include "types.hh"

namespace NoMali {

class GPUBlock;
class GPUControl;
class JobControl;
class MMU;

/**
 * Top-level GPU component (abstract).
 */
class GPU
{
  public:
    /**
     * Instantiate a GPU from a set of functional blocks.
     *
     * @param gpuControl GPU control implementation.
     * @param jobControl Job control implementation.
     * @param mmu MMU implementation.
     */
    GPU(GPUControl &gpuControl, JobControl &jobControl, MMU &mmu);
    virtual ~GPU() = 0;

    /**
     * Reset the whole GPU
     *
     * The default implementation of this method calls the reset() on
     * all the function blocks. Function blocks in turn typically sets
     * all registers to zero.
     */
    virtual void reset();


    /**
     * @{
     * @name Register Interface
     */

    /**
     * Read a register value from the GPU.
     *
     * This method decodes the address to find the function block an
     * access is intended for and forward the register access to that
     * block. The access that reaches the block does not include the
     * block base address.
     *
     * @param addr Register address to read.
     * @return Value of the register.
     */
    virtual uint32_t readReg(RegAddr addr);

    /**
     * Write a register value to the GPU.
     *
     * This method decodes the address to find the function block an
     * access is intended for and forward the register access to that
     * block. The access that reaches the block does not include the
     * block base address.
     *
     * @param addr Target address for the write operation.
     * @param value Value to write.
     */
    virtual void writeReg(RegAddr addr, uint32_t value);

    /**
     * Read a register value from the GPU without side effects.
     *
     * This method decodes the address to find the function block an
     * access is intended for and forward the register access to that
     * block. The access that reaches the block does not include the
     * block base address.
     *
     * Unlike a normal read (readReg()), this method does not include
     * any side effects and reads straight from the register file. It
     * is primarily intended for things checkpointing.
     *
     * @param addr Register address to read.
     * @return Value of the register.
     */
    virtual uint32_t readRegRaw(RegAddr addr);

    /**
     * Write a register value to the GPU without side effects.
     *
     * This method decodes the address to find the function block an
     * access is intended for and forward the register access to that
     * block. The access that reaches the block does not include the
     * block base address.
     *
     * Unlike a normal write (writeReg()), this method does not
     * include any side effects and writes straight into the register
     * file. It is primarily intended for things checkpointing.
     *
     * @param addr Target address for the write operation.
     * @param value Value to write.
     */
    virtual void writeRegRaw(RegAddr addr, uint32_t value);

    /** @} */

    /**
     * @{
     * @name Callbacks
     */

    /**
     * Job interrupt state change
     *
     * @param set Non-zero if raising interrupt, zero if clearing.
     */
    virtual void intJob(int set) {};
    /**
     * MMU interrupt state change
     *
     * @param set Non-zero if raising interrupt, zero if clearing.
     */
    virtual void intMMU(int set) {};
    /**
     * GPU interrupt state change
     *
     * @param set Non-zero if raising interrupt, zero if clearing.
     */
    virtual void intGPU(int set) {};

    /** @} */


    /**
     * Check if the GPU interrupt has been asserted.
     *
     * @see GPUControl::intAsserted()
     *
     * @return true if the GPU control block reports that an interrupt
     * has been asserted.
     */
    bool intGPUAsserted() const;
    /**
     * Check if the job interrupt has been asserted.
     *
     * @see JobControl::intAsserted()
     *
     * @return true if the job control block reports that an interrupt
     * has been asserted.
     */
    bool intJobAsserted() const;
    /**
     * Check if the MMU interrupt has been asserted.
     *
     * @see JobControl::intAsserted()
     *
     * @return true if the GPU control block reports that an interrupt
     * has been asserted.
     */
    bool intMMUAsserted() const;

  private:
    /**
     * Resolve an address into a functional block within the GPU.
     *
     * @return Valid pointer or NULL if address is out of range.
     */
    GPUBlock *getGPUBlock(RegAddr addr);

    GPUControl &gpuControl;
    JobControl &jobControl;
    MMU &mmu;

    /**
     * Vector of control blocks.
     *
     * @note The order <i>MUST</i> have the same correspond to the
     * values in the RegBlock enum.
     */
    const std::vector<GPUBlock *> blocks;
};

}

#endif // _LIBNOMALIMODEL_GPU_HH
