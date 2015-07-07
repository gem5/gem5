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

#ifndef _LIBNOMALIMODEL_GPUBLOCK_HH
#define _LIBNOMALIMODEL_GPUBLOCK_HH

#include "types.hh"

namespace NoMali {

class GPU;

/**
 * Base class for GPU function blocks providing common access
 * functions.
 */
class GPUBlock
{
  public:
    GPUBlock(GPU &_gpu);
    GPUBlock(GPU &_gpu, RegVector::size_type no_regs);
    GPUBlock(GPUBlock &&rhs);
    virtual ~GPUBlock() = 0;

    /**
     * Reset the function block.
     *
     * This method is called to simulated a hard reset of the GPU. It
     * resets all registers to their default state and resets any
     * block-specific state. The default implementation resets all
     * registers to 0.
     */
    virtual void reset();


    /**
     * @{
     * @name Register Interface
     */

    /**
     * Read a register within a function block.
     *
     * @param addr Function-block relative address.
     * @return Register value (32-bits)
     */
    virtual uint32_t readReg(RegAddr addr);

    /**
     * Write to a register within a function block.
     *
     * @param addr Function-block relative address.
     * @param value New value (32-bits)
     */
    virtual void writeReg(RegAddr addr, uint32_t value);


    /**
     * Read a register within a function block without side effects.
     *
     * Unlike a normal read (readReg()), this method does not include
     * any side effects and reads straight from the register file. It
     * is primarily intended for things checkpointing.
     *
     * @param addr Function-block relative address.
     * @return Register value (32-bits)
     */
    virtual uint32_t readRegRaw(RegAddr addr);

    /**
     * Write to a register within a function block without side
     * effects.
     *
     * Unlike a normal write (writeReg()), this method does not
     * include any side effects and writes straight into the register
     * file. It is primarily intended for things checkpointing.
     *
     * @param addr Function-block relative address.
     * @param value New value (32-bits)
     */
    virtual void writeRegRaw(RegAddr addr, uint32_t value);

    /** @} */

  protected:
    /** Reference to the top-level GPU component */
    GPU &gpu;

    /** GPU block register file */
    RegVector regs;


  private:
    /** Disable the default constructor */
    GPUBlock();

    /** Disable the copy constructor */
    GPUBlock(GPUBlock &_rhs);

    /** Disable the assignment operator */
    GPUBlock &operator=(GPUBlock &_rhs);
};

/**
 * Base class for interrupt enabled GPU function blocks.
 *
 * Function blocks with interrupt functionality implement four
 * different registers controlling interrupts:
 * <ul>
 *   <li>XX_IRQ_RAWSTAT -- Raw interrupt state bit mask. (RW)
 *   <li>XX_IRQ_CLEAR -- Interrupt clear register. (WO)
 *   <li>XX_IRQ_MASK -- Bitmaks of enabled interrupts. (RW)
 *   <li>XX_IRQ_STATUS -- Currently pending unmasked interrupts. (RO)
 * </ul>
 *
 * This class provides implements the handling of the registers above
 * and utility functions to raise interrupts from the function block
 * models.
 */
class GPUBlockInt
    : public GPUBlock
{
  public:
    GPUBlockInt(GPU &_gpu,
                const RegAddr &irq_raw_stat,
                const RegAddr &irq_clear,
                const RegAddr &irq_mask,
                const RegAddr &irq_stat);
    virtual ~GPUBlockInt() = 0;

    uint32_t readReg(RegAddr addr) override;
    void writeReg(RegAddr addr, uint32_t value) override;

    /**
     * Raise an interrupt from this function block.
     *
     * Calling this method flags the interrupts in ints as pending in
     * the raw interrupt status register. If this operation asserts a
     * new unmasked interrupt (i.e., the state of the interrupt status
     * register changes), the onInterrupt() callback is called to
     * signal an interrupt state change.
     *
     * @param ints Bitfield representing interrupts to raise.
     */
    void raiseInterrupt(uint32_t ints);

    /**
     * Clear an interrupt from this function block.
     *
     * Calling this method clears the interrupts in ints in the raw
     * interrupt status register. If this operation clears a an
     * existing interrupt (i.e., the state of the interrupt status
     * register changes), the onInterrupt() callback is called to
     * signal an interrupt state change.
     *
     * @param ints Bitfield representing interrupts to raise.
     */
    void clearInterrupt(uint32_t ints);

    /**
     * Current interrupt status
     *
     * @return The value of the raw interrupt status register
     * logically anded with the interrupt mask register.
     */
    uint32_t irqStatus() const;

    /**
     * Are there unmasked interrupts pending?
     *
     * @return true if the interrupt status register is non-zero,
     * false otherwise.
     */
    bool intAsserted() const { return !!irqStatus(); }

  protected:
    /**
     * Callback method for interrupt status change.
     *
     * This method is called whenever the interrupt signal going out
     * of this GPU block changes. The new state of the signal can be
     * determined from the 'set' parameter which is non-zero if the
     * inerrupt is raised and zero if it is cleared. The state of the
     * interrupt signal can also be queried using the irqStatus()
     * method.
     *
     * @see raiseInterrupt()
     * @see clearInterrupt()
     * @see irqStatus()
     *
     * @param set Non-zero to raise interrupt, zero to clear
     * interrupt.
     */
    virtual void onInterrupt(int set) = 0;

  private:
    const RegAddr addrIrqRawStat;
    const RegAddr addrIrqClear;
    const RegAddr addrIrqMask;
    const RegAddr addrIrqStat;
};

}

#endif // _LIBNOMALIMODEL_GPUBLOCK_HH
