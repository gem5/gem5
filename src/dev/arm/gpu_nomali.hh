/*
 * Copyright (c) 2014-2016 ARM Limited
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

#ifndef __DEV_ARM_NOMALI_GPU_HH__
#define __DEV_ARM_NOMALI_GPU_HH__

#include <map>

#include "dev/io_device.hh"
#include "libnomali/nomali.h"

class NoMaliGpuParams;
class CustomNoMaliGpuParams;
class RealView;

class NoMaliGpu : public PioDevice
{
  public:
    NoMaliGpu(const NoMaliGpuParams *p);
    virtual ~NoMaliGpu();

    void init() override;

  public: /* Checkpointing */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: /* IO device interfaces */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    AddrRangeList getAddrRanges() const override;

  protected: /* API wrappers/helpers */
    /**
     * @{
     * @name API wrappers
     */

    /** Wrapper around nomali_reset(). */
    void reset();

    /** Wrapper around nomali_reg_read(). */
    uint32_t readReg(nomali_addr_t reg);
    /** Wrapper around nomali_reg_write(). */
    void writeReg(nomali_addr_t reg, uint32_t value);

    /** Wrapper around nomali_reg_read_raw(). */
    uint32_t readRegRaw(nomali_addr_t reg) const;
    /** Wrapper around nomali_reg_write_raw(). */
    void writeRegRaw(nomali_addr_t reg, uint32_t value);

    /**
     * Wrapper around nomali_int_state()
     *
     * @param intno Interrupt number
     * @return True if asserted, false otherwise.
     */
    bool intState(nomali_int_t intno);

    /** @} */

    /**
     * Format a NoMali error into an error message and panic.
     *
     * @param err Error code from the NoMali library.
     * @param msg Message to print.
     */
    static void gpuPanic(nomali_error_t err, const char *msg) M5_ATTR_NORETURN;
    /**
     * Panic if the NoMali returned an error, do nothing otherwise.
     *
     * @param err Error code from the NoMali library.
     * @param msg Message to print when panicking.
     */
    static void panicOnErr(nomali_error_t err, const char *msg) {
        if (err != NOMALI_E_OK)
            gpuPanic(err, msg);
    }

  protected: /* Callbacks */
    /**
     * @{
     * @name Callbacks
     */

    /**
     * Interrupt callback from the NoMali library
     *
     * This method is called whenever there is an interrupt state change.
     *
     * @param intno Interrupt number
     * @param set True is the interrupt is being asserted, false otherwise.
     */
    virtual void onInterrupt(nomali_int_t intno, bool set);

    /**
     * Reset callback from the NoMali library
     *
     * This method is called whenever the GPU is reset through the
     * register interface or the API (reset() or nomali_reset()).
     */
    virtual void onReset();

    /** @} */

  private: /* Callback helpers */
    /** Wrapper around nomali_set_callback() */
    void setCallback(const nomali_callback_t &callback);

    /**
     * Interrupt callback from the NoMali library.
     *
     * This method calls onInterrupt() on the NoMaliGpu owning this
     * device.
     *
     * @param h NoMali library handle.
     * @param usr Pointer to an instance of the NoMaliGpu
     * @param intno GPU interrupt type
     * @param set Was the interrupt raised (1) or lowered (0)?
     */
    static void _interrupt(nomali_handle_t h, void *usr,
                           nomali_int_t intno, int set);

    /**
     * Reset callback from the NoMali library.
     *
     * This method calls onReset() on the NoMaliGpu owning this
     * device.
     *
     * @param h NoMali library handle.
     * @param usr Pointer to an instance of the NoMaliGpu
     */
    static void _reset(nomali_handle_t h, void *usr);

  protected:
    /** Device base address */
    const Addr pioAddr;

    /** Platform, used to discover GIC */
    RealView *const platform;

    /** Map between NoMali interrupt types and actual GIC
     * interrupts */
    const std::map<nomali_int_t, uint32_t> interruptMap;

    /** Cached information struct from the NoMali library */
    nomali_info_t nomaliInfo;

    /** Handle of a NoMali library instance */
    nomali_handle_t nomali;
};


class CustomNoMaliGpu : public NoMaliGpu
{
  public:
    CustomNoMaliGpu(const CustomNoMaliGpuParams *p);
    virtual ~CustomNoMaliGpu();

  protected:
    void onReset() override;

  private:
    /** Map between GPU registers and their custom reset values */
    std::map<nomali_addr_t, uint32_t> idRegs;
};

#endif // __DEV_ARM_NOMALI_GPU_HH__
