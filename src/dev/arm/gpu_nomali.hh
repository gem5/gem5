/*
 * Copyright (c) 2014-2015 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#ifndef __DEV_ARM_NOMALI_GPU_HH__
#define __DEV_ARM_NOMALI_GPU_HH__

#include <map>

#include "dev/io_device.hh"
#include "libnomali/nomali.h"

class NoMaliGpuParams;
class RealView;

class NoMaliGpu : public PioDevice
{
  public:
    NoMaliGpu(const NoMaliGpuParams *p);
    virtual ~NoMaliGpu();

  public: /* Checkpointing */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: /* IO device interfaces */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    AddrRangeList getAddrRanges() const override;

  private:
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

    void onInterrupt(nomali_handle_t h, nomali_int_t intno, bool set);

    /** Wrapper around nomali_reg_read(). */
    uint32_t readReg(nomali_addr_t reg);
    /** Wrapper around nomali_reg_write(). */
    void writeReg(nomali_addr_t reg, uint32_t value);

    /** Wrapper around nomali_reg_read_raw(). */
    uint32_t readRegRaw(nomali_addr_t reg) const;
    /** Wrapper around nomali_reg_write_raw(). */
    void writeRegRaw(nomali_addr_t reg, uint32_t value);

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


#endif // __DEV_ARM_NOMALI_GPU_HH__
