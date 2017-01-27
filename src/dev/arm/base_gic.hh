/*
 * Copyright (c) 2012-2013, 2017 ARM Limited
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

/** @file
 * Base class for ARM GIC implementations
 */

#ifndef __DEV_ARM_BASE_GIC_H__
#define __DEV_ARM_BASE_GIC_H__

#include "dev/io_device.hh"

class Platform;

class BaseGic :  public PioDevice
{
  public:
    typedef struct BaseGicParams Params;

    BaseGic(const Params *p);
    virtual ~BaseGic();

    const Params * params() const;

    /**
     * Post an interrupt from a device that is connected to the GIC.
     *
     * Depending on the configuration, the GIC will pass this interrupt
     * on through to a CPU.
     *
     * @param num number of interrupt to send
     */
    virtual void sendInt(uint32_t num) = 0;

    /**
     * Interface call for private peripheral interrupts.
     *
     * @param num number of interrupt to send
     * @param cpu CPU to forward interrupt to
     */
    virtual void sendPPInt(uint32_t num, uint32_t cpu) = 0;
    virtual void clearPPInt(uint32_t num, uint32_t cpu) = 0;

    /**
     * Clear an interrupt from a device that is connected to the GIC.
     *
     * Depending on the configuration, the GIC may de-assert it's CPU
     * line.
     *
     * @param num number of interrupt to send
     */
    virtual void clearInt(uint32_t num) = 0;

  protected:
    /** Platform this GIC belongs to. */
    Platform *platform;
};

class BaseGicRegisters
{
  public:
    virtual uint32_t readDistributor(ContextID ctx, Addr daddr) = 0;
    virtual uint32_t readCpu(ContextID ctx, Addr daddr) = 0;

    virtual void writeDistributor(ContextID ctx, Addr daddr,
                                  uint32_t data) = 0;
    virtual void writeCpu(ContextID ctx, Addr daddr, uint32_t data) = 0;
};

#endif
