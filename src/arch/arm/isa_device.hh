/*
 * Copyright (c) 2014 ARM Limited
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

#ifndef __ARCH_ARM_ISA_DEVICE_HH__
#define __ARCH_ARM_ISA_DEVICE_HH__

#include "arch/arm/registers.hh"
#include "base/compiler.hh"

namespace ArmISA
{

class ISA;

/**
 * Base class for devices that use the MiscReg interfaces.
 *
 * This class provides a well-defined interface that the ArmISA class
 * can use when forwarding MiscReg accesses to a device model (e.g., a
 * PMU or GIC).
 */
class BaseISADevice
{
  public:
    BaseISADevice();
    virtual ~BaseISADevice() {}

    virtual void setISA(ISA *isa);

    /**
     * Write to a system register belonging to this device.
     *
     * @param misc_reg Register number (see miscregs.hh)
     * @param val Value to store
     */
    virtual void setMiscReg(int misc_reg, MiscReg val) = 0;

    /**
     * Read a system register belonging to this device.
     *
     * @param misc_reg Register number (see miscregs.hh)
     * @return Register value.
     */
    virtual MiscReg readMiscReg(int misc_reg) = 0;

  protected:
    ISA *isa;
};

/**
 * Dummy device that prints a warning when it is accessed.
 *
 * This device can be used as a placeholder when a real device model
 * is not present. For example, the ISA code uses it to avoid having
 * to check for a PMU in the register access code.
 */
class DummyISADevice : public BaseISADevice
{
  public:
    DummyISADevice()
        : BaseISADevice() {}
    ~DummyISADevice() {}

    void setMiscReg(int misc_reg, MiscReg val) override;
    MiscReg readMiscReg(int misc_reg) override;
};

}

#endif // __ARCH_ARM_ISA_DEVICE_HH__
