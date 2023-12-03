/*
 * Copyright (c) 2012 ARM Limited
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*
 */

/** @file
 * All i2c devices should derive from this class.
 */

#ifndef __DEV_I2C_DEVICE_HH__
#define __DEV_I2C_DEVICE_HH__

#include "base/types.hh"
#include "params/I2CDevice.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class I2CDevice : public SimObject
{
  protected:
    uint8_t _addr;

  public:
    I2CDevice(const I2CDeviceParams &p) : SimObject(p), _addr(p.i2c_addr) {}

    virtual ~I2CDevice() {}

    /**
     * Return the next message that the device expects to send. This
     * will likely have side effects (e.g., incrementing a register
     * pointer).
     *
     * @return 8-bit message the device has been set up to send
     */
    virtual uint8_t read() = 0;

    /**
     * Perform any actions triggered by an i2c write (save msg in a
     * register, perform an interrupt, update a register pointer or
     * command register, etc...)
     *
     * @param msg 8-bit message from master
     */
    virtual void write(uint8_t msg) = 0;

    /**
     * Perform any initialization necessary for the device when it
     * received a start signal from the bus master (devices frequently
     * expect the first write to be a register address)
     */
    virtual void i2cStart() = 0;

    uint8_t
    i2cAddr() const
    {
        return _addr;
    }
};

} // namespace gem5

#endif // __DEV_I2C_DEVICE__
