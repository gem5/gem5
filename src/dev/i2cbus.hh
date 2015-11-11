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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Peter Enns
 */


/** @file
 * Implementiation of an i2c bus
 */

#ifndef __DEV_I2CBUS__
#define __DEV_I2CBUS__

#include <map>

#include "dev/i2cdev.hh"
#include "dev/io_device.hh"
#include "params/I2CBus.hh"

class I2CBus : public BasicPioDevice
{
  protected:

    enum I2CState {
        IDLE,
        RECEIVING_ADDR,
        RECEIVING_DATA,
        SENDING_DATA,
    };

    /**
     * Read [and Set] serial control bits:
     * Bit [0] is SCL
     * Bit [1] is SDA
     *
     * http://infocenter.arm.com/help/topic/com.arm.doc.dui0440b/Bbajdjeg.html
     */
    static const int SB_CONTROLS = 0x0;
    /** Clear control bits. Analogous to SB_CONTROLS */
    static const int SB_CONTROLC = 0x4;

    /** I2C clock wire (0, 1). */
    uint8_t scl;
    /** I2C data wire (0, 1) */
    uint8_t sda;

    /**
     * State used by I2CBus::write to determine what stage of an i2c
     * transmission it is currently in.
     */
    enum I2CState state;

    /**
     * Order of the bit of the current message that is being sent or
     * received (0 - 7).
     */
    int currBit;

    /**
     * Key used to access a device in the slave devices map. This
     * is the same address that is specified in kernel board
     * initialization code (e.g., arch/arm/mach-realview/core.c).
     */
    uint8_t i2cAddr;

    /** 8-bit buffer used to send and receive messages bit by bit. */
    uint8_t message;

    /**
     * All the slave i2c devices that are connected to this
     * bus. Each device has an address that points to the actual
     * device.
     */
    std::map<uint8_t, I2CDevice*> devices;

    /**
     * Update data (sda) and clock (scl) to match any transitions
     * specified by pkt.
     *
     * @param pkt memory request packet
     */
    void updateSignals(PacketPtr pkt);

    /**
     * Clock set check
     *
     * @param pkt memory request packet
     * @return true if pkt indicates that scl transition from 0 to 1
     */
    bool isClockSet(PacketPtr pkt) const;

    /**
     * i2c start signal check
     *
     * @param pkt memory request packet
     * @return true if pkt indicates a new transmission
     */
    bool isStart(PacketPtr pkt) const;

    /**
     * i2c end signal check
     *
     * @param pkt memory request packet
     * @return true if pkt indicates stopping the current transmission
     */
    bool isEnd(PacketPtr pkt) const;

  public:

    I2CBus(const I2CBusParams* p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif //__DEV_I2CBUS
