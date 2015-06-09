/*
 * Copyright (c) 2010 ARM Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
 * All rights reserved.
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
 * Authors: Ali Saidi
 */


/** @file
 * This is a base class for AMBA devices that have to respond to Device and
 * Implementer ID calls.
 */

#ifndef __DEV_ARM_AMBA_DEVICE_HH__
#define __DEV_ARM_AMBA_DEVICE_HH__

#include "dev/arm/base_gic.hh"
#include "dev/dma_device.hh"
#include "dev/io_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/AmbaPioDevice.hh"
#include "params/AmbaDmaDevice.hh"
#include "params/AmbaIntDevice.hh"


class AmbaDevice
{
  protected:
    static const int AMBA_PER_ID0 = 0xFE0;
    static const int AMBA_PER_ID1 = 0xFE4;
    static const int AMBA_PER_ID2 = 0xFE8;
    static const int AMBA_PER_ID3 = 0xFEC;
    static const int AMBA_CEL_ID0 = 0xFF0;
    static const int AMBA_CEL_ID1 = 0xFF4;
    static const int AMBA_CEL_ID2 = 0xFF8;
    static const int AMBA_CEL_ID3 = 0xFFC;

    bool readId(PacketPtr pkt, uint64_t amba_id, Addr pio_addr);
};


class AmbaPioDevice : public BasicPioDevice, public AmbaDevice
{
  protected:
    uint64_t ambaId;

  public:
    typedef AmbaPioDeviceParams Params;
    AmbaPioDevice(const Params *p, Addr pio_size);
};

class AmbaIntDevice : public AmbaPioDevice
{
  protected:
    int intNum;
    BaseGic *gic;
    Tick intDelay;

  public:
    typedef AmbaIntDeviceParams Params;
    AmbaIntDevice(const Params *p, Addr pio_size);
};

class AmbaDmaDevice : public DmaDevice, public AmbaDevice
{
  protected:
    uint64_t ambaId;
    Addr     pioAddr;
    Addr     pioSize;
    Tick     pioDelay;
    int      intNum;
    BaseGic  *gic;

  public:
    typedef AmbaDmaDeviceParams Params;
    AmbaDmaDevice(const Params *p, Addr pio_size = 0);
};


#endif //__DEV_ARM_AMBA_DEVICE_HH__
