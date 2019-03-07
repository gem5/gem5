/*
 * Copyright (c) 2007 The Regents of The University of Michigan
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

/**
 * @file
 * Base Ethernet Device declaration.
 */

#ifndef __DEV_NET_ETHERDEVICE_HH__
#define __DEV_NET_ETHERDEVICE_HH__

#include "base/statistics.hh"
#include "dev/net/etherobject.hh"
#include "dev/pci/device.hh"
#include "params/EtherDevBase.hh"
#include "params/EtherDevice.hh"
#include "sim/sim_object.hh"

class EtherInt;

class EtherDevice : public PciDevice, public EtherObject
{
  public:
    typedef EtherDeviceParams Params;
    EtherDevice(const Params *params)
        : PciDevice(params)
    {}

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

  public:
    void regStats();

  protected:
    Stats::Scalar txBytes;
    Stats::Scalar rxBytes;
    Stats::Scalar txPackets;
    Stats::Scalar rxPackets;
    Stats::Scalar txIpChecksums;
    Stats::Scalar rxIpChecksums;
    Stats::Scalar txTcpChecksums;
    Stats::Scalar rxTcpChecksums;
    Stats::Scalar txUdpChecksums;
    Stats::Scalar rxUdpChecksums;
    Stats::Scalar descDmaReads;
    Stats::Scalar descDmaWrites;
    Stats::Scalar descDmaRdBytes;
    Stats::Scalar descDmaWrBytes;
    Stats::Formula totBandwidth;
    Stats::Formula totPackets;
    Stats::Formula totBytes;
    Stats::Formula totPacketRate;
    Stats::Formula txBandwidth;
    Stats::Formula rxBandwidth;
    Stats::Formula txPacketRate;
    Stats::Formula rxPacketRate;
    Stats::Scalar postedSwi;
    Stats::Formula coalescedSwi;
    Stats::Scalar totalSwi;
    Stats::Scalar postedRxIdle;
    Stats::Formula coalescedRxIdle;
    Stats::Scalar totalRxIdle;
    Stats::Scalar postedRxOk;
    Stats::Formula coalescedRxOk;
    Stats::Scalar totalRxOk;
    Stats::Scalar postedRxDesc;
    Stats::Formula coalescedRxDesc;
    Stats::Scalar totalRxDesc;
    Stats::Scalar postedTxOk;
    Stats::Formula coalescedTxOk;
    Stats::Scalar totalTxOk;
    Stats::Scalar postedTxIdle;
    Stats::Formula coalescedTxIdle;
    Stats::Scalar totalTxIdle;
    Stats::Scalar postedTxDesc;
    Stats::Formula coalescedTxDesc;
    Stats::Scalar totalTxDesc;
    Stats::Scalar postedRxOrn;
    Stats::Formula coalescedRxOrn;
    Stats::Scalar totalRxOrn;
    Stats::Formula coalescedTotal;
    Stats::Scalar postedInterrupts;
    Stats::Scalar droppedPackets;
};

/**
 * Dummy class to keep the Python class hierarchy in sync with the C++
 * object hierarchy.
 *
 * The Python object hierarchy includes the EtherDevBase class which
 * is used by some ethernet devices as a way to share common
 * configuration information in the generated param structs. Since the
 * Python hierarchy is used to generate a Python interfaces for all C++
 * SimObjects, we need to reflect this in the C++ object hierarchy.
 */
class EtherDevBase : public EtherDevice
{
  public:
    EtherDevBase(const EtherDevBaseParams *params)
        : EtherDevice(params)
    {}

    const EtherDevBaseParams *
    params() const
    {
        return dynamic_cast<const EtherDevBaseParams *>(_params);
    }

};

#endif // __DEV_NET_ETHERDEVICE_HH__

