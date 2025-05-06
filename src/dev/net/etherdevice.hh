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
 */

/**
 * @file
 * Base Ethernet Device declaration.
 */

#ifndef __DEV_NET_ETHERDEVICE_HH__
#define __DEV_NET_ETHERDEVICE_HH__

#include "base/statistics.hh"
#include "dev/pci/device.hh"
#include "params/EtherDevBase.hh"
#include "params/EtherDevice.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class EtherInt;

class EtherDevice : public PciEndpoint
{
  public:
    using Params = EtherDeviceParams;
    EtherDevice(const Params &params)
        : PciEndpoint(params),
          etherDeviceStats(this)
    {}

  protected:
    struct EtherDeviceStats : public statistics::Group
    {
        EtherDeviceStats(statistics::Group *parent);

        statistics::Scalar postedInterrupts;

        statistics::Scalar txBytes;
        statistics::Scalar rxBytes;

        statistics::Scalar txPackets;
        statistics::Scalar rxPackets;

        statistics::Formula txBandwidth;
        statistics::Formula rxBandwidth;

        statistics::Scalar txIpChecksums;
        statistics::Scalar rxIpChecksums;

        statistics::Scalar txTcpChecksums;
        statistics::Scalar rxTcpChecksums;

        statistics::Scalar txUdpChecksums;
        statistics::Scalar rxUdpChecksums;

        statistics::Scalar descDmaReads;
        statistics::Scalar descDmaWrites;

        statistics::Scalar descDmaRdBytes;
        statistics::Scalar descDmaWrBytes;

        statistics::Formula totBandwidth;
        statistics::Formula totPackets;
        statistics::Formula totBytes;
        statistics::Formula totPacketRate;

        statistics::Formula txPacketRate;
        statistics::Formula rxPacketRate;

        statistics::Scalar postedSwi;
        statistics::Scalar totalSwi;
        statistics::Formula coalescedSwi;

        statistics::Scalar postedRxIdle;
        statistics::Scalar totalRxIdle;
        statistics::Formula coalescedRxIdle;

        statistics::Scalar postedRxOk;
        statistics::Scalar totalRxOk;
        statistics::Formula coalescedRxOk;

        statistics::Scalar postedRxDesc;
        statistics::Scalar totalRxDesc;
        statistics::Formula coalescedRxDesc;

        statistics::Scalar postedTxOk;
        statistics::Scalar totalTxOk;
        statistics::Formula coalescedTxOk;

        statistics::Scalar postedTxIdle;
        statistics::Scalar totalTxIdle;
        statistics::Formula coalescedTxIdle;

        statistics::Scalar postedTxDesc;
        statistics::Scalar totalTxDesc;
        statistics::Formula coalescedTxDesc;

        statistics::Scalar postedRxOrn;
        statistics::Scalar totalRxOrn;
        statistics::Formula coalescedRxOrn;

        statistics::Formula coalescedTotal;
        statistics::Scalar droppedPackets;
    } etherDeviceStats;
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
    using Params = EtherDevBaseParams;
    EtherDevBase(const Params &params)
        : EtherDevice(params)
    {}
};

} // namespace gem5

#endif // __DEV_NET_ETHERDEVICE_HH__
