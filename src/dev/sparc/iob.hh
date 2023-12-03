/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

/** @file
 * This device implements the niagara I/O Bridge chip. The device manages
 * internal (ipi) and external (serial, pci via jbus).
 */

#ifndef __DEV_SPARC_IOB_HH__
#define __DEV_SPARC_IOB_HH__

#include "dev/io_device.hh"
#include "params/Iob.hh"

namespace gem5
{

const int MaxNiagaraProcs = 32;
// IOB Managment Addresses
const Addr IntManAddr = 0x0000;
const Addr IntManSize = 0x0020;
const Addr IntCtlAddr = 0x0400;
const Addr IntCtlSize = 0x0020;
const Addr JIntVecAddr = 0x0A00;
const Addr IntVecDisAddr = 0x0800;
const Addr IntVecDisSize = 0x0100;

// IOB Control Addresses
const Addr JIntData0Addr = 0x0400;
const Addr JIntData1Addr = 0x0500;
const Addr JIntDataA0Addr = 0x0600;
const Addr JIntDataA1Addr = 0x0700;
const Addr JIntBusyAddr = 0x0900;
const Addr JIntBusySize = 0x0100;
const Addr JIntABusyAddr = 0x0B00;

// IOB Masks
const uint64_t IntManMask = 0x01F3F;
const uint64_t IntCtlMask = 0x00006;
const uint64_t JIntVecMask = 0x0003F;
const uint64_t IntVecDis = 0x31F3F;
const uint64_t JIntBusyMask = 0x0003F;

class Iob : public PioDevice
{
  private:
    Addr iobManAddr;
    Addr iobManSize;
    Addr iobJBusAddr;
    Addr iobJBusSize;
    Tick pioDelay;

    enum DeviceId
    {
        Interal = 0,
        Error = 1,
        SSI = 2,
        Reserved = 3,
        NumDeviceIds
    };

    struct IntMan
    {
        int cpu;
        int vector;
    };

    struct IntCtl
    {
        bool mask;
        bool pend;
    };

    struct IntBusy
    {
        bool busy;
        int source;
    };

    enum Type
    {
        Interrupt,
        Reset,
        Idle,
        Resume
    };

    IntMan intMan[NumDeviceIds];
    IntCtl intCtl[NumDeviceIds];
    uint64_t jIntVec;
    uint64_t jBusData0[MaxNiagaraProcs];
    uint64_t jBusData1[MaxNiagaraProcs];
    IntBusy jIntBusy[MaxNiagaraProcs];

    void writeIob(PacketPtr pkt);
    void writeJBus(PacketPtr pkt);
    void readIob(PacketPtr pkt);
    void readJBus(PacketPtr pkt);

  public:
    PARAMS(Iob);
    Iob(const Params &p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    void generateIpi(Type type, int cpu_id, int vector);
    void receiveDeviceInterrupt(DeviceId devid);
    bool receiveJBusInterrupt(int cpu_id, int source, uint64_t d0,
                              uint64_t d1);

    AddrRangeList getAddrRanges() const override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif //__DEV_SPARC_IOB_HH__
