/*
 * Copyright (c) 2020 ARM Limited
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

#ifndef __DEV_ARM_CSS_MHU_H__
#define __DEV_ARM_CSS_MHU_H__

#include "dev/arm/doorbell.hh"
#include "dev/io_device.hh"

namespace gem5
{

struct Ap2ScpDoorbellParams;
class ArmInterruptPin;
class MHU;
struct MHUParams;
class Scp;
struct Scp2ApDoorbellParams;

class MhuDoorbell : public Doorbell
{
  public:
    friend class MHU;

    MhuDoorbell(const DoorbellParams &p) : Doorbell(p), channel(0) {}

    void
    set(uint32_t val)
    {
        update(channel | val);
    }

    void
    clear(uint32_t val)
    {
        update(channel & ~val);
    }

  protected:
    void update(uint32_t new_val);

    virtual void raiseInterrupt() = 0;
    virtual void clearInterrupt() = 0;

    uint32_t channel;
};

class Scp2ApDoorbell : public MhuDoorbell
{
  public:
    Scp2ApDoorbell(const Scp2ApDoorbellParams &p);

    void raiseInterrupt() override;
    void clearInterrupt() override;

  private:
    ArmInterruptPin *interrupt;
};

class Ap2ScpDoorbell : public MhuDoorbell
{
  public:
    Ap2ScpDoorbell(const Ap2ScpDoorbellParams &p);

    void
    setScp(Scp *_scp)
    {
        scp = _scp;
    }

    void raiseInterrupt() override;
    void clearInterrupt() override;

  private:
    Scp *scp;
};

/** Message Handling Unit */
class MHU : public BasicPioDevice
{
  public:
    enum
    {
        /** From SCP to Application Processor */
        SCP_INTR_L_STAT = 0x0,
        SCP_INTR_L_SET = 0x8,
        SCP_INTR_L_CLEAR = 0x10,
        SCP_INTR_H_STAT = 0x20,
        SCP_INTR_H_SET = 0x28,
        SCP_INTR_H_CLEAR = 0x30,

        /** From Application Processor to SCP */
        CPU_INTR_L_STAT = 0x100,
        CPU_INTR_L_SET = 0x108,
        CPU_INTR_L_CLEAR = 0x110,
        CPU_INTR_H_STAT = 0x120,
        CPU_INTR_H_SET = 0x128,
        CPU_INTR_H_CLEAR = 0x130,

        SCP_INTR_S_STAT = 0x200,
        SCP_INTR_S_SET = 0x208,
        SCP_INTR_S_CLEAR = 0x210,
        CPU_INTR_S_STAT = 0x300,
        CPU_INTR_S_SET = 0x308,
        CPU_INTR_S_CLEAR = 0x310,

        MHU_SCFG = 0x400,

        PID4 = 0xfd0,
        PID0 = 0xfe0,
        PID1 = 0xfe4,
        PID2 = 0xfe8,
        PID3 = 0xfec,

        COMPID0 = 0xff0,
        COMPID1 = 0xff4,
        COMPID2 = 0xff8,
        COMPID3 = 0xffc
    };

    // Secure Violation Interrupt: used when accessing
    // a secure channel in non secure mode
    static const uint32_t SVI_INT = 0x80000000;

    MHU(const MHUParams &p);

    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    uint32_t read32(const Addr addr, bool secure_access);

    Scp2ApDoorbell *scpLow;
    Scp2ApDoorbell *scpHigh;
    Scp2ApDoorbell *scpSec;

    Ap2ScpDoorbell *apLow;
    Ap2ScpDoorbell *apHigh;
    Ap2ScpDoorbell *apSec;

    // MHU.PIDn, MHU.COMPIDn registers
    uint32_t pid[5];
    uint32_t compid[4];

    uint32_t scfg;
};

} // namespace gem5

#endif // __DEV_ARM_CSS_MHU_H__
