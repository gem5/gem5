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
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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
 * Authors: Gabe Black
 *          Andreas Hansson
 */

#ifndef __ARCH_X86_INTERRUPTS_HH__
#define __ARCH_X86_INTERRUPTS_HH__

#include "arch/x86/regs/apic.hh"
#include "arch/x86/faults.hh"
#include "arch/x86/intmessage.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"
#include "dev/x86/intdev.hh"
#include "dev/io_device.hh"
#include "params/X86LocalApic.hh"
#include "sim/eventq.hh"

class ThreadContext;
class BaseCPU;

int divideFromConf(uint32_t conf);

namespace X86ISA {

ApicRegIndex decodeAddr(Addr paddr);

class Interrupts : public BasicPioDevice, IntDevice
{
  protected:
    // Storage for the APIC registers
    uint32_t regs[NUM_APIC_REGS];

    BitUnion32(LVTEntry)
        Bitfield<7, 0> vector;
        Bitfield<10, 8> deliveryMode;
        Bitfield<12> status;
        Bitfield<13> polarity;
        Bitfield<14> remoteIRR;
        Bitfield<15> trigger;
        Bitfield<16> masked;
        Bitfield<17> periodic;
    EndBitUnion(LVTEntry)

    /*
     * Timing related stuff.
     */
    class ApicTimerEvent : public Event
    {
      private:
        Interrupts *localApic;
      public:
        ApicTimerEvent(Interrupts *_localApic) :
            Event(), localApic(_localApic)
        {}

        void process()
        {
            assert(localApic);
            if (localApic->triggerTimerInterrupt()) {
                localApic->setReg(APIC_INITIAL_COUNT,
                        localApic->readReg(APIC_INITIAL_COUNT));
            }
        }
    };

    ApicTimerEvent apicTimerEvent;

    /*
     * A set of variables to keep track of interrupts that don't go through
     * the IRR.
     */
    bool pendingSmi;
    uint8_t smiVector;
    bool pendingNmi;
    uint8_t nmiVector;
    bool pendingExtInt;
    uint8_t extIntVector;
    bool pendingInit;
    uint8_t initVector;
    bool pendingStartup;
    uint8_t startupVector;
    bool startedUp;

    // This is a quick check whether any of the above (except ExtInt) are set.
    bool pendingUnmaskableInt;

    // A count of how many IPIs are in flight.
    int pendingIPIs;

    /*
     * IRR and ISR maintenance.
     */
    uint8_t IRRV;
    uint8_t ISRV;

    int
    findRegArrayMSB(ApicRegIndex base)
    {
        int offset = 7;
        do {
            if (regs[base + offset] != 0) {
                return offset * 32 + findMsbSet(regs[base + offset]);
            }
        } while (offset--);
        return 0;
    }

    void
    updateIRRV()
    {
        IRRV = findRegArrayMSB(APIC_INTERRUPT_REQUEST_BASE);
    }

    void
    updateISRV()
    {
        ISRV = findRegArrayMSB(APIC_IN_SERVICE_BASE);
    }

    void
    setRegArrayBit(ApicRegIndex base, uint8_t vector)
    {
        regs[base + (vector / 32)] |= (1 << (vector % 32));
    }

    void
    clearRegArrayBit(ApicRegIndex base, uint8_t vector)
    {
        regs[base + (vector / 32)] &= ~(1 << (vector % 32));
    }

    bool
    getRegArrayBit(ApicRegIndex base, uint8_t vector)
    {
        return bits(regs[base + (vector / 32)], vector % 32);
    }

    void requestInterrupt(uint8_t vector, uint8_t deliveryMode, bool level);

    BaseCPU *cpu;

    int initialApicId;

    // Port for receiving interrupts
    IntSlavePort intSlavePort;

  public:

    int getInitialApicId() { return initialApicId; }

    /*
     * Params stuff.
     */
    typedef X86LocalApicParams Params;

    void setCPU(BaseCPU * newCPU);

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    /*
     * Initialize this object by registering it with the IO APIC.
     */
    void init() override;

    /*
     * Functions to interact with the interrupt port from IntDevice.
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    Tick recvMessage(PacketPtr pkt) override;
    Tick recvResponse(PacketPtr pkt) override;

    bool
    triggerTimerInterrupt()
    {
        LVTEntry entry = regs[APIC_LVT_TIMER];
        if (!entry.masked)
            requestInterrupt(entry.vector, entry.deliveryMode, entry.trigger);
        return entry.periodic;
    }

    AddrRangeList getIntAddrRange() const override;

    BaseMasterPort &getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override
    {
        if (if_name == "int_master") {
            return intMasterPort;
        }
        return BasicPioDevice::getMasterPort(if_name, idx);
    }

    BaseSlavePort &getSlavePort(const std::string &if_name,
                                PortID idx = InvalidPortID) override
    {
        if (if_name == "int_slave") {
            return intSlavePort;
        }
        return BasicPioDevice::getSlavePort(if_name, idx);
    }

    /*
     * Functions to access and manipulate the APIC's registers.
     */

    uint32_t readReg(ApicRegIndex miscReg);
    void setReg(ApicRegIndex reg, uint32_t val);
    void
    setRegNoEffect(ApicRegIndex reg, uint32_t val)
    {
        regs[reg] = val;
    }

    /*
     * Constructor.
     */

    Interrupts(Params * p);

    /*
     * Functions for retrieving interrupts for the CPU to handle.
     */

    bool checkInterrupts(ThreadContext *tc) const;
    /**
     * Check if there are pending interrupts without ignoring the
     * interrupts disabled flag.
     *
     * @return true if there are interrupts pending.
     */
    bool checkInterruptsRaw() const;
    /**
     * Check if there are pending unmaskable interrupts.
     *
     * @return true there are unmaskable interrupts pending.
     */
    bool hasPendingUnmaskable() const { return pendingUnmaskableInt; }
    Fault getInterrupt(ThreadContext *tc);
    void updateIntrInfo(ThreadContext *tc);

    /*
     * Serialization.
     */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /*
     * Old functions needed for compatability but which will be phased out
     * eventually.
     */
    void
    post(int int_num, int index)
    {
        panic("Interrupts::post unimplemented!\n");
    }

    void
    clear(int int_num, int index)
    {
        panic("Interrupts::clear unimplemented!\n");
    }

    void
    clearAll()
    {
        panic("Interrupts::clearAll unimplemented!\n");
    }
};

} // namespace X86ISA

#endif // __ARCH_X86_INTERRUPTS_HH__
