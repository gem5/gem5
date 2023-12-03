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
 */

#ifndef __ARCH_X86_INTERRUPTS_HH__
#define __ARCH_X86_INTERRUPTS_HH__

#include "arch/generic/interrupts.hh"
#include "arch/x86/faults.hh"
#include "arch/x86/intmessage.hh"
#include "arch/x86/regs/apic.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"
#include "dev/intpin.hh"
#include "dev/io_device.hh"
#include "dev/x86/intdev.hh"
#include "params/X86LocalApic.hh"
#include "sim/eventq.hh"

namespace gem5
{

class ThreadContext;
class BaseCPU;

int divideFromConf(uint32_t conf);

namespace X86ISA
{

ApicRegIndex decodeAddr(Addr paddr);

class Interrupts : public BaseInterrupts
{
  protected:
    System *sys = nullptr;
    ClockDomain &clockDomain;

    // Storage for the APIC registers
    uint32_t regs[NUM_APIC_REGS] = {};

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
    EventFunctionWrapper apicTimerEvent;
    void processApicTimerEvent();

    /*
     * A set of variables to keep track of interrupts that don't go through
     * the IRR.
     */
    bool pendingSmi = false;
    uint8_t smiVector = 0;
    bool pendingNmi = false;
    uint8_t nmiVector = 0;
    bool pendingExtInt = false;
    uint8_t extIntVector = 0;
    bool pendingInit = false;
    uint8_t initVector = 0;
    bool pendingStartup = false;
    uint8_t startupVector = 0;
    bool startedUp = false;

    // This is a quick check whether any of the above (except ExtInt) are set.
    bool pendingUnmaskableInt = false;

    // A count of how many IPIs are in flight.
    int pendingIPIs = 0;

    /*
     * IRR and ISR maintenance.
     */
    uint8_t IRRV = 0;
    uint8_t ISRV = 0;

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

    Tick
    clockPeriod() const
    {
        return clockDomain.clockPeriod();
    }

    void requestInterrupt(uint8_t vector, uint8_t deliveryMode, bool level);

    int initialApicId = 0;

    // Ports for interrupt messages.
    IntResponsePort<Interrupts> intResponsePort;
    IntRequestPort<Interrupts> intRequestPort;

    // Pins for wired interrupts.
    IntSinkPin<Interrupts> lint0Pin;
    IntSinkPin<Interrupts> lint1Pin;

    // Port for memory mapped register accesses.
    PioPort<Interrupts> pioPort;

    Tick pioDelay = 0;
    Addr pioAddr = MaxAddr;

  public:
    int
    getInitialApicId()
    {
        return initialApicId;
    }

    /*
     * Params stuff.
     */
    using Params = X86LocalApicParams;

    void setThreadContext(ThreadContext *_tc) override;

    /*
     * Initialize this object by registering it with the IO APIC.
     */
    void init() override;

    /*
     * Functions to interact with the interrupt port.
     */
    Tick read(PacketPtr pkt);
    Tick write(PacketPtr pkt);
    Tick recvMessage(PacketPtr pkt);
    void completeIPI(PacketPtr pkt);

    bool
    triggerTimerInterrupt()
    {
        LVTEntry entry = regs[APIC_LVT_TIMER];
        if (!entry.masked)
            requestInterrupt(entry.vector, entry.deliveryMode, entry.trigger);
        return entry.periodic;
    }

    AddrRangeList getAddrRanges() const;
    AddrRangeList getIntAddrRange() const;

    void raiseInterruptPin(int number);
    void lowerInterruptPin(int number);

    Port &
    getPort(const std::string &if_name, PortID idx = InvalidPortID) override
    {
        if (if_name == "int_requestor") {
            return intRequestPort;
        } else if (if_name == "int_responder") {
            return intResponsePort;
        } else if (if_name == "pio") {
            return pioPort;
        } else if (if_name == "lint0") {
            return lint0Pin;
        } else if (if_name == "lint1") {
            return lint1Pin;
        } else {
            return SimObject::getPort(if_name, idx);
        }
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

    Interrupts(const Params &p);

    /*
     * Functions for retrieving interrupts for the CPU to handle.
     */

    bool checkInterrupts() const override;
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
    bool
    hasPendingUnmaskable() const
    {
        return pendingUnmaskableInt;
    }

    Fault getInterrupt() override;
    void updateIntrInfo() override;

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
    post(int int_num, int index) override
    {
        panic("Interrupts::post unimplemented!\n");
    }

    void
    clear(int int_num, int index) override
    {
        panic("Interrupts::clear unimplemented!\n");
    }

    void
    clearAll() override
    {
        panic("Interrupts::clearAll unimplemented!\n");
    }
};

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_INTERRUPTS_HH__
