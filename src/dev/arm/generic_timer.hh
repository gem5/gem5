/*
 * Copyright (c) 2013, 2015, 2017 ARM Limited
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
 * Authors: Giacomo Gabrielli
 *          Andreas Sandberg
 */

#ifndef __DEV_ARM_GENERIC_TIMER_HH__
#define __DEV_ARM_GENERIC_TIMER_HH__

#include "arch/arm/isa_device.hh"
#include "arch/arm/system.hh"
#include "base/bitunion.hh"
#include "dev/arm/base_gic.hh"
#include "sim/core.hh"
#include "sim/sim_object.hh"

/// @file
/// This module implements the global system counter and the local per-CPU
/// architected timers as specified by the ARM Generic Timer extension (ARM
/// ARM, Issue C, Chapter 17).

class Checkpoint;
class GenericTimerParams;
class GenericTimerMemParams;

/// Global system counter.  It is shared by the architected timers.
/// @todo: implement memory-mapped controls
class SystemCounter : public Serializable
{
  protected:
    /// Counter frequency (as specified by CNTFRQ).
    uint64_t _freq;
    /// Cached copy of the counter period (inverse of the frequency).
    Tick _period;
    /// Tick when the counter was reset.
    Tick _resetTick;

    uint32_t _regCntkctl;

  public:
    SystemCounter();

    /// Returns the current value of the physical counter.
    uint64_t value() const
    {
        if (_freq == 0)
            return 0;  // Counter is still off.
        return (curTick() - _resetTick) / _period;
    }

    /// Returns the counter frequency.
    uint64_t freq() const { return _freq; }
    /// Sets the counter frequency.
    /// @param freq frequency in Hz.
    void setFreq(uint32_t freq);

    /// Returns the counter period.
    Tick period() const { return _period; }

    void setKernelControl(uint32_t val) { _regCntkctl = val; }
    uint32_t getKernelControl() { return _regCntkctl; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    // Disable copying
    SystemCounter(const SystemCounter &c);
};

/// Per-CPU architected timer.
class ArchTimer : public Serializable, public Drainable
{
  public:
    class Interrupt
    {
      public:
        Interrupt(BaseGic &gic, unsigned irq)
            : _gic(gic), _ppi(false), _irq(irq), _cpu(0) {}

        Interrupt(BaseGic &gic, unsigned irq, unsigned cpu)
            : _gic(gic), _ppi(true), _irq(irq), _cpu(cpu) {}

        void send();
        void clear();

      private:
        BaseGic &_gic;
        const bool _ppi;
        const unsigned _irq;
        const unsigned _cpu;
    };

  protected:
    /// Control register.
    BitUnion32(ArchTimerCtrl)
    Bitfield<0> enable;
    Bitfield<1> imask;
    Bitfield<2> istatus;
    EndBitUnion(ArchTimerCtrl)

    /// Name of this timer.
    const std::string _name;

    /// Pointer to parent class.
    SimObject &_parent;

    SystemCounter &_systemCounter;

    Interrupt _interrupt;

    /// Value of the control register ({CNTP/CNTHP/CNTV}_CTL).
    ArchTimerCtrl _control;
    /// Programmed limit value for the upcounter ({CNTP/CNTHP/CNTV}_CVAL).
    uint64_t _counterLimit;
    /// Offset relative to the physical timer (CNTVOFF)
    uint64_t _offset;

    /**
     * Timer settings or the offset has changed, re-evaluate
     * trigger condition and raise interrupt if necessary.
     */
    void updateCounter();

    /// Called when the upcounter reaches the programmed value.
    void counterLimitReached();
    EventFunctionWrapper _counterLimitReachedEvent;

    virtual bool scheduleEvents() { return true; }

  public:
    ArchTimer(const std::string &name,
              SimObject &parent,
              SystemCounter &sysctr,
              const Interrupt &interrupt);

    /// Returns the timer name.
    std::string name() const { return _name; }

    /// Returns the CompareValue view of the timer.
    uint64_t compareValue() const { return _counterLimit; }
    /// Sets the CompareValue view of the timer.
    void setCompareValue(uint64_t val);

    /// Returns the TimerValue view of the timer.
    uint32_t timerValue() const { return _counterLimit - value(); }
    /// Sets the TimerValue view of the timer.
    void setTimerValue(uint32_t val);

    /// Sets the control register.
    uint32_t control() const { return _control; }
    void setControl(uint32_t val);

    uint64_t offset() const { return _offset; }
    void setOffset(uint64_t val);

    /// Returns the value of the counter which this timer relies on.
    uint64_t value() const;

    // Serializable
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    // Drainable
    DrainState drain() override;
    void drainResume() override;

  private:
    // Disable copying
    ArchTimer(const ArchTimer &t);
};

class ArchTimerKvm : public ArchTimer
{
  private:
    ArmSystem &system;

  public:
    ArchTimerKvm(const std::string &name,
                 ArmSystem &system,
                 SimObject &parent,
                 SystemCounter &sysctr,
                 const Interrupt &interrupt)
      : ArchTimer(name, parent, sysctr, interrupt), system(system) {}

  protected:
    // For ArchTimer's in a GenericTimerISA with Kvm execution about
    // to begin, skip rescheduling the event.
    // Otherwise, we should reschedule the event (if necessary).
    bool scheduleEvents() override {
        return !system.validKvmEnvironment();
    }
};

class GenericTimer : public ClockedObject
{
  public:
    GenericTimer(GenericTimerParams *p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public:
    void setMiscReg(int misc_reg, unsigned cpu, ArmISA::MiscReg val);
    ArmISA::MiscReg readMiscReg(int misc_reg, unsigned cpu);

  protected:
    struct CoreTimers {
        CoreTimers(GenericTimer &parent, ArmSystem &system, unsigned cpu,
                   unsigned _irqPhys, unsigned _irqVirt)
            : irqPhys(*parent.gic, _irqPhys, cpu),
              irqVirt(*parent.gic, _irqVirt, cpu),
              // This should really be phys_timerN, but we are stuck with
              // arch_timer for backwards compatibility.
              phys(csprintf("%s.arch_timer%d", parent.name(), cpu),
                   system, parent, parent.systemCounter,
                   irqPhys),
              virt(csprintf("%s.virt_timer%d", parent.name(), cpu),
                   system, parent, parent.systemCounter,
                   irqVirt)
        {}

        ArchTimer::Interrupt irqPhys;
        ArchTimer::Interrupt irqVirt;

        ArchTimerKvm phys;
        ArchTimerKvm virt;

      private:
        // Disable copying
        CoreTimers(const CoreTimers &c);
    };

    CoreTimers &getTimers(int cpu_id);
    void createTimers(unsigned cpus);

    /// System counter.
    SystemCounter systemCounter;

    /// Per-CPU physical architected timers.
    std::vector<std::unique_ptr<CoreTimers>> timers;

  protected: // Configuration
    /// ARM system containing this timer
    ArmSystem &system;

    /// Pointer to the GIC, needed to trigger timer interrupts.
    BaseGic *const gic;

    /// Physical timer interrupt
    const unsigned irqPhys;

    /// Virtual timer interrupt
    const unsigned irqVirt;
};

class GenericTimerISA : public ArmISA::BaseISADevice
{
  public:
    GenericTimerISA(GenericTimer &_parent, unsigned _cpu)
        : parent(_parent), cpu(_cpu) {}

    void setMiscReg(int misc_reg, ArmISA::MiscReg val) override {
        parent.setMiscReg(misc_reg, cpu, val);
    }
    ArmISA::MiscReg readMiscReg(int misc_reg) override {
        return parent.readMiscReg(misc_reg, cpu);
    }

  protected:
    GenericTimer &parent;
    unsigned cpu;
};

class GenericTimerMem : public PioDevice
{
  public:
    GenericTimerMem(GenericTimerMemParams *p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: // PioDevice
    AddrRangeList getAddrRanges() const override { return addrRanges; }
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  protected:
    uint64_t ctrlRead(Addr addr, size_t size) const;
    void ctrlWrite(Addr addr, size_t size, uint64_t value);

    uint64_t timerRead(Addr addr, size_t size) const;
    void timerWrite(Addr addr, size_t size, uint64_t value);

  protected: // Registers
    static const Addr CTRL_CNTFRQ          = 0x000;
    static const Addr CTRL_CNTNSAR         = 0x004;
    static const Addr CTRL_CNTTIDR         = 0x008;
    static const Addr CTRL_CNTACR_BASE     = 0x040;
    static const Addr CTRL_CNTVOFF_LO_BASE = 0x080;
    static const Addr CTRL_CNTVOFF_HI_BASE = 0x084;

    static const Addr TIMER_CNTPCT_LO    = 0x000;
    static const Addr TIMER_CNTPCT_HI    = 0x004;
    static const Addr TIMER_CNTVCT_LO    = 0x008;
    static const Addr TIMER_CNTVCT_HI    = 0x00C;
    static const Addr TIMER_CNTFRQ       = 0x010;
    static const Addr TIMER_CNTEL0ACR    = 0x014;
    static const Addr TIMER_CNTVOFF_LO   = 0x018;
    static const Addr TIMER_CNTVOFF_HI   = 0x01C;
    static const Addr TIMER_CNTP_CVAL_LO = 0x020;
    static const Addr TIMER_CNTP_CVAL_HI = 0x024;
    static const Addr TIMER_CNTP_TVAL    = 0x028;
    static const Addr TIMER_CNTP_CTL     = 0x02C;
    static const Addr TIMER_CNTV_CVAL_LO = 0x030;
    static const Addr TIMER_CNTV_CVAL_HI = 0x034;
    static const Addr TIMER_CNTV_TVAL    = 0x038;
    static const Addr TIMER_CNTV_CTL     = 0x03C;

  protected: // Params
    const AddrRange ctrlRange;
    const AddrRange timerRange;
    const AddrRangeList addrRanges;

  protected:
    /// System counter.
    SystemCounter systemCounter;
    ArchTimer physTimer;
    ArchTimer virtTimer;
};

#endif // __DEV_ARM_GENERIC_TIMER_HH__
