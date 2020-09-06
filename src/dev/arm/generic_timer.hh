/*
 * Copyright (c) 2013, 2015, 2017-2018,2020 ARM Limited
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

#ifndef __DEV_ARM_GENERIC_TIMER_HH__
#define __DEV_ARM_GENERIC_TIMER_HH__

#include "arch/arm/isa_device.hh"
#include "arch/arm/system.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/generic_timer_miscregs_types.hh"
#include "sim/core.hh"
#include "sim/sim_object.hh"

/// @file
/// This module implements the global system counter and the local per-CPU
/// architected timers as specified by the ARM Generic Timer extension:
/// Arm ARM (ARM DDI 0487E.a)
///     D11.1.2 - The system counter
///     D11.2 - The AArch64 view of the Generic Timer
///     G6.2  - The AArch32 view of the Generic Timer
///     I2 - System Level Implementation of the Generic Timer

class Checkpoint;
class SystemCounterParams;
class GenericTimerParams;
class GenericTimerFrameParams;
class GenericTimerMemParams;

/// Abstract class for elements whose events depend on the counting speed
/// of the System Counter
class SystemCounterListener : public Serializable
{
  public:
    /// Called from the SystemCounter when a change in counting speed occurred
    /// Events should be rescheduled properly inside this member function
    virtual void notify(void) = 0;
};

/// Global system counter.  It is shared by the architected and memory-mapped
/// timers.
class SystemCounter : public SimObject
{
  protected:
    /// Indicates if the counter is enabled
    bool _enabled;
    /// Counter frequency (as specified by CNTFRQ).
    uint32_t _freq;
    /// Counter value (as specified in CNTCV).
    uint64_t _value;
    /// Value increment in each counter cycle
    uint64_t _increment;
    /// Frequency modes table with all possible frequencies for the counter
    std::vector<uint32_t> _freqTable;
    /// Currently selected entry in the table, its contents should match _freq
    size_t _activeFreqEntry;
    /// Cached copy of the counter period (inverse of the frequency).
    Tick _period;
    /// Counter cycle start Tick when the counter status affecting
    /// its value has been updated
    Tick _updateTick;

    /// Listeners to changes in counting speed
    std::vector<SystemCounterListener *> _listeners;

    /// Maximum architectural number of frequency table entries
    static constexpr size_t MAX_FREQ_ENTRIES = 1004;

  public:
    SystemCounter(SystemCounterParams *const p);

    /// Validates a System Counter reference
    /// @param sys_cnt System counter reference to validate
    static void validateCounterRef(SystemCounter *sys_cnt);

    /// Indicates if the counter is enabled.
    bool enabled() const { return _enabled; }
    /// Returns the counter frequency.
    uint32_t freq() const { return _freq; }
    /// Updates and returns the counter value.
    uint64_t value();
    /// Returns the value increment
    uint64_t increment() const { return _increment; }
    /// Returns a reference to the frequency modes table.
    std::vector<uint32_t>& freqTable() { return _freqTable; }
    /// Returns the currently active frequency table entry.
    size_t activeFreqEntry() const { return _activeFreqEntry; }
    /// Returns the counter period.
    Tick period() const { return _period; }

    /// Enables the counter after a CNTCR.EN == 1
    void enable();
    /// Disables the counter after a CNTCR.EN == 0
    void disable();

    /// Schedules a counter frequency update after a CNTCR.FCREQ == 1
    /// This complies with frequency transitions as per the architecture
    /// @param new_freq_entry Index in CNTFID of the new freq
    void freqUpdateSchedule(size_t new_freq_entry);

    /// Sets the value explicitly from writes to CNTCR.CNTCV
    void setValue(uint64_t new_value);

    /// Called from System Counter Listeners to register
    void registerListener(SystemCounterListener *listener);

    /// Returns the tick at which a certain counter value is reached
    Tick whenValue(uint64_t target_val);
    Tick whenValue(uint64_t cur_val, uint64_t target_val) const;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    // Disable copying
    SystemCounter(const SystemCounter &c);

    /// Frequency update event handling
    EventFunctionWrapper _freqUpdateEvent;
    size_t _nextFreqEntry;
    /// Callback for the frequency update
    void freqUpdateCallback();

    /// Updates the counter value.
    void updateValue(void);

    /// Updates the update tick, normalizes to the lower cycle start tick
    void updateTick(void);

    /// Notifies counting speed changes to listeners
    void notifyListeners(void) const;
};

/// Per-CPU architected timer.
class ArchTimer : public SystemCounterListener, public Drainable
{
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

    ArmInterruptPin * const _interrupt;

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
              ArmInterruptPin *interrupt);

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
    Tick whenValue(uint64_t target_val) {
        return _systemCounter.whenValue(value(), target_val);
    }

    void notify(void) override;

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
                 ArmInterruptPin *interrupt)
      : ArchTimer(name, parent, sysctr, interrupt), system(system) {}

  protected:
    // For ArchTimer's in a GenericTimerISA with Kvm execution about
    // to begin, skip rescheduling the event.
    // Otherwise, we should reschedule the event (if necessary).
    bool scheduleEvents() override {
        return !system.validKvmEnvironment();
    }
};

class GenericTimer : public SimObject
{
  public:
    const GenericTimerParams * params() const;

    GenericTimer(GenericTimerParams *const p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public:
    void setMiscReg(int misc_reg, unsigned cpu, RegVal val);
    RegVal readMiscReg(int misc_reg, unsigned cpu);

  protected:
    class CoreTimers : public SystemCounterListener
    {
      public:
        CoreTimers(GenericTimer &_parent, ArmSystem &system, unsigned cpu,
                   ArmInterruptPin *_irqPhysS, ArmInterruptPin *_irqPhysNS,
                   ArmInterruptPin *_irqVirt, ArmInterruptPin *_irqHyp);

        /// Generic Timer parent reference
        GenericTimer &parent;

        /// System counter frequency as visible from this core
        uint32_t cntfrq;

        /// Kernel control register
        ArmISA::CNTKCTL cntkctl;

        /// Hypervisor control register
        ArmISA::CNTHCTL cnthctl;

        /// Thread (HW) context associated to this PE implementation
        ThreadContext *threadContext;

        ArmInterruptPin const *irqPhysS;
        ArmInterruptPin const *irqPhysNS;
        ArmInterruptPin const *irqVirt;
        ArmInterruptPin const *irqHyp;

        ArchTimerKvm physS;
        ArchTimerKvm physNS;
        ArchTimerKvm virt;
        ArchTimerKvm hyp;

        // Event Stream. Events are generated based on a configurable
        // transitionBit over the counter value. transitionTo indicates
        // the transition direction (0->1 or 1->0)
        struct EventStream
        {
            EventFunctionWrapper event;
            uint8_t transitionTo;
            uint8_t transitionBit;

            uint64_t
            eventTargetValue(uint64_t val) const
            {
                uint64_t bit_val = bits(val, transitionBit);
                uint64_t ret_val = mbits(val, 63, transitionBit);
                uint64_t incr_val = 1 << transitionBit;
                if (bit_val == transitionTo)
                    incr_val *= 2;
                return ret_val + incr_val;
            }
        };

        EventStream physEvStream;
        EventStream virtEvStream;
        void physEventStreamCallback();
        void virtEventStreamCallback();
        void eventStreamCallback() const;
        void schedNextEvent(EventStream &ev_stream, ArchTimer &timer);

        void notify(void) override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

      private:
        // Disable copying
        CoreTimers(const CoreTimers &c);
    };

    CoreTimers &getTimers(int cpu_id);
    void createTimers(unsigned cpus);

    /// System counter reference.
    SystemCounter &systemCounter;

    /// Per-CPU physical architected timers.
    std::vector<std::unique_ptr<CoreTimers>> timers;

  protected: // Configuration
    /// ARM system containing this timer
    ArmSystem &system;

    void handleStream(CoreTimers::EventStream *ev_stream,
        ArchTimer *timer, RegVal old_cnt_ctl, RegVal cnt_ctl);
};

class GenericTimerISA : public ArmISA::BaseISADevice
{
  public:
    GenericTimerISA(GenericTimer &_parent, unsigned _cpu)
        : parent(_parent), cpu(_cpu) {}

    void setMiscReg(int misc_reg, RegVal val) override;
    RegVal readMiscReg(int misc_reg) override;

  protected:
    GenericTimer &parent;
    unsigned cpu;
};

class GenericTimerFrame : public PioDevice
{
  public:
    GenericTimerFrame(GenericTimerFrameParams *const p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /// Indicates if this frame implements a virtual timer
    bool hasVirtualTimer() const;

    /// Returns the virtual offset for this frame if a virtual timer is
    /// implemented
    uint64_t getVirtOffset() const;

    /// Sets the virtual offset for this frame's virtual timer after
    /// a write to CNTVOFF
    void setVirtOffset(uint64_t new_offset);

    /// Indicates if this frame implements a second EL0 view
    bool hasEl0View() const;

    /// Returns the access bits for this frame
    uint8_t getAccessBits() const;

    /// Updates the access bits after a write to CNTCTLBase.CNTACR
    void setAccessBits(uint8_t data);

    /// Indicates if non-secure accesses are allowed to this frame
    bool hasNonSecureAccess() const;

    /// Allows non-secure accesses after an enabling write to
    /// CNTCTLBase.CNTNSAR
    void setNonSecureAccess();

    /// Indicates if CNTVOFF is readable for this frame
    bool hasReadableVoff() const;

  protected:
    AddrRangeList getAddrRanges() const override;
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  private:
    /// CNTBase/CNTEL0Base (Memory-mapped timer frame)
    uint64_t timerRead(Addr addr, size_t size, bool is_sec, bool to_el0) const;
    void timerWrite(Addr addr, size_t size, uint64_t data, bool is_sec,
                    bool to_el0);
    const AddrRange timerRange;
    AddrRange timerEl0Range;

    static const Addr TIMER_CNTPCT_LO          = 0x00;
    static const Addr TIMER_CNTPCT_HI          = 0x04;
    static const Addr TIMER_CNTVCT_LO          = 0x08;
    static const Addr TIMER_CNTVCT_HI          = 0x0c;
    static const Addr TIMER_CNTFRQ             = 0x10;
    static const Addr TIMER_CNTEL0ACR          = 0x14;
    static const Addr TIMER_CNTVOFF_LO         = 0x18;
    static const Addr TIMER_CNTVOFF_HI         = 0x1c;
    static const Addr TIMER_CNTP_CVAL_LO       = 0x20;
    static const Addr TIMER_CNTP_CVAL_HI       = 0x24;
    static const Addr TIMER_CNTP_TVAL          = 0x28;
    static const Addr TIMER_CNTP_CTL           = 0x2c;
    static const Addr TIMER_CNTV_CVAL_LO       = 0x30;
    static const Addr TIMER_CNTV_CVAL_HI       = 0x34;
    static const Addr TIMER_CNTV_TVAL          = 0x38;
    static const Addr TIMER_CNTV_CTL           = 0x3c;

    /// All MMIO ranges GenericTimerFrame responds to
    AddrRangeList addrRanges;

    /// System counter reference.
    SystemCounter &systemCounter;

    /// Physical and virtual timers
    ArchTimer physTimer;
    ArchTimer virtTimer;

    /// Reports access properties of the CNTBase register frame elements
    BitUnion8(AccessBits)
        Bitfield<5> rwpt;
        Bitfield<4> rwvt;
        Bitfield<3> rvoff;
        Bitfield<2> rfrq;
        Bitfield<1> rvct;
        Bitfield<0> rpct;
    EndBitUnion(AccessBits)
    AccessBits accessBits;

    // Reports access properties of the CNTEL0Base register frame elements
    BitUnion16(AccessBitsEl0)
        Bitfield<9> pten;
        Bitfield<8> vten;
        Bitfield<1> vcten;
        Bitfield<0> pcten;
    EndBitUnion(AccessBitsEl0)
    AccessBitsEl0 accessBitsEl0;

    /// Reports whether non-secure accesses are allowed to this frame
    bool nonSecureAccess;

    ArmSystem &system;
};

class GenericTimerMem : public PioDevice
{
  public:
    GenericTimerMem(GenericTimerMemParams *const p);

    /// Validates a Generic Timer register frame address range
    /// @param base_addr Range of the register frame
    static void validateFrameRange(const AddrRange &range);

    /// Validates an MMIO access permissions
    /// @param sys System reference where the acces is being made
    /// @param is_sec If the access is to secure memory
    static bool validateAccessPerm(ArmSystem &sys, bool is_sec);

  protected:
    AddrRangeList getAddrRanges() const override;
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  private:
    /// CNTControlBase (System counter control frame)
    uint64_t counterCtrlRead(Addr addr, size_t size, bool is_sec) const;
    void counterCtrlWrite(Addr addr, size_t size, uint64_t data, bool is_sec);
    const AddrRange counterCtrlRange;

    BitUnion32(CNTCR)
        Bitfield<17,8> fcreq;
        Bitfield<2> scen;
        Bitfield<1> hdbg;
        Bitfield<0> en;
    EndBitUnion(CNTCR)

    BitUnion32(CNTSR)
        Bitfield<31,8> fcack;
    EndBitUnion(CNTSR)

    static const Addr COUNTER_CTRL_CNTCR       = 0x00;
    static const Addr COUNTER_CTRL_CNTSR       = 0x04;
    static const Addr COUNTER_CTRL_CNTCV_LO    = 0x08;
    static const Addr COUNTER_CTRL_CNTCV_HI    = 0x0c;
    static const Addr COUNTER_CTRL_CNTSCR      = 0x10;
    static const Addr COUNTER_CTRL_CNTID       = 0x1c;
    static const Addr COUNTER_CTRL_CNTFID      = 0x20;

    /// CNTReadBase (System counter status frame)
    uint64_t counterStatusRead(Addr addr, size_t size) const;
    void counterStatusWrite(Addr addr, size_t size, uint64_t data);
    const AddrRange counterStatusRange;

    static const Addr COUNTER_STATUS_CNTCV_LO  = 0x00;
    static const Addr COUNTER_STATUS_CNTCV_HI  = 0x04;

    /// CNTCTLBase (Memory-mapped timer global control frame)
    uint64_t timerCtrlRead(Addr addr, size_t size, bool is_sec) const;
    void timerCtrlWrite(Addr addr, size_t size, uint64_t data, bool is_sec);
    const AddrRange timerCtrlRange;

    /// ID register for reporting features of implemented timer frames
    uint32_t cnttidr;

    static const Addr TIMER_CTRL_CNTFRQ        = 0x00;
    static const Addr TIMER_CTRL_CNTNSAR       = 0x04;
    static const Addr TIMER_CTRL_CNTTIDR       = 0x08;
    static const Addr TIMER_CTRL_CNTACR        = 0x40;
    static const Addr TIMER_CTRL_CNTVOFF_LO    = 0x80;
    static const Addr TIMER_CTRL_CNTVOFF_HI    = 0x84;

    /// All MMIO ranges GenericTimerMem responds to
    const AddrRangeList addrRanges;

    /// System counter reference.
    SystemCounter &systemCounter;

    /// Maximum architectural number of memory-mapped timer frames
    static constexpr size_t MAX_TIMER_FRAMES = 8;

    /// Timer frame references
    std::vector<GenericTimerFrame *> frames;

    ArmSystem &system;
};

#endif // __DEV_ARM_GENERIC_TIMER_HH__
