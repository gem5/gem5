/*
 * Copyright (c) 2011-2014, 2017 ARM Limited
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
 * Authors: Dam Sunwoo
 *          Matt Horsnell
 *          Andreas Sandberg
 */
#ifndef __ARCH_ARM_PMU_HH__
#define __ARCH_ARM_PMU_HH__

#include <map>
#include <memory>
#include <vector>

#include "arch/arm/isa_device.hh"
#include "arch/arm/registers.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"

class ArmPMUParams;
class Platform;
class ThreadContext;

namespace ArmISA {


/**
 * Model of an ARM PMU version 3
 *
 * This class implements a subset of the ARM PMU v3 specification as
 * described in the ARMv8 reference manual. It supports most of the
 * features of the PMU, however the following features are known to be
 * missing:
 *
 * <ul>
 *   <li>Event filtering (e.g., from different privilege levels).
 *   <li>Access controls (the PMU currently ignores the execution level).
 *   <li>The chain counter (event no. 0x1E) is unimplemented.
 * </ul>
 *
 * The PMU itself does not implement any events, in merely provides an
 * interface for the configuration scripts to hook up probes that
 * drive events. Configuration scripts should call addEventProbe() to
 * configure custom events or high-level methods to configure
 * architected events. The Python implementation of addEventProbe()
 * automatically delays event type registration until after
 * instantiation.
 *
 * In order to support CPU switching and some combined counters (e.g.,
 * memory references synthesized from loads and stores), the PMU
 * allows multiple probes per event type. When creating a system that
 * switches between CPU models that share the same PMU, PMU events for
 * all of the CPU models can be registered with the PMU.
 *
 * @see The ARM Architecture Refererence Manual (DDI 0487A)
 *
 */
class PMU : public SimObject, public ArmISA::BaseISADevice {
  public:
    PMU(const ArmPMUParams *p);
    ~PMU();

    void addEventProbe(unsigned int id, SimObject *obj, const char *name);

  public: // SimObject and related interfaces
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void drainResume() override;


  public: // ISA Device interface
    /**
     * Set a register within the PMU.
     *
     * @param misc_reg Register number (see miscregs.hh)
     * @param val Value to store
     */
    void setMiscReg(int misc_reg, MiscReg val) override;
    /**
     * Read a register within the PMU.
     *
     * @param misc_reg Register number (see miscregs.hh)
     * @return Register value.
     */
    MiscReg readMiscReg(int misc_reg) override;

  protected: // PMU register types and constants
    BitUnion32(PMCR_t)
        // PMU Enable
        Bitfield<0> e;
        // Event counter reset
        Bitfield<1> p;
        // Cycle counter reset
        Bitfield<2> c;
        // Cycle counter divider enable
        Bitfield<3> d;
        // Export enable
        Bitfield<4> x;
        // Disable PMCCNTR when event counting is prohibited
        Bitfield<5> dp;
        // Long Cycle counter enable
        Bitfield<6> lc;
        // Number of event counters implemented
        Bitfield<15, 11> n;
        // Implementation ID
        Bitfield<23, 16> idcode;
        // Implementer code
        Bitfield<31, 24> imp;
    EndBitUnion(PMCR_t)

    BitUnion32(PMSELR_t)
        // Performance counter selector
        Bitfield<4, 0> sel;
    EndBitUnion(PMSELR_t)

    BitUnion32(PMEVTYPER_t)
        Bitfield<15, 0> evtCount;

        // Secure EL3 filtering
        Bitfield<26> m;
        // Non-secure EL2 mode filtering
        Bitfield<27> nsh;
        // Non-secure EL0 mode filtering
        Bitfield<28> nsu;
        // Non-secure EL1 mode filtering
        Bitfield<29> nsk;
        // EL0 filtering
        Bitfield<30> u;
        // EL1 filtering
        Bitfield<31> p;
    EndBitUnion(PMEVTYPER_t)

    /**
     * Counter ID within the PMU.
     *
     * This value is typically used to index into various registers
     * controlling interrupts and overflows. The value normally in the
     * [0, 31] range, where 31 refers to the cycle counter.
     */
    typedef unsigned int CounterId;

    /** Cycle Count Register Number */
    static const CounterId PMCCNTR = 31;

    /**
     * Event type ID.
     *
     * See the PMU documentation for a list of architected IDs.
     */
    typedef unsigned int EventTypeId;

    /** ID of the software increment event */
    static const EventTypeId ARCH_EVENT_SW_INCR = 0x00;

  protected: /* High-level register and interrupt handling */
    MiscReg readMiscRegInt(int misc_reg);

    /**
     * PMCR write handling
     *
     * The PMCR register needs special handling since writing to it
     * changes PMU-global state (e.g., resets all counters).
     *
     * @param val New PMCR value
     */
    void setControlReg(PMCR_t val);

    /**
     * Reset all event counters excluding the cycle counter to zero.
     */
    void resetEventCounts();

    /**
     * Deliver a PMU interrupt to the GIC
     */
    void raiseInterrupt();

    /**
     * Get the value of a performance counter.
     *
     * This method returns the value of a general purpose performance
     * counter or the fixed-function cycle counter. Non-existing
     * counters are treated as constant '0'.
     *
     * @return Value of the performance counter, 0 if the counter does
     * not exist.
     */
    uint64_t getCounterValue(CounterId id) const {
        return isValidCounter(id) ? getCounter(id).value : 0;
    }

    /**
     * Set the value of a performance counter.
     *
     * This method sets the value of a general purpose performance
     * counter or the fixed-function cycle counter. Writes to
     * non-existing counters are ignored.
     */
    void setCounterValue(CounterId id, uint64_t val);

    /**
     * Get the type and filter settings of a counter (PMEVTYPER)
     *
     * This method implements a read from a PMEVTYPER register. It
     * returns the type value and filter settings of a general purpose
     * performance counter or the cycle counter. Non-existing counters
     * are treated as constant '0'.
     *
     * @param id Counter ID within the PMU.
     * @return Performance counter type ID.
     */
    PMEVTYPER_t getCounterTypeRegister(CounterId id) const;

    /**
     * Set the type and filter settings of a performance counter
     * (PMEVTYPER)
     *
     * This method implements a write to a PMEVTYPER register. It sets
     * the type value and filter settings of a general purpose
     * performance counter or the cycle counter. Writes to
     * non-existing counters are ignored. The method automatically
     * updates the probes used by the counter if it is enabled.
     *
     * @param id Counter ID within the PMU.
     * @param type Performance counter type and filter configuration..
     */
    void setCounterTypeRegister(CounterId id, PMEVTYPER_t type);

  protected: /* Probe handling and counter state */
    class ProbeListener : public ProbeListenerArgBase<uint64_t>
    {
      public:
        ProbeListener(PMU &_pmu, CounterId _id,
                      ProbeManager *pm, const std::string &name)
            : ProbeListenerArgBase(pm, name),
              pmu(_pmu), id(_id) {}

        void notify(const uint64_t &val) override
        {
            pmu.handleEvent(id, val);
        }

      protected:
        PMU &pmu;
        const CounterId id;
    };
    typedef std::unique_ptr<ProbeListener> ProbeListenerUPtr;

    /**
     * Event type configuration
     *
     * The main purpose of this class is to describe how a PMU event
     * type is sampled. It is implemented as a probe factory that
     * returns a probe attached to the object the event is mointoring.
     */
    struct EventType {
        /**
         * @param _obj Target SimObject
         * @param _name Probe name
         */
        EventType(SimObject *_obj, const std::string &_name)
            : obj(_obj), name(_name) {}

        /**
         * Create and attach a probe used to drive this event.
         *
         * @param pmu PMU owning the probe.
         * @param CounterID counter ID within the PMU.
         * @return Pointer to a probe listener.
         */
        std::unique_ptr<ProbeListener> create(PMU &pmu, CounterId cid) const
        {
            std::unique_ptr<ProbeListener> ptr;
            ptr.reset(new ProbeListener(pmu, cid,
                                        obj->getProbeManager(), name));
            return ptr;
        }

        /** SimObject being measured by this probe */
        SimObject *const obj;
        /** Probe name within obj */
        const std::string name;

      private:
        // Disable the default constructor
        EventType();
    };

    /** State of a counter within the PMU. */
    struct CounterState : public Serializable {
        CounterState()
            : eventId(0), filter(0), value(0), enabled(false),
              overflow64(false) {

            listeners.reserve(4);
        }

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp)  override;

        /**
         * Add an event count to the counter and check for overflow.
         *
         * @param delta Number of events to add to the counter.
         * @return true on overflow, false otherwise.
         */
        bool add(uint64_t delta);

      public: /* Serializable state */
        /** Counter event ID */
        EventTypeId eventId;

        /** Filtering settings (evtCount is unused) */
        PMEVTYPER_t filter;

        /** Current value of the counter */
        uint64_t value;

        /** Is the counter enabled? */
        bool enabled;

        /** Is this a 64-bit counter? */
        bool overflow64;

      public: /* Configuration */
        /** Probe listeners driving this counter */
        std::vector<ProbeListenerUPtr> listeners;
    };

    /**
     * Handle an counting event triggered by a probe.
     *
     * This method is called by the ProbeListener class whenever an
     * active probe is triggered. Ths method adds the event count from
     * the probe to the affected counter, checks for overflows, and
     * delivers an interrupt if needed.
     *
     * @param id Counter ID affected by the probe.
     * @param delta Counter increment
     */
    void handleEvent(CounterId id, uint64_t delta);

    /**
     * Is this a valid counter ID?
     *
     * @param id ID of counter within the PMU.
     *
     * @return true if counter is within the allowed range or the
     * cycle counter, false otherwise.
     */
    bool isValidCounter(CounterId id) const {
        return id < counters.size() || id == PMCCNTR;
    }

    /**
     * Return the state of a counter.
     *
     * @param id ID of counter within the PMU.
     * @return Reference to a CounterState instance representing the
     * counter.
     */
    CounterState &getCounter(CounterId id) {
        assert(isValidCounter(id));
        return id == PMCCNTR ? cycleCounter : counters[id];
    }


    /**
     * Return the state of a counter.
     *
     * @param id ID of counter within the PMU.
     * @return Reference to a CounterState instance representing the
     * counter.
     */
    const CounterState &getCounter(CounterId id) const {
        assert(isValidCounter(id));
        return id == PMCCNTR ? cycleCounter : counters[id];
    }

    /**
     * Depending on counter configuration, add or remove the probes
     * driving the counter.
     *
     * Look at the state of a counter and (re-)attach the probes
     * needed to drive a counter if it is currently active. All probes
     * for the counter are detached if the counter is inactive.
     *
     * @param id ID of counter within the PMU.
     * @param ctr Reference to the counter's state
     */
    void updateCounter(CounterId id, CounterState &ctr);

    /**
     * Check if a counter's settings allow it to be counted.
     *
     * @param ctr Counter state instance representing this counter.
     * @return false if the counter is active, true otherwise.
     */
    bool isFiltered(const CounterState &ctr) const;

    /**
     * Call updateCounter() for each counter in the PMU if the
     * counter's state has changed..
     *
     * @see updateCounter()
     */
    void updateAllCounters();

  protected: /* State that needs to be serialized */
    /** Performance Monitor Count Enable Register */
    MiscReg reg_pmcnten;

    /** Performance Monitor Control Register */
    PMCR_t reg_pmcr;

    /** Performance Monitor Selection Register */
    PMSELR_t reg_pmselr;

    /** Performance Monitor Interrupt Enable Register */
    MiscReg reg_pminten;

    /** Performance Monitor Overflow Status Register */
    MiscReg reg_pmovsr;

    /**
     * Performance counter ID register
     *
     * These registers contain a bitmask of available architected
     * counters.
     */
    uint64_t reg_pmceid0;
    uint64_t reg_pmceid1;

    /** Remainder part when the clock counter is divided by 64 */
    unsigned clock_remainder;

    /** State of all general-purpose counters supported by PMU */
    std::vector<CounterState> counters;
    /** State of the cycle counter */
    CounterState cycleCounter;

  protected: /* Configuration and constants */
    /** Constant (configuration-dependent) part of the PMCR */
    PMCR_t reg_pmcr_conf;
    /** PMCR write mask when accessed from the guest */
    static const MiscReg reg_pmcr_wr_mask;

    /** Performance monitor interrupt number */
    const unsigned int pmuInterrupt;
    /** Platform this device belongs to */
    Platform *const platform;

    /**
     * Event types supported by this PMU.
     *
     * Each event type ID can map to multiple EventType structures,
     * which enables the PMU to use multiple probes for a single
     * event. This can be useful in the following cases:
     * <ul>
     *   <li>Some events can are increment by multiple different probe
     *       points (e.g., the CPU memory access counter gets
     *       incremented for both loads and stores).
     *
     *   <li>A system switching between multiple CPU models can
     *       register events for all models that will execute a thread
     *       and tehreby ensure that the PMU continues to work.
     * </ul>
     */
    std::multimap<EventTypeId, EventType> pmuEventTypes;
};

} // namespace ArmISA
#endif
