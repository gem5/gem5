/*
 * Copyright (c) 2011-2014, 2017-2018 ARM Limited
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

#ifndef __ARCH_ARM_PMU_HH__
#define __ARCH_ARM_PMU_HH__

#include <map>
#include <memory>
#include <vector>

#include "arch/arm/isa_device.hh"
#include "arch/arm/registers.hh"
#include "arch/arm/system.hh"
#include "base/cprintf.hh"
#include "cpu/base.hh"
#include "debug/PMUVerbose.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

class ArmPMUParams;
class Platform;
class ThreadContext;
class ArmInterruptPin;

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
    void addSoftwareIncrementEvent(unsigned int id);

    void registerEvent(uint32_t id);

  public: // SimObject and related interfaces
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void drainResume() override;

    void regProbeListeners() override;

  public: // ISA Device interface
    void setThreadContext(ThreadContext *tc) override;

    /**
     * Set a register within the PMU.
     *
     * @param misc_reg Register number (see miscregs.hh)
     * @param val Value to store
     */
    void setMiscReg(int misc_reg, RegVal val) override;
    /**
     * Read a register within the PMU.
     *
     * @param misc_reg Register number (see miscregs.hh)
     * @return Register value.
     */
    RegVal readMiscReg(int misc_reg) override;

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

  protected: /* High-level register and interrupt handling */
    RegVal readMiscRegInt(int misc_reg);

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
     * Clear a PMU interrupt.
     */
    void clearInterrupt();

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
        return isValidCounter(id) ? getCounter(id).getValue() : 0;
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

    /**
     * Used for writing the Overflow Flag Status Register (SET/CLR)
     *
     * This method implements a write to the PMOVSSET/PMOVSCLR registers.
     * It is capturing change of state in the register bits so that
     * the overflow interrupt can be raised/cleared as a side effect
     * of the write.
     *
     * @param new_val New value of the Overflow Status Register
     */
    void setOverflowStatus(RegVal new_val);

  protected: /* Probe handling and counter state */
    struct CounterState;

    /**
     * Event definition base class
     */
    struct PMUEvent {

        PMUEvent() {}

        virtual ~PMUEvent() {}

        /**
         * attach this event to a given counter
         *
         * @param a pointer to the counter where to attach this event
         */
        void attachEvent(PMU::CounterState *user);

        /**
         * detach this event from a given counter
         *
         * @param a pointer to the counter where to detach this event from
         */
        void detachEvent(PMU::CounterState *user);

        /**
         * notify an event increment of val units, all the attached counters'
         * value is incremented by val units.
         *
         * @param the quantity by which to increment the attached counter
         * values
         */
        virtual void increment(const uint64_t val);

        /**
         * Enable the current event
         */
        virtual void enable() = 0;

        /**
         * Disable the current event
         */
        virtual void disable() = 0;

        /**
         *  Method called immediately before a counter access in order for
         *  the associated event to update its state (if required)
         */
        virtual void updateAttachedCounters() {}

      protected:

        /** set of counters using this event  **/
        std::set<PMU::CounterState*> userCounters;
    };

    struct RegularEvent : public PMUEvent {
        typedef std::pair<SimObject*, std::string> EventTypeEntry;

        void addMicroarchitectureProbe(SimObject* object,
            std::string name) {

            panic_if(!object,"malformed probe-point"
                " definition with name %s\n", name);

            microArchitectureEventSet.emplace(object, name);
        }

      protected:
        struct RegularProbe: public  ProbeListenerArgBase<uint64_t>
        {
            RegularProbe(RegularEvent *parent, SimObject* obj,
                std::string name)
                : ProbeListenerArgBase(obj->getProbeManager(), name),
                  parentEvent(parent) {}

            RegularProbe() = delete;

            void notify(const uint64_t &val);

          protected:
            RegularEvent *parentEvent;
        };

        /** The set of events driving the event value **/
        std::set<EventTypeEntry> microArchitectureEventSet;

        /** Set of probe listeners tapping onto each of the input micro-arch
         *  events which compose this pmu event
         */
        std::vector<std::unique_ptr<RegularProbe>> attachedProbePointList;

        void enable() override;

        void disable() override;
    };

    class SWIncrementEvent : public PMUEvent
    {
        void enable() override {}
        void disable() override {}

      public:

        /**
         * write on the sw increment register inducing an increment of the
         * counters with this event selected according to the bitfield written.
         *
         * @param the bitfield selecting the counters to increment.
         */
        void write(uint64_t val);
    };

    /**
     * Obtain the event of a given id
     *
     * @param the id of the event to obtain
     * @return a pointer to the event with id eventId
     */
    PMUEvent* getEvent(uint64_t eventId);

    /** State of a counter within the PMU. **/
    struct CounterState : public Serializable {
        CounterState(PMU &pmuReference, uint64_t counter_id)
            : eventId(0), filter(0), enabled(false),
              overflow64(false), sourceEvent(nullptr),
              counterId(counter_id), value(0), resetValue(false),
              pmu(pmuReference) {}

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp)  override;

        /**
         * Add an event count to the counter and check for overflow.
         *
         * @param delta Number of events to add to the counter.
         * @return the quantity remaining until a counter overflow occurs.
         */
        uint64_t add(uint64_t delta);

        bool isFiltered() const;

        /**
         * Detach the counter from its event
         */
        void detach();

        /**
         * Attach this counter to an event
         *
         * @param the event to attach the counter to
         */
        void attach(PMUEvent* event);

        /**
         * Obtain the counter id
         *
         * @return the pysical counter id
         */
        uint64_t getCounterId() const{
            return counterId;
        }

        /**
         * rReturn the counter value
         *
         * @return the counter value
         */
        uint64_t getValue() const;

        /**
         * overwrite the value of the counter
         *
         * @param the new counter value
         */
        void setValue(uint64_t val);

      public: /* Serializable state */
        /** Counter event ID */
        EventTypeId eventId;

        /** Filtering settings (evtCount is unused) */
        PMEVTYPER_t filter;

        /** Is the counter enabled? */
        bool enabled;

        /** Is this a 64-bit counter? */
        bool overflow64;

      protected: /* Configuration */
        /** PmuEvent currently in use (if any) **/
        PMUEvent *sourceEvent;

        /** id of the counter instance **/
        uint64_t counterId;

        /** Current value of the counter */
        uint64_t value;

        /** Flag keeping track if the counter has been reset **/
        bool resetValue;

        PMU &pmu;

        template <typename ...Args>
        void debugCounter(const char* mainString, Args &...args) const {

            std::string userString = csprintf(mainString, args...);

            warn("[counterId = %d, eventId = %d, sourceEvent = 0x%x] %s",
                counterId, eventId, sourceEvent, userString.c_str());

        }
    };

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
    void updateCounter(CounterState &ctr);

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
    RegVal reg_pmcnten;

    /** Performance Monitor Control Register */
    PMCR_t reg_pmcr;

    /** Performance Monitor Selection Register */
    PMSELR_t reg_pmselr;

    /** Performance Monitor Interrupt Enable Register */
    RegVal reg_pminten;

    /** Performance Monitor Overflow Status Register */
    RegVal reg_pmovsr;

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

    /** The number of regular event counters **/
    uint64_t maximumCounterCount;

    /** State of all general-purpose counters supported by PMU */
    std::vector<CounterState> counters;

    /** State of the cycle counter */
    CounterState cycleCounter;

    /** The id of the counter hardwired to the cpu cycle counter **/
    const uint64_t cycleCounterEventId;

    /** The event that implements the software increment **/
    SWIncrementEvent *swIncrementEvent;

  protected: /* Configuration and constants */
    /** Constant (configuration-dependent) part of the PMCR */
    PMCR_t reg_pmcr_conf;

    /** PMCR write mask when accessed from the guest */
    static const RegVal reg_pmcr_wr_mask;

    /** Performance monitor interrupt number */
    ArmInterruptPin *interrupt;

    /**
     * List of event types supported by this PMU.
     */
    std::map<EventTypeId, PMUEvent*> eventMap;
};

} // namespace ArmISA
#endif
