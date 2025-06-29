/*
 * Copyright (c) 2011-2014, 2017-2019, 2022-2023, 2025 Arm Limited
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

#include "arch/arm/pmu.hh"

#include "arch/arm/isa.hh"
#include "arch/arm/utility.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "debug/Checkpoint.hh"
#include "debug/PMUVerbose.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/generic_timer.hh"
#include "params/ArmPMU.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

namespace ArmISA {

const RegVal PMU::reg_pmcr_wr_mask = 0x39;

PMU::PMU(const ArmPMUParams &p)
    : SimObject(p), BaseISADevice(),
      use64bitCounters(p.use64bitCounters),
      reg_pmcnten(0), reg_pmcr(0),
      reg_pmselr(0), reg_pminten(0), reg_pmovsr(0),
      reg_pmceid0(0),reg_pmceid1(0),
      clock_remainder(0),
      maximumCounterCount(p.eventCounters),
      cycleCounter(*this, maximumCounterCount, p.use64bitCounters),
      cycleCounterEventId(p.cycleEventId),
      swIncrementEvent(nullptr),
      reg_pmcr_conf(0),
      interrupt(nullptr),
      exitOnPMUControl(p.exitOnPMUControl),
      exitOnPMUInterrupt(p.exitOnPMUInterrupt),
      stats(this)
{
    DPRINTF(PMUVerbose, "Initializing the PMU.\n");

    if (maximumCounterCount > 31) {
        fatal("The PMU can only accept 31 counters, %d counters requested.\n",
              maximumCounterCount);
    }

    for (const auto& [key, value] : p.statCounters) {
        statCounters.insert(key);
    }

    warn_if(!p.interrupt, "ARM PMU: No interrupt specified, interrupt " \
            "delivery disabled.\n");

    /* Setup the performance counter ID registers */
    reg_pmcr_conf.imp = 0x41;    // ARM Ltd.
    reg_pmcr_conf.idcode = 0x00;
    reg_pmcr_conf.n = p.eventCounters;

    // Setup the hard-coded cycle counter, which is equivalent to
    // architected counter event type 0x11.
    cycleCounter.eventId = enums::EventTypeId::CPU_CYCLES;
}

PMU::~PMU()
{
}

void
PMU::setThreadContext(ThreadContext *tc)
{
    DPRINTF(PMUVerbose, "Assigning PMU to ContextID %i.\n", tc->contextId());
    const auto &pmu_params = static_cast<const ArmPMUParams &>(params());

    if (pmu_params.interrupt)
        interrupt = pmu_params.interrupt->get(tc);
}

void
PMU::addSoftwareIncrementEvent(EventTypeId id)
{
    auto old_event = eventMap.find(id);
    DPRINTF(PMUVerbose, "PMU: Adding SW increment event with id '0x%x'\n", id);

    if (swIncrementEvent) {
        fatal_if(old_event == eventMap.end() ||
                 old_event->second != swIncrementEvent,
                 "Trying to add a software increment event with multiple"
                 "IDs. This is not supported.\n");
        return;
    }

    fatal_if(old_event != eventMap.end(), "An event with id %d has "
             "been previously defined\n", id);

    auto pmu_stats = statCounters.find(id) != statCounters.end() ?
        &stats : nullptr;
    swIncrementEvent = std::make_shared<SWIncrementEvent>(id, pmu_stats);
    eventMap[id] = swIncrementEvent;
    registerEvent(id);
}

void
PMU::addEventProbe(EventTypeId id, SimObject *obj, const char *probe_name)
{

    DPRINTF(PMUVerbose, "PMU: Adding Probe Driven event with id '0x%x'"
        "as probe %s:%s\n", id, obj->name(), probe_name);

    std::shared_ptr<RegularEvent> event;
    auto event_entry = eventMap.find(id);
    if (event_entry == eventMap.end()) {
        auto pmu_stats = statCounters.find(id) != statCounters.end() ?
            &stats : nullptr;
        event = std::make_shared<RegularEvent>(id, pmu_stats);
        eventMap[id] = event;
    } else {
        event = std::dynamic_pointer_cast<RegularEvent>(event_entry->second);
        fatal_if(!event, "Event with id %d is not probe driven\n", id);
    }
    event->addMicroarchitectureProbe(obj, probe_name);

    registerEvent(id);
}

void
PMU::registerEvent(EventTypeId id)
{
    // Flag the event as available in the corresponding PMCEID register if it
    // is an architected event.
    if (id < 0x20) {
        reg_pmceid0 |= ((uint64_t)1) << id;
    } else if (id > 0x20 && id < 0x40) {
        reg_pmceid1 |= ((uint64_t)1) << (id - 0x20);
    } else if (id >= 0x4000 && id < 0x4020) {
        reg_pmceid0 |= ((uint64_t)1) << (id - 0x4000 + 32);
    } else if (id >= 0x4020 && id < 0x4040) {
        reg_pmceid1 |= ((uint64_t)1) << (id - 0x4020 + 32);
    }
}

void
PMU::drainResume()
{
    // Re-attach enabled counters after a resume in case they changed.
    updateAllCounters();
}

void
PMU::regProbeListeners()
{

    // at this stage all probe configurations are done
    // counters can be configured
    for (uint32_t index = 0; index < maximumCounterCount; index++) {
        counters.emplace_back(*this, index, use64bitCounters);
    }

    std::shared_ptr<PMUEvent> event = getEvent(cycleCounterEventId);
    panic_if(!event, "core cycle event is not present\n");
    cycleCounter.enabled = true;
    cycleCounter.attach(event);

    // By enabling the event we are actually connecting the
    // listener (See PMU::RegularEvent::enable)
    for (auto event_id : statCounters) {
        if (auto stat_event = getEvent(event_id); stat_event) {
            stat_event->enable();
        }
    }
}

void
PMU::setMiscReg(int misc_reg, RegVal val)
{
    DPRINTF(PMUVerbose, "setMiscReg(%s, 0x%x)\n",
            miscRegName[unflattenMiscReg(misc_reg)], val);

    switch (unflattenMiscReg(misc_reg)) {
      case MISCREG_PMCR_EL0:
      case MISCREG_PMCR:
        setControlReg(val);
        return;

      case MISCREG_PMCNTENSET_EL0:
      case MISCREG_PMCNTENSET:
        reg_pmcnten |= val;
        updateAllCounters();
        pmuExitEvent(false);
        return;

      case MISCREG_PMCNTENCLR_EL0:
      case MISCREG_PMCNTENCLR:
        reg_pmcnten &= ~val;
        updateAllCounters();
        pmuExitEvent(false);
        return;

      case MISCREG_PMOVSCLR_EL0:
      case MISCREG_PMOVSR:
        setOverflowStatus(reg_pmovsr & ~val);
        return;

      case MISCREG_PMSWINC_EL0:
      case MISCREG_PMSWINC:
        if (swIncrementEvent) {
            swIncrementEvent->write(val);
        }
        return;

      case MISCREG_PMCCNTR_EL0:
      case MISCREG_PMCCNTR:
        cycleCounter.setValue(val);
        return;

      case MISCREG_PMSELR_EL0:
      case MISCREG_PMSELR:
        reg_pmselr = val;
        return;
      //TODO: implement MISCREF_PMCEID{2,3}
      case MISCREG_PMCEID0_EL0:
      case MISCREG_PMCEID0:
      case MISCREG_PMCEID1_EL0:
      case MISCREG_PMCEID1:
        // Ignore writes
        return;

      case MISCREG_PMEVTYPER0_EL0...MISCREG_PMEVTYPER5_EL0:
        setCounterTypeRegister(misc_reg - MISCREG_PMEVTYPER0_EL0, val);
        return;
      case MISCREG_PMEVTYPER0...MISCREG_PMEVTYPER5:
        setCounterTypeRegister(misc_reg - MISCREG_PMEVTYPER0, val);
        return;

      case MISCREG_PMCCFILTR:
      case MISCREG_PMCCFILTR_EL0:
        DPRINTF(PMUVerbose, "Setting PMCCFILTR: 0x%x\n", val);
        setCounterTypeRegister(PMCCNTR, val);
        return;

      case MISCREG_PMXEVTYPER_PMCCFILTR:
      case MISCREG_PMXEVTYPER_EL0:
      case MISCREG_PMXEVTYPER:
        DPRINTF(PMUVerbose, "Setting counter type: "
                "[PMSELR: 0x%x, PMSELER.sel: 0x%x, EVTYPER: 0x%x]\n",
                reg_pmselr, reg_pmselr.sel, val);
        setCounterTypeRegister(reg_pmselr.sel, val);
        return;

      case MISCREG_PMEVCNTR0_EL0...MISCREG_PMEVCNTR5_EL0:
        setCounterValue(misc_reg - MISCREG_PMEVCNTR0_EL0, val);
        return;
      case MISCREG_PMEVCNTR0...MISCREG_PMEVCNTR5:
        setCounterValue(misc_reg - MISCREG_PMEVCNTR0, val);
        return;

      case MISCREG_PMXEVCNTR_EL0:
      case MISCREG_PMXEVCNTR:
        setCounterValue(reg_pmselr.sel, val);
        return;

      case MISCREG_PMUSERENR_EL0:
      case MISCREG_PMUSERENR:
        // TODO
        break;

      case MISCREG_PMINTENSET_EL1:
      case MISCREG_PMINTENSET:
        reg_pminten |= val;
        return;

      case MISCREG_PMINTENCLR_EL1:
      case MISCREG_PMINTENCLR:
        reg_pminten &= ~val;
        return;

      case MISCREG_PMOVSSET_EL0:
      case MISCREG_PMOVSSET:
        setOverflowStatus(reg_pmovsr | val);
        return;

      default:
        panic("Unexpected PMU register: %i\n", miscRegName[misc_reg]);
    }

    warn("Not doing anything for write to miscreg %s\n",
         miscRegName[misc_reg]);
}

RegVal
PMU::readMiscReg(int misc_reg)
{
    RegVal val(readMiscRegInt(misc_reg));
    DPRINTF(PMUVerbose, "readMiscReg(%s): 0x%x\n",
            miscRegName[unflattenMiscReg(misc_reg)], val);
    return val;
}

RegVal
PMU::readMiscRegInt(int misc_reg)
{
    misc_reg = unflattenMiscReg(misc_reg);
    switch (misc_reg) {
      case MISCREG_PMCR_EL0:
      case MISCREG_PMCR:
        return reg_pmcr_conf | (reg_pmcr & reg_pmcr_wr_mask);

      case MISCREG_PMCNTENSET_EL0:
      case MISCREG_PMCNTENCLR_EL0:
      case MISCREG_PMCNTENSET:
      case MISCREG_PMCNTENCLR:
        return reg_pmcnten;

      case MISCREG_PMOVSCLR_EL0:
      case MISCREG_PMOVSSET_EL0:
      case MISCREG_PMOVSR:  // Overflow Status Register
      case MISCREG_PMOVSSET:
        return reg_pmovsr;

      case MISCREG_PMSWINC_EL0:
      case MISCREG_PMSWINC: // Software Increment Register (RAZ)
        return 0;

      case MISCREG_PMSELR_EL0:
      case MISCREG_PMSELR:
        return reg_pmselr;

      case MISCREG_PMCEID0_EL0:
        return reg_pmceid0;

      case MISCREG_PMCEID1_EL0:
        return reg_pmceid1;

      //TODO: implement MISCREF_PMCEID{2,3}
      case MISCREG_PMCEID0: // Common Event ID register
        return reg_pmceid0 & 0xFFFFFFFF;

      case MISCREG_PMCEID1: // Common Event ID register
        return reg_pmceid1 & 0xFFFFFFFF;

      case MISCREG_PMCCNTR_EL0:
      case MISCREG_PMCCNTR:
        return cycleCounter.getValue();

      case MISCREG_PMEVTYPER0_EL0...MISCREG_PMEVTYPER5_EL0:
        return getCounterTypeRegister(misc_reg - MISCREG_PMEVTYPER0_EL0);
      case MISCREG_PMEVTYPER0...MISCREG_PMEVTYPER5:
        return getCounterTypeRegister(misc_reg - MISCREG_PMEVTYPER0);

      case MISCREG_PMCCFILTR:
      case MISCREG_PMCCFILTR_EL0:
        return getCounterTypeRegister(PMCCNTR);

      case MISCREG_PMXEVTYPER_PMCCFILTR:
      case MISCREG_PMXEVTYPER_EL0:
      case MISCREG_PMXEVTYPER:
        return getCounterTypeRegister(reg_pmselr.sel);

      case MISCREG_PMEVCNTR0_EL0...MISCREG_PMEVCNTR5_EL0:
        return getCounterValue(misc_reg - MISCREG_PMEVCNTR0_EL0) & 0xFFFFFFFF;
      case MISCREG_PMEVCNTR0...MISCREG_PMEVCNTR5:
        return getCounterValue(misc_reg - MISCREG_PMEVCNTR0) & 0xFFFFFFFF;

      case MISCREG_PMXEVCNTR_EL0:
      case MISCREG_PMXEVCNTR:
        return getCounterValue(reg_pmselr.sel) & 0xFFFFFFFF;

      case MISCREG_PMUSERENR_EL0:
      case MISCREG_PMUSERENR:
        // TODO
        return 0;

      case MISCREG_PMINTENSET_EL1:
      case MISCREG_PMINTENCLR_EL1:
      case MISCREG_PMINTENSET:
      case MISCREG_PMINTENCLR:
        return reg_pminten;

      default:
        panic("Unexpected PMU register: %i\n", miscRegName[misc_reg]);
    }

    warn("Not doing anything for read from miscreg %s\n",
         miscRegName[misc_reg]);
    return 0;
}

void
PMU::setControlReg(PMCR_t val)
{
    DPRINTF(PMUVerbose, "Set Control Reg 0x%08x.\n", val);

    if (val.p) {
        DPRINTF(PMUVerbose, "PMU reset all events to zero.\n");
        resetEventCounts();
    }

    if (val.c) {
        DPRINTF(PMUVerbose, "PMU reset cycle counter to zero.\n");
        cycleCounter.setValue(0);
    }

    // Reset the clock remainder if divide by 64-mode is toggled.
    if (reg_pmcr.d != val.d)
        clock_remainder = 0;

    reg_pmcr = val & reg_pmcr_wr_mask;

    updateAllCounters();
    pmuExitEvent(true);
}

void
PMU::pmuExitEvent(bool check_reset)
{
    if (!exitOnPMUControl) {
        return;
    }

    // Optionally exit the simulation on various PMU control events.
    // Exit on enable/disable takes precedence over exit on reset.
    if (bool any_counter_enabled = reg_pmcr.e && reg_pmcnten;
        any_counter_enabled != pmuEnabled) {

        pmuEnabled = any_counter_enabled;
        if (pmuEnabled) {
            inform("Exiting simulation: PMU enable detected");
            exitSimLoop("performance counter enabled", 0);
        } else {
            inform("Exiting simulation: PMU disable detected");
            exitSimLoop("performance counter disabled", 0);
        }
    } else if (check_reset && reg_pmcr.p) {
        inform("Exiting simulation: PMU reset detected");
        exitSimLoop("performance counter reset", 0);
    }
}

void
PMU::updateAllCounters()
{
    const bool global_enable(reg_pmcr.e);

    for (int i = 0; i < counters.size(); ++i) {
        CounterState &ctr(counters[i]);
        const bool enable(global_enable && (reg_pmcnten & (1 << i)));
        if (ctr.enabled != enable) {
            ctr.enabled = enable;
            updateCounter(ctr);
        }
    }

    const bool ccntr_enable(global_enable && (reg_pmcnten & (1 << PMCCNTR)));
    if (cycleCounter.enabled != ccntr_enable) {
        cycleCounter.enabled = ccntr_enable;
        updateCounter(cycleCounter);
    }
}

void
PMU::PMUEvent::attachEvent(PMU::CounterState *user)
{
    // Check if there is already a probe (either non-empty
    // userCounters list, or pmuStats enabled)
    if (userCounters.empty() && !pmuStats) {
        enable();
    }
    userCounters.insert(user);
    updateAttachedCounters();
}

void
PMU::PMUEvent::increment(const uint64_t val)
{
    for (auto& counter: userCounters) {
        counter->add(val);
    }

    if (pmuStats) {
        pmuStats->add(id, val);
    }
}

void
PMU::PMUEvent::detachEvent(PMU::CounterState *user)
{
    userCounters.erase(user);

    // Do not destroy the probe if pmuStats are listening
    // to the event
    if (userCounters.empty() && !pmuStats) {
        disable();
    }
}

void
PMU::RegularEvent::RegularProbe::notify(const uint64_t &val)
{
    parentEvent->increment(val);
}

void
PMU::RegularEvent::enable()
{
    for (auto& subEvents: microArchitectureEventSet) {
        ProbeManager *pm = subEvents.first->getProbeManager();
        ProbePoint *probe = pm->getFirstProbePoint(subEvents.second);
        // The PMU currently handles explicit PMU probes as well as
        // cache events. For any further types of events we will need
        // to handle them in this function.
        if (dynamic_cast<probing::PMU *>(probe)) {
            attachedProbePointList.push_back(
                pm->connect<RegularProbe>(this, subEvents.second));
        } else if (dynamic_cast<ProbePointArg<CacheAccessProbeArg> *>(probe)) {
            attachedProbePointList.push_back(
                pm->connect<CacheProbe>(this, subEvents.second));
        } else {
            panic("Unsupported probe kind for event %s", probe->getName());
        }
    }
}

void
PMU::RegularEvent::disable()
{
    attachedProbePointList.clear();
}

bool
PMU::CounterState::isFiltered() const
{
    assert(pmu.isa);

    const PMEVTYPER_t filter(this->filter);
    const ExceptionLevel el(pmu.isa->currEL());
    const bool secure(pmu.isa->inSecureState());

    switch (el) {
      case EL0:
        return secure ? filter.u : (filter.u != filter.nsu);

      case EL1:
        return secure ? filter.p : (filter.p != filter.nsk);

      case EL2:
        return !filter.nsh;

      case EL3:
        return filter.p != filter.m;

      default:
        panic("Unexpected execution level in PMU::isFiltered.\n");
    }
}

void
PMU::CounterState::detach()
{
    if (sourceEvent) {
        sourceEvent->detachEvent(this);
        sourceEvent = nullptr;
    } else {
        DPRINTFS(PMUVerbose, &pmu,
                 "detaching event %d not currently attached to any event"
                 " [counterId = %d]\n", eventId, counterId);
    }
}

void
PMU::CounterState::attach(const std::shared_ptr<PMUEvent> &event)
{
    if (!resetValue) {
      value = 0;
      resetValue = true;
    }
    sourceEvent = event;
    sourceEvent->attachEvent(this);
}

uint64_t
PMU::CounterState::getValue() const
{
    if (sourceEvent) {
        sourceEvent->updateAttachedCounters();
    } else {
        debugCounter("attempted to get value from a counter without"
            " an associated event\n");
    }
    return value;
}

void
PMU::CounterState::setValue(uint64_t val)
{
    value = val;
    resetValue = true;

    if (sourceEvent) {
        sourceEvent->updateAttachedCounters();
    } else {
        debugCounter("attempted to set value from a counter without"
            " an associated event\n");
    }
}

void
PMU::updateCounter(CounterState &ctr)
{
    if (!ctr.enabled) {
        DPRINTF(PMUVerbose, "updateCounter(%i): Disabling counter\n",
            ctr.getCounterId());
        ctr.detach();

    } else {
        DPRINTF(PMUVerbose, "updateCounter(%i): Enable event id 0x%x\n",
                ctr.getCounterId(), ctr.eventId);

        auto sourceEvent = eventMap.find(ctr.eventId);
        if (sourceEvent == eventMap.end()) {
            warn("Can't enable PMU counter of type '0x%x': "
                 "No such event type.\n", ctr.eventId);
            ctr.detach();
        } else {
            ctr.attach(sourceEvent->second);
        }
    }
}


void
PMU::resetEventCounts()
{
    for (CounterState &ctr : counters)
        ctr.setValue(0);
}

void
PMU::setCounterValue(CounterId id, uint64_t val)
{
    if (!isValidCounter(id)) {
        warn_once("Can't change counter value: Counter %i does not exist.\n",
                  id);
        return;
    }

    CounterState &ctr(getCounter(id));
    ctr.setValue(val);
}

PMU::PMEVTYPER_t
PMU::getCounterTypeRegister(CounterId id) const
{
    if (!isValidCounter(id))
        return 0;

    const CounterState &cs(getCounter(id));
    PMEVTYPER_t type(cs.filter);

    type.evtCount = cs.eventId;

    return type;
}

void
PMU::setCounterTypeRegister(CounterId id, PMEVTYPER_t val)
{
    DPRINTF(PMUVerbose, "Set Event [%d] = 0x%08x\n", id, val);
    if (!isValidCounter(id)) {
        warn_once("Can't change counter type: Counter %i does not exist.\n",
                  id);
        return;
    }

    CounterState &ctr(getCounter(id));
    const EventTypeId old_event_id(ctr.eventId);

    ctr.filter = val;

    // If PMCCNTR Register, do not change event type. PMCCNTR can
    // count processor cycles only. If we change the event type, we
    // need to update the probes the counter is using.
    if (id != PMCCNTR && old_event_id != val.evtCount) {
        ctr.eventId = val.evtCount;
        updateCounter(ctr);
    }
}

void
PMU::setOverflowStatus(RegVal new_val)
{
    const bool int_old = reg_pmovsr != 0;
    const bool int_new = new_val != 0;

    reg_pmovsr = new_val;
    if (int_old && !int_new) {
        clearInterrupt();
    } else if (!int_old && int_new && (reg_pminten & reg_pmovsr)) {
        raiseInterrupt();
    }
}

void
PMU::raiseInterrupt()
{
    if (exitOnPMUInterrupt) {
        inform("Exiting simulation: PMU interrupt detected");
        exitSimLoop("performance counter interrupt", 0);
    }
    if (interrupt) {
        DPRINTF(PMUVerbose, "Delivering PMU interrupt.\n");
        interrupt->raise();
    } else {
        warn_once("Dropping PMU interrupt as no interrupt has "
                  "been specified\n");
    }
}

void
PMU::clearInterrupt()
{
    if (interrupt) {
        DPRINTF(PMUVerbose, "Clearing PMU interrupt.\n");
        interrupt->clear();
    } else {
        warn_once("Dropping PMU interrupt as no interrupt has "
                  "been specified\n");
    }
}

void
PMU::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm PMU\n");

    SERIALIZE_SCALAR(use64bitCounters);
    SERIALIZE_SCALAR(reg_pmcr);
    SERIALIZE_SCALAR(reg_pmcnten);
    SERIALIZE_SCALAR(reg_pmselr);
    SERIALIZE_SCALAR(reg_pminten);
    SERIALIZE_SCALAR(reg_pmovsr);
    SERIALIZE_SCALAR(reg_pmceid0);
    SERIALIZE_SCALAR(reg_pmceid1);
    SERIALIZE_SCALAR(clock_remainder);

    for (size_t i = 0; i < counters.size(); ++i)
        counters[i].serializeSection(cp, csprintf("counters.%i", i));

    cycleCounter.serializeSection(cp, "cycleCounter");
}

void
PMU::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm PMU\n");

    UNSERIALIZE_SCALAR(use64bitCounters);
    UNSERIALIZE_SCALAR(reg_pmcr);
    UNSERIALIZE_SCALAR(reg_pmcnten);
    UNSERIALIZE_SCALAR(reg_pmselr);
    UNSERIALIZE_SCALAR(reg_pminten);
    UNSERIALIZE_SCALAR(reg_pmovsr);

    pmuEnabled = reg_pmcr.e && reg_pmcnten;

    // Old checkpoints used to store the entire PMCEID value in a
    // single 64-bit entry (reg_pmceid). The register was extended in
    // ARMv8.1, so we now need to store it as two 64-bit registers.
    if (!UNSERIALIZE_OPT_SCALAR(reg_pmceid0))
        paramIn(cp, "reg_pmceid", reg_pmceid0);

    if (!UNSERIALIZE_OPT_SCALAR(reg_pmceid1))
        reg_pmceid1 = 0;

    UNSERIALIZE_SCALAR(clock_remainder);

    for (size_t i = 0; i < counters.size(); ++i)
        counters[i].unserializeSection(cp, csprintf("counters.%i", i));

    cycleCounter.unserializeSection(cp, "cycleCounter");
}

std::shared_ptr<PMU::PMUEvent>
PMU::getEvent(EventTypeId eventId)
{
    auto entry = eventMap.find(eventId);

    if (entry == eventMap.end()) {
        warn("event %d does not exist\n", eventId);
        return nullptr;
    } else {
        return entry->second;
    }
}

void
PMU::CounterState::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(eventId);
    SERIALIZE_SCALAR(value);
    SERIALIZE_SCALAR(overflow64);
}

void
PMU::CounterState::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(eventId);
    UNSERIALIZE_SCALAR(value);
    UNSERIALIZE_SCALAR(overflow64);
}

uint64_t
PMU::CounterState::add(uint64_t delta)
{
    uint64_t value_until_overflow;
    if (overflow64) {
        value_until_overflow = UINT64_MAX - value;
    } else {
        value_until_overflow = UINT32_MAX - (uint32_t)value;
    }

    if (isFiltered())
        return value_until_overflow;

    if (resetValue) {
        delta = 0;
        resetValue = false;
    } else {
        value += delta;
    }

    if (delta > value_until_overflow) {

        // overflow situation detected
        // flag the overflow occurence
        pmu.reg_pmovsr |= (1 << counterId);

        // Deliver a PMU interrupt if interrupt delivery is enabled
        // for this counter.
        if (pmu.reg_pminten  & (1 << counterId)) {
            pmu.raiseInterrupt();
        }
        return overflow64 ? UINT64_MAX : UINT32_MAX;
    }
    return value_until_overflow - delta + 1;
}

void
PMU::SWIncrementEvent::write(uint64_t val)
{
    for (auto& counter: userCounters) {
        if (val & (0x1 << counter->getCounterId())) {
            counter->add(1);
        }
    }
}

PMU::Stats::Stats(PMU *parent)
    : statistics::Group(parent), pmu(parent)
{
    auto params = static_cast<const ArmPMUParams&>(pmu->params());
    for (const auto& [key, val] : params.statCounters) {
        registerEvent(key, val.c_str());
    }
}

void
PMU::Stats::registerEvent(EventTypeId id, const char *stat_name)
{
    map.emplace(std::piecewise_construct,
                std::forward_as_tuple(id),
                std::forward_as_tuple(
                    this, stat_name,
                    statistics::units::Count::get(),
                    stat_name));
    map[id]
        .precision(0);
}

void
PMU::Stats::add(EventTypeId id, uint64_t value)
{
    if (auto it = map.find(id); it != map.end()) {
        it->second += value;
    } else {
        panic("Couldn't increment event %d stat counter", id);
    }
}

} // namespace ArmISA
} // namespace gem5
