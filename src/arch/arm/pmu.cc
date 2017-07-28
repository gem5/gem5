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

#include "arch/arm/pmu.hh"

#include "arch/arm/isa.hh"
#include "arch/arm/utility.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "debug/Checkpoint.hh"
#include "debug/PMUVerbose.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/realview.hh"
#include "params/ArmPMU.hh"

namespace ArmISA {

const MiscReg PMU::reg_pmcr_wr_mask = 0x39;

PMU::PMU(const ArmPMUParams *p)
    : SimObject(p), BaseISADevice(),
      reg_pmcnten(0), reg_pmcr(0),
      reg_pmselr(0), reg_pminten(0), reg_pmovsr(0),
      reg_pmceid0(0),reg_pmceid1(0),
      clock_remainder(0),
      counters(p->eventCounters),
      reg_pmcr_conf(0),
      pmuInterrupt(p->pmuInterrupt),
      platform(p->platform)
{
    DPRINTF(PMUVerbose, "Initializing the PMU.\n");

    if (p->eventCounters > 31) {
        fatal("The PMU can only accept 31 counters, %d counters requested.\n",
              p->eventCounters);
    }

    /* Setup the performance counter ID registers */
    reg_pmcr_conf.imp = 0x41;    // ARM Ltd.
    reg_pmcr_conf.idcode = 0x00;
    reg_pmcr_conf.n = p->eventCounters;

    // Setup the hard-coded cycle counter, which is equivalent to
    // architected counter event type 0x11.
    cycleCounter.eventId = 0x11;
}

PMU::~PMU()
{
}

void
PMU::addEventProbe(unsigned int id, SimObject *obj, const char *probe_name)
{
    DPRINTF(PMUVerbose, "PMU: Adding event type '0x%x' as probe %s:%s\n",
            id, obj->name(), probe_name);
    pmuEventTypes.insert(std::make_pair(id, EventType(obj, probe_name)));

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
PMU::setMiscReg(int misc_reg, MiscReg val)
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
        return;

      case MISCREG_PMCNTENCLR_EL0:
      case MISCREG_PMCNTENCLR:
        reg_pmcnten &= ~val;
        updateAllCounters();
        return;

      case MISCREG_PMOVSCLR_EL0:
      case MISCREG_PMOVSR:
        reg_pmovsr &= ~val;
        return;

      case MISCREG_PMSWINC_EL0:
      case MISCREG_PMSWINC:
        for (int i = 0; i < counters.size(); ++i) {
            CounterState &ctr(getCounter(i));
            if (ctr.enabled && (val & (1 << i))
                && ctr.eventId == ARCH_EVENT_SW_INCR ) {
                ++ctr.value;
            }
        }
        break;

      case MISCREG_PMCCNTR_EL0:
      case MISCREG_PMCCNTR:
        cycleCounter.value = val;
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
        reg_pmovsr |= val;
        return;

      default:
        panic("Unexpected PMU register: %i\n", miscRegName[misc_reg]);
    }

    warn("Not doing anything for write to miscreg %s\n",
         miscRegName[misc_reg]);
}

MiscReg
PMU::readMiscReg(int misc_reg)
{
    MiscReg val(readMiscRegInt(misc_reg));
    DPRINTF(PMUVerbose, "readMiscReg(%s): 0x%x\n",
            miscRegName[unflattenMiscReg(misc_reg)], val);
    return val;
}

MiscReg
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
        return cycleCounter.value;

      case MISCREG_PMCCNTR:
        return cycleCounter.value & 0xFFFFFFFF;

      case MISCREG_PMEVTYPER0_EL0...MISCREG_PMEVTYPER5_EL0:
        return getCounterTypeRegister(misc_reg - MISCREG_PMEVTYPER0_EL0);

      case MISCREG_PMCCFILTR:
      case MISCREG_PMCCFILTR_EL0:
        return getCounterTypeRegister(PMCCNTR);

      case MISCREG_PMXEVTYPER_PMCCFILTR:
      case MISCREG_PMXEVTYPER_EL0:
      case MISCREG_PMXEVTYPER:
        return getCounterTypeRegister(reg_pmselr.sel);

      case MISCREG_PMEVCNTR0_EL0...MISCREG_PMEVCNTR5_EL0:
        return getCounterValue(misc_reg - MISCREG_PMEVCNTR0_EL0) & 0xFFFFFFFF;

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
        cycleCounter.value = 0;
    }

    // Reset the clock remainder if divide by 64-mode is toggled.
    if (reg_pmcr.d != val.d)
        clock_remainder = 0;

    reg_pmcr = val & reg_pmcr_wr_mask;
    updateAllCounters();
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
            updateCounter(i, ctr);
        }
    }

    const bool ccntr_enable(global_enable && (reg_pmcnten & (1 << PMCCNTR)));
    if (cycleCounter.enabled != ccntr_enable) {
        cycleCounter.enabled = ccntr_enable;
        updateCounter(PMCCNTR, cycleCounter);
    }
}

bool
PMU::isFiltered(const CounterState &ctr) const
{
    assert(isa);

    const PMEVTYPER_t filter(ctr.filter);
    const SCR scr(isa->readMiscRegNoEffect(MISCREG_SCR));
    const CPSR cpsr(isa->readMiscRegNoEffect(MISCREG_CPSR));
    const ExceptionLevel el(opModeToEL((OperatingMode)(uint8_t)cpsr.mode));
    const bool secure(inSecureState(scr, cpsr));

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
PMU::handleEvent(CounterId id, uint64_t delta)
{
    CounterState &ctr(getCounter(id));
    const bool overflowed(reg_pmovsr & (1 << id));

    if (isFiltered(ctr))
        return;

    // Handle the "count every 64 cycles" mode
    if (id == PMCCNTR && reg_pmcr.d) {
        clock_remainder += delta;
        delta = (clock_remainder >> 6);
        clock_remainder &= 63;
    }

    // Add delta and handle (new) overflows
    if (ctr.add(delta) && !overflowed) {
        DPRINTF(PMUVerbose, "PMU counter '%i' overflowed.\n", id);
        reg_pmovsr |= (1 << id);
        // Deliver a PMU interrupt if interrupt delivery is enabled
        // for this counter.
        if (reg_pminten  & (1 << id))
            raiseInterrupt();
    }
}

void
PMU::updateCounter(CounterId id, CounterState &ctr)
{
    if (!ctr.enabled) {
        if (!ctr.listeners.empty()) {
            DPRINTF(PMUVerbose, "updateCounter(%i): Disabling counter\n", id);
            ctr.listeners.clear();
        }
    } else {
        DPRINTF(PMUVerbose, "updateCounter(%i): Enable event id 0x%x\n",
                id, ctr.eventId);

        // Attach all probes belonging to this event
        auto range(pmuEventTypes.equal_range(ctr.eventId));
        for (auto it = range.first; it != range.second; ++it) {
            const EventType &et(it->second);

            DPRINTF(PMUVerbose, "\tProbe: %s:%s\n", et.obj->name(), et.name);
            ctr.listeners.emplace_back(et.create(*this, id));
        }

        /* The SW_INCR event type is a special case which doesn't need
         * any probes since it is controlled by software and the PMU
         * itself.
         */
        if (ctr.listeners.empty() && ctr.eventId != ARCH_EVENT_SW_INCR) {
            warn("Can't enable PMU counter of type '0x%x': "
                 "No such event type.\n", ctr.eventId);
        }
    }
}


void
PMU::resetEventCounts()
{
    for (CounterState &ctr : counters)
        ctr.value = 0;
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
    ctr.value = val;
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
        updateCounter(reg_pmselr.sel, ctr);
    }
}

void
PMU::raiseInterrupt()
{
    RealView *rv(dynamic_cast<RealView *>(platform));
    if (!rv || !rv->gic) {
        warn_once("ARM PMU: GIC missing, can't raise interrupt.\n");
        return;
    }

    DPRINTF(PMUVerbose, "Delivering PMU interrupt.\n");
    rv->gic->sendInt(pmuInterrupt);
}

void
PMU::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm PMU\n");

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

    UNSERIALIZE_SCALAR(reg_pmcr);
    UNSERIALIZE_SCALAR(reg_pmcnten);
    UNSERIALIZE_SCALAR(reg_pmselr);
    UNSERIALIZE_SCALAR(reg_pminten);
    UNSERIALIZE_SCALAR(reg_pmovsr);

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

void
PMU::CounterState::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(eventId);
    SERIALIZE_SCALAR(value);
    SERIALIZE_SCALAR(enabled);
    SERIALIZE_SCALAR(overflow64);
}

void
PMU::CounterState::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(eventId);
    UNSERIALIZE_SCALAR(value);
    UNSERIALIZE_SCALAR(enabled);
    UNSERIALIZE_SCALAR(overflow64);
}

bool
PMU::CounterState::add(uint64_t delta)
{
    const uint64_t msb(1ULL << (overflow64 ? 63 : 31));
    const uint64_t old_value(value);

    value += delta;

    // Overflow if the msb goes from 1 to 0
    return (old_value & msb) && !(value & msb);
}

} // namespace ArmISA

ArmISA::PMU *
ArmPMUParams::create()
{
    return new ArmISA::PMU(this);
}
