/*
 * Copyright 2018 Google, Inc.
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

#include "systemc/core/kernel.hh"

#include "base/logging.hh"
#include "systemc/core/channel.hh"
#include "systemc/core/module.hh"
#include "systemc/core/port.hh"
#include "systemc/core/sc_main_fiber.hh"
#include "systemc/core/scheduler.hh"

namespace sc_gem5
{

namespace
{

bool stopAfterCallbacks = false;
bool startComplete = false;
bool endComplete = false;

sc_core::sc_status _status = sc_core::SC_ELABORATION;

} // anonymous namespace

bool Kernel::startOfSimulationComplete() { return startComplete; }
bool Kernel::endOfSimulationComplete() { return endComplete; }

sc_core::sc_status Kernel::status() { return _status; }
void Kernel::status(sc_core::sc_status s) { _status = s; }

Kernel::Kernel(const Params &params, int) :
    gem5::SimObject(params),
    t0Event(this, false, gem5::EventBase::Default_Pri - 1)
{
    // Install ourselves as the scheduler's event manager.
    ::sc_gem5::scheduler.setEventQueue(eventQueue());
}

void
Kernel::init()
{
    if (scMainFiber.finished())
        return;

    if (stopAfterCallbacks)
        fatal("Simulation called sc_stop during elaboration.\n");

    status(::sc_core::SC_BEFORE_END_OF_ELABORATION);
    for (auto p: allPorts)
        p->sc_port_base()->before_end_of_elaboration();
    for (auto m: sc_gem5::allModules)
        m->beforeEndOfElaboration();
    for (auto c: sc_gem5::allChannels)
        c->sc_chan()->before_end_of_elaboration();

    ::sc_gem5::scheduler.elaborationDone(true);
}

void
Kernel::regStats()
{
    gem5::SimObject::regStats();

    if (scMainFiber.finished() || stopAfterCallbacks)
        return;

    try {
        for (auto p: allPorts)
            p->finalize();
        for (auto p: allPorts)
            p->regPort();

        status(::sc_core::SC_END_OF_ELABORATION);
        for (auto p: allPorts)
            p->sc_port_base()->end_of_elaboration();
        for (auto m: sc_gem5::allModules)
            m->endOfElaboration();
        for (auto c: sc_gem5::allChannels)
            c->sc_chan()->end_of_elaboration();
    } catch (...) {
        ::sc_gem5::scheduler.throwUp();
    }
}

void
Kernel::startup()
{
    if (scMainFiber.finished())
        return;

    schedule(t0Event, gem5::curTick());

    if (stopAfterCallbacks)
        return;

    try {
        status(::sc_core::SC_START_OF_SIMULATION);
        for (auto p: allPorts)
            p->sc_port_base()->start_of_simulation();
        for (auto m: sc_gem5::allModules)
            m->startOfSimulation();
        for (auto c: sc_gem5::allChannels)
            c->sc_chan()->start_of_simulation();
    } catch (...) {
        ::sc_gem5::scheduler.throwUp();
    }

    startComplete = true;

    if (stopAfterCallbacks)
        stopWork();

    kernel->status(::sc_core::SC_RUNNING);
}

void
Kernel::stop()
{
    if (status() < ::sc_core::SC_RUNNING)
        stopAfterCallbacks = true;
    else
        stopWork();
}

void
Kernel::stopWork()
{
    status(::sc_core::SC_END_OF_SIMULATION);
    try {
        for (auto p: allPorts)
            p->sc_port_base()->end_of_simulation();
        for (auto m: sc_gem5::allModules)
            m->endOfSimulation();
        for (auto c: sc_gem5::allChannels)
            c->sc_chan()->end_of_simulation();
    } catch (...) {
        ::sc_gem5::scheduler.throwUp();
    }

    endComplete = true;

    status(::sc_core::SC_STOPPED);
}

void
Kernel::t0Handler()
{
    if (stopAfterCallbacks) {
        scheduler.clear();
        ::sc_gem5::scheduler.initPhase();
        scheduler.scheduleStop(false);
    } else {
        ::sc_gem5::scheduler.initPhase();
        status(::sc_core::SC_RUNNING);
    }
}

Kernel *kernel;

} // namespace sc_gem5

sc_gem5::Kernel *
gem5::SystemC_KernelParams::create() const
{
    using namespace gem5;
    panic_if(sc_gem5::kernel,
            "Only one systemc kernel object may be defined.\n");
    sc_gem5::kernel = new sc_gem5::Kernel(*this, 0);
    return sc_gem5::kernel;
}
