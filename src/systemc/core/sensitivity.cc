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
 *
 * Authors: Gabe Black
 */

#include "systemc/core/sensitivity.hh"

#include "systemc/core/event.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/sc_export.hh"
#include "systemc/ext/core/sc_interface.hh"
#include "systemc/ext/core/sc_port.hh"

namespace sc_gem5
{

void
Sensitivity::satisfy()
{
    process->satisfySensitivity(this);
}

bool
Sensitivity::notify(Event *e)
{
    if (process->disabled())
        return false;
    return notifyWork(e);
}


void
DynamicSensitivity::addToEvent(const ::sc_core::sc_event *e)
{
    Event::getFromScEvent(e)->addSensitivity(this);
}

void
DynamicSensitivity::delFromEvent(const ::sc_core::sc_event *e)
{
    Event::getFromScEvent(e)->delSensitivity(this);
}

void
StaticSensitivity::addToEvent(const ::sc_core::sc_event *e)
{
    Event::getFromScEvent(e)->addSensitivity(this);
}

void
StaticSensitivity::delFromEvent(const ::sc_core::sc_event *e)
{
    Event::getFromScEvent(e)->delSensitivity(this);
}

void
StaticSensitivityInterface::finalize()
{
    event = &interface->default_event();
    SensitivityEvent::finalize();
}

void
StaticSensitivityPort::finalize()
{
    for (int i = 0; i < port->size(); i++) {
        const ::sc_core::sc_event *event =
            &port->_gem5Interface(i)->default_event();
        events.insert(event);
        addToEvent(event);
    }
}

void
StaticSensitivityExport::finalize()
{
    event = &exp->get_interface()->default_event();
    SensitivityEvent::finalize();
}

void
StaticSensitivityFinder::finalize()
{
    const ::sc_core::sc_port_base *port = finder->port();
    int size = port->size();
    for (int i = 0; i < size; i++) {
        ::sc_core::sc_interface *interface = port->_gem5Interface(i);
        const ::sc_core::sc_event *event = &finder->find_event(interface);
        events.insert(event);
        addToEvent(event);
    }
}

bool
DynamicSensitivityEventOrList::notifyWork(Event *e)
{
    events.erase(e->sc_event());

    // All the other events need this deleted from their lists since this
    // sensitivity has been satisfied without them triggering.
    for (auto le: events)
        delFromEvent(le);

    satisfy();
    return true;
}

DynamicSensitivityEventOrList::DynamicSensitivityEventOrList(
        Process *p, const sc_core::sc_event_or_list *eol) :
    Sensitivity(p), DynamicSensitivity(p), events(eol->events)
{}

void
DynamicSensitivityEventOrList::finalize()
{
    for (auto e: events)
        addToEvent(e);
}

void
DynamicSensitivityEventOrList::clear()
{
    for (auto e: events)
        delFromEvent(e);
}

bool
DynamicSensitivityEventAndList::notifyWork(Event *e)
{
    events.erase(e->sc_event());

    // This sensitivity is satisfied if all events have triggered.
    if (events.empty())
        satisfy();

    return true;
}

DynamicSensitivityEventAndList::DynamicSensitivityEventAndList(
        Process *p, const sc_core::sc_event_and_list *eal) :
    Sensitivity(p), DynamicSensitivity(p), events(eal->events)
{}

void
DynamicSensitivityEventAndList::finalize()
{
    for (auto e: events)
        addToEvent(e);
}

void
DynamicSensitivityEventAndList::clear()
{
    for (auto e: events)
        delFromEvent(e);
}

} // namespace sc_gem5
