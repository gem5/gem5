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

#include "systemc/core/sensitivity.hh"

#include "systemc/core/event.hh"
#include "systemc/core/port.hh"
#include "systemc/core/process.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/sc_in.hh"
#include "systemc/ext/channel/sc_inout.hh"
#include "systemc/ext/channel/sc_out.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_export.hh"
#include "systemc/ext/core/sc_interface.hh"
#include "systemc/ext/core/sc_port.hh"

namespace sc_gem5
{

/*
 * Common sensitivity interface.
 */

void
Sensitivity::satisfy()
{
    process->satisfySensitivity(this);
}

bool
Sensitivity::notifyWork(Event *e)
{
    satisfy();
    return true;
}

bool
Sensitivity::notify(Event *e)
{
    if (scheduler.current() == process) {
        static bool warned = false;
        if (!warned) {
            SC_REPORT_WARNING(sc_core::SC_ID_IMMEDIATE_SELF_NOTIFICATION_,
                    process->name());
            warned = true;
        }
        return false;
    }

    if (process->disabled())
        return false;

    return notifyWork(e);
}

bool
Sensitivity::ofMethod()
{
    return process->procKind() == sc_core::SC_METHOD_PROC_;
}


/*
 * Dynamic vs. static sensitivity.
 */

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


/*
 * Static sensitivities.
 */

void
newStaticSensitivityEvent(Process *p, const sc_core::sc_event *e)
{
    auto s = new StaticSensitivityEvent(p, e);
    s->addToEvent(s->event);
    p->addStatic(s);
}

void
newStaticSensitivityInterface(Process *p, const sc_core::sc_interface *i)
{
    auto s = new StaticSensitivityInterface(p, i);
    s->addToEvent(s->event);
    p->addStatic(s);
}

void
newStaticSensitivityPort(Process *p, const sc_core::sc_port_base *pb)
{
    auto s = new StaticSensitivityPort(p);
    Port *port = Port::fromPort(pb);
    port->sensitive(s);
    p->addStatic(s);
}

void
newStaticSensitivityExport(Process *p, const sc_core::sc_export_base *exp)
{
    auto s = new StaticSensitivityExport(p, exp);
    s->addToEvent(s->event);
    p->addStatic(s);
}

void
newStaticSensitivityFinder(Process *p, const sc_core::sc_event_finder *f)
{
    auto s = new StaticSensitivityFinder(p, f);
    Port *port = Port::fromPort(f->port());
    port->sensitive(s);
    p->addStatic(s);
}


StaticSensitivityInterface::StaticSensitivityInterface(
        Process *p, const sc_core::sc_interface *i) :
    Sensitivity(p), StaticSensitivity(p),
    SensitivityEvent(p, &i->default_event())
{}

StaticSensitivityExport::StaticSensitivityExport(
        Process *p, const sc_core::sc_export_base *exp) :
    Sensitivity(p), StaticSensitivity(p),
    SensitivityEvent(p, &exp->get_interface()->default_event())
{}

const ::sc_core::sc_event &
StaticSensitivityFinder::find(::sc_core::sc_interface *i)
{
    return finder->find_event(i);
}


/*
 * Dynamic sensitivities.
 */

void
newDynamicSensitivityEvent(Process *p, const sc_core::sc_event *e)
{
    auto s = new DynamicSensitivityEvent(p, e);
    s->addToEvent(s->event);
    p->setDynamic(s);
}

void
newDynamicSensitivityEventOrList(
        Process *p, const sc_core::sc_event_or_list *eol)
{
    auto s = new DynamicSensitivityEventOrList(p, eol);
    for (auto event: s->events)
        s->addToEvent(event);
    p->setDynamic(s);
}

void newDynamicSensitivityEventAndList(
        Process *p, const sc_core::sc_event_and_list *eal)
{
    auto s = new DynamicSensitivityEventAndList(p, eal);
    for (auto event: s->events)
        s->addToEvent(event);
    p->setDynamic(s);
}


DynamicSensitivityEventOrList::DynamicSensitivityEventOrList(
        Process *p, const sc_core::sc_event_or_list *eol) :
    Sensitivity(p), DynamicSensitivity(p), SensitivityEvents(p, eol->events)
{}

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

DynamicSensitivityEventAndList::DynamicSensitivityEventAndList(
        Process *p, const sc_core::sc_event_and_list *eal) :
    Sensitivity(p), DynamicSensitivity(p), SensitivityEvents(p, eal->events)
{}

bool
DynamicSensitivityEventAndList::notifyWork(Event *e)
{
    events.erase(e->sc_event());

    // This sensitivity is satisfied if all events have triggered.
    if (events.empty())
        satisfy();

    return true;
}

} // namespace sc_gem5
