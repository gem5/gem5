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

#include "systemc/core/event.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_module.hh"

namespace sc_core
{


/*
 * sc_event_and_list
 */

sc_event_and_list::sc_event_and_list() : autoDelete(false), busy(0) {}

sc_event_and_list::sc_event_and_list(const sc_event_and_list &eal) :
    events(eal.events), autoDelete(false), busy(0)
{}

sc_event_and_list::sc_event_and_list(const sc_event &e) : sc_event_and_list()
{
    insert(e);
}

sc_event_and_list::sc_event_and_list(bool auto_delete) :
    autoDelete(auto_delete), busy(0)
{}

sc_event_and_list::~sc_event_and_list() {}

sc_event_and_list &
sc_event_and_list::operator = (const sc_event_and_list &eal)
{
    events = eal.events;
    return *this;
}

int
sc_event_and_list::size() const
{
    return events.size();
}

void
sc_event_and_list::swap(sc_event_and_list &eal)
{
    events.swap(eal.events);
}

sc_event_and_list &
sc_event_and_list::operator &= (const sc_event &e)
{
    insert(e);
    return *this;
}

sc_event_and_list &
sc_event_and_list::operator &= (const sc_event_and_list &eal)
{
    insert(eal);
    return *this;
}

sc_event_and_expr
sc_event_and_list::operator & (const sc_event &e) const
{
    sc_event_and_expr expr;
    expr.insert(*this);
    expr.insert(e);
    return expr;
}

sc_event_and_expr
sc_event_and_list::operator & (const sc_event_and_list &eal) const
{
    sc_event_and_expr expr;
    expr.insert(*this);
    expr.insert(eal);
    return expr;
}

void
sc_event_and_list::insert(sc_event const &e)
{
    events.insert(&e);
}

void
sc_event_and_list::insert(sc_event_and_list const &eal)
{
    events.insert(eal.events.begin(), eal.events.end());
}


/*
 * sc_event_or_list
 */

sc_event_or_list::sc_event_or_list() : autoDelete(false), busy(0) {}

sc_event_or_list::sc_event_or_list(const sc_event_or_list &eol) :
    events(eol.events), autoDelete(false), busy(0)
{}

sc_event_or_list::sc_event_or_list(const sc_event &e) : sc_event_or_list()
{
    insert(e);
}

sc_event_or_list::sc_event_or_list(bool auto_delete) :
    autoDelete(auto_delete), busy(0)
{}

sc_event_or_list &
sc_event_or_list::operator = (const sc_event_or_list &eol)
{
    events = eol.events;
    return *this;
}

sc_event_or_list::~sc_event_or_list() {}

int
sc_event_or_list::size() const
{
    return events.size();
}

void
sc_event_or_list::swap(sc_event_or_list &eol)
{
    events.swap(eol.events);
}

sc_event_or_list &
sc_event_or_list::operator |= (const sc_event &e)
{
    insert(e);
    return *this;
}

sc_event_or_list &
sc_event_or_list::operator |= (const sc_event_or_list &eol)
{
    insert(eol);
    return *this;
}

sc_event_or_expr
sc_event_or_list::operator | (const sc_event &e) const
{
    sc_event_or_expr expr;
    expr.insert(*this);
    expr.insert(e);
    return expr;
}

sc_event_or_expr
sc_event_or_list::operator | (const sc_event_or_list &eol) const
{
    sc_event_or_expr expr;
    expr.insert(*this);
    expr.insert(eol);
    return expr;
}

void
sc_event_or_list::insert(sc_event const &e)
{
    events.insert(&e);
}

void
sc_event_or_list::insert(sc_event_or_list const &eol)
{
    events.insert(eol.events.begin(), eol.events.end());
}


/*
 * sc_event_and_expr
 */

// Move semantics
sc_event_and_expr::sc_event_and_expr(sc_event_and_expr const &e) :
    list(e.list)
{
    e.list = nullptr;
}

sc_event_and_expr::operator const sc_event_and_list &() const
{
    sc_event_and_list *temp = list;
    list = nullptr;
    return *temp;
}

void
sc_event_and_expr::insert(sc_event const &e) const
{
    assert(list);
    list->insert(e);
}

void
sc_event_and_expr::insert(sc_event_and_list const &eal) const
{
    assert(list);
    list->insert(eal);
}

sc_event_and_expr::~sc_event_and_expr() { delete list; }

sc_event_and_expr::sc_event_and_expr() : list(new sc_event_and_list(true)) {}

sc_event_and_expr
operator & (sc_event_and_expr expr, sc_event const &e)
{
    expr.insert(e);
    return expr;
}

sc_event_and_expr
operator & (sc_event_and_expr expr, sc_event_and_list const &eal)
{
    expr.insert(eal);
    return expr;
}


/*
 * sc_event_or_expr
 */

// Move semantics
sc_event_or_expr::sc_event_or_expr(sc_event_or_expr const &e) :
    list(e.list)
{
    e.list = nullptr;
}

sc_event_or_expr::operator const sc_event_or_list &() const
{
    sc_event_or_list *temp = list;
    list = NULL;
    return *temp;
}

void
sc_event_or_expr::insert(sc_event const &e) const
{
    assert(list);
    list->insert(e);
}

void
sc_event_or_expr::insert(sc_event_or_list const &eol) const
{
    assert(list);
    list->insert(eol);
}

sc_event_or_expr::~sc_event_or_expr() { delete list; }

sc_event_or_expr::sc_event_or_expr() : list(new sc_event_or_list(true)) {}

sc_event_or_expr
operator | (sc_event_or_expr expr, sc_event const &e)
{
    expr.insert(e);
    return expr;
}

sc_event_or_expr
operator | (sc_event_or_expr expr, sc_event_or_list const &eol)
{
    expr.insert(eol);
    return expr;
}


/*
 * sc_event
 */

sc_event::sc_event() :
    _gem5_event(new ::sc_gem5::Event(
                this, sc_core::sc_gen_unique_name("event")))
{}

sc_event::sc_event(const char *_name) :
    _gem5_event(new ::sc_gem5::Event(this, _name))
{}

sc_event::~sc_event() { delete _gem5_event; }

const char *sc_event::name() const { return _gem5_event->name().c_str(); }
const char *
sc_event::basename() const
{
    return _gem5_event->basename().c_str();
}
bool sc_event::in_hierarchy() const { return _gem5_event->inHierarchy(); }

sc_object *
sc_event::get_parent_object() const
{
    return _gem5_event->getParentObject();
}

void sc_event::notify() { _gem5_event->notify(); }
void sc_event::notify(const sc_time &t) { _gem5_event->notify(t); }
void sc_event::notify(double d, sc_time_unit u) { _gem5_event->notify(d, u); }
void sc_event::cancel() { _gem5_event->cancel(); }
bool sc_event::triggered() const { return _gem5_event->triggered(); }
void
sc_event::notify_delayed()
{
    _gem5_event->notifyDelayed(SC_ZERO_TIME);
}
void
sc_event::notify_delayed(const sc_time &t)
{
    _gem5_event->notifyDelayed(t);
}

sc_event_and_expr
sc_event::operator & (const sc_event &e) const
{
    sc_event_and_expr expr;
    expr.insert(*this);
    expr.insert(e);
    return expr;
}

sc_event_and_expr
sc_event::operator & (const sc_event_and_list &eal) const
{
    sc_event_and_expr expr;
    expr.insert(*this);
    expr.insert(eal);
    return expr;
}

sc_event_or_expr
sc_event::operator | (const sc_event &e) const
{
    sc_event_or_expr expr;
    expr.insert(*this);
    expr.insert(e);
    return expr;
}

sc_event_or_expr
sc_event::operator | (const sc_event_or_list &eol) const
{
    sc_event_or_expr expr;
    expr.insert(*this);
    expr.insert(eol);
    return expr;
}

sc_event::sc_event(bool) :
    _gem5_event(new ::sc_gem5::Event(
                this, sc_core::sc_gen_unique_name(
                    "$$$internal kernel event$$$"), true))
{}

sc_event::sc_event(bool, const char *_name) :
    _gem5_event(new ::sc_gem5::Event(
                this,
                (std::string("$$$internal kernel event$$$") + _name).c_str(),
                true))
{}

const std::vector<sc_event *> &
sc_get_top_level_events()
{
    return ::sc_gem5::topLevelEvents;
}

sc_event *
sc_find_event(const char *name)
{
    std::string str(name);
    ::sc_gem5::EventsIt it = ::sc_gem5::findEvent(str);
    return it == ::sc_gem5::allEvents.end() ? nullptr : *it;
}

} // namespace sc_core

namespace sc_gem5
{

InternalScEvent::InternalScEvent() : sc_event(true) {}
InternalScEvent::InternalScEvent(const char *_name) : sc_event(true, _name) {}

} // namespace sc_gem5
