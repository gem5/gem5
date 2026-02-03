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

#include "systemc/core/event.hh"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <utility>

#include "systemc/core/module.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_module.hh"

namespace sc_gem5
{

namespace
{

EventsIt
findEventIn(Events &events, const std::string &name)
{
    EventsIt it;
    for (it = events.begin(); it != events.end(); it++)
        if (!strcmp((*it)->name(), name.c_str()))
            break;

    return it;
}

void
addEvent(Events *events, sc_core::sc_event *event)
{
    events->emplace(events->end(), event);
}

void
popEvent(Events* events, const std::string &name)
{
    EventsIt it = findEventIn(*events, name);
    assert(it != events->end());
    std::swap(events->back(), *it);
    events->pop_back();
}

std::shared_mutex globalEventLock;

} // anonymous namespace

Events topLevelEvents;
Events allEvents;

Event::Event(sc_core::sc_event *_sc_event, bool internal) :
    Event(_sc_event, nullptr, internal)
{}

Event::Event(sc_core::sc_event *_sc_event, const char *_basename_cstr,
        bool internal) :
    _sc_event(_sc_event), _basename(_basename_cstr ? _basename_cstr : ""),
    _inHierarchy(!internal), delayedNotify([this]() { this->notify(); }),
    _triggeredStamp(~0ULL)
{
    [[maybe_unused]] std::unique_lock lock(globalEventLock);

    if (_basename == "" && ::sc_core::sc_is_running())
        _basename = ::sc_core::sc_gen_unique_name("event");

    parent = internal ? nullptr : pickParentObj();

    if (internal) {
        _basename = globalNameGen.gen(_basename);
        _name = _basename;
    } else {
        std::string original_name = _basename;
        _basename = pickUniqueName(parent, _basename);

        if (parent) {
            Object *obj = Object::getFromScObject(parent);
            obj->addChildEvent(_sc_event);
        } else {
            addEvent(&topLevelEvents, _sc_event);
        }

        std::string path = parent ? (std::string(parent->name()) + ".") : "";

        if (original_name != "" && _basename != original_name) {
            std::string message = path + original_name +
                ". Latter declaration will be renamed to " +
                path + _basename;
            SC_REPORT_WARNING(sc_core::SC_ID_INSTANCE_EXISTS_,
                    message.c_str());
        }

        _name = path + _basename;
    }

    addEvent(&allEvents, _sc_event);

    // Determine if we're in the hierarchy (created once initialization starts
    // means no).
}

Event::~Event()
{
    [[maybe_unused]] std::unique_lock lock(globalEventLock);

    if (parent) {
        Object *obj = Object::getFromScObject(parent);
        obj->delChildEvent(_sc_event);
    } else if (inHierarchy()) {
        popEvent(&topLevelEvents, _name);
    }

    popEvent(&allEvents, _name);

    if (delayedNotify.scheduled())
        scheduler.deschedule(&delayedNotify);
}

const std::string &
Event::name() const
{
    return _name;
}

const std::string &
Event::basename() const
{
    return _basename;
}

bool
Event::inHierarchy() const
{
    return _inHierarchy;
}

sc_core::sc_object *
Event::getParentObject() const
{
    return parent;
}

void
Event::notify(StaticSensitivities &senses)
{
    for (auto s: senses)
        s->notify(this);
}

void
Event::notify(DynamicSensitivities &senses)
{
    int size = senses.size();
    int pos = 0;
    while (pos < size) {
        if (senses[pos]->notify(this))
            senses[pos] = senses[--size];
        else
            pos++;
    }
    senses.resize(size);
}

void
Event::notify()
{
    if (scheduler.inUpdate())
        SC_REPORT_ERROR(sc_core::SC_ID_IMMEDIATE_NOTIFICATION_, "");

    // An immediate notification overrides any pending delayed notification.
    if (delayedNotify.scheduled())
        scheduler.deschedule(&delayedNotify);

    _triggeredStamp = scheduler.changeStamp();
    notify(staticSenseMethod);
    notify(dynamicSenseMethod);
    notify(staticSenseThread);
    notify(dynamicSenseThread);
}

void
Event::notify(const sc_core::sc_time &t)
{
    if (delayedNotify.scheduled()) {
        if (scheduler.delayed(t) >= delayedNotify.when())
            return;

        scheduler.deschedule(&delayedNotify);
    }
    scheduler.schedule(&delayedNotify, t);
}

void
Event::notifyDelayed(const sc_core::sc_time &t)
{
    if (delayedNotify.scheduled())
        SC_REPORT_ERROR(sc_core::SC_ID_NOTIFY_DELAYED_, "");
    notify(t);
}

void
Event::cancel()
{
    if (delayedNotify.scheduled())
        scheduler.deschedule(&delayedNotify);
}

bool
Event::triggered() const
{
    return _triggeredStamp == scheduler.changeStamp();
}

void
Event::clearParent()
{
    [[maybe_unused]] std::unique_lock lock(globalEventLock);

    if (!parent)
        return;
    Object::getFromScObject(parent)->delChildEvent(_sc_event);
    parent = nullptr;
    addEvent(&topLevelEvents, _sc_event);
}

sc_core::sc_event *
findEvent(const char *name)
{
    [[maybe_unused]] std::shared_lock lock(globalEventLock);

    EventsIt it = findEventIn(allEvents, name);
    return it == allEvents.end() ? nullptr : *it;
}

} // namespace sc_gem5
