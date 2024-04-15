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

#ifndef __SYSTEMC_CORE_EVENT_HH__
#define __SYSTEMC_CORE_EVENT_HH__

#include <list>
#include <string>
#include <vector>

#include "sim/eventq.hh"
#include "systemc/core/list.hh"
#include "systemc/core/object.hh"
#include "systemc/core/process.hh"
#include "systemc/core/sched_event.hh"
#include "systemc/core/sensitivity.hh"
#include "systemc/ext/core/sc_prim.hh"
#include "systemc/ext/core/sc_time.hh"

namespace sc_core
{

class sc_event;

} // namespace sc_core

namespace sc_gem5
{

typedef std::vector<sc_core::sc_event *> Events;

class Sensitivity;

class Event
{
  public:
    Event(sc_core::sc_event *_sc_event, bool internal = false);
    Event(sc_core::sc_event *_sc_event, const char *_basename,
          bool internal = false);

    ~Event();

    sc_core::sc_event *
    sc_event()
    {
        return _sc_event;
    }

    const std::string &name() const;
    const std::string &basename() const;
    bool inHierarchy() const;
    sc_core::sc_object *getParentObject() const;

    void notify(StaticSensitivities &senses);
    void notify(DynamicSensitivities &senses);

    void notify();
    void notify(const sc_core::sc_time &t);

    void
    notify(double d, sc_core::sc_time_unit &u)
    {
        notify(sc_core::sc_time(d, u));
    }

    void notifyDelayed(const sc_core::sc_time &t);
    void cancel();

    bool triggered() const;

    uint64_t
    triggeredStamp() const
    {
        return _triggeredStamp;
    }

    static Event *
    getFromScEvent(sc_core::sc_event *e)
    {
        return e->_gem5_event;
    }

    static const Event *
    getFromScEvent(const sc_core::sc_event *e)
    {
        return e->_gem5_event;
    }

    void
    addSensitivity(StaticSensitivity *s) const
    {
        // Insert static sensitivities in reverse order to match Accellera's
        // implementation.
        auto &senses = s->ofMethod() ? staticSenseMethod : staticSenseThread;
        senses.insert(senses.begin(), s);
    }

    void
    delSensitivity(StaticSensitivity *s) const
    {
        auto &senses = s->ofMethod() ? staticSenseMethod : staticSenseThread;
        for (auto &t : senses) {
            if (t == s) {
                t = senses.back();
                senses.pop_back();
                break;
            }
        }
    }

    void
    addSensitivity(DynamicSensitivity *s) const
    {
        auto &senses = s->ofMethod() ? dynamicSenseMethod : dynamicSenseThread;
        senses.push_back(s);
    }

    void
    delSensitivity(DynamicSensitivity *s) const
    {
        auto &senses = s->ofMethod() ? dynamicSenseMethod : dynamicSenseThread;
        for (auto &t : senses) {
            if (t == s) {
                t = senses.back();
                senses.pop_back();
                break;
            }
        }
    }

    void clearParent();

  private:
    sc_core::sc_event *_sc_event;

    std::string _basename;
    std::string _name;
    bool _inHierarchy;

    sc_core::sc_object *parent;

    ScEvent delayedNotify;
    mutable uint64_t _triggeredStamp;

    mutable StaticSensitivities staticSenseMethod;
    mutable StaticSensitivities staticSenseThread;
    mutable DynamicSensitivities dynamicSenseMethod;
    mutable DynamicSensitivities dynamicSenseThread;
};

extern Events topLevelEvents;
extern Events allEvents;

EventsIt findEvent(const std::string &name);

} // namespace sc_gem5

#endif //__SYSTEMC_CORE_EVENT_HH__
