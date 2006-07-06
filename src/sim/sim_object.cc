/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * All rights reserved.
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
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#include <assert.h>

#include "base/callback.hh"
#include "base/inifile.hh"
#include "base/match.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "base/stats/events.hh"
#include "base/serializer.hh"
#include "sim/host.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "sim/param.hh"

using namespace std;


////////////////////////////////////////////////////////////////////////
//
// SimObject member definitions
//
////////////////////////////////////////////////////////////////////////

//
// static list of all SimObjects, used for initialization etc.
//
SimObject::SimObjectList SimObject::simObjectList;

namespace Stats {
    extern ObjectMatch event_ignore;
}

//
// SimObject constructor: used to maintain static simObjectList
//
SimObject::SimObject(Params *p)
    : _params(p)
{
#ifdef DEBUG
    doDebugBreak = false;
#endif

    doRecordEvent = !Stats::event_ignore.match(name());
    simObjectList.push_back(this);
    state = Atomic;
}

//
// SimObject constructor: used to maintain static simObjectList
//
SimObject::SimObject(const string &_name)
    : _params(new Params)
{
    _params->name = _name;
#ifdef DEBUG
    doDebugBreak = false;
#endif

    doRecordEvent = !Stats::event_ignore.match(name());
    simObjectList.push_back(this);
    state = Atomic;
}

void
SimObject::connect()
{
}

void
SimObject::init()
{
}

//
// no default statistics, so nothing to do in base implementation
//
void
SimObject::regStats()
{
}

void
SimObject::regFormulas()
{
}

void
SimObject::resetStats()
{
}

//
// static function:
//   call regStats() on all SimObjects and then regFormulas() on all
//   SimObjects.
//
struct SimObjectResetCB : public Callback
{
    virtual void process() { SimObject::resetAllStats(); }
};

namespace {
    static SimObjectResetCB StatResetCB;
}

void
SimObject::regAllStats()
{
    SimObjectList::iterator i;
    SimObjectList::iterator end = simObjectList.end();

    /**
     * @todo change cprintfs to DPRINTFs
     */
    for (i = simObjectList.begin(); i != end; ++i) {
#ifdef STAT_DEBUG
        cprintf("registering stats for %s\n", (*i)->name());
#endif
        (*i)->regStats();
    }

    for (i = simObjectList.begin(); i != end; ++i) {
#ifdef STAT_DEBUG
        cprintf("registering formulas for %s\n", (*i)->name());
#endif
        (*i)->regFormulas();
    }

    Stats::registerResetCallback(&StatResetCB);
}

//
// static function: call connect() on all SimObjects.
//
void
SimObject::connectAll()
{
    SimObjectList::iterator i = simObjectList.begin();
    SimObjectList::iterator end = simObjectList.end();

    for (; i != end; ++i) {
        SimObject *obj = *i;
        obj->connect();
    }
}

//
// static function: call init() on all SimObjects.
//
void
SimObject::initAll()
{
    SimObjectList::iterator i = simObjectList.begin();
    SimObjectList::iterator end = simObjectList.end();

    for (; i != end; ++i) {
        SimObject *obj = *i;
        obj->init();
    }
}

//
// static function: call resetStats() on all SimObjects.
//
void
SimObject::resetAllStats()
{
    SimObjectList::iterator i = simObjectList.begin();
    SimObjectList::iterator end = simObjectList.end();

    for (; i != end; ++i) {
        SimObject *obj = *i;
        obj->resetStats();
    }
}

//
// static function: serialize all SimObjects.
//
void
SimObject::serializeAll(ostream &os)
{
    SimObjectList::reverse_iterator ri = simObjectList.rbegin();
    SimObjectList::reverse_iterator rend = simObjectList.rend();

    for (; ri != rend; ++ri) {
        SimObject *obj = *ri;
        obj->nameOut(os);
        obj->serialize(os);
   }
}

void
SimObject::unserializeAll(Checkpoint *cp)
{
    SimObjectList::reverse_iterator ri = simObjectList.rbegin();
    SimObjectList::reverse_iterator rend = simObjectList.rend();

    for (; ri != rend; ++ri) {
        SimObject *obj = *ri;
        DPRINTFR(Config, "Unserializing '%s'\n",
                 obj->name());
        if(cp->sectionExists(obj->name()))
            obj->unserialize(cp, obj->name());
        else
            warn("Not unserializing '%s': no section found in checkpoint.\n",
                 obj->name());
   }
}

#ifdef DEBUG
//
// static function: flag which objects should have the debugger break
//
void
SimObject::debugObjectBreak(const string &objs)
{
    SimObjectList::const_iterator i = simObjectList.begin();
    SimObjectList::const_iterator end = simObjectList.end();

    ObjectMatch match(objs);
    for (; i != end; ++i) {
        SimObject *obj = *i;
        obj->doDebugBreak = match.match(obj->name());
   }
}

void
debugObjectBreak(const char *objs)
{
    SimObject::debugObjectBreak(string(objs));
}
#endif

void
SimObject::recordEvent(const std::string &stat)
{
    if (doRecordEvent)
        Stats::recordEvent(stat);
}

bool
SimObject::quiesce(Event *quiesce_event)
{
    if (state != QuiescedAtomic && state != Atomic) {
        panic("Must implement your own quiesce function if it is to be used "
              "in timing mode!");
    }
    state = QuiescedAtomic;
    return false;
}

void
SimObject::resume()
{
    if (state == QuiescedAtomic) {
        state = Atomic;
    } else if (state == QuiescedTiming) {
        state = Timing;
    }
}

void
SimObject::setMemoryMode(State new_mode)
{
    assert(new_mode == Timing || new_mode == Atomic);
    if (state == QuiescedAtomic && new_mode == Timing) {
        state = QuiescedTiming;
    } else if (state == QuiescedTiming && new_mode == Atomic) {
        state = QuiescedAtomic;
    } else {
        state = new_mode;
    }
}

void
SimObject::switchOut()
{
    panic("Unimplemented!");
}

void
SimObject::takeOverFrom(BaseCPU *cpu)
{
    panic("Unimplemented!");
}

DEFINE_SIM_OBJECT_CLASS_NAME("SimObject", SimObject)
