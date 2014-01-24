/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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

#include <cassert>

#include "base/callback.hh"
#include "base/inifile.hh"
#include "base/match.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Checkpoint.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

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

//
// SimObject constructor: used to maintain static simObjectList
//
SimObject::SimObject(const Params *p)
    : EventManager(getEventQueue(p->eventq_index)), _params(p)
{
#ifdef DEBUG
    doDebugBreak = false;
#endif
    simObjectList.push_back(this);
    probeManager = new ProbeManager(this);
}

void
SimObject::init()
{
}

void
SimObject::loadState(Checkpoint *cp)
{
    if (cp->sectionExists(name())) {
        DPRINTF(Checkpoint, "unserializing\n");
        unserialize(cp, name());
    } else {
        DPRINTF(Checkpoint, "no checkpoint section found\n");
    }
}

void
SimObject::initState()
{
}

void
SimObject::startup()
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
SimObject::resetStats()
{
}

/**
 * No probe points by default, so do nothing in base.
 */
void
SimObject::regProbePoints()
{
}

/**
 * No probe listeners by default, so do nothing in base.
 */
void
SimObject::regProbeListeners()
{
}

ProbeManager *
SimObject::getProbeManager()
{
    return probeManager;
}

//
// static function: serialize all SimObjects.
//
void
SimObject::serializeAll(std::ostream &os)
{
    SimObjectList::reverse_iterator ri = simObjectList.rbegin();
    SimObjectList::reverse_iterator rend = simObjectList.rend();

    for (; ri != rend; ++ri) {
        SimObject *obj = *ri;
        obj->nameOut(os);
        obj->serialize(os);
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

unsigned int
SimObject::drain(DrainManager *drain_manager)
{
    setDrainState(Drained);
    return 0;
}


SimObject *
SimObject::find(const char *name)
{
    SimObjectList::const_iterator i = simObjectList.begin();
    SimObjectList::const_iterator end = simObjectList.end();

    for (; i != end; ++i) {
        SimObject *obj = *i;
        if (obj->name() == name)
            return obj;
    }

    return NULL;
}
