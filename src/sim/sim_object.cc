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
 */

#include "sim/sim_object.hh"

#include <cassert>

#include "base/logging.hh"
#include "base/match.hh"
#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

////////////////////////////////////////////////////////////////////////
//
// SimObject member definitions
//
////////////////////////////////////////////////////////////////////////

//
// static list of all SimObjects, used for initialization etc.
//
SimObject::SimObjectList SimObject::simObjectList;
SimObjectResolver *SimObject::_objNameResolver = NULL;

//
// SimObject constructor: used to maintain static simObjectList
//
SimObject::SimObject(const Params &p)
    : EventManager(getEventQueue(p.eventq_index)),
      statistics::Group(nullptr),
      Named(p.name),
      _params(p)
{
    simObjectList.push_back(this);
    probeManager = new ProbeManager(this);
}

SimObject::~SimObject() { delete probeManager; }

void
SimObject::init()
{}

void
SimObject::loadState(CheckpointIn &cp)
{
    if (cp.sectionExists(name())) {
        DPRINTF(Checkpoint, "unserializing\n");
        // This works despite name() returning a fully qualified name
        // since we are at the top level.
        unserializeSection(cp, name());
    } else {
        DPRINTF(Checkpoint, "no checkpoint section found\n");
    }
}

void
SimObject::initState()
{}

void
SimObject::startup()
{}

/**
 * No probe points by default, so do nothing in base.
 */
void
SimObject::regProbePoints()
{}

/**
 * No probe listeners by default, so do nothing in base.
 */
void
SimObject::regProbeListeners()
{}

ProbeManager *
SimObject::getProbeManager()
{
    return probeManager;
}

Port &
SimObject::getPort(const std::string &if_name, PortID idx)
{
    fatal("%s does not have any port named %s\n", name(), if_name);
}

//
// static function: serialize all SimObjects.
//
void
SimObject::serializeAll(const std::string &cpt_dir)
{
    std::ofstream cp;
    Serializable::generateCheckpointOut(cpt_dir, cp);

    SimObjectList::reverse_iterator ri = simObjectList.rbegin();
    SimObjectList::reverse_iterator rend = simObjectList.rend();

    for (; ri != rend; ++ri) {
        SimObject *obj = *ri;
        // This works despite name() returning a fully qualified name
        // since we are at the top level.
        obj->serializeSection(cp, obj->name());
    }
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

void
SimObject::setSimObjectResolver(SimObjectResolver *resolver)
{
    assert(!_objNameResolver);
    _objNameResolver = resolver;
}

SimObjectResolver *
SimObject::getSimObjectResolver()
{
    assert(_objNameResolver);
    return _objNameResolver;
}

void
objParamIn(CheckpointIn &cp, const std::string &name, SimObject *&param)
{
    const std::string &section(Serializable::currentSection());
    std::string path;
    if (!cp.find(section, name, path)) {
        fatal("Can't unserialize '%s:%s'\n", section, name);
    }
    param = SimObject::getSimObjectResolver()->resolveSimObject(path);
}

void
debug_serialize(const std::string &cpt_dir)
{
    SimObject::serializeAll(cpt_dir);
}

} // namespace gem5
