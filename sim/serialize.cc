/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <sys/time.h>

#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/str.hh"

#include "sim/eventq.hh"
#include "sim/param.hh"
#include "sim/serialize.hh"
#include "base/inifile.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "base/trace.hh"
#include "sim/config_node.hh"

using namespace std;

Serializer *Serializeable::serializer = NULL;

Serializeable::Serializeable()
    : serialized(false)
{ }

Serializeable::~Serializeable()
{ }

void
Serializeable::mark()
{
    if (!serialized)
        serializer->add_object(this);

    serialized = true;
}

void
Serializeable::nameOut(ostream &os)
{
    os << "\n[" << name() << "]\n";
}

void
Serializeable::nameOut(ostream &os, const string &_name)
{
    os << "\n[" << _name << "]\n";
}

template <class T>
void
paramOut(ostream &os, const std::string &name, const T& param)
{
    os << name << "=";
    showParam(os, param);
    os << "\n";
}


template <class T>
void
paramIn(Checkpoint *cp, const std::string &section,
        const std::string &name, T& param)
{
    std::string str;
    if (!cp->find(section, name, str) || !parseParam(str, param)) {
        fatal("Can't unserialize '%s:%s'\n", section, name);
    }
}


template <class T>
void
arrayParamOut(ostream &os, const std::string &name,
              const T *param, int size)
{
    os << name << "=";
    if (size > 0)
        showParam(os, param[0]);
    for (int i = 1; i < size; ++i) {
        os << " ";
        showParam(os, param[i]);
    }
    os << "\n";
}


template <class T>
void
arrayParamIn(Checkpoint *cp, const std::string &section,
             const std::string &name, T *param, int size)
{
    std::string str;
    if (!cp->find(section, name, str)) {
        fatal("Can't unserialize '%s:%s'\n", section, name);
    }

    // code below stolen from VectorParam<T>::parse().
    // it would be nice to unify these somehow...

    vector<string> tokens;

    tokenize(tokens, str, ' ');

    // Need this if we were doing a vector
    // value.resize(tokens.size());

    if (tokens.size() != size) {
        fatal("Array size mismatch on %s:%s'\n", section, name);
    }

    for (int i = 0; i < tokens.size(); i++) {
        // need to parse into local variable to handle vector<bool>,
        // for which operator[] returns a special reference class
        // that's not the same as 'bool&', (since it's a packed
        // vector)
        T scalar_value;
        if (!parseParam(tokens[i], scalar_value)) {
            string err("could not parse \"");

            err += str;
            err += "\"";

            fatal(err);
        }

        // assign parsed value to vector
        param[i] = scalar_value;
    }
}


void
objParamIn(Checkpoint *cp, const std::string &section,
           const std::string &name, Serializeable * &param)
{
    if (!cp->findObj(section, name, param)) {
        fatal("Can't unserialize '%s:%s'\n", section, name);
    }
}


#define INSTANTIATE_PARAM_TEMPLATES(type)				\
template void								\
paramOut(ostream &os, const std::string &name, type const &param);	\
template void								\
paramIn(Checkpoint *cp, const std::string &section,			\
        const std::string &name, type & param);				\
template void								\
arrayParamOut(ostream &os, const std::string &name,			\
              type const *param, int size);				\
template void								\
arrayParamIn(Checkpoint *cp, const std::string &section,		\
             const std::string &name, type *param, int size);


INSTANTIATE_PARAM_TEMPLATES(int8_t)
INSTANTIATE_PARAM_TEMPLATES(uint8_t)
INSTANTIATE_PARAM_TEMPLATES(int16_t)
INSTANTIATE_PARAM_TEMPLATES(uint16_t)
INSTANTIATE_PARAM_TEMPLATES(int32_t)
INSTANTIATE_PARAM_TEMPLATES(uint32_t)
INSTANTIATE_PARAM_TEMPLATES(int64_t)
INSTANTIATE_PARAM_TEMPLATES(uint64_t)
INSTANTIATE_PARAM_TEMPLATES(bool)
INSTANTIATE_PARAM_TEMPLATES(string)


#if 0
// unneeded?
void
Serializeable::childOut(const string &name, Serializeable *child)
{
    child->mark();
    if (child->name() == "")
        panic("child is unnamed");

    out() << name << "=" << child->name() << "\n";
}
#endif

Serializer::Serializer()
{ }

Serializer::~Serializer()
{ }

ostream &
Serializer::out() const
{
    if (!output)
        panic("must set output before serializing");

    return *output;
}

void
Serializer::add_object(Serializeable *obj)
{
    objects.push_back(obj);
}

void
Serializer::add_objects()
{
    mainEventQueue.mark();

    SimObject::SimObjectList::iterator i = SimObject::simObjectList.begin();
    SimObject::SimObjectList::iterator end = SimObject::simObjectList.end();

    while (i != end) {
        (*i)->mark();
        ++i;
    }
}

void
Serializer::serialize()
{
    if (Serializeable::serializer != NULL)
        panic("in process of serializing!");

    Serializeable::serializer = this;

    file = CheckpointFile();
    string cpt_file = file + ".cpt";
    output = new ofstream(cpt_file.c_str());
    time_t t = time(NULL);
    *output << "// checkpoint generated: " << ctime(&t);

    serlist_t list;

    add_objects();
    while (!objects.empty()) {
        Serializeable *obj = objects.front();
        DPRINTF(Serialize, "Serializing %s\n", obj->name());
        obj->nameOut(out());
        obj->serialize(out());
        objects.pop_front();
        list.push_back(obj);
    }

    while (!list.empty()) {
        list.front()->serialized = false;
        list.pop_front();
    }

    Serializeable::serializer = NULL;

    delete output;
    output = NULL;
    file = "";
}

class SerializeEvent : public Event
{
  protected:
    string file;
    Tick repeat;

  public:
    SerializeEvent(Tick _when, Tick _repeat);
    virtual void process();
    virtual void serialize(std::ostream &os)
    {
        panic("Cannot serialize the SerializeEvent");
    }

};

SerializeEvent::SerializeEvent(Tick _when, Tick _repeat)
    : Event(&mainEventQueue, 990), repeat(_repeat)
{
    setFlags(AutoDelete);
    schedule(_when);
}

void
SerializeEvent::process()
{
    Serializer serial;
    serial.serialize();
    if (repeat)
        schedule(curTick + repeat);
}

string __CheckpointFileBase;

string
CheckpointFile()
{
    if (__CheckpointFileBase.empty())
        return __CheckpointFileBase;

    return csprintf("%s.%d", __CheckpointFileBase, curTick);
}

void
SetupCheckpoint(Tick when, Tick period)
{
    new SerializeEvent(when, period);
}

class SerializeParamContext : public ParamContext
{
  private:
    SerializeEvent *event;

  public:
    SerializeParamContext(const string &section);
    ~SerializeParamContext();
    void checkParams();
};

SerializeParamContext serialParams("serialize");

Param<string> serialize_file(&serialParams,
                             "file",
                             "file to write to", "m5");

Param<Counter> serialize_cycle(&serialParams,
                                "cycle",
                                "cycle to serialize",
                                0);

Param<Counter> serialize_period(&serialParams,
                                "period",
                                "period to repeat serializations",
                                0);



SerializeParamContext::SerializeParamContext(const string &section)
    : ParamContext(section), event(NULL)
{ }

SerializeParamContext::~SerializeParamContext()
{
}

void
SerializeParamContext::checkParams()
{
    __CheckpointFileBase = serialize_file;
    if (serialize_cycle > 0)
        SetupCheckpoint(serialize_cycle, serialize_period);
}

void
debug_serialize()
{
    Serializer serial;
    serial.serialize();
}

void
debug_serialize(Tick when)
{
    new SerializeEvent(when, 0);
}

////////////////////////////////////////////////////////////////////////
//
// SerializeableClass member definitions
//
////////////////////////////////////////////////////////////////////////

// Map of class names to SerializeableBuilder creation functions.
// Need to make this a pointer so we can force initialization on the
// first reference; otherwise, some SerializeableClass constructors
// may be invoked before the classMap constructor.
map<string,SerializeableClass::CreateFunc> *SerializeableClass::classMap = 0;

// SerializeableClass constructor: add mapping to classMap
SerializeableClass::SerializeableClass(const string &className,
                                       CreateFunc createFunc)
{
    if (classMap == NULL)
        classMap = new map<string,SerializeableClass::CreateFunc>();

    if ((*classMap)[className])
    {
        cerr << "Error: simulation object class " << className << " redefined"
             << endl;
        fatal("");
    }

    // add className --> createFunc to class map
    (*classMap)[className] = createFunc;
}


//
//
Serializeable *
SerializeableClass::createObject(Checkpoint *cp,
                                 const std::string &section)
{
    string className;

    if (!cp->find(section, "type", className)) {
        fatal("Serializeable::create: no 'type' entry in section '%s'.\n",
              section);
    }

    CreateFunc createFunc = (*classMap)[className];

    if (createFunc == NULL) {
        fatal("Serializeable::create: no create function for class '%s'.\n",
              className);
    }

    Serializeable *object = createFunc(cp, section);

    assert(object != NULL);

    return object;
}


Serializeable *
Serializeable::create(Checkpoint *cp, const std::string &section)
{
    Serializeable *object = SerializeableClass::createObject(cp, section);
    object->unserialize(cp, section);
    return object;
}


Checkpoint::Checkpoint(const std::string &filename, const std::string &path,
                       const ConfigNode *_configNode)
    : db(new IniFile), basePath(path), configNode(_configNode)
{
    if (!db->load(filename)) {
        fatal("Can't load checkpoint file '%s'\n", filename);
    }

    mainEventQueue.unserialize(this, "MainEventQueue");
}


bool
Checkpoint::find(const std::string &section, const std::string &entry,
                 std::string &value)
{
    return db->find(section, entry, value);
}


bool
Checkpoint::findObj(const std::string &section, const std::string &entry,
                    Serializeable *&value)
{
    string path;

    if (!db->find(section, entry, path))
        return false;

    if ((value = configNode->resolveSimObject(path)) != NULL)
        return true;

    if ((value = objMap[path]) != NULL)
        return true;

    return false;
}


bool
Checkpoint::sectionExists(const std::string &section)
{
    return db->sectionExists(section);
}
