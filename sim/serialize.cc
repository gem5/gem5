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
#include <sys/types.h>
#include <sys/stat.h>

#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "sim/config_node.hh"
#include "sim/eventq.hh"
#include "sim/param.hh"
#include "sim/serialize.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"

using namespace std;

void
Serializable::nameOut(ostream &os)
{
    os << "\n[" << name() << "]\n";
}

void
Serializable::nameOut(ostream &os, const string &_name)
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
           const std::string &name, Serializable * &param)
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


/////////////////////////////

/// Container for serializing global variables (not associated with
/// any serialized object).
class Globals : public Serializable
{
  public:
    string name() const;
    void serialize(ostream& os);
    void unserialize(Checkpoint *cp);
};

/// The one and only instance of the Globals class.
Globals globals;

string
Globals::name() const
{
    return "Globals";
}

void
Globals::serialize(ostream& os)
{
    nameOut(os);
    SERIALIZE_SCALAR(curTick);

    nameOut(os, "MainEventQueue");
    mainEventQueue.serialize(os);
}

void
Globals::unserialize(Checkpoint *cp)
{
    const string &section = name();
    UNSERIALIZE_SCALAR(curTick);

    mainEventQueue.unserialize(cp, "MainEventQueue");
}

void
Serializable::serializeAll()
{
    string dir = Checkpoint::dir();
    if (mkdir(dir.c_str(), 0775) == -1 && errno != EEXIST)
            fatal("couldn't mkdir %s\n", dir);

    string cpt_file = dir + Checkpoint::baseFilename;
    ofstream outstream(cpt_file.c_str());
    time_t t = time(NULL);
    outstream << "// checkpoint generated: " << ctime(&t);

    globals.serialize(outstream);
    SimObject::serializeAll(outstream);
}


void
Serializable::unserializeGlobals(Checkpoint *cp)
{
    globals.unserialize(cp);
}


class SerializeEvent : public Event
{
  protected:
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
    : Event(&mainEventQueue, Serialize_Pri), repeat(_repeat)
{
    setFlags(AutoDelete);
    schedule(_when);
}

void
SerializeEvent::process()
{
    Serializable::serializeAll();
    if (repeat)
        schedule(curTick + repeat);
}

const char *Checkpoint::baseFilename = "m5.cpt";

static string checkpointDirBase;

string
Checkpoint::dir()
{
    // use csprintf to insert curTick into directory name if it
    // appears to have a format placeholder in it.
    return (checkpointDirBase.find("%") != string::npos) ?
        csprintf(checkpointDirBase, curTick) : checkpointDirBase;
}

void
Checkpoint::setup(Tick when, Tick period)
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

Param<string> serialize_dir(&serialParams, "dir",
                            "dir to stick checkpoint in "
                            "(sprintf format with cycle #)");

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
    if (serialize_dir.isValid()) {
        checkpointDirBase = serialize_dir;
    } else {
        if (outputDirectory.empty())
            checkpointDirBase = "m5.%012d";
        else
            checkpointDirBase = outputDirectory + "cpt.%012d";
    }

    // guarantee that directory ends with a '/'
    if (checkpointDirBase[checkpointDirBase.size() - 1] != '/')
        checkpointDirBase += "/";

    if (serialize_cycle > 0)
        Checkpoint::setup(serialize_cycle, serialize_period);
}

void
debug_serialize()
{
    Serializable::serializeAll();
}

void
debug_serialize(Tick when)
{
    new SerializeEvent(when, 0);
}

////////////////////////////////////////////////////////////////////////
//
// SerializableClass member definitions
//
////////////////////////////////////////////////////////////////////////

// Map of class names to SerializableBuilder creation functions.
// Need to make this a pointer so we can force initialization on the
// first reference; otherwise, some SerializableClass constructors
// may be invoked before the classMap constructor.
map<string,SerializableClass::CreateFunc> *SerializableClass::classMap = 0;

// SerializableClass constructor: add mapping to classMap
SerializableClass::SerializableClass(const string &className,
                                       CreateFunc createFunc)
{
    if (classMap == NULL)
        classMap = new map<string,SerializableClass::CreateFunc>();

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
Serializable *
SerializableClass::createObject(Checkpoint *cp,
                                 const std::string &section)
{
    string className;

    if (!cp->find(section, "type", className)) {
        fatal("Serializable::create: no 'type' entry in section '%s'.\n",
              section);
    }

    CreateFunc createFunc = (*classMap)[className];

    if (createFunc == NULL) {
        fatal("Serializable::create: no create function for class '%s'.\n",
              className);
    }

    Serializable *object = createFunc(cp, section);

    assert(object != NULL);

    return object;
}


Serializable *
Serializable::create(Checkpoint *cp, const std::string &section)
{
    Serializable *object = SerializableClass::createObject(cp, section);
    object->unserialize(cp, section);
    return object;
}


Checkpoint::Checkpoint(const std::string &cpt_dir, const std::string &path,
                       const ConfigNode *_configNode)
    : db(new IniFile), basePath(path), configNode(_configNode)
{
    string filename = cpt_dir + "/" + Checkpoint::baseFilename;
    if (!db->load(filename)) {
        fatal("Can't load checkpoint file '%s'\n", filename);
    }
}


bool
Checkpoint::find(const std::string &section, const std::string &entry,
                 std::string &value)
{
    return db->find(section, entry, value);
}


bool
Checkpoint::findObj(const std::string &section, const std::string &entry,
                    Serializable *&value)
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
