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

#include <assert.h>

#include "base/inifile.hh"
#include "base/misc.hh"
#include "sim/builder.hh"
#include "sim/configfile.hh"
#include "sim/config_node.hh"
#include "sim/host.hh"
#include "sim/sim_object.hh"
#include "sim/universe.hh"

using namespace std;


ostream &
builderStream()
{
    static ofstream file;
    static ostream *stream = NULL;

    if (!stream) {
        if (!outputDirectory.empty()) {
            string filename = outputDirectory + "builder.txt";
            file.open(filename.c_str());
            stream = &file;
        } else {
            stream = outputStream;
        }
    }

    return *stream;
}

SimObjectBuilder::SimObjectBuilder(const string &_configClass,
                                   const string &_instanceName,
                                   ConfigNode *_configNode,
                                   const string &_simObjClassName)
    : ParamContext(_configClass, true),
      instanceName(_instanceName),
      configNode(_configNode),
      simObjClassName(_simObjClassName)
{
}

SimObjectBuilder::~SimObjectBuilder()
{
}

///////////////////////////////////////////
//
// SimObjectBuilder member definitions
//
///////////////////////////////////////////

// override ParamContext::parseParams() to check params based on
// instance name first.  If not found, then check based on iniSection
// (as in default ParamContext implementation).
void
SimObjectBuilder::parseParams(IniFile &iniFile)
{
    iniFilePtr = &iniFile;	// set object member

    ParamList::iterator i;

    for (i = paramList->begin(); i != paramList->end(); ++i) {
        string string_value;

        if (iniFile.findDefault(instanceName, (*i)->name, string_value)) {
            (*i)->parse(string_value);
        }
        else if (iniFile.findDefault(iniSection, (*i)->name, string_value)) {
            (*i)->parse(string_value);
        }
    }
}


void
SimObjectBuilder::printErrorProlog(ostream &os)
{
    os << "Error creating object '" << getInstanceName()
       << "' of type '" << simObjClassName
       << "', section '" << iniSection << "':" << endl;
}


////////////////////////////////////////////////////////////////////////
//
// SimObjectClass member definitions
//
////////////////////////////////////////////////////////////////////////

// Map of class names to SimObjectBuilder creation functions.  Need to
// make this a pointer so we can force initialization on the first
// reference; otherwise, some SimObjectClass constructors may be invoked
// before the classMap constructor.
map<string,SimObjectClass::CreateFunc> *SimObjectClass::classMap = NULL;

// SimObjectClass constructor: add mapping to classMap
SimObjectClass::SimObjectClass(const string &className, CreateFunc createFunc)
{
    if (classMap == NULL)
        classMap = new map<string,SimObjectClass::CreateFunc>();

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
SimObject *
SimObjectClass::createObject(IniFile &configDB,
                             const string &configClassName,
                             const string &objName,
                             ConfigNode *configNode)
{
    // find simulation object class name from configuration class
    // (specified by 'type=' parameter)
    string simObjClassName;

    if (!configNode->find("type", simObjClassName)) {
        cerr << "Configuration class '" << configClassName << "' not found."
             << endl;
        abort();
    }

    // look up className to get appropriate createFunc
    if (classMap->find(simObjClassName) == classMap->end()) {
        cerr << "Simulator object class '" << simObjClassName << "' not found."
             << endl;
        abort();
    }

    CreateFunc createFunc = (*classMap)[simObjClassName];

    // call createFunc with config hierarchy node to get object
    // builder instance (context with parameters for object creation)
    SimObjectBuilder *objectBuilder = (*createFunc)(configClassName,
                                                    objName, configNode,
                                                    simObjClassName);

    assert(objectBuilder != NULL);

    // parse all parameters in context to generate parameter values
    objectBuilder->parseParams(configDB);

    // now create the actual simulation object
    SimObject *object = objectBuilder->create();

    assert(object != NULL);

    // echo object parameters to stats file (for documenting the
    // config used to generate the associated stats)
    builderStream() << "[" << object->name() << "]" << endl;
    builderStream() << "type=" << simObjClassName << endl;
    objectBuilder->showParams(builderStream());
    builderStream() << endl;

    // done with the SimObjectBuilder now
    delete objectBuilder;

    return object;
}


//
// static method:
//
void
SimObjectClass::describeAllClasses(ostream &os)
{
    map<string,CreateFunc>::iterator iter;

    for (iter = classMap->begin(); iter != classMap->end(); ++iter) {
        const string &className = iter->first;
        CreateFunc createFunc = iter->second;

        os << "[" << className << "]\n";

        // create dummy object builder just to instantiate parameters
        SimObjectBuilder *objectBuilder = (*createFunc)("", "", NULL, "");

        // now get the object builder to describe ite params
        objectBuilder->describeParams(os);

        os << endl;

        // done with the object builder now
        delete objectBuilder;
    }
}
