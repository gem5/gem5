/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 */

#include <assert.h>

#include "base/inifile.hh"
#include "base/misc.hh"
#include "sim/builder.hh"
#include "sim/host.hh"
#include "sim/sim_object.hh"
#include "sim/root.hh"

using namespace std;

SimObjectBuilder::SimObjectBuilder(const std::string &_iniSection)
    : ParamContext(_iniSection)
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
        if (iniFile.find(iniSection, (*i)->name, string_value))
            (*i)->parse(string_value);
    }
}


void
SimObjectBuilder::printErrorProlog(ostream &os)
{
    ccprintf(os, "Error creating object '%s':\n", iniSection);
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
        panic("Error: simulation object class '%s' redefined\n", className);

    // add className --> createFunc to class map
    (*classMap)[className] = createFunc;
}


//
//
SimObject *
SimObjectClass::createObject(IniFile &configDB, const std::string &iniSection)
{
    string type;
    if (!configDB.find(iniSection, "type", type)) {
        // no C++ type associated with this object
        return NULL;
    }

    // look up className to get appropriate createFunc
    if (classMap->find(type) == classMap->end())
        panic("Simulator object type '%s' not found.\n", type);


    CreateFunc createFunc = (*classMap)[type];

    // call createFunc with config hierarchy node to get object
    // builder instance (context with parameters for object creation)
    SimObjectBuilder *objectBuilder = (*createFunc)(iniSection);

    assert(objectBuilder != NULL);

    // parse all parameters in context to generate parameter values
    objectBuilder->parseParams(configDB);

    // now create the actual simulation object
    SimObject *object = objectBuilder->create();

    assert(object != NULL);

    // echo object parameters to stats file (for documenting the
    // config used to generate the associated stats)
    ccprintf(*configStream, "[%s]\n", object->name());
    ccprintf(*configStream, "type=%s\n", type);
    objectBuilder->showParams(*configStream);
    ccprintf(*configStream, "\n");

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
        SimObjectBuilder *objectBuilder = (*createFunc)("");

        // now get the object builder to describe ite params
        objectBuilder->describeParams(os);

        os << endl;

        // done with the object builder now
        delete objectBuilder;
    }
}
