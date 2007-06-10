/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

%module sim_object

%{
#include "python/swig/pyobject.hh"
%}

// import these files for SWIG to wrap
%include "stdint.i"
%include "std_string.i"
%include "sim/host.hh"

class BaseCPU;

class SimObject {
  public:
    enum State {
      Running,
      Draining,
      Drained
    };

    enum MemoryMode {
      Invalid,
      Atomic,
      Timing
    };

    unsigned int drain(Event *drain_event);
    void resume();
    void switchOut();
    void takeOverFrom(BaseCPU *cpu);
    SimObject(const std::string &_name);
};

class System {
    private:
      System();
    public:
      SimObject::MemoryMode getMemoryMode();
      void setMemoryMode(SimObject::MemoryMode mode);
};

SimObject *createSimObject(const std::string &name);

int connectPorts(SimObject *o1, const std::string &name1, int i1,
                 SimObject *o2, const std::string &name2, int i2);

BaseCPU *convertToBaseCPUPtr(SimObject *obj);
System *convertToSystemPtr(SimObject *obj);

void serializeAll(const std::string &cpt_dir);
void unserializeAll(const std::string &cpt_dir);

void initAll();
void regAllStats();

%wrapper %{
// fix up module name to reflect the fact that it's inside the m5 package
#undef SWIG_name
#define SWIG_name "m5.internal._sim_object"

// Convert a pointer to the Python object that SWIG wraps around a
// C++ SimObject pointer back to the actual C++ pointer.
SimObject *
convertSwigSimObjectPtr(PyObject *pyObj)
{
    SimObject *so;
    if (SWIG_ConvertPtr(pyObj, (void **) &so, SWIGTYPE_p_SimObject, 0) == -1)
        return NULL;
    return so;
}
%}
