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

%module(package="m5.internal") stats

%include <std_list.i>
%include <std_string.i>
%include <std_vector.i>
%include <stdint.i>

%{
#include "base/stats/text.hh"
#include "base/stats/types.hh"
#include "base/callback.hh"
#include "base/misc.hh"
#include "base/statistics.hh"
#include "sim/core.hh"
#include "sim/stat_control.hh"
#include "sim/stat_register.hh"

namespace Stats {
template <class T>
inline T
cast_info(Info *info)
{
    return dynamic_cast<T>(info);
}

inline FlagsType
Stats_Info_flags_get(Info *info)
{
    return info->flags;
}

inline void
Stats_Info_flags_set(Info *info, FlagsType flags)
{
    info->flags = flags;
}

inline char *
PCC(const char *string)
{
    return const_cast<char *>(string);
}

void
call_module_function(const char *module_name, const char *func_name)
{
    PyObject *module = PyImport_ImportModule(PCC(module_name));
    if (module == NULL)
        panic("Could not import %s", module);

    PyObject *result = PyObject_CallMethod(module, PCC(func_name), PCC(""));
    if (result == NULL) {
        PyErr_Print();
        panic("failure on call to function %s", func_name);
    }

    Py_DECREF(module);
    Py_DECREF(result);
}

void
pythonDump()
{
    call_module_function("m5.stats", "dump");
}

void
pythonReset()
{
    call_module_function("m5.stats", "reset");
}

} // namespace Stats
%}

%extend Stats::Info {
    short flags;
}

%ignore Stats::Info::flags;

%import  "base/stats/types.hh"
%import  "base/types.hh"

%include "base/stats/info.hh"
%include "base/stats/output.hh"

namespace std {
%template(list_info) list<Stats::Info *>;
%template(vector_double) vector<double>;
%template(vector_string) vector<string>;
%template(vector_DistData) vector<Stats::DistData>;
}

namespace Stats {

template <class T> T cast_info(Info *info);

%template(dynamic_ScalarInfo) cast_info<ScalarInfo *>;
%template(dynamic_VectorInfo) cast_info<VectorInfo *>;
%template(dynamic_DistInfo) cast_info<DistInfo *>;
%template(dynamic_VectorDistInfo) cast_info<VectorDistInfo *>;
%template(dynamic_Vector2dInfo) cast_info<Vector2dInfo *>;
%template(dynamic_FormulaInfo) cast_info<FormulaInfo *>;
%template(dynamic_SparseHistInfo) cast_info<SparseHistInfo *>;

void initSimStats();
Output *initText(const std::string &filename, bool desc);

void registerPythonStatsHandlers();

void schedStatEvent(bool dump, bool reset,
                    Tick when = curTick(), Tick repeat = 0);

void periodicStatDump(Tick period = 0);

void updateEvents();

void processResetQueue();
void processDumpQueue();
void enable();
bool enabled();

std::list<Info *> &statsList();

} // namespace Stats
