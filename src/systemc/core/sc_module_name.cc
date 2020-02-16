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


#include "systemc/core/module.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_module_name.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

sc_module_name::sc_module_name(const char *name) :
    _name(name), _gem5_module(nullptr), _on_the_stack(true)
{
    if (sc_is_running())
        SC_REPORT_ERROR(SC_ID_INSERT_MODULE_, "simulation running");
    else if (::sc_gem5::scheduler.elaborationDone())
        SC_REPORT_ERROR(SC_ID_INSERT_MODULE_, "elaboration done");
    else
        _gem5_module = new sc_gem5::Module(name);
}

sc_module_name::sc_module_name(const sc_module_name &other) :
    _name(other._name), _gem5_module(other._gem5_module), _on_the_stack(false)
{}

sc_module_name::~sc_module_name()
{
    if (_on_the_stack) {
        _gem5_module->pop();
    }
}

sc_module_name::operator const char *() const
{
    return _name;
}

} // namespace sc_core
