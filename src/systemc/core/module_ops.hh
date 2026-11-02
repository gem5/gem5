/*
 * Copyright 2026 Google LLC
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

#ifndef __SYSTEMC_CORE_MODULE_OPS_HH__
#define __SYSTEMC_CORE_MODULE_OPS_HH__

/*
 * Free-function interface to sc_gem5::Module and the active-module stack,
 * used by core/sc_module.cc, sc_module_name.cc, sc_port.cc and
 * sc_export.cc. It needs only forward declarations, so its callers do not
 * have to see the kernel's module.hh.
 */

#include <vector>

namespace sc_core
{

class sc_bind_proxy;
class sc_export_base;
class sc_port_base;

} // namespace sc_core

namespace sc_gem5
{

class Module;
class Object;

/*
 * Module construction lifecycle: sc_module_name calls newModule() to stage an
 * unfinished Module; sc_module's sc_object base constructor finishes it and
 * pushes it onto the active-module and parent-object stacks; ~sc_module_name
 * pops it via popModule(); ~sc_module deletes it via destroyModule().
 */
Module *newModule(const char *name);
Module *getUnfinishedModuleChecked();
void popModule(Module *m);
void destroyModule(Module *m);

// Active module on the constructor stack vs. enclosing module on the
// parent-object stack (which also covers elaboration/simulation callbacks).
Module *getCurrentModule();
Module *pickParentModule();

const char *getModuleName(const Module *m);
Object *getModuleObject(Module *m);

// sc_module::end_module() and the deprecated string-constructor check.
void endModule(Module *m);
void markModuleDeprecatedConstructor(Module *m);

void bindModulePorts(Module *m,
                     std::vector<const sc_core::sc_bind_proxy *> &proxies);
void addModulePort(Module *m, sc_core::sc_port_base *port);
void addModuleExport(Module *m, sc_core::sc_export_base *exp);

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_MODULE_OPS_HH__
