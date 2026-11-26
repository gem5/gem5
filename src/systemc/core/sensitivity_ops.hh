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

#ifndef __SYSTEMC_CORE_SENSITIVITY_OPS_HH__
#define __SYSTEMC_CORE_SENSITIVITY_OPS_HH__

/*
 * Free-function interface to static sensitivities, used by
 * core/sc_sensitive.cc and sc_spawn.cc. It needs only forward declarations,
 * so its callers do not have to see the kernel's sensitivity.hh.
 */

namespace sc_core
{

class sc_event;
class sc_event_finder;
class sc_export_base;
class sc_interface;
class sc_port_base;

} // namespace sc_core

namespace sc_gem5
{

class Process;

void newStaticSensitivityEvent(Process *p, const sc_core::sc_event *e);
void newStaticSensitivityInterface(Process *p, const sc_core::sc_interface *i);
void newStaticSensitivityPort(Process *p, const sc_core::sc_port_base *pb);
void newStaticSensitivityExport(Process *p,
                                const sc_core::sc_export_base *exp);
void newStaticSensitivityFinder(Process *p, const sc_core::sc_event_finder *f);

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_SENSITIVITY_OPS_HH__
