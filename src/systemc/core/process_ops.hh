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

#ifndef __SYSTEMC_CORE_PROCESS_OPS_HH__
#define __SYSTEMC_CORE_PROCESS_OPS_HH__

/*
 * Free-function interface to sc_gem5::Process and sc_gem5::Reset, used by
 * the sources that create resets (core/sc_module.cc, sc_spawn.cc)
 * and by channel/sc_signal.cc. It needs only forward declarations, so its
 * operations do not require callers to see the kernel's process.hh.
 */

namespace sc_core
{

class sc_object;
class sc_port_base;
template <class T> class sc_signal_in_if;

} // namespace sc_core

namespace sc_gem5
{

class Process;
class Reset;

void newReset(const sc_core::sc_port_base *pb, Process *p, bool sync,
              bool value);
void newReset(const sc_core::sc_signal_in_if<bool> *sig, Process *p, bool sync,
              bool value);
void updateReset(Reset *reset);

sc_core::sc_object *getProcessObject(Process *p);

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_PROCESS_OPS_HH__
