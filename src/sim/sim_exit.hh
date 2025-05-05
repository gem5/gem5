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
 */

#ifndef __SIM_EXIT_HH__
#define __SIM_EXIT_HH__

#include <functional>
#include <string>

#include "base/types.hh"

namespace gem5
{

Tick curTick();

/// Register a callback to be called when Python exits.  Defined in
/// sim/main.cc.
void registerExitCallback(const std::function<void()> &);

/// Schedule an event to exit the simulation loop (returning to
/// Python) at the end of the current cycle (curTick()).  The message
/// and exit_code parameters are saved in the SimLoopExitEvent to
/// indicate why the exit occurred.
void exitSimLoop(const std::string &message, int exit_code = 0,
                 Tick when = curTick(), Tick repeat = 0,
                 bool serialize = false);
/// Schedule an event as above, but make it high priority so it runs before
/// any normal events which are schedule at the current time.
void exitSimLoopNow(const std::string &message, int exit_code = 0,
                    Tick repeat = 0, bool serialize = false);

void exitSimLoopWithHypercall(const std::string &message, int exit_code,
                              Tick when, Tick repeat,
                              std::map<std::string, std::string> payload,
                              uint64_t hypercall_id, bool serialize);

void exitSimulationLoop(uint64_t type_id,
    std::map<std::string, std::string> payload=
        std::map<std::string, std::string>(),
    Tick when=curTick());

void
exitSimulationLoopNow(uint64_t type_id,
    std::map<std::string, std::string> payload=
        std::map<std::string, std::string>());

} // namespace gem5

#endif // __SIM_EXIT_HH__
