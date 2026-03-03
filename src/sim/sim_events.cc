/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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

#include "sim/sim_events.hh"

#include <charconv>
#include <string>
#include <system_error>
#include <utility>

#include "base/callback.hh"
#include "base/logging.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/stats.hh"

namespace gem5
{

namespace
{

constexpr uint64_t
classicGeneratorHypercallId()
{ return static_cast<uint64_t>(ExitHypercall::CLASSIC_GENERATOR); }

std::string
classicGeneratorCause(const ExitPayload &payload)
{
    const auto it = payload.find("cause");
    return it == payload.end() ? "" : it->second;
}

int
classicGeneratorCode(const ExitPayload &payload)
{
    const auto it = payload.find("code");
    fatal_if(it == payload.end(),
             "ExitHypercall::CLASSIC_GENERATOR requires a 'code' payload "
             "entry.");

    int code;
    const auto &value = it->second;
    const auto [end, error] =
        std::from_chars(value.data(), value.data() + value.size(), code);
    fatal_if(value.empty() || error != std::errc() ||
                 end != value.data() + value.size(),
             "ExitHypercall::CLASSIC_GENERATOR requires 'code' to be a "
             "base-10 integer; got '%s'.",
             value.c_str());
    return code;
}

void
validateClassicGeneratorPayload(const ExitPayload &payload)
{
    const auto cause = payload.find("cause");
    fatal_if(cause == payload.end() || payload.find("code") == payload.end(),
             "ExitHypercall::CLASSIC_GENERATOR requires 'cause' and 'code' "
             "payload entries.");
    fatal_if(cause->second.empty(),
             "ExitHypercall::CLASSIC_GENERATOR requires a non-empty 'cause' "
             "payload entry.");
    classicGeneratorCode(payload);
}

void
syncClassicGeneratorFields(std::string &cause, int &code, ExitPayload &payload)
{
    if (payload.empty()) {
        payload = classicGeneratorPayload(cause, code);
        return;
    }

    validateClassicGeneratorPayload(payload);
    cause = classicGeneratorCause(payload);
    code = classicGeneratorCode(payload);
}

} // anonymous namespace

GlobalSimLoopExitEvent::GlobalSimLoopExitEvent(
    Tick when, const std::string &_cause, int c, Tick r, uint64_t hypercall_id,
    std::map<std::string, std::string> payload)
    : GlobalEvent(when, Sim_Exit_Pri, IsExitEvent),
      cause(_cause),
      code(c),
      repeat(r),
      hypercall_id(hypercall_id),
      payload(std::move(payload))
{
    if (this->hypercall_id == classicGeneratorHypercallId()) {
        syncClassicGeneratorFields(cause, code, this->payload);
    }
}

GlobalSimLoopExitEvent::GlobalSimLoopExitEvent(
    const std::string &_cause, int c, Tick r, uint64_t hypercall_id,
    std::map<std::string, std::string> payload)
    : GlobalEvent(curTick(), Minimum_Pri, IsExitEvent),
      cause(_cause),
      code(c),
      repeat(r),
      hypercall_id(hypercall_id),
      payload(std::move(payload))
{
    if (this->hypercall_id == classicGeneratorHypercallId()) {
        syncClassicGeneratorFields(cause, code, this->payload);
    }
}

GlobalSimLoopExitEvent::GlobalSimLoopExitEvent(
    Tick when, uint64_t hypercall_id,
    std::map<std::string, std::string> payload, Tick repeat)
    : GlobalSimLoopExitEvent(when, "", 0, repeat, hypercall_id,
                             std::move(payload))
{
}

GlobalSimLoopExitEvent::GlobalSimLoopExitEvent(uint64_t hypercall_id,
        std::map<std::string, std::string> payload)
    : GlobalSimLoopExitEvent("", 0, 0, hypercall_id, payload)
{
    assert(hypercall_id != 0); // 0 is reserved for the "old style" exitSimLoop
}

const char *
GlobalSimLoopExitEvent::description() const
{
    return "global simulation loop exit";
}

//
// handle termination event
//
void
GlobalSimLoopExitEvent::process()
{
    if (repeat) {
        schedule(curTick() + repeat);
    }
}

/**
 * Deprecated compatibility wrappers for the old-style exitSimLoop API.
 */

void
exitSimLoop(const std::string &message, int exit_code, Tick when, Tick repeat,
            bool serialize)
{ exitSimulationLoopClassic(message, exit_code, when, repeat, serialize); }

void
exitSimLoopNow(const std::string &message, int exit_code, Tick repeat,
               bool serialize)
{ exitSimulationLoopClassicNow(message, exit_code, repeat, serialize); }

void
exitSimulationLoopClassic(const std::string &message, int exit_code, Tick when,
                          Tick repeat, bool serialize)
{
    warn_if(serialize && (when != curTick() || repeat),
            "exitSimulationLoopClassic called with serialize=true for a "
            "delayed or repeating exit. Delay and repeat are supported, but "
            "auto-serialization of scheduled global exit events is not; the "
            "serialize argument is kept only for legacy API compatibility.");

    exitSimulationLoop(ExitHypercall::CLASSIC_GENERATOR,
                       classicGeneratorPayload(message, exit_code),
                       when + simQuantum, repeat);
}

void
exitSimulationLoopClassicNow(const std::string &message, int exit_code,
                             Tick repeat, bool /*serialize*/)
{
    new GlobalSimLoopExitEvent(message, exit_code, repeat,
                               classicGeneratorHypercallId(),
                               classicGeneratorPayload(message, exit_code));
}

/**
 * The hypercall-based exitSimulationLoop functions.
 */
void
exitSimulationLoop(ExitHypercallId hypercall_id, ExitPayload payload,
                   Tick when, Tick repeat)
{
    if (hypercall_id == classicGeneratorHypercallId()) {
        validateClassicGeneratorPayload(payload);
        new GlobalSimLoopExitEvent(when, classicGeneratorCause(payload),
                                   classicGeneratorCode(payload), repeat,
                                   hypercall_id, std::move(payload));
        return;
    }

    new GlobalSimLoopExitEvent(when, hypercall_id, std::move(payload), repeat);
}

void
exitSimulationLoopNow(ExitHypercallId hypercall_id, ExitPayload payload)
{
    if (hypercall_id == classicGeneratorHypercallId()) {
        validateClassicGeneratorPayload(payload);
        new GlobalSimLoopExitEvent(classicGeneratorCause(payload),
                                   classicGeneratorCode(payload), 0,
                                   hypercall_id, payload);
        return;
    }

    new GlobalSimLoopExitEvent(hypercall_id, payload);
}

LocalSimLoopExitEvent::LocalSimLoopExitEvent(const std::string &_cause, int c,
                                   Tick r)
    : Event(Sim_Exit_Pri, IsExitEvent),
      cause(_cause), code(c), repeat(r)
{
}

//
// handle termination event
//
void
LocalSimLoopExitEvent::process()
{ exitSimulationLoopClassic(cause, 0); }

const char *
LocalSimLoopExitEvent::description() const
{
    return "simulation loop exit";
}

void
LocalSimLoopExitEvent::serialize(CheckpointOut &cp) const
{
    Event::serialize(cp);

    SERIALIZE_SCALAR(cause);
    SERIALIZE_SCALAR(code);
    SERIALIZE_SCALAR(repeat);
}

void
LocalSimLoopExitEvent::unserialize(CheckpointIn &cp)
{
    Event::unserialize(cp);

    UNSERIALIZE_SCALAR(cause);
    UNSERIALIZE_SCALAR(code);
    UNSERIALIZE_SCALAR(repeat);
}

//
// constructor: automatically schedules at specified time
//
CountedExitEvent::CountedExitEvent(const std::string &_cause, int &counter)
    : Event(Sim_Exit_Pri), cause(_cause), downCounter(counter)
{
    // catch stupid mistakes
    assert(downCounter > 0);
}


//
// handle termination event
//
void
CountedExitEvent::process()
{
    if (--downCounter == 0) {
        exitSimulationLoopClassic(cause, 0);
    }
}


const char *
CountedExitEvent::description() const
{
    return "counted exit";
}

} // namespace gem5
