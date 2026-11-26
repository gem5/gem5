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
#include <map>
#include <string>
#include <utility>

#include "base/types.hh"
#include "gem5/hypercall_ids.h"

namespace gem5
{

Tick curTick();

/// Built-in hypercall identifiers used when scheduling exits with
/// exitSimulationLoop(). A hypercall is the broader guest/simulator request
/// mechanism; ``ExitHypercall`` is gem5's finite set of hypercalls that route
/// through the simulation-exit/stdlib ``ExitHandler`` path. Users may still
/// use custom integer hypercall IDs with custom handlers. ID 0 is reserved for
/// the legacy generator-based exit handling path.
enum class ExitHypercall : uint64_t
{
#define GEM5_DEFINE_EXIT_ENUM(name, value, desc) name = value,
    GEM5_FOREACH_EXIT_HYPERCALL(GEM5_DEFINE_EXIT_ENUM)
#undef GEM5_DEFINE_EXIT_ENUM
};

struct ExitHypercallDescriptor
{
    ExitHypercall id;
    const char *name;
    const char *description;
};

#define GEM5_DEFINE_EXIT_DESCRIPTOR(name, value, desc)                        \
    {ExitHypercall::name, #name, desc},
inline constexpr ExitHypercallDescriptor kExitHypercallDescriptors[] = {
    GEM5_FOREACH_EXIT_HYPERCALL(GEM5_DEFINE_EXIT_DESCRIPTOR)};
#undef GEM5_DEFINE_EXIT_DESCRIPTOR

using ExitHypercallId = uint64_t;
using ExitPayload = std::map<std::string, std::string>;

inline ExitPayload
classicGeneratorPayload(const std::string &cause, int code = 0)
{ return {{"cause", cause}, {"code", std::to_string(code)}}; }

/// Register a callback to be called when Python exits.  Defined in
/// sim/main.cc.
void registerExitCallback(const std::function<void()> &);

/**
 * Legacy "classic" exit path that predates hypercalls. This schedules a
 * generator-based exit event that reports only a human-readable message and an
 * optional integer code. Modern stdlib simulations should prefer the
 * hypercall-based APIs below so Python ExitHandlers can dispatch on an ID and
 * structured payload.
 *
 * Use ``exitSimulationLoopClassic`` for legacy generator exits that must
 * still preserve ``getCause()``/``getCode()``.
 */
[[deprecated("exitSimLoop is deprecated. Use exitSimulationLoop for "
             "hypercall-based exits, or exitSimulationLoopClassic only "
             "when legacy cause/code compatibility is required.")]]
void exitSimLoop(const std::string &message, int exit_code = 0,
                 Tick when = curTick(), Tick repeat = 0,
                 bool serialize = false);

/// Legacy high-priority variant of ``exitSimLoop`` (see note above).
[[deprecated("exitSimLoopNow is deprecated. Use exitSimulationLoopNow for "
             "hypercall-based exits, or exitSimulationLoopClassicNow only "
             "when legacy cause/code compatibility is required.")]]
void exitSimLoopNow(const std::string &message, int exit_code = 0,
                    Tick repeat = 0, bool serialize = false);

/**
 * Compatibility helper for legacy "classic" simulation exits.
 *
 * "Classic" means the exit is still identified by the legacy string message
 * instead of by a dedicated hypercall ID. This helper sends that string and
 * code through the ``CLASSIC_GENERATOR`` hypercall payload. The Python
 * classic-generator handler then maps the string into the legacy
 * ``ExitEvent`` path, for example ``"checkpoint"`` maps to
 * ``ExitEvent.CHECKPOINT``.
 *
 * This preserves ``GlobalSimLoopExitEvent::getCause()/getCode()`` and supplies
 * the matching ``{"cause": message, "code": exit_code}`` payload expected by
 * the Python classic-generator handler. Use this only when legacy cause/code
 * or ``ExitEvent`` string-dispatch compatibility is required.
 */
void exitSimulationLoopClassic(const std::string &message, int exit_code = 0,
                               Tick when = curTick(), Tick repeat = 0,
                               bool serialize = false);

/**
 * Immediate variant of ``exitSimulationLoopClassic`` that preserves the
 * minimum-priority scheduling behavior of the legacy ``exitSimLoopNow`` API.
 *
 * The ``serialize`` parameter is accepted for compatibility with
 * ``exitSimLoopNow``. That API has ignored the parameter since it was added.
 */
void exitSimulationLoopClassicNow(const std::string &message,
                                  int exit_code = 0, Tick repeat = 0,
                                  bool serialize = false);

/**
 * Preferred hypercall-based exit API. ``hypercall_id`` should be one of
 * ``ExitHypercall``'s values, or a custom ID if you have registered a matching
 * ExitHandler.
 *
 * This API does not normally populate legacy ``getCause()``/``getCode()``.
 * ``ExitHypercall::CLASSIC_GENERATOR`` is the compatibility exception: callers
 * must provide ``cause`` and ``code`` payload entries, which are mirrored into
 * the legacy fields.
 *
 * Examples:
 *
 * ```
 * exitSimulationLoop(ExitHypercall::SCHEDULED_EXIT,
 *     { {"justification", "Stop after ROI"} },
 *     curTick() + 1000);
 * exitSimulationLoop(static_cast<uint64_t>(myCustomId), {{"foo", "bar"}});
 * ```
 *
 * @param hypercall_id Identifier for the ExitHandler to run.
 * @param payload   Optional metadata consumed by the ExitHandler.
 * @param when      Tick at which the exit event should fire.
 * @param repeat    Interval to reschedule the event; 0 disables repetition.
 */
void exitSimulationLoop(ExitHypercallId hypercall_id,
                        ExitPayload payload = ExitPayload(),
                        Tick when = curTick(), Tick repeat = 0);

/**
 * Immediate variant of ``exitSimulationLoop`` that schedules the event for
 * the current tick and does not repeat.
 */
void exitSimulationLoopNow(ExitHypercallId hypercall_id,
                           ExitPayload payload = ExitPayload());

/// Convenience overloads so callers can pass ``ExitHypercall`` directly.
inline void
exitSimulationLoop(ExitHypercall hypercall,
                   ExitPayload payload = ExitPayload(), Tick when = curTick(),
                   Tick repeat = 0)
{
    exitSimulationLoop(static_cast<uint64_t>(hypercall), std::move(payload),
                       when, repeat);
}

inline void
exitSimulationLoopNow(ExitHypercall hypercall,
                      ExitPayload payload = ExitPayload())
{
    exitSimulationLoopNow(static_cast<uint64_t>(hypercall),
                          std::move(payload));
}

} // namespace gem5

#endif // __SIM_EXIT_HH__
