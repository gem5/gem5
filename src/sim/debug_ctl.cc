/*
 * Copyright (c) 2025 Polydoros Petrakis
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

#include "sim/debug_ctl.hh"

#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <fstream>
#include <string>

#include "base/debug.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * Self-deleting event that disables a named debug flag at a future tick.
 * Scheduled by processDebugCmd() when a +FLAG:Nt duration is given.
 */
class DebugFlagOffEvent : public Event
{
  public:
    DebugFlagOffEvent(const std::string &name)
        : Event(Event::Default_Pri, AutoDelete), flagName(name)
    {}

    void
    process() override
    {
        if (!debug::changeFlag(flagName.c_str(), false)) {
            warn("gem5_debug_ctl: auto-disable: unknown flag '%s'\n",
                 flagName);
        } else {
            inform("gem5_debug_ctl: auto-disabled '%s' at tick %lu\n",
                   flagName.c_str(), curTick());
        }
    }

    const char *
    description() const override
    {
        return "DebugFlagOffEvent";
    }

  private:
    std::string flagName;
};

void
processDebugCmd()
{
    std::string path = "/tmp/gem5_debug_" + std::to_string(getpid()) + ".cmd";

    std::ifstream f(path);
    if (!f.is_open()) {
        warn("gem5_debug_ctl: cannot open command file '%s'\n", path);
        return;
    }

    std::string line;
    while (std::getline(f, line)) {
        // Strip inline comments.
        auto hash = line.find('#');
        if (hash != std::string::npos) {
            line = line.substr(0, hash);
        }

        // Strip trailing whitespace.
        while (!line.empty() && isspace((unsigned char)line.back())) {
            line.pop_back();
        }

        if (line.empty()) {
            continue;
        }

        // Handle global trace master toggles.
        if (line == "trace:on") {
            trace::enable();
            inform("gem5_debug_ctl: trace enabled at tick %lu\n", curTick());
            continue;
        }
        if (line == "trace:off") {
            trace::disable();
            inform("gem5_debug_ctl: trace disabled at tick %lu\n", curTick());
            continue;
        }

        // All remaining commands start with '+' (enable) or '-' (disable).
        if (line[0] != '+' && line[0] != '-') {
            warn("gem5_debug_ctl: unrecognised command '%s'\n", line);
            continue;
        }
        bool enable = (line[0] == '+');
        std::string rest = line.substr(1);

        // Parse optional tick duration suffix: FLAG:Nt
        Tick duration = 0;
        std::string flagName = rest;
        auto colon = rest.find(':');
        if (colon != std::string::npos) {
            flagName = rest.substr(0, colon);
            std::string durStr = rest.substr(colon + 1);
            if (durStr.size() < 2 || durStr.back() != 't') {
                warn("gem5_debug_ctl: invalid duration '%s'"
                     " (expected format: <N>t for ticks)\n",
                     durStr);
                continue;
            } else {
                try {
                    duration =
                        std::stoull(durStr.substr(0, durStr.size() - 1));
                } catch (const std::exception &e) {
                    warn("gem5_debug_ctl: invalid duration value"
                         " '%s': %s\n",
                         durStr, e.what());
                    continue;
                }
            }
        }

        if (flagName.empty()) {
            warn("gem5_debug_ctl: empty flag name in command '%s'\n", line);
            continue;
        }

        if (!debug::changeFlag(flagName.c_str(), enable)) {
            warn("gem5_debug_ctl: unknown debug flag '%s'\n", flagName);
            continue;
        }

        inform("gem5_debug_ctl: %s flag '%s' at tick %lu\n",
               enable ? "enabled" : "disabled", flagName.c_str(), curTick());

        // Schedule auto-disable if a duration was given.
        if (enable && duration > 0) {
            auto *ev = new DebugFlagOffEvent(flagName);
            curEventQueue()->schedule(ev, curTick() + duration);
            inform("gem5_debug_ctl: will auto-disable '%s' after"
                   " %lu ticks (at tick %lu)\n",
                   flagName.c_str(), duration, curTick() + duration);
        }
    }

    // Remove the command file so stale commands are not re-processed
    // if the signal is somehow delivered twice.
    ::remove(path.c_str());
}

} // namespace gem5
