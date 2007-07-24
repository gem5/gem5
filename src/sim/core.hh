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
 *          Steve Reinhardt
 */

#ifndef __SIM_CORE_HH__
#define __SIM_CORE_HH__

#include <string>

#include "sim/host.hh"

/// The universal simulation clock.
extern Tick curTick;
const Tick retryTime = 1000;

namespace Clock {
/// The simulated frequency of curTick.
extern Tick Frequency;

namespace Float {
extern double s;
extern double ms;
extern double us;
extern double ns;
extern double ps;

extern double Hz;
extern double kHz;
extern double MHz;
extern double GHZ;
/* namespace Float */ }

namespace Int {
extern Tick s;
extern Tick ms;
extern Tick us;
extern Tick ns;
extern Tick ps;
/* namespace Int */ }
/* namespace Clock */ }

void setClockFrequency(Tick ticksPerSecond);

/// Output stream for simulator messages (e.g., cprintf()).  Also used
/// as default stream for tracing and DPRINTF() messages (unless
/// overridden with trace:file option).
extern std::ostream *outputStream;
void setOutputFile(const std::string &file);
void setOutputDir(const std::string &dir);

struct Callback;
void registerExitCallback(Callback *callback);
void doExitCleanup();

#endif /* __SIM_CORE_HH__ */
