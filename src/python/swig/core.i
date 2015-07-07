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

%module(package="m5.internal") core

%{
#include "base/misc.hh"
#include "base/random.hh"
#include "base/socket.hh"
#include "base/types.hh"
#include "python/swig/pyobject.hh"
#include "sim/core.hh"

extern const char *compileDate;

#ifdef DEBUG
const bool flag_DEBUG = true;
#else
const bool flag_DEBUG = false;
#endif
#ifdef NDEBUG
const bool flag_NDEBUG = true;
#else
const bool flag_NDEBUG = false;
#endif
const bool flag_TRACING_ON = TRACING_ON;

inline void disableAllListeners() { ListenSocket::disableAll(); }

inline void
seedRandom(uint64_t seed)
{
    random_mt.init(seed);
}

%}

%include <std_string.i>
%include <stdint.i>

%include "base/types.hh"

void setOutputDir(const std::string &dir);
void doExitCleanup();
void disableAllListeners();
void seedRandom(uint64_t seed);

%immutable compileDate;
char *compileDate;
const bool flag_DEBUG;
const bool flag_NDEBUG;
const bool flag_TRACING_ON;

void setClockFrequency(Tick ticksPerSecond);

Tick curTick();

class Checkpoint;

void serializeAll(const std::string &cpt_dir);
CheckpointIn *getCheckpoint(const std::string &cpt_dir);
void unserializeGlobals(CheckpointIn &cp);

bool want_warn, warn_verbose;
bool want_info, info_verbose;
bool want_hack, hack_verbose;

