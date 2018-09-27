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
 *
 * Authors: Gabe Black
 */

#include "systemc/core/time.hh"

#include "systemc/ext/core/sc_time.hh"

namespace sc_gem5
{

const char *TimeUnitNames[] = {
    [::sc_core::SC_FS] = "fs",
    [::sc_core::SC_PS] = "ps",
    [::sc_core::SC_NS] = "ns",
    [::sc_core::SC_US] = "us",
    [::sc_core::SC_MS] = "ms",
    [::sc_core::SC_SEC] = "s"
};

const char *TimeUnitConstantNames[] = {
    [::sc_core::SC_FS] = "SC_FS",
    [::sc_core::SC_PS] = "SC_PS",
    [::sc_core::SC_NS] = "SC_NS",
    [::sc_core::SC_US] = "SC_US",
    [::sc_core::SC_MS] = "SC_MS",
    [::sc_core::SC_SEC] = "SC_SEC"
};

double TimeUnitScale[] = {
    [::sc_core::SC_FS] = 1.0e-15,
    [::sc_core::SC_PS] = 1.0e-12,
    [::sc_core::SC_NS] = 1.0e-9,
    [::sc_core::SC_US] = 1.0e-6,
    [::sc_core::SC_MS] = 1.0e-3,
    [::sc_core::SC_SEC] = 1.0
};

Tick TimeUnitFrequency[] = {
    [::sc_core::SC_FS] = 1ULL * 1000 * 1000 * 1000 * 1000 * 1000,
    [::sc_core::SC_PS] = 1ULL * 1000 * 1000 * 1000 * 1000,
    [::sc_core::SC_NS] = 1ULL * 1000 * 1000 * 1000,
    [::sc_core::SC_US] = 1ULL * 1000 * 1000,
    [::sc_core::SC_MS] = 1ULL * 1000,
    [::sc_core::SC_SEC] = 1ULL
};

} // namespace sc_gem5
