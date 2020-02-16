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
 */

#include "systemc/utils/tracefile.hh"

#include <ctime>
#include <iomanip>

#include "base/output.hh"
#include "sim/core.hh"
#include "systemc/core/time.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_time.hh"
#include "systemc/ext/utils/functions.hh"

namespace sc_gem5
{

TraceFile::TraceFile(const std::string &name) :
    _os(simout.create(name, true, true)), timeUnitTicks(0),
    timeUnitValue(0.0), timeUnitUnit(::sc_core::SC_PS), _traceDeltas(false)
{}

TraceFile::~TraceFile()
{
    simout.close(_os);
}

std::ostream &TraceFile::stream() { return *_os->stream(); }

void
TraceFile::set_time_unit(double d, ::sc_core::sc_time_unit tu)
{
    timeUnitValue = d;
    timeUnitUnit = tu;

    double secs = d * TimeUnitScale[tu];
    for (tu = ::sc_core::SC_SEC; tu > ::sc_core::SC_FS;
            tu = (::sc_core::sc_time_unit)(tu - 1)) {
        if (TimeUnitScale[tu] <= secs)
            break;
    }

    uint64_t i = static_cast<uint64_t>(secs / TimeUnitScale[tu]);
    std::ostringstream ss;
    ss << i << " " << TimeUnitNames[tu] << " (" << _os->name() << ")";
    SC_REPORT_INFO("(I703) tracing timescale unit set", ss.str().c_str());
}

void
TraceFile::finalizeTime()
{
    ::sc_core::sc_time time;
    if (timeUnitValue == 0.0) {
        // The time scale was never set. Use the global time resolution.
        time = ::sc_core::sc_get_time_resolution();
    } else {
        time = ::sc_core::sc_time(timeUnitValue, timeUnitUnit);
    }
    timeUnitTicks = time.value();
}

} // namespace sc_gem5
