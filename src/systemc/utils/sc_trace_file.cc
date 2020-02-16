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

#include <vector>

#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/sc_signal_in_if.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_time.hh"
#include "systemc/ext/dt/bit/sc_bv_base.hh"
#include "systemc/ext/dt/bit/sc_logic.hh"
#include "systemc/ext/dt/bit/sc_lv_base.hh"
#include "systemc/ext/dt/fx/sc_fxnum.hh"
#include "systemc/ext/dt/fx/sc_fxval.hh"
#include "systemc/ext/dt/int/sc_int_base.hh"
#include "systemc/ext/dt/int/sc_signed.hh"
#include "systemc/ext/dt/int/sc_uint_base.hh"
#include "systemc/ext/dt/int/sc_unsigned.hh"
#include "systemc/ext/utils/sc_trace_file.hh"
#include "systemc/utils/vcd.hh"

namespace sc_core
{

sc_trace_file::sc_trace_file() {}
sc_trace_file::~sc_trace_file() {}

sc_trace_file *
sc_create_vcd_trace_file(const char *name)
{
    auto tf = new ::sc_gem5::VcdTraceFile(name);
    ::sc_gem5::scheduler.registerTraceFile(tf);
    return tf;
}

void
sc_close_vcd_trace_file(sc_trace_file *tf)
{
    ::sc_gem5::scheduler.unregisterTraceFile(
            static_cast<::sc_gem5::TraceFile *>(tf));
    delete tf;
}

void
sc_write_comment(sc_trace_file *tf, const std::string &comment)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->writeComment(comment);
}

void
sc_trace(sc_trace_file *tf, const bool &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const bool *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const float &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const float *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const double &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const double *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_logic &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_logic *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_int_base &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_int_base *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_uint_base &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_uint_base *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_signed &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_signed *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_unsigned &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_unsigned *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_bv_base &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_bv_base *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_lv_base &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_lv_base *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxval &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxval *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxval_fast &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxval_fast *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxnum &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxnum *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxnum_fast &v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::sc_fxnum_fast *v,
        const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_event &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_event *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_time &v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name);
}

void
sc_trace(sc_trace_file *tf, const sc_time *v, const std::string &name)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name);
}

void
sc_trace(sc_trace_file *tf, const unsigned char &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned char *v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned short &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned short *v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned int &v,
        const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned int *v,
        const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned long &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned long *v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const char &v, const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const char *v, const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const short &v,
        const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const short *v,
        const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const int &v, const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const int *v, const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const long &v, const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const long *v, const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::int64 &v,
        const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::int64 *v,
        const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::uint64 &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(&v, name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_dt::uint64 *v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->addTraceVal(v, name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_signal_in_if<char> &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->
        addTraceVal(&v.read(), name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_signal_in_if<short> &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->
        addTraceVal(&v.read(), name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_signal_in_if<int> &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->
        addTraceVal(&v.read(), name, width);
}

void
sc_trace(sc_trace_file *tf, const sc_signal_in_if<long> &v,
         const std::string &name, int width)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->
        addTraceVal(&v.read(), name, width);
}

void
sc_trace(sc_trace_file *tf, const unsigned int &v,
         const std::string &name, const char **enum_literals)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->
        addTraceVal(&v, name, enum_literals);
}

void
sc_trace_delta_cycles(sc_trace_file *tf, bool on)
{
    static_cast<::sc_gem5::TraceFile *>(tf)->traceDeltas(on);
}

} // namespace sc_core
