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

#include "base/logging.hh"
#include "systemc/ext/utils/sc_report.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

sc_report::sc_report(sc_severity _severity, const char *_msgType,
        const char *_msg, int _verbosity, const char *_fileName,
        int _lineNumber, sc_time _time, const char *_processName, int _id) :
    _severity(_severity), _msgType(_msgType), _msg(_msg),
    _verbosity(_verbosity), _fileName(_fileName), _lineNumber(_lineNumber),
    _time(_time), _processName(_processName), _id(_id)
{
    _what = sc_report_compose_message(*this);
}

sc_report::sc_report(const sc_report &r) :
    sc_report(r._severity, r._msgType, r._msg, r._verbosity, r._fileName,
            r._lineNumber, r._time, r._processName, r._id)
{}

sc_report &
sc_report::operator = (const sc_report &r)
{
    _severity = r._severity;
    _msgType = r._msgType;
    _msg = r._msg;
    _verbosity = r._verbosity;
    _fileName = r._fileName;
    _lineNumber = r._lineNumber;
    _time = r._time;
    _processName = r._processName;
    _id = r._id;
    return *this;
}

sc_report::~sc_report() throw() {}

const char *
sc_report::what() const throw()
{
    return _what.c_str();
}

const char *
sc_report::get_message(int id)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

bool
sc_report::is_suppressed(int id)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

void
sc_report::make_warnings_errors(bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_report::register_id(int id, const char *msg)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_report::suppress_id(int id, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_report::suppress_infos(bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_report::suppress_warnings(bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_abort()
{
    panic("simulation aborted");
}

} // namespace sc_core
