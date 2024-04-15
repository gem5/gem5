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

#include <cstring>

#include "base/logging.hh"
#include "systemc/ext/utils/messages.hh"
#include "systemc/ext/utils/sc_report.hh"
#include "systemc/ext/utils/sc_report_handler.hh"
#include "systemc/utils/report.hh"

namespace sc_core
{

sc_report::sc_report(sc_severity _severity, const char *msg_type,
                     const char *msg, int _verbosity, const char *_fileName,
                     int _lineNumber, sc_time _time, const char *_processName,
                     int _id)
    : _severity(_severity),
      _msgType(msg_type),
      _msg(msg),
      _verbosity(_verbosity),
      _fileName(_fileName),
      _lineNumber(_lineNumber),
      _time(_time),
      _processName(_processName),
      _id(_id)
{
    if (_msgType)
        _msgType = strdup(_msgType);
    if (_msg)
        _msg = strdup(_msg);
    _what = sc_report_compose_message(*this);
}

sc_report::sc_report(const sc_report &r)
    : sc_report(r._severity, r._msgType, r._msg, r._verbosity, r._fileName,
                r._lineNumber, r._time, r._processName, r._id)
{}

sc_report &
sc_report::operator=(const sc_report &r)
{
    _severity = r._severity;
    free((void *)_msgType);
    _msgType = r._msgType ? strdup(r._msgType) : nullptr;
    free((void *)_msg);
    _msg = r._msg ? strdup(r._msg) : nullptr;
    _verbosity = r._verbosity;
    _fileName = r._fileName;
    _lineNumber = r._lineNumber;
    _time = r._time;
    _processName = r._processName;
    _id = r._id;
    return *this;
}

sc_report::~sc_report() throw()
{
    free((void *)_msgType);
    free((void *)_msg);
}

const char *
sc_report::what() const throw()
{
    return _what.c_str();
}

const char *
sc_report::get_message(int id)
{
    auto it = sc_gem5::reportIdToMsgMap().find(id);
    if (it == sc_gem5::reportIdToMsgMap().end())
        return "unknown id";
    else
        return it->second.c_str();
}

bool
sc_report::is_suppressed(int id)
{
    auto it = sc_gem5::reportIdToMsgMap().find(id);
    if (it == sc_gem5::reportIdToMsgMap().end())
        return false;

    auto &msgInfo = sc_gem5::reportMsgInfoMap()[it->second];

    return (msgInfo.actions == SC_DO_NOTHING ||
            (msgInfo.sevActions[SC_INFO] == SC_DO_NOTHING &&
             msgInfo.sevActions[SC_WARNING] == SC_DO_NOTHING));
}

void
sc_report::make_warnings_errors(bool val)
{
    sc_gem5::reportWarningsAsErrors = val;
}

void
sc_report::register_id(int id, const char *msg)
{
    if (id < 0) {
        SC_REPORT_ERROR(SC_ID_REGISTER_ID_FAILED_, "invalid report id");
        return;
    }
    if (!msg) {
        SC_REPORT_ERROR(SC_ID_REGISTER_ID_FAILED_, "invalid report message");
        return;
    }
    auto p = sc_gem5::reportIdToMsgMap().insert(
        std::pair<int, std::string>(id, msg));
    if (!p.second) {
        SC_REPORT_ERROR(SC_ID_REGISTER_ID_FAILED_, "report id already exists");
    } else {
        sc_gem5::reportMsgInfoMap()[msg].id = id;
    }
}

void
sc_report::suppress_id(int id, bool suppress)
{
    auto it = sc_gem5::reportIdToMsgMap().find(id);
    if (it == sc_gem5::reportIdToMsgMap().end())
        return;

    if (suppress) {
        sc_gem5::reportMsgInfoMap()[it->second].sevActions[SC_INFO] =
            SC_DO_NOTHING;
        sc_gem5::reportMsgInfoMap()[it->second].sevActions[SC_WARNING] =
            SC_DO_NOTHING;
    } else {
        sc_gem5::reportMsgInfoMap()[it->second].sevActions[SC_INFO] =
            SC_UNSPECIFIED;
        sc_gem5::reportMsgInfoMap()[it->second].sevActions[SC_WARNING] =
            SC_UNSPECIFIED;
    }
}

void
sc_report::suppress_infos(bool suppress)
{
    if (suppress)
        sc_gem5::reportSevInfos[SC_INFO].actions = SC_DO_NOTHING;
    else
        sc_gem5::reportSevInfos[SC_INFO].actions = SC_DEFAULT_INFO_ACTIONS;
}

void
sc_report::suppress_warnings(bool suppress)
{
    if (suppress) {
        sc_gem5::reportSevInfos[SC_WARNING].actions = SC_DO_NOTHING;
    } else {
        sc_gem5::reportSevInfos[SC_WARNING].actions =
            SC_DEFAULT_WARNING_ACTIONS;
    }
}

void
sc_abort()
{
    panic("simulation aborted");
}

} // namespace sc_core
