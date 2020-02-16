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

#include "systemc/utils/report.hh"

namespace sc_gem5
{

const char *reportSeverityNames[] = {
    [sc_core::SC_INFO] = "Info",
    [sc_core::SC_WARNING] = "Warning",
    [sc_core::SC_ERROR] = "Error",
    [sc_core::SC_FATAL] = "Fatal"
};

ReportSevInfo reportSevInfos[sc_core::SC_MAX_SEVERITY] =
{
    [sc_core::SC_INFO] = ReportSevInfo(sc_core::SC_DEFAULT_INFO_ACTIONS),
    [sc_core::SC_WARNING] = ReportSevInfo(sc_core::SC_DEFAULT_WARNING_ACTIONS),
    [sc_core::SC_ERROR] = ReportSevInfo(sc_core::SC_DEFAULT_ERROR_ACTIONS),
    [sc_core::SC_FATAL] = ReportSevInfo(sc_core::SC_DEFAULT_FATAL_ACTIONS)
};

std::map<std::string, ReportMsgInfo> &
reportMsgInfoMap()
{
    static std::map<std::string, ReportMsgInfo> m;
    return m;
}

std::map<int, std::string> &
reportIdToMsgMap()
{
    static std::map<int, std::string> m;
    return m;
}

int reportVerbosityLevel = sc_core::SC_MEDIUM;

sc_core::sc_actions reportSuppressedActions = sc_core::SC_UNSPECIFIED;
sc_core::sc_actions reportForcedActions = sc_core::SC_UNSPECIFIED;
sc_core::sc_actions reportCatchActions = sc_core::SC_DISPLAY;

sc_core::sc_report_handler_proc reportHandlerProc =
    &sc_core::sc_report_handler::default_handler;

std::unique_ptr<sc_core::sc_report> globalReportCache;

bool reportWarningsAsErrors = false;

DefaultReportMessages::DefaultReportMessages(
        std::initializer_list<std::pair<int, const char *>> msgs)
{
    for (auto &p: msgs)
        sc_core::sc_report::register_id(p.first, p.second);
}

} // namespace sc_gem5
