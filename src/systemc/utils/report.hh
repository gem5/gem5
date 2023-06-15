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

#ifndef __SYSTEMC_UTILS_REPORT_HH__
#define __SYSTEMC_UTILS_REPORT_HH__

#include <initializer_list>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "systemc/ext/utils/sc_report.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_gem5
{

struct ReportMsgInfo
{
    explicit ReportMsgInfo() :
        actions(sc_core::SC_UNSPECIFIED), count(0), limit(-1),
        sevActions{ sc_core::SC_UNSPECIFIED, sc_core::SC_UNSPECIFIED,
                sc_core::SC_UNSPECIFIED, sc_core::SC_UNSPECIFIED },
        sevCounts{0, 0, 0, 0}, sevLimits{-1, -1, -1, -1}, id(-1)
    {}

    void
    checkLimits(sc_core::sc_severity severity, sc_core::sc_actions &actions)
    {
        int sevLimit = sevLimits[severity];
        int sevCount = sevCounts[severity];
        if ((limit != -1 && limit >= count) ||
                (sevLimit != 1 && sevLimit >= sevCount)) {
            actions |= sc_core::SC_STOP;
        }
    }

    sc_core::sc_actions actions;
    int count;
    int limit;

    sc_core::sc_actions sevActions[sc_core::SC_MAX_SEVERITY];
    int sevCounts[sc_core::SC_MAX_SEVERITY];
    int sevLimits[sc_core::SC_MAX_SEVERITY];

    int id;
};

struct ReportSevInfo
{
    explicit ReportSevInfo(sc_core::sc_actions actions) :
        actions(actions), count(0), limit(-1)
    {}

    void
    checkLimit(sc_core::sc_actions &actions)
    {
        if (limit != -1 && limit >= count)
            actions |= sc_core::SC_STOP;
    }

    sc_core::sc_actions actions;
    int count;
    int limit;
};

extern const char *reportSeverityNames[sc_core::SC_MAX_SEVERITY];
extern ReportSevInfo reportSevInfos[sc_core::SC_MAX_SEVERITY];

std::map<std::string, ReportMsgInfo> &reportMsgInfoMap();
std::map<int, std::string> &reportIdToMsgMap();

extern int reportVerbosityLevel;

extern sc_core::sc_actions reportSuppressedActions;
extern sc_core::sc_actions reportForcedActions;
extern sc_core::sc_actions reportCatchActions;

extern sc_core::sc_report_handler_proc reportHandlerProc;

// gem5-specific support for extra SystemC report handlers. Called _after_
// the default/set handler.
const std::list<sc_core::sc_report_handler_proc>
    &getExtraSystemCReportHandlers();
void addExtraSystemCReportHandler(sc_core::sc_report_handler_proc proc);
void removeExtraSystemCReportHandler(sc_core::sc_report_handler_proc proc);

extern std::unique_ptr<sc_core::sc_report> globalReportCache;

extern bool reportWarningsAsErrors;

struct DefaultReportMessages
{
  public:
    DefaultReportMessages(std::initializer_list<std::pair<int, const char *>>);
};

} // namespace sc_gem5

#endif // __SYSTEMC_UTILS_REPORT_HH__
