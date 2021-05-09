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

#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "base/cprintf.hh"
#include "systemc/core/process.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/utils/messages.hh"
#include "systemc/ext/utils/sc_report_handler.hh"
#include "systemc/utils/report.hh"

namespace sc_core
{

namespace
{

std::unique_ptr<std::string> logFileName;
std::unique_ptr<std::ofstream> logFile;

} // anonymous namespace

void
sc_report_handler::report(sc_severity severity, const char *msg_type,
                          const char *msg, const char *file, int line)
{
    report(severity, msg_type, msg, SC_MEDIUM, file, line);
}

void
sc_report_handler::report(sc_severity severity, const char *msg_type,
                          const char *msg, int verbosity, const char *file,
                          int line)
{
    if (!msg_type)
        msg_type = SC_ID_UNKNOWN_ERROR_;

    if (severity == SC_INFO && verbosity > sc_gem5::reportVerbosityLevel)
        return;

    sc_gem5::ReportSevInfo &sevInfo = sc_gem5::reportSevInfos[severity];
    sc_gem5::ReportMsgInfo &msgInfo = sc_gem5::reportMsgInfoMap()[msg_type];

    sevInfo.count++;
    msgInfo.count++;
    msgInfo.sevCounts[severity]++;

    sc_actions actions = SC_UNSPECIFIED;
    if (msgInfo.sevActions[severity] != SC_UNSPECIFIED)
        actions = msgInfo.sevActions[severity];
    else if (msgInfo.actions != SC_UNSPECIFIED)
        actions = msgInfo.actions;
    else if (sevInfo.actions != SC_UNSPECIFIED)
        actions = sevInfo.actions;

    actions &= ~sc_gem5::reportSuppressedActions;
    actions |= sc_gem5::reportForcedActions;

    msgInfo.checkLimits(severity, actions);
    sevInfo.checkLimit(actions);

    ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
    sc_report report(severity, msg_type, msg, verbosity, file, line,
            sc_time::from_value(::sc_gem5::scheduler.getCurTick()),
            current ? current->name() : nullptr, msgInfo.id);

    if (actions & SC_CACHE_REPORT) {
        if (current) {
            current->lastReport(&report);
        } else {
            sc_gem5::globalReportCache =
                std::unique_ptr<sc_report>(new sc_report(report));
        }
    }

    sc_gem5::reportHandlerProc(report, actions);
}

void
sc_report_handler::report(sc_severity severity, int id, const char *msg,
                          const char *file, int line)
{
    std::string &msg_type = sc_gem5::reportIdToMsgMap()[id];

    if (sc_gem5::reportWarningsAsErrors && severity == SC_WARNING)
        severity = SC_ERROR;

    report(severity, msg_type.c_str(), msg, file, line);
}

sc_actions
sc_report_handler::set_actions(sc_severity severity, sc_actions actions)
{
    sc_gem5::ReportSevInfo &info = sc_gem5::reportSevInfos[severity];
    sc_actions previous = info.actions;
    info.actions = actions;
    return previous;
}

sc_actions
sc_report_handler::set_actions(const char *msg_type, sc_actions actions)
{
    if (!msg_type)
        msg_type = SC_ID_UNKNOWN_ERROR_;

    sc_gem5::ReportMsgInfo &info = sc_gem5::reportMsgInfoMap()[msg_type];
    sc_actions previous = info.actions;
    info.actions = actions;
    return previous;
}

sc_actions
sc_report_handler::set_actions(
        const char *msg_type, sc_severity severity, sc_actions actions)
{
    if (!msg_type)
        msg_type = SC_ID_UNKNOWN_ERROR_;

    sc_gem5::ReportMsgInfo &info = sc_gem5::reportMsgInfoMap()[msg_type];
    sc_actions previous = info.sevActions[severity];
    info.sevActions[severity] = actions;
    return previous;
}

int
sc_report_handler::stop_after(sc_severity severity, int limit)
{
    sc_gem5::ReportSevInfo &info = sc_gem5::reportSevInfos[severity];
    int previous = info.limit;
    info.limit = limit;
    return previous;
}

int
sc_report_handler::stop_after(const char *msg_type, int limit)
{
    if (!msg_type)
        msg_type = SC_ID_UNKNOWN_ERROR_;

    sc_gem5::ReportMsgInfo &info = sc_gem5::reportMsgInfoMap()[msg_type];
    int previous = info.limit;
    info.limit = limit;
    return previous;
}

int
sc_report_handler::stop_after(
        const char *msg_type, sc_severity severity, int limit)
{
    if (!msg_type)
        msg_type = SC_ID_UNKNOWN_ERROR_;

    sc_gem5::ReportMsgInfo &info = sc_gem5::reportMsgInfoMap()[msg_type];
    int previous = info.sevLimits[severity];
    info.sevLimits[severity] = limit;
    return previous;
}

int
sc_report_handler::get_count(sc_severity severity)
{
    return sc_gem5::reportSevInfos[severity].count;
}

int
sc_report_handler::get_count(const char *msg_type)
{
    if (!msg_type)
        msg_type = SC_ID_UNKNOWN_ERROR_;

    return sc_gem5::reportMsgInfoMap()[msg_type].count;
}

int
sc_report_handler::get_count(const char *msg_type, sc_severity severity)
{
    if (!msg_type)
        msg_type = SC_ID_UNKNOWN_ERROR_;

    return sc_gem5::reportMsgInfoMap()[msg_type].sevCounts[severity];
}

int
sc_report_handler::set_verbosity_level(int vl)
{
    int previous = sc_gem5::reportVerbosityLevel;
    sc_gem5::reportVerbosityLevel = vl;
    return previous;
}

int
sc_report_handler::get_verbosity_level()
{
    return sc_gem5::reportVerbosityLevel;
}


sc_actions
sc_report_handler::suppress(sc_actions actions)
{
    sc_actions previous = sc_gem5::reportSuppressedActions;
    sc_gem5::reportSuppressedActions = actions;
    return previous;
}

sc_actions
sc_report_handler::suppress()
{
    return suppress(SC_UNSPECIFIED);
}

sc_actions
sc_report_handler::force(sc_actions actions)
{
    sc_actions previous = sc_gem5::reportForcedActions;
    sc_gem5::reportForcedActions = actions;
    return previous;
}

sc_actions
sc_report_handler::force()
{
    return force(SC_UNSPECIFIED);
}


sc_actions
sc_report_handler::set_catch_actions(sc_actions actions)
{
    sc_actions previous = sc_gem5::reportCatchActions;
    sc_gem5::reportCatchActions = actions;
    return previous;
}

sc_actions
sc_report_handler::get_catch_actions()
{
    return sc_gem5::reportCatchActions;
}


void
sc_report_handler::set_handler(sc_report_handler_proc proc)
{
    sc_gem5::reportHandlerProc = proc;
}

void
sc_report_handler::default_handler(
        const sc_report &report, const sc_actions &actions)
{
    if (actions & SC_DISPLAY)
        gem5::cprintf("\n%s\n", sc_report_compose_message(report));

    if ((actions & SC_LOG) && logFile) {
        gem5::ccprintf(*logFile, "%s: %s\n", report.get_time().to_string(),
                 sc_report_compose_message(report));
    }
    if (actions & SC_STOP) {
        sc_stop_here(report.get_msg_type(), report.get_severity());
        sc_stop();
    }
    if (actions & SC_INTERRUPT)
        sc_interrupt_here(report.get_msg_type(), report.get_severity());
    if (actions & SC_ABORT)
        sc_abort();
    if (actions & SC_THROW) {
        ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
        if (current)
            current->isUnwinding(false);
        throw report;
    }
}

sc_actions
sc_report_handler::get_new_action_id()
{
    static sc_actions maxAction = SC_ABORT;
    maxAction = maxAction << 1;
    return maxAction;
}

sc_report *
sc_report_handler::get_cached_report()
{
    ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
    if (current)
        return current->lastReport();
    return ::sc_gem5::globalReportCache.get();
}

void
sc_report_handler::clear_cached_report()
{
    ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
    if (current) {
        current->lastReport(nullptr);
    } else {
        ::sc_gem5::globalReportCache = nullptr;
    }
}

bool
sc_report_handler::set_log_file_name(const char *new_name)
{
    if (!new_name) {
        logFile = nullptr;
        logFileName = nullptr;
        return false;
    } else {
        if (logFileName)
            return false;
        logFileName = std::unique_ptr<std::string>(new std::string(new_name));
        logFile = std::unique_ptr<std::ofstream>(new std::ofstream(new_name));
        return true;
    }
}

const char *
sc_report_handler::get_log_file_name()
{
    if (!logFileName)
        return nullptr;
    else
        return logFileName->c_str();
}

void
sc_interrupt_here(const char *msg_type, sc_severity)
{
    // Purposefully empty, for setting breakpoints supposedly.
}

void
sc_stop_here(const char *msg_type, sc_severity)
{
    // Purposefully empty, for setting breakpoints supposedly.
}

const std::string
sc_report_compose_message(const sc_report &report)
{
    std::ostringstream str;

    const char *sevName = sc_gem5::reportSeverityNames[report.get_severity()];
    int id = report.get_id();

    str << sevName << ": ";
    if (id >= 0) {
        gem5::ccprintf(str, "(%c%d) ", sevName[0], id);
    }
    str << report.get_msg_type();

    const char *msg = report.get_msg();
    if (msg && msg[0])
        str << ": " << msg;

    if (report.get_severity() > SC_INFO) {
        gem5::ccprintf(str, "\nIn file: %s:%d", report.get_file_name(),
                 report.get_line_number());

        ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
        const char *name = report.get_process_name();
        if (current && sc_is_running() && name) {
            gem5::ccprintf(str, "\nIn process: %s @ %s", name,
                    report.get_time().to_string());
        }
    }

    return str.str();
}

bool
sc_report_close_default_log()
{
    if (logFile) {
        logFile = nullptr;
        logFileName = nullptr;
        return false;
    }
    return true;
}

} // namespace sc_core
