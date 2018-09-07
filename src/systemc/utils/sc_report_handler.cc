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

#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "base/logging.hh"
#include "systemc/core/process.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

namespace
{

std::unique_ptr<std::string> logFileName;
std::unique_ptr<std::ofstream> logFile;

struct ReportCatInfo
{
    explicit ReportCatInfo(sc_actions actions) :
        actions(actions), count(0), limit(-1)
    {}
    ReportCatInfo() : ReportCatInfo(SC_UNSPECIFIED) {}

    bool
    checkLimit(sc_actions &actions)
    {
        if (limit == 0 || count < limit)
            return false;
        if (limit != -1)
            actions |= SC_STOP;
        return true;
    }

    sc_actions actions;
    int count;
    int limit;
};

const char *severityNames[] = {
    [SC_INFO] = "Info",
    [SC_WARNING] = "Warning",
    [SC_ERROR] = "Error",
    [SC_FATAL] = "Fatal"
};

ReportCatInfo catForSeverity[SC_MAX_SEVERITY] =
{
    [SC_INFO] = ReportCatInfo(SC_DEFAULT_INFO_ACTIONS),
    [SC_WARNING] = ReportCatInfo(SC_DEFAULT_WARNING_ACTIONS),
    [SC_ERROR] = ReportCatInfo(SC_DEFAULT_ERROR_ACTIONS),
    [SC_FATAL] = ReportCatInfo(SC_DEFAULT_FATAL_ACTIONS)
};

std::map<std::string, ReportCatInfo> catForMsgType;
std::map<std::pair<std::string, sc_severity>, ReportCatInfo>
    catForSeverityAndMsgType;

int verbosityLevel = SC_MEDIUM;

sc_actions suppressedActions = SC_UNSPECIFIED;
sc_actions forcedActions = SC_UNSPECIFIED;
sc_actions catchActions = SC_UNSPECIFIED;

sc_report_handler_proc reportHandlerProc = &sc_report_handler::default_handler;

sc_actions maxAction = SC_ABORT;

std::unique_ptr<sc_report> globalReportCache;

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
    if (severity == SC_INFO && verbosity > verbosityLevel)
        return;

    ReportCatInfo &sevInfo = catForSeverity[severity];
    ReportCatInfo &msgInfo = catForMsgType[msg_type];
    ReportCatInfo &sevMsgInfo = catForSeverityAndMsgType[
        std::make_pair(std::string(msg_type), severity)];

    sevInfo.count++;
    msgInfo.count++;
    sevMsgInfo.count++;

    sc_actions actions = SC_UNSPECIFIED;
    if (sevMsgInfo.actions != SC_UNSPECIFIED)
        actions = sevMsgInfo.actions;
    else if (msgInfo.actions != SC_UNSPECIFIED)
        actions = msgInfo.actions;
    else if (sevInfo.actions != SC_UNSPECIFIED)
        actions = sevInfo.actions;

    actions &= ~suppressedActions;
    actions |= forcedActions;

    if (sevMsgInfo.checkLimit(actions) && msgInfo.checkLimit(actions))
        sevInfo.checkLimit(actions);

    ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
    sc_report report(severity, msg_type, msg, verbosity, file, line,
            sc_time::from_value(::sc_gem5::scheduler.getCurTick()),
            current ? current->name() : nullptr, -1);

    if (actions & SC_CACHE_REPORT) {
        if (current) {
            current->lastReport(&report);
        } else {
            globalReportCache =
                std::unique_ptr<sc_report>(new sc_report(report));
        }
    }

    reportHandlerProc(report, actions);
}

void
sc_report_handler::report(sc_severity, int id, const char *msg,
                          const char *file, int line)
{
    warn("%s:%d %s\n", file, line, msg);
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_actions
sc_report_handler::set_actions(sc_severity severity, sc_actions actions)
{
    ReportCatInfo &info = catForSeverity[severity];
    sc_actions previous = info.actions;
    info.actions = actions;
    return previous;
}

sc_actions
sc_report_handler::set_actions(const char *msg_type, sc_actions actions)
{
    ReportCatInfo &info = catForMsgType[msg_type];
    sc_actions previous = info.actions;
    info.actions = actions;
    return previous;
}

sc_actions
sc_report_handler::set_actions(
        const char *msg_type, sc_severity severity, sc_actions actions)
{
    ReportCatInfo &info = catForSeverityAndMsgType[
        std::make_pair(std::string(msg_type), severity)];
    sc_actions previous = info.actions;
    info.actions = actions;
    return previous;
}

int
sc_report_handler::stop_after(sc_severity severity, int limit)
{
    ReportCatInfo &info = catForSeverity[severity];
    int previous = info.limit;
    info.limit = limit;
    return previous;
}

int
sc_report_handler::stop_after(const char *msg_type, int limit)
{
    ReportCatInfo &info = catForMsgType[msg_type];
    int previous = info.limit;
    info.limit = limit;
    return previous;
}

int
sc_report_handler::stop_after(
        const char *msg_type, sc_severity severity, int limit)
{
    ReportCatInfo &info = catForSeverityAndMsgType[
        std::make_pair(std::string(msg_type), severity)];
    int previous = info.limit;
    info.limit = limit;
    return previous;
}

int
sc_report_handler::get_count(sc_severity severity)
{
    return catForSeverity[severity].count;
}

int
sc_report_handler::get_count(const char *msg_type)
{
    return catForMsgType[msg_type].count;
}

int
sc_report_handler::get_count(const char *msg_type, sc_severity severity)
{
    return catForSeverityAndMsgType[
        std::make_pair(std::string(msg_type), severity)].count;
}

int
sc_report_handler::set_verbosity_level(int vl)
{
    int previous = verbosityLevel;
    verbosityLevel = vl;
    return previous;
}

int
sc_report_handler::get_verbosity_level()
{
    return verbosityLevel;
}


sc_actions
sc_report_handler::suppress(sc_actions actions)
{
    sc_actions previous = suppressedActions;
    suppressedActions = actions;
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
    sc_actions previous = forcedActions;
    forcedActions = actions;
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
    sc_actions previous = catchActions;
    catchActions = actions;
    return previous;
}

sc_actions
sc_report_handler::get_catch_actions()
{
    return catchActions;
}


void
sc_report_handler::set_handler(sc_report_handler_proc proc)
{
    reportHandlerProc = proc;
}

void
sc_report_handler::default_handler(
        const sc_report &report, const sc_actions &actions)
{
    if (actions & SC_DISPLAY)
        cprintf("\n%s\n", sc_report_compose_message(report));

    if ((actions & SC_LOG) && logFile) {
        ccprintf(*logFile, "%s: %s\n", report.get_time().to_string(),
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
    maxAction = maxAction << 1;
    return maxAction;
}

sc_report_handler_proc
sc_report_handler::get_handler()
{
    return reportHandlerProc;
}

sc_report *
sc_report_handler::get_cached_report()
{
    ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
    if (current)
        return current->lastReport();
    return globalReportCache.get();
}

void
sc_report_handler::clear_cached_report()
{
    ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
    if (current) {
        current->lastReport(nullptr);
    } else {
        globalReportCache = nullptr;
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

    const char *sevName = severityNames[report.get_severity()];
    int id = report.get_id();

    str << sevName << ": ";
    if (id >= 0) {
        ccprintf(str, "(%c%d) ", sevName[0], id);
    }
    str << report.get_msg_type();

    const char *msg = report.get_msg();
    if (msg && msg[0])
        str << ": " << msg;

    if (report.get_severity() > SC_INFO) {
        ccprintf(str, "\nIn file: %s:%d", report.get_file_name(),
                 report.get_line_number());

        ::sc_gem5::Process *current = ::sc_gem5::scheduler.current();
        const char *name = report.get_process_name();
        if (current && sc_is_running() && name) {
            ccprintf(str, "\nIn process: %s @ %s", name,
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
