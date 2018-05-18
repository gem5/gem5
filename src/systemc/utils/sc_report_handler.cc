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
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

void
sc_report_handler::report(sc_severity, const char *msg_type, const char *msg,
                          const char *file, int line)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_report_handler::report(sc_severity, const char *msg_type, const char *msg,
                          int verbosity, const char *file, int line)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_actions
sc_report_handler::set_actions(sc_severity, sc_actions)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}

sc_actions
sc_report_handler::set_actions(const char *msg_type, sc_actions)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}

sc_actions
sc_report_handler::set_actions(const char *msg_type, sc_severity, sc_actions)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}

int
sc_report_handler::stop_after(sc_severity, int limit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

int
sc_report_handler::stop_after(const char *msg_type, int limit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

int
sc_report_handler::stop_after(const char *msg_type, sc_severity, sc_actions)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

int
sc_report_handler::get_count(sc_severity)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

int
sc_report_handler::get_count(const char *msg_type)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

int
sc_report_handler::get_count(const char *msg_type, sc_severity)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

int
sc_report_handler::set_verbosity_level(int)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

int
sc_report_handler::get_verbosity_level()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}


sc_actions
sc_report_handler::suppress(sc_actions)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}

sc_actions
sc_report_handler::suppress()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}

sc_actions
sc_report_handler::force(sc_actions)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}

sc_actions
sc_report_handler::force()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}


void
sc_report_handler::set_handler(sc_report_handler_proc)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_report_handler::default_handler(const sc_report &, const sc_actions &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_actions
sc_report_handler::get_new_action_id()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_UNSPECIFIED;
}

sc_report *
sc_report_handler::get_cached_report()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return nullptr;
}

void
sc_report_handler::clear_cached_report()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

bool
sc_report_handler::set_log_file_name(const char *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

const char *
sc_report_handler::get_log_file_name()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return nullptr;
}

void
sc_interrupt_here(const char *msg_type, sc_severity)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_stop_here(const char *msg_type, sc_severity)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

} // namespace sc_core
