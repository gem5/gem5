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

namespace sc_core
{

sc_report::sc_report(const sc_report &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_report &
sc_report::operator = (const sc_report &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_report::~sc_report() throw() {}

sc_severity
sc_report::get_severity() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_FATAL;
}

const char *
sc_report::get_msg_type() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

const char *
sc_report::get_msg() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

int
sc_report::get_verbosity() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_NONE;
}

const char *
sc_report::get_file_name() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

int
sc_report::get_line_number() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

const sc_time &
sc_report::get_time() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const sc_time *)nullptr;
}

const char *
sc_report::get_process_name() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

const char *
sc_report::what() const throw()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

void
sc_abort()
{
    panic("%s not implemented.\n", __PRETTY_FUNCTION__);
}

} // namespace sc_core
