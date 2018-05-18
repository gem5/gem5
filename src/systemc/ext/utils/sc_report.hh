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

#ifndef __SYSTEMC_EXT_UTIL_SC_REPORT_HH__
#define __SYSTEMC_EXT_UTIL_SC_REPORT_HH__

#include <exception>

namespace sc_core
{

class sc_time;

enum sc_severity
{
    SC_INFO = 0,
    SC_WARNING,
    SC_ERROR,
    SC_FATAL,
    SC_MAX_SEVERITY
};

enum sc_verbosity
{
    SC_NONE = 0,
    SC_LOW = 100,
    SC_MEDIUM = 200,
    SC_HIGH = 300,
    SC_FULL = 400,
    SC_DEBUG = 500
};

class sc_report : public std::exception
{
  public:
    sc_report(const sc_report &);
    sc_report &operator = (const sc_report &);
    virtual ~sc_report() throw();

    sc_severity get_severity() const;
    const char *get_msg_type() const;
    const char *get_msg() const;
    int get_verbosity() const;
    const char *get_file_name() const;
    int get_line_number() const;

    const sc_time &get_time() const;
    const char *get_process_name() const;

    virtual const char *what() const throw();
};

// A non-standard function the Accellera datatypes rely on.
[[noreturn]] void sc_abort();

} // namespace sc_core

#endif  //__SYSTEMC_EXT_UTIL_SC_REPORT_HH__
