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

#ifndef __SYSTEMC_EXT_UTILS__USING_HH__
#define __SYSTEMC_EXT_UTILS__USING_HH__

#include "_utils.hh"

using sc_core::sc_severity;
using sc_core::SC_INFO;
using sc_core::SC_WARNING;
using sc_core::SC_ERROR;
using sc_core::SC_FATAL;
using sc_core::SC_MAX_SEVERITY;
using sc_core::sc_verbosity;
using sc_core::SC_NONE;
using sc_core::SC_LOW;
using sc_core::SC_MEDIUM;
using sc_core::SC_HIGH;
using sc_core::SC_FULL;
using sc_core::SC_DEBUG;
using sc_core::sc_report;

using sc_core::sc_actions;
using sc_core::SC_UNSPECIFIED;
using sc_core::SC_DO_NOTHING;
using sc_core::SC_THROW;
using sc_core::SC_LOG;
using sc_core::SC_DISPLAY;
using sc_core::SC_CACHE_REPORT;
using sc_core::SC_INTERRUPT;
using sc_core::SC_STOP;
using sc_core::SC_ABORT;
using sc_core::sc_report_handler_proc;
using sc_core::sc_report_handler;
using sc_core::sc_interrupt_here;
using sc_core::sc_stop_here;
using sc_core::sc_report_compose_message;
using sc_core::sc_report_close_default_log;
using sc_core::SC_DEFAULT_INFO_ACTIONS;
using sc_core::SC_DEFAULT_WARNING_ACTIONS;
using sc_core::SC_DEFAULT_ERROR_ACTIONS;
using sc_core::SC_DEFAULT_FATAL_ACTIONS;

using sc_core::sc_trace_file;
using sc_core::sc_create_vcd_trace_file;
using sc_core::sc_close_vcd_trace_file;
using sc_core::sc_write_comment;
using sc_core::sc_trace;
using sc_core::sc_trace_delta_cycles;

using sc_core::sc_exception;

using sc_core::sc_vector_base;
using sc_core::sc_vector_iter;
using sc_core::sc_vector;
using sc_core::sc_vector_assembly;

using sc_dt::sc_abs;
using sc_dt::sc_max;
using sc_dt::sc_min;

using sc_core::sc_version_major;
using sc_core::sc_version_minor;
using sc_core::sc_version_patch;
using sc_core::sc_version_originator;
using sc_core::sc_version_release_date;
using sc_core::sc_version_prerelease;
using sc_core::sc_version_string;
using sc_core::sc_copyright_string;
using sc_core::sc_release;
using sc_core::sc_copyright;
using sc_core::sc_version;

using sc_core::SC_ID_UNKNOWN_ERROR_;
using sc_core::SC_ID_WITHOUT_MESSAGE_;
using sc_core::SC_ID_NOT_IMPLEMENTED_;
using sc_core::SC_ID_INTERNAL_ERROR_;
using sc_core::SC_ID_ASSERTION_FAILED_;
using sc_core::SC_ID_OUT_OF_BOUNDS_;
using sc_core::SC_ID_ABORT_;

using sc_core::SC_ID_REGISTER_ID_FAILED_;
using sc_core::SC_ID_STRING_TOO_LONG_;
using sc_core::SC_ID_FRONT_ON_EMPTY_LIST_;
using sc_core::SC_ID_BACK_ON_EMPTY_LIST_;
using sc_core::SC_ID_IEEE_1666_DEPRECATION_;
using sc_core::SC_ID_VECTOR_INIT_CALLED_TWICE_;
using sc_core::SC_ID_VECTOR_BIND_EMPTY_;
using sc_core::SC_ID_VECTOR_NONOBJECT_ELEMENTS_;

#endif  //__SYSTEMC_EXT_UTILS__USING_HH__
