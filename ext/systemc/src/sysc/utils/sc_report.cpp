/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_report.cpp -- Run-time logging and reporting facilities

  Interface design by SystemC Verification Working Group.
  Implementation by Alex Riesen, Synopsys Inc.
  Original implementation by Martin Janssen, Synopsys Inc.
  Reference implementation by Cadence Design Systems, Inc., 2002-09-23:
  Norris Ip, Dean Shea, John Rose, Jasvinder Singh, William Paulsen,
  John Pierce, Rachida Kebichi, Ted Elkind, David Bailey.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/


#include <stdlib.h>
#include <string.h>

#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/utils/sc_stop_here.h"
#include "sysc/utils/sc_report.h"
#include "sysc/utils/sc_utils_ids.h"
#include <algorithm> // std::swap

namespace sc_core {


static void sc_deprecated_report_ids(const char* method)
{
    static bool warn_report_ids_deprecated=true;
    if ( warn_report_ids_deprecated )
    {
        std::string message;
	message = "integer report ids are deprecated, use string values: ";
	message += method;
        warn_report_ids_deprecated=false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_, message.c_str());
    }
}

static char empty_str[] = "";
static inline char * empty_dup(const char * p)
{
    if ( p && *p )
    {
        char* result;
        result = (char*)malloc(strlen(p)+1);
        strcpy(result, p);
        return result;
    }
    else
    {
        return empty_str;
    }
}

sc_report::sc_report() 
: severity(SC_INFO),
  md(0),
  msg(empty_dup(0)),
  file(empty_dup(0)),
  line(0),
  timestamp(new sc_time(sc_time_stamp())),
  process(0),
  m_verbosity_level(SC_MEDIUM),
  m_what(empty_dup(0))
{
}

sc_report::sc_report(sc_severity severity_,
		     const sc_msg_def* md_,
		     const char* msg_,
		     const char* file_,
		     int line_,
		     int verbosity_level)
: severity(severity_),
  md(md_),
  msg(empty_dup(msg_)),
  file(empty_dup(file_)),
  line(line_),
  timestamp(new sc_time(sc_time_stamp())),
  process(sc_get_current_process_b()),
  m_verbosity_level(verbosity_level),
  m_what( empty_dup( sc_report_compose_message(*this).c_str() ) )
{
}

sc_report::sc_report(const sc_report& other)
: std::exception(other),
  severity(other.severity),
  md(other.md),
  msg(empty_dup(other.msg)),
  file(empty_dup(other.file)),
  line(other.line),
  timestamp(new sc_time(*other.timestamp)),
  process(other.process),
  m_verbosity_level(other.m_verbosity_level),
  m_what(empty_dup(other.m_what))
{
}

sc_report & sc_report::operator=(const sc_report& other)
{
    sc_report copy(other);
    swap( copy );
    return *this;
}

void
sc_report::swap( sc_report & that )
{
    using std::swap;
    swap( severity,          that.severity );
    swap( md,                that.md );
    swap( msg,               that.msg );
    swap( file,              that.file );
    swap( line,              that.line );
    swap( timestamp,         that.timestamp );
    swap( process,           that.process );
    swap( m_verbosity_level, that.m_verbosity_level );
    swap( m_what,            that.m_what );
} 

sc_report::~sc_report() throw()
{
    if ( file != empty_str )
	free(file);
    if ( msg != empty_str )
	free(msg);
    delete timestamp;
    if ( m_what != empty_str )
    free(m_what);
}

const char * sc_report::get_msg_type() const
{
    return md->msg_type;
}

//
// backward compatibility with 2.0+
//

static bool warnings_are_errors = false;
static const char unknown_id[] = "unknown id";

void sc_report_handler::report(sc_severity severity_,
			       int         id_,
			       const char* msg_,
			       const char* file_,
			       int         line_ )
{
    sc_msg_def * md = sc_report_handler::mdlookup(id_);

    if ( !md )
    {
	md = sc_report_handler::add_msg_type(unknown_id);
	md->id = id_;
    }

    if ( severity_ == SC_WARNING && warnings_are_errors )
	severity_ = SC_ERROR;

    sc_actions actions = execute(md, severity_);
    sc_report rep(severity_, md, msg_, file_, line_);

    if ( actions & SC_CACHE_REPORT )
	cache_report(rep);

    if ( severity_ == SC_ERROR )
	actions |= SC_THROW;
    else if ( severity_ == SC_FATAL )
	actions |= SC_ABORT;

    handler(rep, actions);
}

void sc_report::register_id( int id, const char* msg )
{
    sc_deprecated_report_ids("sc_report::register_id()");
    if( id < 0 ) {
	SC_REPORT_ERROR( SC_ID_REGISTER_ID_FAILED_,
			 "invalid report id" );
    }
    if( msg == 0 ) {
	SC_REPORT_ERROR( SC_ID_REGISTER_ID_FAILED_,
			 "invalid report message" );
    }
    sc_msg_def * md = sc_report_handler::mdlookup(id);

    if ( !md )
	md = sc_report_handler::add_msg_type(msg);

    if ( !md ) {
	SC_REPORT_ERROR( SC_ID_REGISTER_ID_FAILED_,
			 "report_map insertion error" );
    }

    if( md->id != -1 ) {
	if( strcmp( msg, md->msg_type ) != 0 ) {
	    SC_REPORT_ERROR( SC_ID_REGISTER_ID_FAILED_,
			     "report id already exists" );
	}
	return;
    }
    md->id = id;
}

const char* sc_report::get_message( int id )
{
    sc_deprecated_report_ids("sc_report::get_message()");
    sc_msg_def* md = sc_report_handler::mdlookup(id);

    return md ? md->msg_type: unknown_id;
}

bool sc_report::is_suppressed( int id )
{
    sc_deprecated_report_ids("sc_report::is_suppressed()");
    sc_msg_def* md = sc_report_handler::mdlookup(id);

    return md ? md->actions == SC_DO_NOTHING: false; // only do-nothing set
}

void sc_report::suppress_id(int id_, bool suppress)
{
    sc_deprecated_report_ids("sc_report::suppress_id()");
    sc_msg_def* md = sc_report_handler::mdlookup(id_);

    if ( md )
	md->actions = suppress ? SC_DO_NOTHING: SC_UNSPECIFIED;
}

void sc_report::suppress_infos(bool suppress)
{
    sc_deprecated_report_ids("sc_report::supress_infos");
    sc_report_handler::sev_actions[SC_INFO] =
	suppress ? SC_DO_NOTHING: SC_DEFAULT_INFO_ACTIONS;
}

void sc_report::suppress_warnings(bool suppress)
{
    sc_deprecated_report_ids("sc_report::suppress_warnings");
    sc_report_handler::sev_actions[SC_WARNING] =
	suppress ? SC_DO_NOTHING: SC_DEFAULT_WARNING_ACTIONS;
}

void sc_report::make_warnings_errors(bool flag)
{
    sc_deprecated_report_ids("sc_report::make_warnings_errors");
    warnings_are_errors = flag;
}

int sc_report::get_id() const
{
    return md->id;
}

} // namespace sc_core

// $Log: sc_report.cpp,v $
// Revision 1.8  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.7  2011/08/26 20:43:01  acg
//  Andy Goodrich:
//    (1) Replaced strdup with new and strcpy to eliminate issue with the
//        Greenhills compiler.
//    (2) Moved modification log to the end of the file to eliminate line
//        skew when check-ins are done.
//
// Revision 1.6  2011/08/24 22:05:56  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.5  2011/05/05 17:46:04  acg
//  Philip A. Hartmann: changes in "swap" support.
//
// Revision 1.4  2011/03/23 16:16:48  acg
//  Andy Goodrich: finish message verbosity support.
//
// Revision 1.3  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.2  2011/02/01 23:02:05  acg
//  Andy Goodrich: IEEE 1666 2011 changes.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.7  2006/03/21 00:00:37  acg
//   Andy Goodrich: changed name of sc_get_current_process_base() to be
//   sc_get_current_process_b() since its returning an sc_process_b instance.
//
// Revision 1.6  2006/01/25 00:31:27  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.5  2006/01/24 22:02:30  acg
//  Andy Goodrich: switch deprecated features warnings to use a single message
//  id, SC_ID_IEEE_1666_DEPRECATION_.
//
// Revision 1.4  2006/01/24 20:53:41  acg
// Andy Goodrich: added warnings indicating that use of integer ids in reports
// is deprecated. Added tracing/sc_trace_ids.h to message list.
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// taf
