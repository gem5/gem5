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

  sc_report_handler.cpp - 

  Original Author: Alex Riesen, Synopsys, Inc.
  see also sc_report.cpp

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#include <cstdio>
#include <stdlib.h>
#include <string.h>

#include "sysc/utils/sc_iostream.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/utils/sc_stop_here.h"
#include "sysc/utils/sc_report_handler.h"
#include "sysc/utils/sc_report.h"

namespace std {}

namespace sc_core {

int sc_report_handler::verbosity_level = SC_MEDIUM;

// not documented, but available
const std::string sc_report_compose_message(const sc_report& rep)
{
    static const char * severity_names[] = {
	"Info", "Warning", "Error", "Fatal"
    };
    std::string str;

    str += severity_names[rep.get_severity()];
    str += ": ";

    if ( rep.get_id() >= 0 ) // backward compatibility with 2.0+
    {
	char idstr[64];
	std::sprintf(idstr, "(%c%d) ",
		"IWEF"[rep.get_severity()], rep.get_id());
	str += idstr;
    }
    str += rep.get_msg_type();

    if( *rep.get_msg() )
    {
	str += ": ";
	str += rep.get_msg();
    }
    if( rep.get_severity() > SC_INFO )
    {
        char line_number_str[16];
	str += "\nIn file: ";
	str += rep.get_file_name();
	str += ":";
	std::sprintf(line_number_str, "%d", rep.get_line_number());
	str += line_number_str;
	sc_simcontext* simc = sc_get_curr_simcontext();

	if( simc && sc_is_running() )
	{
	    const char* proc_name = rep.get_process_name();

	    if( proc_name )
	    {
		str += "\nIn process: ";
		str += proc_name;
		str += " @ ";
		str += rep.get_time().to_string();
	    }
	}
    }

    return str;
}
bool sc_report_close_default_log();

static ::std::ofstream* log_stream = 0;
static
struct auto_close_log
{
    ~auto_close_log()
    {
	sc_report_close_default_log();
    }
} auto_close;

const char* sc_report::get_process_name() const
{
	return process ? process->name() : 0;
}


//
// The official handler of the exception reporting
//

void sc_report_handler::default_handler(const sc_report& rep,
					const sc_actions& actions)
{
    if ( actions & SC_DISPLAY )
	::std::cout << ::std::endl << sc_report_compose_message(rep) << 
		::std::endl;

    if ( (actions & SC_LOG) && get_log_file_name() )
    {
	if ( !log_stream )
	    log_stream = new ::std::ofstream(get_log_file_name()); // ios::trunc

	*log_stream << rep.get_time() << ": "
	    << sc_report_compose_message(rep) << ::std::endl;
    }
    if ( actions & SC_STOP )
    {
	sc_stop_here(rep.get_msg_type(), rep.get_severity());
	sc_stop();
    }
    if ( actions & SC_INTERRUPT )
	sc_interrupt_here(rep.get_msg_type(), rep.get_severity());

    if ( actions & SC_ABORT )
	abort();

    if ( actions & SC_THROW ) {
        sc_process_b* proc_p = sc_get_current_process_b();
        if( proc_p && proc_p->is_unwinding() )
            proc_p->clear_unwinding();
        throw rep; 
    }
}

// not documented, but available
bool sc_report_close_default_log()
{
    delete log_stream;
    sc_report_handler::set_log_file_name(NULL);

    if ( !log_stream )
	return false;

    log_stream = 0;
    return true;
}

int sc_report_handler::get_count(sc_severity severity_) 
{ 
   return sev_call_count[severity_]; 
} 

int sc_report_handler::get_count(const char* msg_type_) 
{ 
    sc_msg_def * md = mdlookup(msg_type_); 

    if ( !md ) 
        md = add_msg_type(msg_type_); 

    return md->call_count; 
} 

int sc_report_handler::get_count(const char* msg_type_, sc_severity severity_) 
{ 
    sc_msg_def * md = mdlookup(msg_type_); 

    if ( !md ) 
        md = add_msg_type(msg_type_); 

    return md->sev_call_count[severity_]; 
} 


//
// CLASS: sc_report_handler
// implementation
//

sc_msg_def * sc_report_handler::mdlookup(const char * msg_type_)
{
    if( !msg_type_ ) // if msg_type is NULL, report unknown error
        msg_type_ = SC_ID_UNKNOWN_ERROR_;

    for ( msg_def_items * item = messages; item; item = item->next )
    {
	for ( int i = 0; i < item->count; ++i )
	    if ( !strcmp(msg_type_, item->md[i].msg_type) )
		return item->md + i;
    }
    return 0;
}

// The calculation of actions to be executed
sc_actions sc_report_handler::execute(sc_msg_def* md, sc_severity severity_)
{
    sc_actions actions = md->sev_actions[severity_]; // high prio

    if ( SC_UNSPECIFIED == actions ) // middle prio
	actions = md->actions;

    if ( SC_UNSPECIFIED == actions ) // the lowest prio
	actions = sev_actions[severity_];

    actions &= ~suppress_mask; // higher than the high prio
    actions |= force_mask; // higher than above, and the limit is the highest

    unsigned * limit = 0;
    unsigned * call_count = 0;

    // just increment counters and check for overflow
    if ( md->sev_call_count[severity_] < UINT_MAX )
	md->sev_call_count[severity_]++;
    if ( md->call_count < UINT_MAX )
	md->call_count++;
    if ( sev_call_count[severity_] < UINT_MAX )
	sev_call_count[severity_]++;

    if ( md->limit_mask & (1 << (severity_ + 1)) )
    {
	limit = md->sev_limit + severity_;
	call_count = md->sev_call_count + severity_;
    }
    if ( !limit && (md->limit_mask & 1) )
    {
	limit = &md->limit;
	call_count = &md->call_count;
    }
    if ( !limit )
    {
	limit = sev_limit + severity_;
	call_count = sev_call_count + severity_;
    }
    if ( *limit == 0 )
    {
	// stop limit disabled
    }
    else if ( *limit != UINT_MAX )
    {
	if ( *call_count >= *limit )
	    actions |= SC_STOP; // force sc_stop()
    }
    return actions;
}

void sc_report_handler::report( sc_severity severity_, 
                                const char* msg_type_, 
				const char* msg_, 
				int verbosity_, 
				const char* file_, 
				int line_ )
{
    sc_msg_def * md = mdlookup(msg_type_);

    // If the severity of the report is SC_INFO and the specified verbosity 
    // level is greater than the maximum verbosity level of the simulator then 
    // return without any action.

    if ( (severity_ == SC_INFO) && (verbosity_ > verbosity_level) ) return;

    // Process the report:

    if ( !md )
	md = add_msg_type(msg_type_);

    sc_actions actions = execute(md, severity_);
    sc_report rep(severity_, md, msg_, file_, line_, verbosity_);

    if ( actions & SC_CACHE_REPORT )
	cache_report(rep);

    handler(rep, actions);
}

void sc_report_handler::report(sc_severity severity_,
			       const char * msg_type_,
			       const char * msg_,
			       const char * file_,
			       int line_)
{
    sc_msg_def * md = mdlookup(msg_type_);

    // If the severity of the report is SC_INFO and the maximum verbosity
    // level is less than SC_MEDIUM return without any action.

    if ( (severity_ == SC_INFO) && (SC_MEDIUM > verbosity_level) ) return;

    // Process the report:


    if ( !md )
	md = add_msg_type(msg_type_);

    sc_actions actions = execute(md, severity_);
    sc_report rep(severity_, md, msg_, file_, line_);

    if ( actions & SC_CACHE_REPORT )
	cache_report(rep);

    handler(rep, actions);
}

// The following method is never called by the simulator.

void sc_report_handler::initialize()
{
#if 0 // actually, i do not know whether we have to reset these.
    suppress();
    force();
    set_actions(SC_INFO,    SC_DEFAULT_INFO_ACTIONS);
    set_actions(SC_WARNING, SC_DEFAULT_WARNING_ACTIONS);
    set_actions(SC_ERROR,   SC_DEFAULT_ERROR_ACTIONS);
    set_actions(SC_FATAL,   SC_DEFAULT_FATAL_ACTIONS);
#endif

    sev_call_count[SC_INFO]    = 0;
    sev_call_count[SC_WARNING] = 0;
    sev_call_count[SC_ERROR]   = 0;
    sev_call_count[SC_FATAL]   = 0;

    msg_def_items * items = messages;

    while ( items != &msg_terminator )
    {
	for ( int i = 0; i < items->count; ++i )
	{
	    items->md[i].call_count = 0;
	    items->md[i].sev_call_count[SC_INFO]    = 0;
	    items->md[i].sev_call_count[SC_WARNING] = 0;
	    items->md[i].sev_call_count[SC_ERROR]   = 0;
	    items->md[i].sev_call_count[SC_FATAL]   = 0;
	}
	items = items->next;
    }

    // PROCESS ANY ENVIRONMENTAL OVERRIDES:

    const char* deprecation_warn = std::getenv("SC_DEPRECATION_WARNINGS");
    if ( (deprecation_warn!=0) && !strcmp(deprecation_warn,"DISABLE") )
    {
        set_actions("/IEEE_Std_1666/deprecated", SC_DO_NOTHING);
    }
}

// free the sc_msg_def's allocated by add_msg_type
// (or implicit msg_type registration: set_actions, abort_after)
// clear last_global_report.
void sc_report_handler::release()
{
    delete last_global_report;
    last_global_report = 0;
    sc_report_close_default_log();

    msg_def_items * items = messages, * newitems = &msg_terminator;
    messages = &msg_terminator;

    while ( items != &msg_terminator )
    {
	for ( int i = 0; i < items->count; ++i )
	    if ( items->md[i].msg_type == items->md[i].msg_type_data )
		free(items->md[i].msg_type_data);

	msg_def_items * prev = items;
	items = items->next;

	if ( prev->allocated )
	{
	    delete [] prev->md;
	    delete prev;
	}
	else
	{
	    prev->next = newitems;
	    newitems = prev;
	}
    }
    messages = newitems;
}

sc_msg_def * sc_report_handler::add_msg_type(const char * msg_type_)
{
    sc_msg_def * md = mdlookup(msg_type_);
    int          msg_type_len;

    if ( md )
	return md;

    msg_def_items * items = new msg_def_items;

    if ( !items )
	return 0;

    items->count = 1;
    items->md = new sc_msg_def[items->count];

    if ( !items->md )
    {
	delete items;
	return 0;
    }
    memset(items->md, 0, sizeof(sc_msg_def) * items->count);
    msg_type_len = strlen(msg_type_);
    if ( msg_type_len > 0 )
    {
	items->md->msg_type_data = (char*) malloc(msg_type_len+1);
	strcpy( items->md->msg_type_data, msg_type_ );
	items->md->id = -1; // backward compatibility with 2.0+
    }
    else
    {
	delete items->md;
	delete items;
	return 0;
    }
    items->md->msg_type = items->md->msg_type_data;
    add_static_msg_types(items);
    items->allocated = true;

    return items->md;
}

void sc_report_handler::add_static_msg_types(msg_def_items * items)
{
    items->allocated = false;
    items->next = messages;
    messages = items;
}

sc_actions sc_report_handler::set_actions(sc_severity severity_,
					  sc_actions actions_)
{
    sc_actions old = sev_actions[severity_];
    sev_actions[severity_] = actions_;
    return old;
}

sc_actions sc_report_handler::set_actions(const char * msg_type_,
					  sc_actions actions_)
{
    sc_msg_def * md = mdlookup(msg_type_);

    if ( !md )
	md = add_msg_type(msg_type_);

    sc_actions old = md->actions;
    md->actions = actions_;

    return old;
}

sc_actions sc_report_handler::set_actions(const char * msg_type_,
					  sc_severity severity_,
					  sc_actions actions_)
{
    sc_msg_def * md = mdlookup(msg_type_);

    if ( !md )
	md = add_msg_type(msg_type_);

    sc_actions old = md->sev_actions[severity_];
    md->sev_actions[severity_] = actions_;

    return old;
}

int sc_report_handler::stop_after(sc_severity severity_, int limit)
{
    int old = sev_limit[severity_];

    sev_limit[severity_] = limit < 0 ? UINT_MAX: (unsigned) limit;

    return old;
}

int sc_report_handler::stop_after(const char * msg_type_, int limit)
{
    sc_msg_def * md = mdlookup(msg_type_);

    if ( !md )
	md = add_msg_type(msg_type_);

    int old = md->limit_mask & 1 ? md->limit: UINT_MAX;

    if ( limit < 0 )
	md->limit_mask &= ~1;
    else
    {
	md->limit_mask |= 1;
	md->limit = limit;
    }
    return old;
}

int sc_report_handler::stop_after(const char * msg_type_,
				  sc_severity severity_,
				  int limit)
{
    sc_msg_def * md = mdlookup(msg_type_);

    if ( !md )
	md = add_msg_type(msg_type_);

    int mask = 1 << (severity_ + 1);
    int old = md->limit_mask & mask ?  md->sev_limit[severity_]: UINT_MAX;

    if ( limit < 0 )
	md->limit_mask &= ~mask;
    else
    {
	md->limit_mask |= mask;
	md->sev_limit[severity_] = limit;
    }
    return old;
}

sc_actions sc_report_handler::suppress(sc_actions mask)
{
    sc_actions old = suppress_mask;
    suppress_mask = mask;
    return old;
}

sc_actions sc_report_handler::suppress()
{
    return suppress(0);
}

sc_actions sc_report_handler::force(sc_actions mask)
{
    sc_actions old = force_mask;
    force_mask = mask;
    return old;
}

sc_actions sc_report_handler::force()
{
    return force(0);
}

sc_report_handler_proc
sc_report_handler::set_handler(sc_report_handler_proc handler_)
{
    sc_report_handler_proc old = handler;
    handler = handler_ ? handler_: &sc_report_handler::default_handler;
    return old;
}

sc_report_handler_proc
sc_report_handler::get_handler()
{
    return handler;
}

sc_report* sc_report_handler::get_cached_report()
{
    sc_process_b * proc = sc_get_current_process_b();

    if ( proc )
	return proc->get_last_report();

    return last_global_report;
}

void sc_report_handler::clear_cached_report()
{
    sc_process_b * proc = sc_get_current_process_b();

    if ( proc )
	proc->set_last_report(0);
    else
    {
	delete last_global_report;
	last_global_report = 0;
    }
}

sc_actions sc_report_handler::get_new_action_id()
{
    for ( sc_actions p = 1; p; p <<= 1 )
    {
	if ( !(p & available_actions) ) // free
	{
	    available_actions |= p;
	    return p;
	}
    }
    return SC_UNSPECIFIED;
}

bool sc_report_handler::set_log_file_name(const char* name_)
{
    if ( !name_ )
    {
	free(log_file_name);
	log_file_name = 0;
	return false;
    }
    if ( log_file_name )
	return false;

    log_file_name = (char*)malloc(strlen(name_)+1);
    strcpy(log_file_name, name_);
    return true;
}

const char * sc_report_handler::get_log_file_name()
{
    return log_file_name;
}

void sc_report_handler::cache_report(const sc_report& rep)
{
    sc_process_b * proc = sc_get_current_process_b();
    if ( proc )
	proc->set_last_report(new sc_report(rep));
    else
    {
	delete last_global_report;
	last_global_report = new sc_report(rep);
    }
}

//
// backward compatibility with 2.0+
//

sc_msg_def * sc_report_handler::mdlookup(int id)
{
    for ( msg_def_items * item = messages; item; item = item->next )
    {
	for ( int i = 0; i < item->count; ++i )
	    if ( id == item->md[i].id )
		return item->md + i;
    }
    return 0;
}

int sc_report_handler::get_verbosity_level() { return verbosity_level; }

int sc_report_handler::set_verbosity_level( int level )
{
    int result = verbosity_level;
    verbosity_level = level;
    return result;
}

//
// CLASS: sc_report_handler
// static variables
//

sc_actions sc_report_handler::suppress_mask = 0;
sc_actions sc_report_handler::force_mask = 0;

sc_actions sc_report_handler::sev_actions[SC_MAX_SEVERITY] =
{
    /* info  */ SC_DEFAULT_INFO_ACTIONS,
    /* warn  */ SC_DEFAULT_WARNING_ACTIONS,
    /* error */ SC_DEFAULT_ERROR_ACTIONS,
    /* fatal */ SC_DEFAULT_FATAL_ACTIONS
};

// Note that SC_FATAL has a limit of 1 by default

sc_actions sc_report_handler::sev_limit[SC_MAX_SEVERITY] =
{
    UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX
};
sc_actions sc_report_handler::sev_call_count[SC_MAX_SEVERITY] = { 0, 0, 0, 0 };

sc_report* sc_report_handler::last_global_report = NULL;
sc_actions sc_report_handler::available_actions =
    SC_DO_NOTHING |
    SC_THROW |
    SC_LOG |
    SC_DISPLAY |
    SC_CACHE_REPORT |
    SC_INTERRUPT |
    SC_STOP |
    SC_ABORT;

sc_report_handler_proc sc_report_handler::handler =
    &sc_report_handler::default_handler;

char * sc_report_handler::log_file_name = 0;

sc_report_handler::msg_def_items * sc_report_handler::messages =
    &sc_report_handler::msg_terminator;


//
// predefined messages
//

const char SC_ID_REGISTER_ID_FAILED_[] = "register_id failed";
const char SC_ID_UNKNOWN_ERROR_[]      = "unknown error";
const char SC_ID_WITHOUT_MESSAGE_[]    = "";
const char SC_ID_NOT_IMPLEMENTED_[]    = "not implemented";
const char SC_ID_INTERNAL_ERROR_[]     = "internal error";
const char SC_ID_ASSERTION_FAILED_[]   = "assertion failed";
const char SC_ID_OUT_OF_BOUNDS_[]      = "out of bounds";

#define DEFINE_MSG(id,n)                                                     \
    {                                                                        \
	(id),                                                                \
	0u, {0u}, /* actions */                                              \
	0u, {0u}, 0u, /* limits */                                           \
	0u, {0u}, NULL, /* call counters */                                  \
	n                                                                    \
    }

static sc_msg_def default_msgs[] = {
    DEFINE_MSG(SC_ID_REGISTER_ID_FAILED_, 800),
    DEFINE_MSG(SC_ID_UNKNOWN_ERROR_, 0),
    DEFINE_MSG(SC_ID_WITHOUT_MESSAGE_, 1),
    DEFINE_MSG(SC_ID_NOT_IMPLEMENTED_, 2),
    DEFINE_MSG(SC_ID_INTERNAL_ERROR_, 3),
    DEFINE_MSG(SC_ID_ASSERTION_FAILED_, 4),
    DEFINE_MSG(SC_ID_OUT_OF_BOUNDS_, 5)
};

sc_report_handler::msg_def_items sc_report_handler::msg_terminator =
{
    default_msgs,
    sizeof(default_msgs)/sizeof(*default_msgs),
    false,
    NULL
};

} // namespace sc_core

// $Log: sc_report_handler.cpp,v $
// Revision 1.9  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.8  2011/08/26 20:46:19  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.7  2011/08/07 19:08:08  acg
//  Andy Goodrich: moved logs to end of file so line number synching works
//  better between versions.
//
// Revision 1.6  2011/08/07 18:56:03  acg
//  Philipp A. Hartmann: added cast to ? : to eliminate clang warning message.
//
// Revision 1.5  2011/03/23 16:16:49  acg
//  Andy Goodrich: finish message verbosity support.
//
// Revision 1.4  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.3  2011/02/11 13:25:55  acg
//  Andy Goodrich: Philipp's changes for sc_unwind_exception.
//
// Revision 1.2  2011/02/01 23:02:05  acg
//  Andy Goodrich: IEEE 1666 2011 changes.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.7  2006/05/26 20:35:52  acg
//  Andy Goodrich: removed debug message that should not have been left in.
//
// Revision 1.6  2006/03/21 00:00:37  acg
//   Andy Goodrich: changed name of sc_get_current_process_base() to be
//   sc_get_current_process_b() since its returning an sc_process_b instance.
//
// Revision 1.5  2006/01/31 21:42:07  acg
//  Andy Goodrich: Added checks for SC_DEPRECATED_WARNINGS being defined as
//  DISABLED. If so, we turn off the /IEEE_Std_1666/deprecated message group.
//
// Revision 1.4  2006/01/26 21:08:17  acg
//  Andy Goodrich: conversion to use sc_is_running instead of deprecated
//  sc_simcontext::is_running()
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// Taf!
