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

  sc_report_handler.h - 

  Original Author: Alex Riesen, Synopsys, Inc.
  see also sc_report.h

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef SC_REPORT_HANDLER_H
#define SC_REPORT_HANDLER_H

namespace sc_core {

// ----------------------------------------------------------------------------
//  STRUCT : sc_msg_def
//
//  Exception message definition structure
// ----------------------------------------------------------------------------

struct sc_msg_def
{
    const char*  msg_type;
    sc_actions   actions;
    sc_actions   sev_actions[SC_MAX_SEVERITY];
    unsigned     limit;
    unsigned     sev_limit[SC_MAX_SEVERITY];
    unsigned     limit_mask; // 0 - limit, 1..4 - sev_limit
    unsigned     call_count;
    unsigned     sev_call_count[SC_MAX_SEVERITY];
    char*        msg_type_data;

    int          id; // backward compatibility with 2.0+
};

typedef void (* sc_report_handler_proc)(const sc_report&, const sc_actions &);
class sc_report;
extern bool sc_report_close_default_log();
class sc_report_handler
{
public:
    static void report(sc_severity,
		       const char* msg_type,
		       const char* msg,
		       const char* file,
		       int line);

    static void report( sc_severity, 
                        const char* msg_type, 
			const char* msg, 
                        int verbosity, 
			const char* file, 
			int line );

    static sc_actions set_actions(sc_severity,
				  sc_actions = SC_UNSPECIFIED);

    static sc_actions set_actions(const char * msg_type,
				  sc_actions = SC_UNSPECIFIED);

    static sc_actions set_actions(const char * msg_type,
				  sc_severity,
				  sc_actions = SC_UNSPECIFIED);

    static int stop_after(sc_severity, int limit = -1);
    static int stop_after(const char* msg_type, int limit = -1);
    static int stop_after(const char* msg_type, sc_severity, int limit = -1);

    static sc_actions suppress(sc_actions);
    static sc_actions suppress();
    static sc_actions force(sc_actions);
    static sc_actions force();

    static int get_count(sc_severity severity_);
    static int get_count(const char* msg_type_);
    static int get_count(const char* msg_type_, sc_severity severity_);

    static int get_verbosity_level();
    static int set_verbosity_level( int level );


    static void initialize(); // just reset counters
    static void release(); // initialize() needed for reports after it

    static sc_report_handler_proc set_handler(sc_report_handler_proc);
    static sc_report_handler_proc get_handler();
    // use set_handler(NULL); to restore default handler
    static void default_handler(const sc_report&, const sc_actions&);

    static sc_actions get_new_action_id();

    static sc_report* get_cached_report();
    static void clear_cached_report();

    // if filename is NULL, the previous log file name will be removed.
    // The provider of a report_handler supposed to handle this.
    // Return false if filename is not NULL and filename is already set.
    static bool set_log_file_name(const char* filename);
    static const char* get_log_file_name();

public: // private, actually

    struct msg_def_items
    {
	sc_msg_def*     md;        // have to point to sc_msg_def-s
	int             count;     // set to number of items in md[]
	bool            allocated; // used internally, previous value ignored
	msg_def_items*  next;      // used internally, previous value ignored
    };

    static void add_static_msg_types(msg_def_items *);
    static sc_msg_def* add_msg_type(const char * msg_type);

protected:

    static void cache_report(const sc_report&);
    static sc_actions execute(sc_msg_def*, sc_severity);

    static sc_actions   suppress_mask;
    static sc_actions   force_mask;
    static sc_actions   sev_actions[SC_MAX_SEVERITY];
    static unsigned     sev_limit[SC_MAX_SEVERITY];
    static unsigned     sev_call_count[SC_MAX_SEVERITY];
    static sc_report*   last_global_report;
    static sc_actions   available_actions;
    static char*        log_file_name;
    static int          verbosity_level;

    static msg_def_items*  messages;
    static msg_def_items   msg_terminator;

    static sc_report_handler_proc  handler;

    static sc_msg_def* mdlookup(const char* msg_type);

private: // backward compatibility with 2.0+

    friend class sc_report;
    static sc_msg_def* mdlookup(int id);

public:

    static void report(sc_severity,
		       int         id,
		       const char* add_msg,
		       const char* file,
		       int         line);

};

} // namespace sc_core

// $Log: sc_report_handler.h,v $
// Revision 1.5  2011/08/26 20:46:19  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/03/23 16:16:49  acg
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
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.
//

#endif

// Taf!
