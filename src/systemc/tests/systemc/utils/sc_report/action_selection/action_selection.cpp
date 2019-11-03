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

  sc_report -- test error reporting, sepcifically which actions are selected

  An important part of the sc_report functionality is to translate
  an reported tupel (id,severity) into a set of actions. This test
  is dedicated to this functionality. It uses user-defined actions and
  user-defined handler. 

  Original Author: Ulli Holtmann, Synopsys, Inc., Jan 2003

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>


/*
  Allocate user-defined action:

  Allocate as many user-defined actions as possible. 
  Note: We need to have at least usr1..usr6 action which are used in 
  subsequent tests.

*/

const unsigned num_usr_actions = 6;
sc_actions usr_actions[num_usr_actions];

sc_actions& usr1 = usr_actions[0];
sc_actions& usr2 = usr_actions[1];
sc_actions& usr3 = usr_actions[2];
sc_actions& usr4 = usr_actions[3];
sc_actions& usr5 = usr_actions[4];
sc_actions& usr6 = usr_actions[5];

const char* id1 = "ID1";
const char* id2 = "ID2";
const char* id3 = "ID3";
const char* id4 = "ID4";


void allocate_user_actions( )
{
    sc_actions usr;
    for (unsigned int n=0; n<1000; n++) {
	usr = sc_report_handler::get_new_action_id();
	if ( usr == SC_UNSPECIFIED ) {
	    cout << "We got " << n << " user-defined actions\n";
	    break;
	}
	// make sure we don't get the same usr action again and that it
	// is really new
	sc_assert (usr!=SC_UNSPECIFIED && usr!=SC_DO_NOTHING && usr!=SC_THROW &&
		usr!=SC_LOG         && usr!=SC_DISPLAY    && usr!=SC_CACHE_REPORT &&
		usr!=SC_STOP        && usr!=SC_ABORT );
	if ( n < num_usr_actions ) {
	    // save for later use
	    usr_actions[n] = usr;
	    for (unsigned int i=0; i<n; i++) {
		// lso check that is is new
		sc_assert( usr!=usr_actions[i]);
	    }
	}
    }
}


static const char* severity2str[] = {
    "INFO", "WARNING", "ERROR", "FATAL", "UNKNOWN_SEVERITY"
};

// custom handler which is used to dump out reports and actions
void dump_all_handler( const sc_report& report, const sc_actions& actions)
{
    // dump out report
    cout << "report: " << report.get_msg_type() 
	 << " " << severity2str[ report.get_severity() ];
    cout << " --> ";
    for (int n=0; n<32; n++) {
	sc_actions action = actions & 1<<n;
	if (action) {
	    cout << " ";
	    switch(action) {
	    case SC_UNSPECIFIED:  cout << "unspecified"; break;
	    case SC_DO_NOTHING:   cout << "do-nothing"; break;
	    case SC_THROW:        cout << "throw"; break;
	    case SC_LOG:          cout << "log"; break;         
	    case SC_DISPLAY:      cout << "display"; break;     
	    case SC_CACHE_REPORT: cout << "cache-report"; break;
	    case SC_INTERRUPT:    cout << "interrupt"; break;   
	    case SC_STOP:         cout << "stop"; break;     
	    case SC_ABORT:        cout << "abort"; break;     
	    default:
		bool found=false;
		for (unsigned int u=0; u<num_usr_actions; u++)
		    if (action == usr_actions[u]) {
			cout << "usr" << u+1;
			found=true;
			break;
		    }
		if (!found)
		    cout << "UNKNOWN";
	    }
	}
    }
    cout << endl;
    cout << " msg="      << report.get_msg()
	 << " file="     << report.get_file_name() 
	 << " line "     << report.get_line_number()
	 << " time="     << report.get_time();
	 const char* name = report.get_process_name();
    cout << " process=" << (name ? name : "<none>") << endl;
}


/*
  Test selection schema  id x severity :
  
            ID
  Severity   1       2       3

  info      usr2   usr1     usr5*   
  warning   usr3   usr1     usr3
  error     usr4   usr1     usr1*
  fatal     usr5   usr1     usr5

  usr1..usr5 are user-defined actions

  ID 1 selects by severity rule which has lowest priority,
  ID 2 selects by ID rule,
  ID 3 selects by  individual severity x ID rules (highest priority)

*/
void set_rules()
{
    // set rule 1: by severity
    sc_report_handler::set_actions( SC_INFO,    usr2 );
    sc_report_handler::set_actions( SC_WARNING, usr3 );
    sc_report_handler::set_actions( SC_ERROR,   usr4 );
    sc_report_handler::set_actions( SC_FATAL,   usr5 );
    
    // set rule 2: by id
    sc_report_handler::set_actions( id2, usr1 );

    // set rule 3: by (id,severity)
    sc_report_handler::set_actions ( id3, SC_INFO,  usr5 );
    sc_report_handler::set_actions ( id3, SC_ERROR, usr1 );
}
void query_rules( const char* id )
{
    sc_report_handler::report( SC_INFO,    id, "extra_msg_for_info",   "no_specific_file", 0);
    sc_report_handler::report( SC_WARNING, id, "extra_msg_for_warning","no_specific_file", 1);
    sc_report_handler::report( SC_ERROR,   id, "extra_msg_for_error",  "no_specific_file", 2);
    sc_report_handler::report( SC_FATAL,   id, "extra_msg_for_fatal",  "no_specific_file", 3);
    cout << endl;
}
void query_rules()
{
    query_rules( id1 );
    query_rules( id2 );
    query_rules( id3 );
}


int sc_main(int,char**)
{
    allocate_user_actions();
    sc_report_handler::set_handler( &dump_all_handler );

    // disable automatic stop 
    sc_report_handler::stop_after( SC_ERROR, 0 );
    sc_report_handler::stop_after( SC_FATAL, 0 );

    // Don't emit error|fatal for ID4 because this would 
    // terminate the simulation.
      cout << "Default settings for ID4\n";
      query_rules( id4 );

    // check default setting of rules
    cout << "Specific settings for ID1..ID3\n";
    set_rules();
    query_rules();

    // temporarily suppress usr4:
    // - check which actions are emitted
    // - check that suppress() restores old state
    // - check return value of suppress(.)
    cout << "temporarily suppress usr4\n";
    sc_start( 1,SC_NS );
    sc_report_handler::suppress( usr3 );
    sc_assert( sc_report_handler::suppress( usr4 ) == usr3 );
    query_rules( id1 );
    sc_assert( sc_report_handler::suppress() == usr4 );
    query_rules( id1 );

    // temporarily force usr1: same checking as with suppress
    cout << "temporarily force usr1\n";
    sc_start( 1,SC_NS );
    sc_report_handler::force( usr2 );
    sc_assert( sc_report_handler::force( usr1 ) == usr2 );
    query_rules( id1 );
    sc_assert( sc_report_handler::force() == usr1 );
    query_rules( id1 );

    // temporarily force usr1: same checking as with suppress
    cout << "temporarily suppress {usr3,usr4} and force {usr1,usr3}\n";
    sc_start( 1,SC_NS );
    sc_report_handler::force   ( usr1|usr3 );
    sc_report_handler::suppress( usr3|usr4 );
    query_rules( id1 );
    sc_report_handler::force();
    sc_report_handler::suppress();
    query_rules( id1 );

    return 0;
}
