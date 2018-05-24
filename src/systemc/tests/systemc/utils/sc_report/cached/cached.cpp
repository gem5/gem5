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

  sc_report -- caching of report in process context

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

  id1 has actions: display, cache
  id2 has actions: display

 Issue report in this order in the following contextes: 

   retrieve report from thread1   -> no report
   retrieve report from method1   -> no report
   retrieve report from global    -> no report

     info/id1/a -> thread1  
  warning/id1/b -> method1
     info/id1/c -> global
  warning/id1/d -> thread2  
     info/id1/e -> method2

     info/id2/f -> thread1  
  warning/id2/g -> method1
     info/id2/h -> global
  warning/id2/i -> thread2  
     info/id2/j -> method2
  
   retrieve report from thread1   ->    info/id1/a
   retrieve report from method1   -> warning/id1/b
   retrieve report from global    ->    info/id1/c
   retrieve report from thread2   -> no report
   retrieve report from method2   -> no report
*/


static const char* severity2str[] = {
    "INFO", "WARNING", "ERROR", "FATAL", "UNKNOWN_SEVERITY"
};

void dump_cached_report(const char* ctx)
{
    sc_report* report = sc_report_handler::get_cached_report();
    cout << sc_time_stamp() 
	 << " from context '" << ctx << "' ";
    if (report) {
	cout << report->get_msg_type() 
	     << " " << severity2str[ report->get_severity() ] << endl
	     << " msg="      << report->get_msg()
	     << " file="     << report->get_file_name() 
	     << " line "     << report->get_line_number()
	     << " time="     << report->get_time();
	const char* name = report->get_process_name();
	cout << " process=" << (name ? name : "<none>") << endl;
    } else {
	cout << "<no cached report>\n";
    }
    sc_report_handler::clear_cached_report();
}

SC_MODULE( M )
{
    sc_in<bool> emit;  // 1: emit, 0: dump cahced report
    sc_in<const char*> id;
    sc_in<bool> ofs; 
    sc_event t1, t2, m1, m2;

    SC_CTOR( M ) {
	SC_THREAD( thread1 );
	sensitive << t1;
	dont_initialize();

	SC_THREAD( thread2 );
	sensitive << t2;
	dont_initialize();

	SC_METHOD( method1 );
	sensitive << m1;
	dont_initialize();

	SC_METHOD( method2 );
	sensitive << m2;
	dont_initialize();
    }
    void thread1() {
	while(1) {
	    if (emit)
		sc_report_handler::report(SC_INFO, id.read(), "aa"+ofs, "file_t1", 110+ofs);
	    else 
		dump_cached_report("t1");
	    wait();
	}
    }
    void method1() {
	if (emit)
	    sc_report_handler::report(SC_WARNING, id.read(), "bb"+ofs, "file_m1", 210+ofs);
	else 
	    dump_cached_report("m1");
    }
    void thread2() {
	while(1) {
	    if (emit)
		sc_report_handler::report(SC_WARNING, id.read(), "dd"+ofs, "file_t2", 120+ofs);
	    else 
		dump_cached_report("t2");
	    wait();
	}
    }
    void method2() {
	if (emit)
	    sc_report_handler::report(SC_INFO, id.read(), "ee"+ofs, "file_m2", 220+ofs);
	else 
	    dump_cached_report("m2");
    }
};


int sc_main(int,char**)
{
    sc_report_handler::set_actions( "ID1", SC_DISPLAY | SC_CACHE_REPORT );
    sc_report_handler::set_actions( "ID2", SC_DISPLAY );

    sc_signal<bool> emit;
    sc_signal<const char*> ID;
    sc_signal<bool> ofs;
    M uut("M");
    uut( emit,ID,ofs );

    emit = 0;
    ID="ID3";
    ofs=0;
    sc_start( 1,SC_NS );

    // dump initial cached reports
    cout << "Initial status:\n";
    uut.t1.notify();
    uut.m1.notify();
    uut.t2.notify();
    uut.m2.notify();
    dump_cached_report("global");
    sc_start( 1, SC_NS );

    // emit report ID1 everywhere
    emit = 1;
    ID="ID1";
    sc_start( 1,SC_NS );
    cout << "\n\nEmit ID1\n";
    uut.t1.notify();
    uut.m1.notify();
    sc_start( 1, SC_NS );
    sc_report_handler::report(SC_INFO, ID.read(), "cc", "file_g", 300);
    uut.t2.notify();
    uut.m2.notify();
    sc_start( 1, SC_NS );
    
    // emit report ID2 everywhere
    cout << "\n\nEmit ID2\n";
    emit = 1;
    ID="ID2";
    ofs=1;
    sc_start( 1,SC_NS );
    uut.t1.notify();
    uut.m1.notify();
    sc_start( 1, SC_NS );
    sc_report_handler::report(SC_INFO, ID.read(), "c", "file_g", 310);
    uut.t2.notify();
    uut.m2.notify();
    sc_start( 1, SC_NS );
    
    // dump cached reports: should be all ID1
    emit = 0;
    sc_start( 1,SC_NS );
    cout << "\n\nStatus:\n";
    uut.t1.notify();
    uut.t2.notify();
    uut.m1.notify();
    uut.m2.notify();
    dump_cached_report("global");
    sc_start( 1, SC_NS );

    // dump cached reports again
    // (should be all empty because dump clears cached report)
    cout << "\n\nStatus:\n";
    uut.t1.notify();
    uut.t2.notify();
    uut.m1.notify();
    uut.m2.notify();
    dump_cached_report("global");
    sc_start( 1, SC_NS );

    return 0;
}
