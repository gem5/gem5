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

  rdy.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

/******************************************************************************/
/***************************    rdy Function             **********************/
/******************************************************************************/

SC_MODULE( RDY )
{
    SC_HAS_PROCESS( RDY );

    sc_in_clk clk;

  /*** Input and Output Ports ***/
  sc_signal<bool>& data;
 
  /*** Constructor ***/
  RDY (   sc_module_name        	NAME,
                sc_clock&		TICK_N,
                sc_signal<bool>&  	DATA )
 
    : 
		data (DATA)

    {
        clk (TICK_N);
		SC_CTHREAD( entry, clk.neg() );
    }
 
  /*** Call to Process Functionality ***/
  void entry();
 
};

void
RDY::entry()
{
  // int a;
  int a = 0;

  cout << "\nSTART OF SIM -- CLOCK AT NEGEDGE (10,30,50,...) " << endl;
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;

  a = 0;		cout << "\t\t\t a =    0 " << endl;
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  data.write(0);	cout << "      ready =    0 " << endl; 
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  wait();		cout << "\nCLK " << endl;
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;

  a = 1;		cout << "\t\t   a =    1 " << endl;
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  data.write(1);	cout << "      ready =    1 " << endl; 
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  wait(); 		cout << "\nCLK " << endl;	
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;

  a = 0;		cout << "\t\t   a =    0 " << endl;
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  data.write(0);	cout << "      ready =    0 " << endl; 
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  wait();		cout << "\nCLK " << endl;
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;

  a = 1;		cout << "\t\t   a =    1 " << endl;
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  data.write(1);	cout << "      ready =    1 " << endl; 
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;
  wait(); 		cout << "\nCLK " << endl;	
  cout << sc_time_stamp() << " : "	
       << " ready[S] = " << data 
       << " a[V] = " << a
       << endl;

  halt();
}
