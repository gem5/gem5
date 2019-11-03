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
  // IMPLICIT wait(); AT FIRST NEGEDGE
  cout << sc_time_stamp() << " : "	// Time 10
       << "ready = " << data 
       << "\t\t (RDY) "
       << endl;
  data.write(0); 

  wait();						// Time 30
  cout << sc_time_stamp() << " : "	
       << "ready = " << data 
       << "\t\t (RDY) "
       << endl;
  data.write(1); 

  wait(); 						// Time 50
  cout << sc_time_stamp() << " : "
       << "ready = " << data 
       << "\t\t (RDY) "
       << endl;
  data.write(0); 

  wait(); 						// Time 70
  cout << sc_time_stamp() << " : "
       << "ready = " << data 
       << "\t\t (RDY) "
       << endl;
  data.write(1); 

  wait(); 						// Time 90
  cout << sc_time_stamp() << " : "
       << "ready = " << data 
       << "\t\t (RDY) "
       << endl;

  halt();
}
