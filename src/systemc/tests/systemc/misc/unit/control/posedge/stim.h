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

  stim.h -- 

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
/***************************    stim Function            **********************/
/******************************************************************************/

SC_MODULE( STIM )
{
  SC_HAS_PROCESS( STIM );

  sc_in_clk clk;

  /*** Input and Output Ports ***/
  const sc_signal<bool>& data;
 
  /*** Constructor ***/
  STIM (   sc_module_name        	NAME,
                 sc_clock&		TICK_P,
           const sc_signal<bool>&  	DATA )
 
    : 
		data (DATA)

    {
      clk (TICK_P);
	  SC_CTHREAD( entry, clk.pos() );
    }
 
  /*** Call to Process Functionality ***/
  void entry();
 
};

void
STIM::entry()
{
  while(true) {
    at_posedge(data);			
    cout << sc_time_stamp() << " : "	
         << "POSEDGE READY DETECTED" 
	 << endl;
  }
}
