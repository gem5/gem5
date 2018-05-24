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

  reset_stim.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "common.h"
#include "add_chain.h"

/******************************************************************************/
/***************************   reset_stim Function       **********************/
/******************************************************************************/
bool_vector8 mem[LIMIT + 1];  // Stimulus input memory

SC_MODULE( RESET_STIM )
{
    SC_HAS_PROCESS( RESET_STIM );

    sc_in_clk clk;

  /*** Input and Output Ports ***/
  sc_signal<bool>& 	ready;
  sc_signal<bool>& 	reset;
  sc_signal<int>&	addr;

  /*** Constructor ***/
  RESET_STIM (   sc_module_name    	NAME,
                      sc_clock&         TICK_N,
                      sc_signal<bool>&  READY,
                      sc_signal<bool>&  RESET,
  		      sc_signal<int>&	ADDR   )
 
    : 
		ready (READY),
		reset (RESET),
		addr  (ADDR)

    {
	    clk (TICK_N);
        SC_CTHREAD( entry, clk.neg() );
    }
 
  /*** Call to Process Functionality ***/
  void entry();
 
};
 
void
RESET_STIM::entry()
{

/**  LOAD MEMORY WITH DATA AT TIME ZERO  **/

  ifstream 		stimulus ("add_chain_FUNC/add_chain.dat");
  char			buffer[WIDTH+1];

  for(int i=1; i < LIMIT+1; i++) {
      stimulus >> buffer;
      mem[i] = buffer;  
  }
  
  stimulus.close();
 
/**  INITIALIZE reset AND addr, THEN REMOVE RESET AFTER 2 CLOCK CYCLES  **/ 

  reset.write(0);	// reset = 0
  addr.write(1);	// addr = 1
  wait(2);

  reset.write(1);	// reset = 1

/** WAIT FOR LAST MEMORY ADDRESS, THEN 3 CLOCKS, THEN STOP SIMULATION  **/

// do { wait(); } while (addr == LIMIT);
  do { wait(); } while (!(addr == LIMIT));
  wait(LATENCY);
  do { wait(); } while (ready != 1);			
  sc_stop();
  halt();
}

void
f_RESET_STIM (   const char*        NAME,
                       sc_clock&    TICK,
                       sc_signal<bool>&  READY,
                       sc_signal<bool>&  RESET,
  		       sc_signal<int>&	 ADDR   )

{
	new RESET_STIM(NAME, TICK, READY,RESET, ADDR);
}
