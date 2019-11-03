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

  data_gen.cpp -- 

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
/***************************    data_gen Function        **********************/
/******************************************************************************/

SC_MODULE( DATA_GEN )
{
    SC_HAS_PROCESS( DATA_GEN );

    sc_in_clk clk;

  /*** Input and Output Ports ***/
  const sc_signal<bool>&       	ready;
        signal_bool_vector8&   	data;
        sc_signal<int>&		addr;
 
  /*** Constructor ***/
  DATA_GEN (   sc_module_name                	NAME,
                     sc_clock&            	TICK_N,
               const sc_signal<bool>&    	READY,
                     signal_bool_vector8& 	DATA,
  		     sc_signal<int>&		ADDR   )
 
    : 
		ready (READY),
              	data  (DATA),	// 8 bits
		addr  (ADDR)

    { 
	    clk (TICK_N);
        SC_CTHREAD( entry, clk.neg() );
    }
 
  /*** Call to Process Functionality ***/
  void entry();
 
};
 
void
DATA_GEN::entry()
{
  while(true) {

/**  WAIT FOR POSEDGE OF ready  **/

    do { wait(); } while (ready == 1);		// Posedge ready
    do { wait(); } while (ready == 0);

/**  CHECK TO SEE IF THE END OF MEMORY HAS BEEN REACHED  **/

    if(addr.read() > LIMIT) { 		// if(addr > LIMIT)
	break; 
    }

/**  WRITE VALUE OF MEMORY AT CURRENT ADDRESS TO data  **/

    data.write(mem[addr.read()]);	// data = mem[addr]

/**  INCREMENT addr BY 1  **/

    addr.write(addr.read() + 1);   	// addr = addr + 1 
  }

}

void
f_DATA_GEN (   const char*          NAME,
                     sc_clock&      TICK,
               const sc_signal<bool>&     READY,
                     signal_bool_vector8& DATA,
                     sc_signal<int>&      ADDR   )
 
{
        new DATA_GEN(NAME, TICK, READY, DATA, ADDR); 
}

