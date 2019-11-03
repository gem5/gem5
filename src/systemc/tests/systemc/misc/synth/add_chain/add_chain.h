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

  add_chain.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "common.h"

/******************************************************************************/
/***************************   add_chain Class Definition  ********************/
/******************************************************************************/

SC_MODULE( ADD_CHAIN )
{
    SC_HAS_PROCESS( ADD_CHAIN );

    sc_in_clk clk;

    const sc_signal<bool>& 	rst;
    const signal_bool_vector8& 	a_in;
          signal_bool_vector4& 	sum_out;
    	  sc_signal<bool>& 	ready;
 
    ADD_CHAIN( sc_module_name  		NAME,
	       sc_clock& 		TICK_P,

    		const sc_signal<bool>&		RST,
    		const signal_bool_vector8&	A_IN,
          	      signal_bool_vector4&	SUM_OUT,
    	  	      sc_signal<bool>& 		READY
              ) 
        :
		rst	(RST), 
		a_in	(A_IN), 
		sum_out	(SUM_OUT),
		ready	(READY)	
    {
        clk(TICK_P);
        SC_CTHREAD( entry, clk.pos() );
	reset_signal_is(rst, false); 
    }
    void entry();
};

/******************************************************************************/
/***************************   add_chain Entry Function  **********************/
/******************************************************************************/
/**									     **/
/**  This function sums the number of 1's contained in a 8-bit data stream   **/ 
/**									     **/
/******************************************************************************/
void
ADD_CHAIN::entry()
{
  bool_vector4 sum;
  bool_vector8 a; 

    /***** Reset Initialization *****/
    sum_out.write(0);
    ready.write(1);
    wait(); 

    /***** MAIN LOOP *****/
    while(true) { 

      /***** Handshake *****/
      ready.write(0);
      wait();
 
      /***** Computation *****/
      sum = 0;
      a = a_in.read();	
 
      for (int i=0; i<=7; i=i+1) {
        sum = sum.to_uint() + a[i].to_bool();
        } 
 
      sum_out.write(sum);
 
      /***** Handshake *****/
      ready.write(1);
      wait();
    }
}
