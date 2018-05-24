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

  display.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "common.h"

/****************************************************************/
/**    Display to standard out and to logfile "systemc.log"     **/
/****************************************************************/

ofstream lout ("systemc.log");                   // Output log file

/******************************************************************************/
/***************************    Output Display Function  **********************/
/******************************************************************************/

SC_MODULE( DISPLAY )
{
    SC_HAS_PROCESS( DISPLAY );

// INPUTS & OUTPUTS TO DISPLAY

    const sc_signal<bool>&      reset;
    const sc_signal<bool>&      in_ok;
    const sc_signal<bool>&      out_ok;
    const sc_signal<bool>&      instrb;
    const sc_signal<bool>&      outstrb;
    const signal_bool_vector    &a1,&a2,&a3,&a4,&a5,&a6,&a7,&a8; 
    const signal_bool_vector    &d1,&d2,&d3,&d4,&d5,&d6,&d7,&d8;  
 
// CONSTRUCTOR DEFINITION

    DISPLAY(    sc_module_name                  NAME,
                const sc_signal<bool>&          RESET,
                const sc_signal<bool>&          IN_OK,
                const sc_signal<bool>&          OUT_OK,
                const sc_signal<bool>&          INSTRB,
                const sc_signal<bool>&          OUTSTRB,
                const signal_bool_vector&       A1,
                const signal_bool_vector&       A2,
                const signal_bool_vector&       A3,
                const signal_bool_vector&       A4,
                const signal_bool_vector&       A5,
                const signal_bool_vector&       A6,
                const signal_bool_vector&       A7,
                const signal_bool_vector&       A8,
                const signal_bool_vector&       D1,
                const signal_bool_vector&       D2,
                const signal_bool_vector&       D3,
                const signal_bool_vector&       D4,
                const signal_bool_vector&       D5,
                const signal_bool_vector&       D6,
                const signal_bool_vector&       D7,
                const signal_bool_vector&       D8
              )

        :
                reset   (RESET),
                in_ok   (IN_OK),
                out_ok  (OUT_OK),
                instrb  (INSTRB),
                outstrb (OUTSTRB),
                a1      (A1), a2(A2), a3(A3), a4(A4),
                a5      (A5), a6(A6), a7(A7), a8(A8),
                d1      (D1), d2(D2), d3(D3), d4(D4),
                d5      (D5), d6(D6), d7(D7), d8(D8)

    {
        SC_METHOD( entry );
	sensitive << reset;
	sensitive << in_ok;
	sensitive << out_ok;
	sensitive << instrb;
	sensitive << outstrb;
	sensitive << a1;
	sensitive << a2;
	sensitive << a3;
	sensitive << a4;
	sensitive << a5;
	sensitive << a6;
	sensitive << a7;
	sensitive << a8;
	sensitive << d1;
	sensitive << d2;
	sensitive << d3;
	sensitive << d4;
	sensitive << d5;
	sensitive << d6;
	sensitive << d7;
	sensitive << d8;
    }
 
  /*** Call to Process Functionality ***/
  void entry();
 
};
 
void
DISPLAY::entry()
{
//  DISPLAYS ALL SIGNALS USED TO DEBUG DESIGN

      lout << " reset = " << reset 		
         << " in_ok = " << in_ok 	
         << " out_ok = " << out_ok 
         << " instrb = " << instrb
         << " outstrb = " << outstrb 		
	 << "\n"
         << " a1 = " << a1 		
         << " a2 = " << a2 	
         << " a3 = " << a3 
         << " a4 = " << a4
	 << "\n"
         << " a5 = " << a5 				
         << " a6 = " << a6 			
         << " a7 = " << a7 		
         << " a8 = " << a8 	
	 << "\n"
         << " d1 = " << d1 
         << " d2 = " << d2 					
         << " d3 = " << d3 				
         << " d4 = " << d4 			
	 << "\n"
         << " d5 = " << d5 		
         << " d6 = " << d6 	
         << " d7 = " << d7 				
         << " d8 = " << d8 			
         << endl;			

}
