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

/******************************************************************************/
/***************************  stimulus Class Definition    ********************/
/******************************************************************************/

#include "common.h"

SC_MODULE( STIM )
{
    SC_HAS_PROCESS( STIM );

    sc_in_clk clk;

          sc_signal<bool>& 	reset;
          sc_signal<bool>&    	in_ok;
          sc_signal<bool>&    	out_ok;
    const sc_signal<bool>&     	instrb;
    const sc_signal<bool>&     	outstrb;
          signal_bool_vector    &a1,&a2,&a3,&a4,&a5,&a6,&a7,&a8;// -128 to 127
    const signal_bool_vector    &d1,&d2,&d3,&d4,&d5,&d6,&d7,&d8;

 
    STIM( 	sc_module_name  		NAME,
	       	      sc_clock& 		TICK_P,
    		      sc_signal<bool>& 		RESET,
    		      sc_signal<bool>& 		IN_OK,
    		      sc_signal<bool>& 		OUT_OK,
    		const sc_signal<bool>&     	INSTRB,
    		const sc_signal<bool>&    	OUTSTRB,
    		      signal_bool_vector&       A1,
		      signal_bool_vector&	A2,
		      signal_bool_vector&	A3,
		      signal_bool_vector&	A4,
		      signal_bool_vector&	A5,
		      signal_bool_vector&	A6,
		      signal_bool_vector&	A7,
		      signal_bool_vector&	A8,
    		      signal_bool_vector&       D1,
		      signal_bool_vector&	D2,
		      signal_bool_vector&	D3,
		      signal_bool_vector&	D4,
		      signal_bool_vector&	D5,
		      signal_bool_vector&	D6,
		      signal_bool_vector&	D7,
		      signal_bool_vector&	D8
              ) 
        :
		reset	(RESET),
		in_ok	(IN_OK),
		out_ok	(OUT_OK),
		instrb 	(INSTRB),
		outstrb (OUTSTRB),
		a1      (A1), a2(A2), a3(A3), a4(A4),
                a5      (A5), a6(A6), a7(A7), a8(A8),
		d1      (D1), d2(D2), d3(D3), d4(D4),
                d5      (D5), d6(D6), d7(D7), d8(D8)
    {
        clk(TICK_P); 
        SC_CTHREAD( entry, clk.neg() );
    }
    void entry();
};

/******************************************************************************/
/*************************** testbench Entry Function    **********************/
/******************************************************************************/
void
STIM::entry()
{

// INITIAL INPUT VALUES

  a1.write(0);		// Are quotes necessary ???
  a2.write(0);
  a3.write(0);
  a4.write(0);
  a5.write(0);
  a6.write(0);
  a7.write(0);
  a8.write(0);
  in_ok.write(0);
  out_ok.write(0);
  reset.write(1);
  wait(2);

// REMOVE RESET 

  reset.write(0);
  wait();

// WAIT FOR REQUEST FOR INPUT 

  do { wait(); } while (instrb == 0);

// SEND INPUT DATA TO BE SORTED
  a1.write(-76);
  a2.write(  1);
  a3.write( 12);
  a4.write( 85);
  a5.write( 15);	
  a6.write(103);	
  a7.write( -2);	
  a8.write(  3);	
  in_ok.write(1);
  wait(); 

// WAIT FOR OUTPUT READY

  do { wait(); } while (outstrb == 0);

// READ OUTPUT & DISPLAY RESULTS

  cout << "\n" << endl;
  cout << "\t\t INPUT DATA \t\t SORTED DATA" << endl;
  cout << "\t\t " << a1.read().to_int() << "   \t\t " 
		  << d1.read().to_int() << endl;
  cout << "\t\t " << a2.read().to_int() << "   \t\t " 
		  << d2.read().to_int() << endl;
  cout << "\t\t " << a3.read().to_int() << "   \t\t " 
		  << d3.read().to_int() << endl;
  cout << "\t\t " << a4.read().to_int() << "   \t\t " 
		  << d4.read().to_int() << endl;
  cout << "\t\t " << a5.read().to_int() << "   \t\t " 
		  << d5.read().to_int() << endl;
  cout << "\t\t " << a6.read().to_int() << "   \t\t " 
		  << d6.read().to_int() << endl;
  cout << "\t\t " << a7.read().to_int() << "   \t\t " 
		  << d7.read().to_int() << endl;
  cout << "\t\t " << a8.read().to_int() << "   \t\t " 
		  << d8.read().to_int() << endl;
  cout << "\n" << endl;
 
// SEND FINISHED READING OUTPUT FLAG 

  in_ok.write(0);
  out_ok.write(1);
  wait();

// STOP SIMULATION

  sc_stop();
}
