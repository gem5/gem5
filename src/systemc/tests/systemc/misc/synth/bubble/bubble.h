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

  bubble.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/******************************************************************************/
/***************************   bubble Class Definition     ********************/
/******************************************************************************/

#include "common.h"

SC_MODULE( BUBBLE )
{
    SC_HAS_PROCESS( BUBBLE );

    sc_in_clk clk;

    const sc_signal<bool>& 	reset;
    const sc_signal<bool>& 	in_ok;
    const sc_signal<bool>& 	out_ok;
    	  sc_signal<bool>& 	instrb;
    	  sc_signal<bool>& 	outstrb;
    const signal_bool_vector    &a1,&a2,&a3,&a4,&a5,&a6,&a7,&a8;// -128 to 127
          signal_bool_vector    &d1,&d2,&d3,&d4,&d5,&d6,&d7,&d8;// -128 to 127
 
    BUBBLE( 	sc_module_name  		NAME,
	       	      sc_clock& 		TICK_P,
    		const sc_signal<bool>& 		RESET,
    		const sc_signal<bool>& 		IN_OK,
    		const sc_signal<bool>& 		OUT_OK,
    	  	      sc_signal<bool>& 		INSTRB,
    	  	      sc_signal<bool>& 		OUTSTRB,
          	const signal_bool_vector& 	A1,
		const signal_bool_vector&	A2,
		const signal_bool_vector&	A3,
		const signal_bool_vector&	A4,
		const signal_bool_vector&	A5,
		const signal_bool_vector&	A6,
		const signal_bool_vector&	A7,
		const signal_bool_vector&	A8,
          	      signal_bool_vector& 	D1,
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
		instrb	(INSTRB),
		outstrb	(OUTSTRB),
		a1	(A1), a2(A2), a3(A3), a4(A4),
		a5	(A5), a6(A6), a7(A7), a8(A8),
		d1	(D1), d2(D2), d3(D3), d4(D4),
		d5	(D5), d6(D6), d7(D7), d8(D8)
    {
        clk(TICK_P); 
        SC_CTHREAD( entry, clk.pos() );
	reset_signal_is(reset,true);
    }
    void entry();
};

/******************************************************************************/
/***************************   bubble Entry Function     **********************/
/******************************************************************************/
/**									     **/
/**  This function is most likely a bubble sort algorithm ???		     **/ 
/**									     **/
/******************************************************************************/
void
BUBBLE::entry()
{
  bool_vector B[9];
  bool_vector C[9];
  int     		minel; 
  int     		x; 
  int			i;
  int			j;

// RESET INITIALIZATION

  while(true) {

    instrb.write(false);
    outstrb.write(false);
    minel = -500;
    x = 0;
    d1.write(0);
    d2.write(0);
    d3.write(0);
    d4.write(0);
    d5.write(0);
    d6.write(0);
    d7.write(0);
    d8.write(0);
    for (i = 1; i <= 8; i++) {
        B[i] = 0;
    }
    for (i = 1; i <= 8; i++) {
        C[i] = 0;
    }
    wait(); 

// READY SIGNAL FOR INPUT DATA

    wait(); 
    instrb.write(true);
    wait(); 

// INPUT HANDSHAKE & INPUT READ

    do { wait(); } while (!in_ok);

    instrb.write(false);
    wait();

    B[1] = a1.read(); B[2] = a2.read(); B[3] = a3.read(); B[4] = a4.read();	
    B[5] = a5.read(); B[6] = a6.read(); B[7] = a7.read(); B[8] = a8.read();	
    wait();

lout << "STARTING BUBBLE SORT" << endl;
// BUBBLE SORT ALGORITHM

    // EVERY ELEMENT
    for (i = 1; i <= 7; i++) {

      if (B[i].to_int() > B[i+1].to_int()) {  	// if #1 
         for (j = 1; j <= 8; j++) {
           C[j] = B[j];                          	// COPY
         } 

         B[i] = B[i+1];
         B[i+1] = C[i];
         minel = C[i].to_int();                    	// MOVE

         for (j = 1; j <= 8; j++) {
           C[j] = B[j];                          	// COPY
         } 

         if (i >= 2) {  // if #2
            x = i;
            while (x > 1) {
 	      if (B[x].to_int() > B[x-1].to_int()) {  
                  break;
              }
              else {
                  for (j = 1; j <= 8; j++) {
                    C[j] = B[j]; 
                  }
              
                  B[x] = B[x-1];
                  B[x-1] = C[x];                  	// MOVE

                  for (j = 1; j <= 8; j++) {
                    C[j] = B[j];               		// COPY
	          }
	      }  // end else
              x = x-1;
            }  //  end WHL Loop
         }  // end if #2
      }  // end if #1;
    }  // end FL3 Loop
    wait();
    
// WRITE OUTPUT & OUTPUT HANDSHAKE
 
    d1.write(C[1]); d2.write(C[2]); d3.write(C[3]); d4.write(C[4]);
    d5.write(C[5]); d6.write(C[6]); d7.write(C[7]); d8.write(C[8]);
    outstrb.write(true);	// Ready to give output data
    wait();
   
    do { wait(); } while (!out_ok); 

    outstrb.write(false);
    wait();
 
  }  // end Reset Loop
 
}
