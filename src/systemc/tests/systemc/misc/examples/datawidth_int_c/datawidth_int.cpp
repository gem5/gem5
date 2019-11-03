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

  datawidth_int.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "datawidth.h" 	
#include "stimgen.h" 	

                /*******************************************/
                /* Implementation Filename:  datawidth.cc  */
                /*******************************************/
 
void
datawidth::entry()
{
  int 	tmp_a;
  int	tmp_b;
  int  	tmp_result;

  while (true) {
    
    // HANDSHAKING
    do { wait(); } while (ready == 0);

    // COMPUTATION
    tmp_a = in1.read();
    tmp_b = in2.read();
    tmp_result = tmp_a + tmp_b;

    // WRITE OUTPUT
    result.write(tmp_result);		// result = in1 + in2
    wait();
  }
}
                /*****************************************/
                /* Implementation Filename:  stimgen.cc  */
                /*****************************************/
 
void
stimgen::entry()
{
  int i;
  int j;

  ready.write(0);

  for (i = 0; i < 64; i++) {		// integer in1 (6 bits of data)
    for (j = 0; j < 64; j++) {		// integer in2 (6 bits of data)
      in1.write(i);
      in2.write(j);
      ready.write(1);
      wait();
 
      ready.write(0);
      wait();

      cout << in1.read() << " + " << in2.read() 
	   << " = " << result.read() << endl;
    }
  }

  sc_stop();
}

                /***************************************/
                /* Main Filename:       main.cc        */
                /***************************************/
		/*				       */
		/*	int = int + int		       */
                /*                                     */
                /*      Max addition is 63 + 63        */
		/*				       */
                /***************************************/
 
int
sc_main(int ac, char *av[])
{

// Signal Instantiation
  sc_signal<int>   	  in1 		("in1");
  sc_signal<int>   	  in2		("in2");
  sc_signal<int>   	  result 	("result");   
  sc_signal<bool> 	  ready 	("ready");     

// Clock Instantiation
  sc_clock clk( "clock", 10, SC_NS, 0.5, 0, SC_NS); 

// Process Instantiation
  datawidth	D1 ("D1", clk, in1, in2, ready, result);

  stimgen	T1 ("T1", clk, result, in1, in2, ready);

// Simulation Run Control
  sc_start(); 
  return 0;
}
