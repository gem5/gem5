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

  op_queue.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename op_queue.h */
/* This is the interface file for synchronous process `op_queue' */

#include "systemc.h"

SC_MODULE( op_queue )
{
  SC_HAS_PROCESS( op_queue );

  sc_in_clk clk;

  sc_in<float>  in;
  sc_in<bool>   pop;
  sc_out<float> out;

  const int queue_size; //internal variable
  float *queue; //internal variable

  // Constructor 
  op_queue( sc_module_name NAME,
	    sc_clock& CLK,
	    sc_signal<float>& IN1,
	    sc_signal<bool>& POP,
	    sc_signal<float>& OUT1,
	    int QUEUE_SIZE = 4 )
	: queue_size(QUEUE_SIZE)
  {
    clk(CLK);
    in(IN1); 
	pop(POP); 
	out(OUT1); 
	SC_CTHREAD( entry, clk.pos() );
    queue = new float[queue_size];
  }

  // Process functionality in member function below
  void entry();
};
