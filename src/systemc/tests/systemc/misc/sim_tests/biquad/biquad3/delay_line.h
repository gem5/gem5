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

  delay_line.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename delay_line.h */
/* This is the interface file for synchronous process `delay_line' */

#include "systemc.h"

SC_MODULE( delay_line )
{
  SC_HAS_PROCESS( delay_line );

  sc_in<float>  in;
  sc_out<float> out;

  const int delay; //internal variable
  float *line; // delay line

  // Constructor 
  delay_line( sc_module_name NAME,
	      sc_signal<float>& IN1,
	      sc_signal<float>& OUT1,
	      int DELAY=4 )
    : delay(DELAY)
  {
    in(IN1); 
	out(OUT1);
	line = new float[delay];
    for(int i=0; i<delay; i++) line[i] = 0.0;

    SC_METHOD( entry );
    sensitive << in;
  }

  // Process functionality in member function below
  void entry();
};
