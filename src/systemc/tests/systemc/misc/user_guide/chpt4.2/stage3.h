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

  stage3.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename stage3.h */
/* This is the interface file for synchronous process `stage3' */

#include "systemc.h"

SC_MODULE( stage3 )
{
  SC_HAS_PROCESS( stage3 );

  sc_in_clk clk;

  const sc_signal<double>& prod; //input
  const sc_signal<double>& quot; //input
  sc_signal<double>& powr; //output

  //Constructor 
  stage3(sc_module_name NAME,
	 sc_clock& CLK,
	 const sc_signal<double>& PROD,
	 const sc_signal<double>& QUOT,
	 sc_signal<double>& POWR)
    : prod(PROD), quot(QUOT), powr(POWR)
  {
    clk(CLK);
	SC_CTHREAD( entry, clk.pos() );
  }

  // Process functionality in member function below
  void entry();
};


