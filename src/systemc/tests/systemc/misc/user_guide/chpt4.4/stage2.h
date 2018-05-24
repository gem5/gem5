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

  stage2.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename stage2.h */
/* This is the interface file for synchronous process `stage2' */

#include "systemc.h"

SC_MODULE( stage2 )
{
  SC_HAS_PROCESS( stage2 );

  sc_in_clk clk;

  const sc_signal<double>& sum; //input
  const sc_signal<double>& diff; //input
  sc_signal<double>& prod; //output
  sc_signal<double>& quot; //output

  //Constructor 
  stage2(sc_module_name NAME,
	 sc_clock& CLK,
	 const sc_signal<double>& SUM,
	 const sc_signal<double>& DIFF,
	 sc_signal<double>& PROD,
	 sc_signal<double>& QUOT)
    : sum(SUM), diff(DIFF), prod(PROD), quot(QUOT)
  {
    clk(CLK);
	SC_CTHREAD( entry, clk.pos() );
  }

  // Process functionality in member function below
  void entry();
};


