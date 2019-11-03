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

  tb.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Common interface file for test bench
   Author: PRP 
   */

SC_MODULE( tb )
{
    SC_HAS_PROCESS( tb );

    sc_in_clk clk;

        // Output Reset Port
        sc_signal<bool>& reset_sig;

        // Output Data Ports
	sc_signal<int>& i1;
	sc_signal<int>& i2;
	sc_signal<int>& i3;
	sc_signal<int>& i4;
	sc_signal<int>& i5;

        // Output Control Ports
	sc_signal<bool>& cont1;
	sc_signal<bool>& cont2;
	sc_signal<bool>& cont3;

        // Input Data Ports
	const sc_signal<int>& o1;
	const sc_signal<int>& o2;
	const sc_signal<int>& o3;
	const sc_signal<int>& o4;
	const sc_signal<int>& o5;

	// Constructor
	tb (	
        sc_module_name NAME, 
	sc_clock& CLK,

        sc_signal<bool>& RESET_SIG,

	sc_signal<int>& I1,
	sc_signal<int>& I2,
	sc_signal<int>& I3,
	sc_signal<int>& I4,
	sc_signal<int>& I5,

	sc_signal<bool>& CONT1,
	sc_signal<bool>& CONT2,
	sc_signal<bool>& CONT3,

	const sc_signal<int>& O1,
	const sc_signal<int>& O2,
	const sc_signal<int>& O3,
	const sc_signal<int>& O4,
	const sc_signal<int>& O5)
	  : reset_sig(RESET_SIG), i1(I1),  i2(I2),  
	    i3(I3),  i4(I4), i5(I5), cont1 (CONT1), cont2 (CONT2), 
	    cont3 (CONT3), o1(O1),  o2(O2),  o3(O3),  o4(O4),  o5(O5) 
        {
	  clk(CLK);
             SC_CTHREAD( entry, clk.pos() );
	}

  void entry();
};
