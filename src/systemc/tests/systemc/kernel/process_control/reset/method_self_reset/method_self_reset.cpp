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

  method_self_reset.cpp -- Test of method self reset.

  Original Author: Andy Goodrich

 *****************************************************************************/
// $Log: method_self_reset.cpp,v $
// Revision 1.4  2011/07/24 15:59:53  acg
//  Andy Goodrich: add statement I missed installing Philipp's patch.
//
// Revision 1.3  2011/07/24 15:58:39  acg
//  Philipp A. Hartmann: convert first time toggle to counter since need 3
//  states.
//
// Revision 1.2  2011/02/04 15:26:33  acg
//  Andy Goodrich: regolden for proper process control semantics.
//
// Revision 1.1  2011/01/28 19:48:36  acg
//  Andy Goodrich: first check in.
//

#include "systemc.h"

SC_MODULE(DUT)
{
    SC_CTOR(DUT)
    {
        SC_METHOD(method);
	sensitive << m_clk.pos();
    }
    void method()
    {
        static int trigger = 0;
        cout << "Entry " << endl;
	switch( trigger++ )
	{
	  case 0:
	    cout << "Issuing self reset " << endl;
	    sc_get_current_process_handle().reset();
	    sc_assert( false );
	  case 1:
	    break;
	  default:
	    trigger = 0;
	}
        cout << "Exit " << endl;
    }
    sc_in<bool> m_clk;
};

int sc_main(int argc, char* argv[])
{
    sc_clock        clock;
    DUT             dut("dut");

    dut.m_clk(clock);

    sc_start(3, SC_NS);

    cout << "Program completed" << endl;
    return 0;
}
