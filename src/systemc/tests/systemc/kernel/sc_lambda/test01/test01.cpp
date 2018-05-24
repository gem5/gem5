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

  test01.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-03-22
                   Ucar Aziz, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems, 2005-11-10
  Description of Modification: Removal of Lambda exressions

 *****************************************************************************/
// $Log: test01.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:26:04  acg
// systemc_tests-2.3
//
// Revision 1.2  2006/01/24 21:04:54  acg
//  Andy Goodrich: replacement of deprecated features with their non-deprecated
//  counterparts.
//

// test of sc_lambda-style wait_until w/normal loop.

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in_clk   clk1;
    sc_in<bool> clk2;

    void main_action()
    {
	int i = 0;

	while( true ) {
	    do { wait(); } while ( !(clk2 == true) );
	    cout << "i = " << i << endl;
	    i ++;
	    wait();
	}
    }

    SC_CTOR( mod_a )
    {
	SC_CTHREAD( main_action, clk1.pos() );
    }
};

int
sc_main( int, char*[] )
{
    mod_a a( "a" );
    sc_clock clk1( "clk1", 0.1, SC_NS );
    sc_clock clk2( "clk2", 0.5, SC_NS );
    a.clk1( clk1 );
    a.clk2( clk2 );

    sc_start( 3, SC_NS );

    return 0;
}
