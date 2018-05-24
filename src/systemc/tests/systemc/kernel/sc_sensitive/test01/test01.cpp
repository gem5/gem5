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

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: test01.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:26:04  acg
// systemc_tests-2.3
//
// Revision 1.2  2006/01/19 00:46:58  acg
// Andy Goodrich: Added CVS logging.
//

// test of sc_sensitive::operator()(sc_cthread_process*, sc_in(inout)<bool>)

#include "systemc.h"


SC_MODULE( mod_a )
{
    sc_in<bool>    clk;
    sc_in<bool>    in1;
 
    void main_action1()
    { 
	int i = 0;
	while( true ) {
	    wait();
	    cout << "i = " << i << endl;
	    i ++;
	}
    }

    void main_action2()
    { 
	int j = 0;
	while( true ) {
	    wait();
	    cout << "j = " << j << endl;
	    j ++;
	}
    }
 
    SC_CTOR(mod_a)
    {
	SC_CTHREAD( main_action1, clk );
	SC_CTHREAD( main_action2, in1 );
    }
};

SC_MODULE( mod_b )
{
    sc_in<bool>    clk;
    sc_inout<bool> in1;
 
    void main_action()
    { 
	bool j = true;
	while( true ) {
	    wait();
	    in1->write( j );
	    j = !j;
	}
    }
 
    SC_CTOR( mod_b )
    {
	SC_CTHREAD( main_action, clk );
    }
};

int
sc_main( int, char*[] )
{
    sc_clock clk( "clk", 10, SC_NS );
    sc_clock clk1( "clk1", 5, SC_NS );
    sc_signal<bool> channel;
    mod_a a( "a" );
    mod_b b( "b" );

    b.clk( clk1 );
    b.in1( channel );   
    a.clk( clk );
    a.in1( channel );

    sc_start( 100, SC_NS );   

    return 0;
}
