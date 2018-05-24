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

  test08.cpp --

  Original Author: Andy Goodrich, Forte Design Systems, 9 November 2007

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test concatenation uses that were reported not to work with SystemC 2.1.v1

#include "systemc.h"

int sc_main(int argc, char* argv[])
{
    { 
	    sc_uint<2> a = 1;
	    sc_uint<2> b = 2;
		sc_uint<10> f = 0 - (a,b);

		// subtraction of a concat from 0.

		if ( f != 0x3fa )
		{
			cout << __FILE__ << " " << __LINE__ << ": expected " << 0x3fa
			     << " actual " << f << dec << endl;
		}

	    // subtraction of a concat from a concat if left < right.

		f = (a,b) -  (b,a);
		if ( f != 0x3fd )
		{
			cout << __FILE__ << " " << __LINE__ << ": expected " << 0x3fd
			     << " actual " << f << dec << endl;
		}
	    
    }
	{ // multiplication times a negative integer.

		sc_uint<4> a = 10;
		int        b = 0xdd6283e4;
	    sc_uint<2> c = 1;
		sc_uint<40> e("0xfc7c016528");
		sc_uint<40> f = (c,a)*b;
		if ( f != e )
        {
			cout << __FILE__ << " " << __LINE__ << ": expected " << hex << e
			     << " actual " << f << dec << endl;
		}
	}

    { // divsion where dividend negative and of type int/long
		sc_uint<4> a = 14;
		int        b = 0xb292b9fe;
        sc_uint<2> c = 3;
		sc_uint<50> e = b / sc_biguint<6>(0x3e); 
        sc_uint<50> f = b / (c,a);

		if ( f != e )
        {
			cout << __FILE__ << " " << __LINE__ << ": expected " << hex << e
			     << " actual " << f << dec << endl;
		}

	}
	{ // modulo for negative int % concat
	  
	    sc_uint<4> a = 10;
		int        b = 0xbb6283e4;
		sc_uint<2> c = 1;
		sc_uint<50> e = b % sc_biguint<6>(0x1a); 
		sc_uint<50> f = b %(c,a);
		if ( f != e )
        {
			cout << __FILE__ << " " << __LINE__ << ": expected " << hex << e
			     << " actual " << f << dec << endl;
		}
	}

	{ // Bitwise or and xor, when operands are concat and unsigned int
	    
		sc_uint<4> a = 6;
		unsigned int b = 0xcc6690e6; 
		sc_uint<2> c = 3;
		sc_uint<34> e;
		sc_uint<34> f;
		e = b | 0x36;
		f = b | (c,a);
		if ( f != e )
        {
			cout << __FILE__ << " " << __LINE__ << ": expected " << hex << e
			     << " actual " << f << dec << endl;
		}

		e = b ^ 0x36; 
		f = b ^ (c,a);
		if ( f != e )
        {
			cout << __FILE__ << " " << __LINE__ << ": expected " << hex << e
			     << " actual " << f << dec << endl;
		}
	}

	cout << "Program completed" << endl;
	return 0;
}
