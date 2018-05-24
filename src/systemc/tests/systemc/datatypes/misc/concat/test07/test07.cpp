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

  test07.cpp --

  Original Author: Andy Goodrich, Forte Design Systems, 29 October 2007

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test various width concatenations for each major base type.

#include "systemc.h"

void checkout( const sc_unsigned* actual_p, const sc_unsigned* expected_p )
{
    cout << *actual_p <<" " << *expected_p << endl;
	cout <<"another line" << endl;
}

#define TEST(LEFT,RIGHT) \
{ \
    const sc_unsigned* actual_p = &(LEFT,RIGHT).value(); \
    const sc_unsigned* expected_p = &(high_int,low_int).value(); \
	if ( *actual_p != *expected_p ) \
	{ \
	    checkout(actual_p, expected_p); \
		cout << "!!! ERROR (" << #LEFT << "," << #RIGHT << "):" << endl; \
		cout << "    expected:  " << hex << *expected_p  << endl; \
		cout << "    actual:    " << hex << *actual_p << endl; \
		cout << "    diff mask: " << hex << ((high_int,low_int) ^ (LEFT,RIGHT))  << endl; \
	} \
}

int sc_main(int argc, char* argv[])
{
    for ( int cycle_i = 0; cycle_i < 3; cycle_i++ )
	{
		for ( int high_i = 64; high_i > 0; high_i-- )
		{
			for ( int low_i = 64; low_i > 0; --low_i )
			{
				sc_int_base  high_int(high_i);
				sc_int_base  low_int(low_i);
				sc_signed    high_signed(high_i);
				sc_signed    low_signed(low_i);
				sc_uint_base high_uint(high_i);
				sc_uint_base low_uint(low_i);
				sc_unsigned  high_unsigned(high_i);
				sc_unsigned  low_unsigned(low_i);

				switch ( cycle_i )
				{
				  case 0:
					low_int =       "0xaaaaaaaa55555555";
					low_signed =    "0xaaaaaaaa55555555";
					low_uint =      "0xaaaaaaaa55555555";
					low_unsigned =  "0xaaaaaaaa55555555";
					high_int =      "0xfedcba9876543210";
					high_signed =   "0xfedcba9876543210";
					high_uint =     "0xfedcba9876543210";
					high_unsigned = "0xfedcba9876543210";
				    break;
			      case 1:	
					low_int =       "0x0000000000000000";
					low_signed =    "0x0000000000000000";
					low_uint =      "0x0000000000000000";
					low_unsigned =  "0x0000000000000000";
					high_int =      "0xffffffffffffffff";
					high_signed =   "0xffffffffffffffff";
					high_uint =     "0xffffffffffffffff";
					high_unsigned = "0xffffffffffffffff";
					break;
			      case 2:	
					low_int =       "0xffffffffffffffff";
					low_signed =    "0xffffffffffffffff";
					low_uint =      "0xffffffffffffffff";
					low_unsigned =  "0xffffffffffffffff";
					high_int =      "0x0000000000000000";
					high_signed =   "0x0000000000000000";
					high_uint =     "0x0000000000000000";
					high_unsigned = "0x0000000000000000";
					break;
				}

				cout << endl << dec << "[" << high_i << "," << low_i << "]:" 
					 << endl;

				cout << hex << "  int      " << (high_int,low_int) << " <- (  " 
							<< high_int  << " , " << low_int << " )" << endl;

				TEST(high_int,low_signed);
				TEST(high_int,low_uint);
				TEST(high_int,low_unsigned);

				TEST(high_signed,low_int);
				TEST(high_signed,low_signed);
				TEST(high_signed,low_uint);
				TEST(high_signed,low_unsigned);

				TEST(high_uint,low_int);
				TEST(high_uint,low_signed);
				TEST(high_uint,low_uint);
				TEST(high_uint,low_unsigned);

				TEST(high_unsigned,low_int);
				TEST(high_unsigned,low_signed);
				TEST(high_unsigned,low_uint);
				TEST(high_unsigned,low_unsigned);
			}
		}
    }

	cout << "Program completed" << endl;
	return 0;
}
