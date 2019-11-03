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

  test12.cpp -- Test that part selections are returning the proper values
                when they are part of a concatenation. This tests is here
                because the wrong width mask was being used in the 2.2
                source.

  Original Author: Andy Goodrich, Forte Design Systems, 03 Feb 2010

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "systemc.h"

#define TEST(TARGET) \
    expected = TARGET(high,low); \
    result = (sc_uint<6>(0),TARGET(high,low)); \
    if ( result != expected ) \
    { \
	cout << dec; \
	cout << "Mismatch for " << #TARGET << "(" << high << "," << low << "):"\
	     << endl; \
	cout << "Expected " << hex << expected << " got " << result \
	     << dec << endl; \
    }

int sc_main(int argc, char* argv[])
{
    sc_bigint<37>  bi37;
    sc_biguint<37> bu37;
    sc_int<37>     i37;
    sc_uint<37>    u37;

    bi37 = 0x1fc070000ULL;
    bu37 = 0x1fc070000ULL;
    i37 = 0x1fc070000ULL;
    u37 = 0x1fc070000ULL;

    // Test sc_int<W>:

    cout << endl << "sc_int test" << endl;
    cout << "i37 = " << hex << i37 << endl;
    cout << "i37(33,0) = " << hex << i37(33,0) << endl;
    cout << "(sc_uint<6>(0),i37(33,0)) = " 
         << hex << (sc_uint<6>(0),i37(33,0)) 
         << endl;

    // Test sc_uint<W>:

    cout << endl << "sc_uint test" << endl;
    cout << "u37 = " << hex << u37 << endl;
    cout << "u37(33,0) = " << hex << u37(33,0) << endl;
    cout << "(sc_uint<6>(0),u37(33,0)) = " 
         << hex << (sc_uint<6>(0),u37(33,0)) 
         << endl;

    // Test sc_bigint<W>:

    cout << endl << "sc_bigint test" << endl;
    cout << "bi37 = " << hex << bi37 << endl;
    cout << "bi37(33,0) = " << hex << bi37(33,0) << endl;
    cout << "(sc_uint<6>(0),bi37(33,0)) = " 
         << hex << (sc_uint<6>(0),bi37(33,0)) 
         << endl;

    // Test sc_biguint<W>:

    cout << endl << "sc_biguint test" << endl;
    cout << "bu37 = " << hex << bu37 << endl;
    cout << "bu37(33,0) = " << hex << bu37(33,0) << endl;
    cout << "(sc_uint<6>(0),bu37(33,0)) = " 
         << hex << (sc_uint<6>(0),bu37(33,0)) 
         << endl;

    sc_bigint<64>  bi64;
    sc_biguint<64> bu64;
    sc_uint<64>    expected;
    sc_int<64>     i64;
    sc_uint<64>    result;
    sc_uint<64>    u64;

    bi64 = -1;
    bu64 = -1;
    i64 = -1;
    u64 = -1;
    for ( size_t low = 0; low < 64; low++ )
    {
	for ( size_t high = low; high < 64; high++ )
	{
	    TEST(bi64);
	    TEST(bu64);
	    TEST(i64);
	    TEST(u64);
	}
    }

    bi64 = 0x6666666666666666ULL;
    bu64 = 0x6aaaaaaaaaaaaaaaULL;
    i64 = 0x6aaaaaaaaaaaaaaaULL;
    u64 = 0x6aaaaaaaaaaaaaaaULL;
    for ( size_t low = 0; low < 64; low++ )
    {
	for ( size_t high = low; high < 64; high++ )
	{
	    TEST(bi64);
	    TEST(bu64);
	    TEST(i64);
	    TEST(u64);
	}
    }

    bi64 = 0xaaaaaaaaaaaaaaaaULL;
    bu64 = 0xaaaaaaaaaaaaaaaaULL;
    i64 = 0xaaaaaaaaaaaaaaaaULL;
    u64 = 0xaaaaaaaaaaaaaaaaULL;
    for ( size_t low = 0; low < 64; low++ )
    {
	for ( size_t high = low; high < 64; high++ )
	{
	    TEST(bi64);
	    TEST(bu64);
	    TEST(i64);
	    TEST(u64);
	}
    }

        
    cout << endl << "Program completed" << endl;

    return 0;
}
