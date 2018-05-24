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

  test02.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test string conversion of sc_big[u]int

#include "systemc.h"

void
test_string_conversions()
{
    cout << "*** test_string_conversions" << endl;
    {
	cout << "sc_bigint" << endl;

        sc_bigint<8> a = -1;
        sc_bigint<8> b;
	std::string s;

	s = a.to_string();
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_BIN );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_BIN_US );
	cout << s << endl;
	// b = s.c_str();
	// sc_assert( b == a );

	s = a.to_string( SC_BIN_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_OCT );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_OCT_US );
	cout << s << endl;
	// b = s.c_str();
	// sc_assert( b == a );

	s = a.to_string( SC_OCT_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_HEX );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_HEX_US );
	cout << s << endl;
	// b = s.c_str();
	// sc_assert( b == a );

	s = a.to_string( SC_HEX_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_DEC );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_CSD );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	sc_bigint<8> c( a.to_string().c_str() );
	cout << c.to_string() << endl;

	c.range( 7, 0 ) = a.to_string().c_str();
	cout << c.range( 7, 0 ).to_string() << endl;
    }
    {
	cout << "sc_biguint" << endl;

        sc_biguint<8> a = -1;
        sc_biguint<8> b;
	std::string s;

	s = a.to_string();
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_BIN );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_BIN_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_BIN_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_OCT );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_OCT_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_OCT_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_HEX );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_HEX_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_HEX_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_DEC );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	s = a.to_string( SC_CSD );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );

	sc_biguint<8> c( a.to_string().c_str() );
	cout << c.to_string() << endl;

	c.range( 7, 0 ) = a.to_string().c_str();
	cout << c.range( 7, 0 ).to_string() << endl;
    }
}

int
sc_main( int, char*[] )
{
    test_string_conversions();

    return 0;
}
