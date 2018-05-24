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

  test02.cpp

  Original Author: Andy Goodrich, Forte Design Systems, 7 Apr 2005

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

#define GET_UNSIGNED(VALUE,OFFSET,EXPECTED) \
{ \
	right_sc_biguint32 = 1 << OFFSET; \
	VALUE.concat_get_data( right_sc_biguint32.get_raw(), OFFSET); \
	if ( right_sc_biguint32 != ((EXPECTED)<<OFFSET) ) \
	cout << __FILE__ << "(" << __LINE__ << ") : " << \
		#VALUE << ".concat_get_data(ulong*, " << #OFFSET << ") expected " \
		<< ((EXPECTED)<<OFFSET) << " got " << right_sc_biguint32 << endl; \
}

#define GET_UNSIGNEDS(OFFSET,EXPECTED) \
{ \
	GET_UNSIGNED(left_sc_bigint12,OFFSET,EXPECTED); \
	GET_UNSIGNED(left_sc_biguint12,OFFSET,EXPECTED); \
	GET_UNSIGNED(left_sc_int12,OFFSET,EXPECTED); \
	GET_UNSIGNED(left_sc_uint12,OFFSET,EXPECTED); \
	GET_UNSIGNED(left_sc_int12[1],OFFSET,((EXPECTED>>1)&1)); \
	GET_UNSIGNED(left_sc_uint12[1],OFFSET,((EXPECTED>>1)&1)); \
	GET_UNSIGNED(left_sc_int12(7,2),OFFSET,((EXPECTED>>2)&0x3f)); \
	GET_UNSIGNED(left_sc_uint12(7,2),OFFSET,((EXPECTED>>2)&0x3f)); \
}

#define GET_UINT64(VALUE,EXPECTED) \
{ \
	uint64 actual = VALUE.concat_get_uint64(); \
	if ( actual != (EXPECTED) ) \
	cout << __FILE__ << "(" << __LINE__ << ") : " << \
		#VALUE << ".const_get_uint64() expected " << (EXPECTED) << " got " \
		<< actual << endl; \
}

#define GET_UINT64S(EXPECTED) \
{ \
	GET_UINT64(left_sc_bigint12,EXPECTED) \
	GET_UINT64(left_sc_biguint12,EXPECTED) \
	GET_UINT64(left_sc_int12,EXPECTED) \
	GET_UINT64(left_sc_uint12,EXPECTED) \
	GET_UINT64(left_sc_int12[1],         ((EXPECTED>>1)&1)) \
	GET_UINT64(left_sc_uint12[1],        ((EXPECTED>>1)&1)) \
	GET_UINT64(left_sc_int12(7,2),       ((EXPECTED>>2)&0x3f)) \
	GET_UINT64(left_sc_uint12(7,2),((EXPECTED>>2)&0x3f)) \
}

#define LENGTH(LEFT,WIDTH) \
{ \
	int width = LEFT.concat_length(0); \
	if ( width != (WIDTH) ) \
    cout << __FILE__ << "(" << __LINE__ << ") : " \
		<< #LEFT << ".concat_length() expected " << (WIDTH) \
		<< " got " << width << endl; \
}

#define LENGTHS(WIDTH) \
{ \
	LENGTH(left_sc_bigint12,WIDTH) \
	LENGTH(left_sc_biguint12,WIDTH) \
	LENGTH(left_sc_int12,WIDTH) \
	LENGTH(left_sc_uint12,WIDTH) \
}

#define SET(LEFT,RIGHT,VALUE,OFFSET,EXPECTED) \
{ \
	LEFT.concat_set(RIGHT,OFFSET); \
	wait(); \
	uint64 actual = LEFT.concat_get_uint64(); \
	if ( actual != (EXPECTED) ) \
	cout << #LEFT << ".const_set_uint64(" << #RIGHT <<", " << VALUE << ") \
		<< expected " << (EXPECTED) << " got " << actual << endl; \
}
    
#define SET_SIGNED(VALUE,OFFSET,EXPECTED) \
{ \
	right_sc_bigint32 = VALUE; \
	SET(left_sc_bigint12,right_sc_bigint32,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_biguint12,right_sc_bigint32,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_int12,right_sc_bigint32,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_uint12,right_sc_bigint32,VALUE,OFFSET,EXPECTED); \
}

#define SET_S64(VALUE,OFFSET,EXPECTED) \
{ \
	right_s64 = VALUE; \
	SET(left_sc_bigint12,right_s64,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_biguint12,right_s64,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_int12,right_s64,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_uint12,right_s64,VALUE,OFFSET,EXPECTED); \
}

#define SET_UNSIGNED(VALUE,OFFSET,EXPECTED) \
{ \
	right_sc_biguint32 = VALUE; \
	SET(left_sc_bigint12,right_sc_biguint32,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_biguint12,right_sc_biguint32,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_int12,right_sc_biguint32,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_uint12,right_sc_biguint32,VALUE,OFFSET,EXPECTED); \
}

#define SET_U64(VALUE,OFFSET,EXPECTED) \
{ \
	right_u64 = VALUE; \
	SET(left_sc_bigint12,right_u64,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_biguint12,right_u64,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_int12,right_u64,VALUE,OFFSET,EXPECTED); \
	SET(left_sc_uint12,right_u64,VALUE,OFFSET,EXPECTED); \
}

#define SETS(VALUE,OFFSET,EXPECTED) \
	SET_S64(VALUE,OFFSET,EXPECTED)  \
	SET_SIGNED(VALUE,OFFSET,EXPECTED)  \
	SET_UNSIGNED(VALUE,OFFSET,EXPECTED)  \
	SET_U64(VALUE,OFFSET,EXPECTED) 

SC_MODULE(X)
{
    SC_CTOR(X)
	{
		SC_CTHREAD(sync, clk.pos());
	}
	void sync()
	{
		// for (;; )
		{
			LENGTHS(12);
			SETS(0x87654321,0,0x321);
			SETS(0x87654321,4,0x432);
			GET_UINT64S(0x432);
			GET_UNSIGNEDS(0,0x432);
			GET_UNSIGNEDS(4,0x432);
		}
	}

	sc_in_clk                	clk;
	sc_int<12>               	left_sc_int12;
	sc_bigint<12>            	left_sc_bigint12;
	sc_biguint<12>           	left_sc_biguint12;
	sc_uint<12>              	left_sc_uint12;

	sc_int<32>               right_sc_int32;
	sc_bigint<32>            right_sc_bigint32;
	sc_biguint<32>           right_sc_biguint32;
	sc_uint<32>              right_sc_uint32;
	int                      right_si;
	long                     right_sl;
	int64                    right_s64;
	unsigned int             right_ui;
	unsigned long            right_ul;
	uint64                   right_u64;
};

int sc_main( int argc, char* argv[] )
{	
	sc_clock clock;
	X x("x");
	x.clk(clock);
	sc_start(1000, SC_NS);

	cerr << "Program completed\n";
	return 0;
}
