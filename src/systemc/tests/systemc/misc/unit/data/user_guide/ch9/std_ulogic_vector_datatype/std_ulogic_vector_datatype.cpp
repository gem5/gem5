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

  std_ulogic_vector_datatype.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Main file for "std_ulogic_vector" data type */ 

#include "systemc.h"

typedef sc_logic std_ulogic;

#define std_ulogic_vector sc_lv
#define bool_vector       sc_bv

int sc_main(int ac, char *av[])
{

// 0. SIZE OF TYPES
  int 		integer;
  short		short_integer;
  long		long_integer;
  unsigned long	unsigned_long;  
  signed long	signed_long;  

  cout << "\nINTEGER SIZE \t\t= " 	<< sizeof integer	<< " bytes"
       << "\nSHORT INTEGER SIZE \t= " 	<< sizeof short_integer	<< " bytes"
       << "\nLONG INTEGER SIZE \t= " 	<< sizeof long_integer	<< " bytes"
       << "\nUNSIGNED LONG SIZE \t= " 	<< sizeof unsigned_long	<< " bytes"
       << "\nSIGNED LONG SIZE \t= " 	<< sizeof signed_long	<< " bytes"
       << "\n" << endl;

// 1. DECLARATION SYNTAX
  std_ulogic_vector<9>		a;  
  std_ulogic_vector<9>		b;  
  std_ulogic_vector<68>		big;
  std_ulogic_vector<1284>	huge_;

// 2. TYPE CONVERSION

  // std_ulogic_vector <- C++ string
  a = "01XZUWLH-"; 
  b = "ZZ1XX0UU1WWW";
  big = "11110000111100001111000011110000111100001111000011110000111100001111";
  huge_ = "111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111";

  cout 	<< "std_ulogic_vector \t<=\t C++ string"
	<< "\n-------------------------------------------"
	<< "\nA = " << a << "\t\t \"01XZUWLH-\" "
	<< "\nB = " << b << "\t\t \"ZZ1XX0UU1WWW\" "
	<< "\nBIG  = " << big << "\n      "
  <<"\"11110000111100001111000011110000111100001111000011110000111100001111\""
	<< "\nHUGE = " << huge_.range(0,63) << "\n       "
		       << huge_.range(64,127) << "\n       " 
		       << huge_.range(128,191) << "\n       " 
		       << huge_.range(192,255) << "\n       " 
		       << huge_.range(256,319) << "\n       " 
		       << huge_.range(320,383) << "\n       " 
		       << huge_.range(384,447) << "\n       " 
		       << huge_.range(448,511) << "\n       " 
		       << huge_.range(512,575) << "\n       " 
		       << huge_.range(576,639) << "\n       " 
		       << huge_.range(640,703) << "\n       " 
		       << huge_.range(704,767) << "\n       " 
		       << huge_.range(768,831) << "\n       " 
		       << huge_.range(832,895) << "\n       " 
		       << huge_.range(896,959) << "\n       " 
		       << huge_.range(960,1023) << "\n       " 
		       << huge_.range(1024,1087) << "\n       " 
		       << huge_.range(1088,1151) << "\n       " 
		       << huge_.range(1152,1215) << "\n       " 
		       << huge_.range(1216,1279) << "\n       " 
		       << huge_.range(1280,1283) 
	<< "\n" << endl;

  // std_ulogic_vector <- std_ulogic_vector
  std_ulogic_vector<9>	c;  
  std_ulogic_vector<68>	big2;

  c = b;
  big2 = big;

  cout 	<< "std_ulogic_vector \t<=\t std_ulogic_vector"
	<< "\n--------------------------------------------------"
	<< "\nC = " << c << "\t\t ZZ1XX0XX1XXX" 
	<< "\nBIG2 = " << big2 << "\n      "
  <<"\"11110000111100001111000011110000111100001111000011110000111100001111\""
	<< "\n" << endl;

  // std_ulogic_vector <- C++ array of std_ulogic
  std_ulogic_vector<9>	d;
  std_ulogic_vector<9>	e;
  std_ulogic_vector<68>	big3;
  std_ulogic	cb1[9]  = { sc_logic( 'U' ), sc_logic( 0 )  , sc_logic( 1 ),
                            sc_logic( 'X' ), sc_logic( 'Z' ), sc_logic( 'W' ),
                            sc_logic( 'H' ), sc_logic( 'L' ), sc_logic( '-' ) };
  std_ulogic	cb2[12] = { sc_logic( 'U' ), sc_logic( 'U' ), sc_logic( 'X' ),
                            sc_logic( 'X' ), sc_logic( 1 )  , sc_logic( 1 ),
                            sc_logic( 0 )  , sc_logic( 0 )  , sc_logic( 1 ),
                            sc_logic( 1 )  , sc_logic( 'X' ), sc_logic( 'X' ) };
  std_ulogic	cb3[80] = { sc_logic( 1 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 1 ), sc_logic( 1 ), sc_logic( 1 ),
                            sc_logic( 1 ), sc_logic( 0 ), sc_logic( 0 ),
                            sc_logic( 0 ), sc_logic( 0 ) };

  d = cb1;
  e = cb2; 
  big3 = cb3;

  cout 	<< "std_ulogic_vector \t<=\t C++ array of bool"
	<< "\n--------------------------------------------------"
        << "\nD = " << d << "\t\t -, L, H, W, Z, X, 1, 0, U" 
        << "\nE = " << e << "\t\t X, X, 1, 1, 0, 0, 1, 1, X, X, U, U" 
	<< "\nBIG3 = " << big3 << "\n      "
    <<"\"11110000111100001111000011110000111100001111000011110000111100001111"
  	<<"\n       000011110000\""
	<< "\n" << endl;

  // std_ulogic_vector <- bool_vector 
  std_ulogic_vector<4>	f;
  bool_vector<4>	sv1;

  sv1 = "1010";

  f = sv1; 

  cout 	<< "std_ulogic_vector \t<=\t bool_vector" 
	<< "\n--------------------------------------------"
	<< "\nF = " << f << "\t\t \"1010\" " 
	<< "\n" << endl;

  // std_ulogic_vector <- unsigned long 
  std_ulogic_vector<4>	h;
  std_ulogic_vector<32>	i;
  std_ulogic_vector<40>	j;
  unsigned long		ul1 = 137; 	// ...10001001
  unsigned long		ul2 = 137; 	// ...10001001
  unsigned long		ul3 = 137; 	// ...10001001

  h = ul1; 
  i = ul2;
  j = ul3;

  cout 	<< "std_ulogic_vector \t<=\t unsigned long" 
	<< "\n----------------------------------------------"
	<< "\nH = " << h << "\t\t\t\t\t ...10001001 (137)" 
	<< "\nI = " << i << "\t\t ...10001001 (137)" 
	<< "\nJ = " << j << "\t ...10001001 (137)" 
	<< "\n" << endl;

  // std_ulogic_vector <- sc_unsigned
  std_ulogic_vector<4>	k;
  std_ulogic_vector<4>	l;
  std_ulogic_vector<4>	m;
  sc_biguint<2>		scu1;
  sc_biguint<4>		scu2;
  sc_biguint<8>		scu3;

  scu1 = 3;	// .........11
  scu2 = 13; 	// .......1101
  scu3 = 137; 	// ...10001001

  k = scu1; 
  l = scu2;
  m = scu3;

  cout 	<< "std_ulogic_vector \t<=\t sc_unsigned" 
	<< "\n--------------------------------------------"
	<< "\nK = " << k << "\t\t       11 (3)" 
	<< "\nL = " << l << "\t\t     1101 (13)" 
	<< "\nM = " << m << "\t\t 10001001 (137)" 
	<< "\n" << endl;

  // std_ulogic_vector <- signed long
  std_ulogic_vector<5>	n;
  std_ulogic_vector<32>	o;
  std_ulogic_vector<40>	p;
  std_ulogic_vector<5> 	q;
  std_ulogic_vector<32>	r;
  std_ulogic_vector<40>	s;
  signed long		sl1 = 137;      // ...010001001
  signed long		sl2 = 137;      // ...010001001
  signed long		sl3 = 137;      // ...010001001
  signed long		sl4 = -137;     // ...101110111
  signed long		sl5 = -137;     // ...101110111
  signed long		sl6 = -137;     // ...101110111
 
  n = sl1;
  o = sl2;
  p = sl3;
  q = sl4;
  r = sl5;
  s = sl6;
 
  cout  << "std_ulogic_vector \t<=\t signed long"
        << "\n--------------------------------------------"
        << "\nN = " << n << "\t\t\t\t\t ...010001001 (137)"
        << "\nO = " << o << "\t\t ...010001001 (137)"
        << "\nP = " << p << "\t ...010001001 (137)"
        << "\nQ = " << q << "\t\t\t\t\t ...101110111 (-137)"
        << "\nR = " << r << "\t\t ...101110111 (-137)"
        << "\nS = " << s << "\t ...101110111 (-137)"
        << "\n" << endl;

  // std_ulogic_vector <- sc_signed
  std_ulogic_vector<5>   t;
  std_ulogic_vector<5>   u;
  std_ulogic_vector<5>   v;
  std_ulogic_vector<5>   w;
  std_ulogic_vector<5>   x;
  std_ulogic_vector<5>   y;
  sc_bigint<3>   scs1;
  sc_bigint<5>   scs2;
  sc_bigint<9>   scs3;
  sc_bigint<3>   scs4;
  sc_bigint<5>   scs5;
  sc_bigint<9>   scs6;
 
  scs1 = 3;     // ........011
  scs2 = 13;    // ......01101
  scs3 = 137;   // ..010001001
  scs4 = -3;    // ........101
  scs5 = -13;   // ......10011
  scs6 = -137;  // ..101110111
 
  t = scs1;
  u = scs2;
  v = scs3;
  w = scs4;
  x = scs5;
  y = scs6;
 
  cout  << "std_ulogic_vector \t<=\t sc_signed"
        << "\n------------------------------------------"
        << "\nT = " << t << "\t\t       011 (3)"
        << "\nU = " << u << "\t\t     01101 (13)"
        << "\nV = " << v << "\t\t 010001001 (137)"
        << "\nW = " << w << "\t\t       101 (-3)"
        << "\nX = " << x << "\t\t     10011 (-13)"
        << "\nY = " << y << "\t\t 101110111 (-137)"
        << "\n" << endl;

  // std_ulogic_vector	.to_uint()
  std_ulogic_vector<4> 	tu1;
  std_ulogic_vector<32>	tu2;
  std_ulogic_vector<40>	tu3;
  std_ulogic_vector<4> 	tu4;
  sc_biguint<2>		tu5;
  sc_biguint<4>   	tu6;
  sc_biguint<8>   	tu7;
 
  tu1 =                                     "1001";	// 9
  tu2 =         "10000000000000000000000000000001"; 	// 2147483649 
  tu3 = "0000000110000000000000000000000000000001"; 	// 6442450945 
  tu4 = "1101";
  tu5 = tu4.to_uint();
  tu6 = tu4.to_uint();
  tu7 = tu4.to_uint();

  cout  << "std_ulogic_vector \t\t<=\t\t to_uint()"
        << "\n-----------------------------------------------------------------"
        << "\nTU1 = \t\t\t\t          " << tu1 << "\t " << tu1.to_uint()
        << "\nTU2 =         " << tu2 << "\t " << tu2.to_uint()
        << "\nTU3 = " << tu3 << "\t " << tu3.to_uint()
        << "\nTU4 = " << tu4 << " \t\t\t\t\t " << tu5 << "\t      (" 
	   << tu5[1] << tu5[0] << ")"
        << "\nTU4 = " << tu4 << " \t\t\t\t\t " << tu6 << "\t    ("
	   << tu6[3] << tu6[2] << tu6[1] << tu6[0] << ")"
        << "\nTU4 = " << tu4 << " \t\t\t\t\t " << tu7 << "\t("
	   << tu7[7] << tu7[6] << tu7[5] << tu7[4] 
	   << tu7[3] << tu7[2] << tu7[1] << tu7[0] << ")" 
      	<< "\n" << endl;

  // std_ulogic_vector	.to_int()
  std_ulogic_vector<4> 	ts1;
  std_ulogic_vector<32>	ts2;
  std_ulogic_vector<40>	ts3;
  std_ulogic_vector<5> 	ts4;
  sc_bigint<3>	ts5;
  sc_bigint<5> 	ts6;
  sc_bigint<9> 	ts7;
 
  ts1 =                                     "1001";     // -7
  ts2 =         "11111111111111111111101111111001";     // -1031 
  ts3 = "0000000111111111111111111111101111111001";     // 8589933561 
  ts4 = "11001";
  ts5 = ts4.to_int();
  ts6 = ts4.to_int();
  ts7 = ts4.to_int();
 
  cout  << "std_ulogic_vector \t\t<=\t\t to_int()"
        << "\n-----------------------------------------------------------------"
        << "\nTS1 = \t\t\t\t          " << ts1 << "\t " << ts1.to_int()
        << "\nTS2 =         " << ts2 << "\t " << ts2.to_int()
        << "\nTS3 = " << ts3 << "\t " << ts3.to_int()
        << "\nTS4 = " << ts4 << " \t\t\t\t\t " << ts5 << "\t      (" 
           << ts5[2] << ts5[1] << ts5[0] << ")"
        << "\nTS4 = " << ts4 << " \t\t\t\t\t " << ts6 << "\t    ("
           << ts6[4] << ts6[3] << ts6[2] << ts6[1] << ts6[0] << ")"
        << "\nTS4 = " << ts4 << " \t\t\t\t\t " << ts7 << "\t("
           << ts7[8] << ts7[7] << ts7[6] << ts7[5] 
           << ts7[4] << ts7[3] << ts7[2] << ts7[1] << ts7[0] << ")"
        << "\n" << endl;

  // std_ulogic_vector       Typecasted to sc_unsigned 
  std_ulogic_vector<4> 	tcu1;
  sc_biguint<2>   	tcu2;
  sc_biguint<4>   	tcu3;
  sc_biguint<8>   	tcu4;

  tcu1 = "1101";
  tcu2 = tcu1;
  tcu3 = tcu1;
  tcu4 = tcu1;
 
  cout  << "std_ulogic_vector \t\t<=\t\t Typecast sc_unsigned"
        << "\n-----------------------------------------------------------------"
        << "\nTCU1 = " << tcu1 << " \t\t\t\t\t " << tcu2 << "\t      ("
           << tcu2[1] << tcu2[0] << ")"
        << "\nTCU1 = " << tcu1 << " \t\t\t\t\t " << tcu3 << "\t    ("
           << tcu3[3] << tcu3[2] << tcu3[1] << tcu3[0] << ")"
        << "\nTCU1 = " << tcu1 << " \t\t\t\t\t " << tcu4 << "\t("
           << tcu4[7] << tcu4[6] << tcu4[5] << tcu4[4]
           << tcu4[3] << tcu4[2] << tcu4[1] << tcu4[0] << ")"
        << "\n" << endl;

  // std_ulogic_vector       Typecasted to sc_signed 
  std_ulogic_vector<5> 	tcs1;
  sc_bigint<3> 		tcs2;
  sc_bigint<5> 		tcs3;
  sc_bigint<9> 		tcs4;
 
  tcs1 = "11001";
  tcs2 = sc_bigint<3>(tcs1);
  tcs3 = sc_bigint<5>(tcs1);
  tcs4 = sc_bigint<9>(tcs1);
 
  cout  << "std_ulogic_vector \t\t<=\t\t Typecast sc_signed"
        << "\n-----------------------------------------------------------------"
        << "\nTCS1 = " << tcs1 << " \t\t\t\t\t " << tcs2 << "\t      ("
           << tcs2[2] << tcs2[1] << tcs2[0] << ")"
        << "\nTCS1 = " << tcs1 << " \t\t\t\t\t " << tcs3 << "\t    ("
           << tcs3[4] << tcs3[3] << tcs3[2] << tcs3[1] << tcs3[0] << ")"
        << "\nTCS1 = " << tcs1 << " \t\t\t\t\t " << tcs4 << "\t("
           << tcs4[8] << tcs4[7] << tcs4[6] << tcs4[5]
           << tcs4[4] << tcs4[3] << tcs4[2] << tcs4[1] << tcs4[0] << ")"
        << "\n" << endl;

  // std_ulogic_vector     .to_string() 
  std_ulogic_vector<9>	tstr;
  std::string           str;

  tstr = "UXZ01WLH-";
  str  = tstr.to_string();
   
  cout  << "std_ulogic_vector \t<=\t to_string()"
        << "\n--------------------------------------------"
        << "\nTSTR = " << tstr << " \t\t " << str
        << endl;


// 3. OPERATORS
//    Supported operators:      ~ & ^ | &= ^= |= = [] range() 
//				and_reduce() or_reduce() xor_reduce()
  std_ulogic_vector<4>	ra;
  std_ulogic_vector<4>	rb;
  std_ulogic_vector<4>	rc;
  std_ulogic_vector<9>	rd;
  std_ulogic_vector<4>	re;
  std_ulogic_vector<4>	rf;
  std_ulogic_vector<4>	rg;
  std_ulogic_vector<9>	rh;
  std_ulogic_vector<4>	ri;
  std_ulogic_vector<4>	rj;
  std_ulogic_vector<4>	rl;

  std_ulogic_vector<4>	rdata4;
  std_ulogic_vector<9>	rdata9;

  rdata4 = "1000";
  rdata9 = "UXZ01WHL-";
 
  ra.range(0,3) = rdata4;
  rb.range(3,0) = rdata4;
  ( rc.range(1,3), rc.range(0,0) ) = rdata4;
  ( rd.range(8,6), rd.range(5,0) ) = rdata9;
  re = "1111"; 
  re.range(2,2) = std_ulogic_vector<1>( rdata4[1] );
 
  rf = rdata4.range(0,3);
  rg = rdata4.range(3,0);
  rh = ( rdata9.range(8,6), rdata9.range(5,0) );
  ri = ( rdata4.range(0,1), rdata4.range(2,3) );
  rj = "1111"; 
  rj[1] = rdata4.range(2,2)[0];
  rl = ( rdata4.range(1,1), rdata4.range(3,3),
        rdata4.range(0,0), rdata4.range(2,2) );
 
  cout.precision(15);
  cout  << "\nrange() tests"
        << "\n-----------------------------------------------------------------"
        << "\nINITIAL 4-BIT \t" << rdata4
        << "\nINITIAL 9-BIT \t" << rdata9 << "\n"
        << "\nLVALUE RISE \t"
                << ra[0] << "\t" << ra[1] << "\t" << ra[2] << "\t" << ra[3]
        << "\nLVALUE FALL \t"
                << rb[0] << "\t" << rb[1] << "\t" << rb[2] << "\t" << rb[3]
        << "\nLVALUE SUB RISE "
                << rc[0] << "\t" << rc[1] << "\t" << rc[2] << "\t" << rc[3]
        << "\nLVALUE SUB FALL "
                << rd[0] << "\t" << rd[1] << "\t" << rd[2] << "\t"
                << rd[3] << "\t" << rd[4] << "\t" << rd[5] << "\t"
                << rd[6] << "\t" << rd[7] << "\t" << rd[8]
        << "\nLVALUE BIT \t"
                << re[0] << "\t" << re[1] << "\t" << re[2] << "\t" << re[3]
        << "\n\nRVALUE RISE \t"
                << rf[0] << "\t" << rf[1] << "\t" << rf[2] << "\t" << rf[3]
        << "\nRVALUE FALL \t"
                << rg[0] << "\t" << rg[1] << "\t" << rg[2] << "\t" << rg[3]
        << "\nRVALUE SUB FALL "
                << rh[0] << "\t" << rh[1] << "\t" << rh[2] << "\t"
                << rh[3] << "\t" << rh[4] << "\t" << rh[5] << "\t"
                << rh[6] << "\t" << rh[7] << "\t" << rh[8]
        << "\nRVALUE SUB RISE "
                << ri[0] << "\t" << ri[1] << "\t" << ri[2] << "\t" << ri[3]
        << "\nRVALUE BIT [] \t"
                << rj[0] << "\t" << rj[1] << "\t" << rj[2] << "\t" << rj[3]
        << "\nRVALUE BIT \t"
                << rl[0] << "\t" << rl[1] << "\t" << rl[2] << "\t" << rl[3]
        << endl;

#define VAL1	"1010"
#define VAL2	"1000"
#define VAL3	"111011"

  std_ulogic_vector<4> 	op1;
  op1 = VAL1;
  std_ulogic_vector<4>	op2;
  op2 = VAL2;
  std_ulogic_vector<4> 	r1, r2, r3, r4, r5, r6, r7, r8; 
  std_ulogic_vector<4> 	r9,  r10, r11, r12, r13, r14;
  std_ulogic_vector<4>	r15, r16, r17, r18, r19;  
  std_ulogic_vector<4> 	r20, r21, r22, r23, r24;
  std_ulogic_vector<4>	r25, r26, r27, r28, r29;  
  std_ulogic_vector<4>	r30, r31, r32, r33, r34, r35;
  std_ulogic_vector<4>	r36, r37, r38, r39, r40, r41;
  std_ulogic 		r42, r43, r44;

//  r1 = op1 * op2;			// Multiplication

//  r2 = op1 / op2;			// Division

//  r3 = op1 % op2;			// Modulus

//  r4 = op1 + op2;			// Addition

//  r5 = op1 - op2;			// Subtraction

//  r6 = !op1;				// Logical NOT

//  r7 = op1 && op2;			// Logical AND

//  r8 = op1 || op2;			// Logical OR

//  r9 = op1 < op2;			// Less than

//  r10 = op1 <= op2;			// Less than or equal

//  r11 = op1 > op2;			// Greater than

//  r12 = op1 >= op2;			// Greater than or equal

//  r13 = op1 += op2;			// Compound addition
//    op1 = VAL1; op2 = VAL2;

//  r14 = op1 -= op2;			// Compound subtraction
//    op1 = VAL1; op2 = VAL2;

//  r15 = op1 *= op2;			// Compound multiplication
//    op1 = VAL1; op2 = VAL2;

//  r16 = op1 /= op2;			// Compound division
//    op1 = VAL1; op2 = VAL2;

//  r17 = op1 %= op2;			// Compound modulus
//    op1 = VAL1; op2 = VAL2;

//  r18 = op1 <<= op2;			// Compound shift left 
//    op1 = VAL1; op2 = VAL2;

//  r19 = op1 >>= op2;			// Compound shift right 
//    op1 = VAL1; op2 = VAL2;

  r20 = op1 &= op2;			// Compound bitwise AND 
    op1 = VAL1; op2 = VAL2;
  r36 = op1 &= VAL3;			
    op1 = VAL1; 

  r21 = op1 ^= op2;			// Compound bitwise XOR 
    op1 = VAL1; op2 = VAL2;
  r37 = op1 ^= VAL3;
    op1 = VAL1; 

  r22 = op1 |= op2;			// Compound bitwise OR 
    op1 = VAL1; op2 = VAL2;
  r38 = op1 |= VAL3;	
    op1 = VAL1; 

//  r23 = op2++;				// Postfix increment 
//    op1 = VAL1; op2 = VAL2;

//  r24 = ++op2;				// Prefix increment 
//    op1 = VAL1; op2 = VAL2;

//  r25 = op2--;				// Postfix decrement 
//    op1 = VAL1; op2 = VAL2;

//  r26 = --op2;				// Prefix decrement 
//    op1 = VAL1; op2 = VAL2;

//  r27 = (op1 > op2) ? true : false;	// Arithmetic if
//  r28 = (op1 < op2) ? true : false;	// Arithmetic if

//  r29 = op1, r29 = op2;		 	// Comma  

  r30 = ~op1;				// Bitwise NOT

//  r31 = op1 << op2;			// Left shift 
//    op1 = VAL1; op2 = VAL2;

//  r32 = op1 >> op2;			// Right shift 
//    op1 = VAL1; op2 = VAL2;

  r33 = op1 & op2;			// Bitwise AND 
  r39 = op1 & VAL3;

  r34 = op1 ^ op2;			// Bitwise XOR 
  r40 = op1 ^ VAL3;

  r35 = op1 | op2;			// Bitwise OR 
  r41 = op1 | VAL3;

  r42 = and_reduce(op1);		// AND reduction

  r43 = or_reduce(op1);			// OR reduction

  r44 = xor_reduce(op1);		// XOR reduction

cout << "\nop1\t operator\t op2\t result  [All operands are std_ulogic_vector]"
	<< "\n----------------------------------------------------------------"
// 	<< "\n" << op1 << "\t    * \t\t " << op2 << "\t = " << r1
// 	<< "\n" << op1 << "\t    / \t\t " << op2 << "\t = " << r2
// 	<< "\n" << op1 << "\t    % \t\t " << op2 << "\t = " << r3
// 	<< "\n" << op1 << "\t    + \t\t " << op2 << "\t = " << r4
// 	<< "\n" << op1 << "\t    - \t\t " << op2 << "\t = " << r5
//  	<< "\n!(" << op1 << ") \t\t\t\t = " << r6 
//  	<< "\n" << op1 << "\t    && \t\t " << op2 << "\t = " << r7
//  	<< "\n" << op1 << "\t    || \t\t " << op2 << "\t = " << r8
//  	<< "\n" << op1 << "\t    < \t\t "  << op2 << "\t = " << r9
//  	<< "\n" << op1 << "\t    <= \t\t " << op2 << "\t = " << r10
//  	<< "\n" << op1 << "\t    > \t\t "  << op2 << "\t = " << r11
//  	<< "\n" << op1 << "\t    >= \t\t " << op2 << "\t = " << r12
//  	<< "\n" << op1 << "\t    += \t\t " << op2 << "\t = " << r13 
//  	<< "\n" << op1 << "\t    -= \t\t " << op2 << "\t = " << r14 
//  	<< "\n" << op1 << "\t    *= \t\t " << op2 << "\t = " << r15 
//  	<< "\n" << op1 << "\t    /= \t\t " << op2 << "\t = " << r16 
//  	<< "\n" << op1 << "\t    %= \t\t " << op2 << "\t = " << r17 
//  	<< "\n" << op1 << "\t    <<=\t\t " << op2 << "\t = " << r18 
//  	<< "\n" << op1 << "\t    >>=\t\t " << op2 << "\t = " << r19 
  	<< "\n" << op1 << "\t    &= \t\t " << op2 << "\t = " << r20 
  	<< "\n" << op1 << "\t    ^= \t\t " << op2 << "\t = " << r21 
  	<< "\n" << op1 << "\t    |= \t\t " << op2 << "\t = " << r22 
//  	<< "\n" << "\t    ()++ \t " << op2 << "\t = " << r23 
//  	<< "\n" << "\t    ++() \t " << op2 << "\t = " << r24 
//  	<< "\n" << "\t    ()-- \t " << op2 << "\t = " << r25 
//  	<< "\n" << "\t    --() \t " << op2 << "\t = " << r26 
//  	<< "\n" << op1 << "\t    > ?: \t " << op2 << "\t = " << r27 
//  	<< "\n" << op1 << "\t    < ?: \t " << op2 << "\t = " << r28 
//  	<< "\n" << op1 << "\t    , \t\t " << op2 << "\t = " << r29 
  	<< "\n~(" << op1 << ") \t\t\t = " << r30 
//  	<< "\n" << op1 << "\t    << \t\t " << op2 << "\t = " << r31 
//  	<< "\n" << op1 << "\t    >> \t\t " << op2 << "\t = " << r32 
  	<< "\n" << op1 << "\t    & \t\t " << op2 << "\t = " << r33 
  	<< "\n" << op1 << "\t    ^ \t\t " << op2 << "\t = " << r34 
  	<< "\n" << op1 << "\t    | \t\t " << op2 << "\t = " << r35 
  	<< "\n\n" << op1 << "\t    &= \t\t " << VAL3 << "\t = " << r36 
  	<< "\n" << op1 << "\t    ^= \t\t " << VAL3 << "\t = " << r37 
  	<< "\n" << op1 << "\t    |= \t\t " << VAL3 << "\t = " << r38 
  	<< "\n" << op1 << "\t    & \t\t " << VAL3 << "\t = " << r39 
  	<< "\n" << op1 << "\t    ^ \t\t " << VAL3 << "\t = " << r40 
  	<< "\n" << op1 << "\t    | \t\t " << VAL3 << "\t = " << r41 
  	<< "\n\n" << op1 << "\t    and_reduce() \t = " << r42 
  	<< "\n" << op1 << "\t    or_reduce() \t = " << r43 
  	<< "\n" << op1 << "\t    xor_reduce() \t = " << r44 
 	<< endl;

  if (op1 == op2)			// Equality
   cout << "\n" << op1 << "\t    == \t\t " << op2 << "\t -> true" << endl;
  else
   cout << "\n" << op1 << "\t    == \t\t " << op2 << "\t -> false" << endl;

  if (op1 != op2)			// Inequality
   cout << op1 << "\t    != \t\t " << op2 << "\t -> true" << endl;
  else
   cout << op1 << "\t    != \t\t " << op2 << "\t -> false" << endl;

  op1 = op2 = "1111";		// Assignment operator concatenation
   cout << "\n" << op1 << "\t    = \t\t " << op2 << endl;


// 4. OPERATOR DEFINITIONS
//      & | ^ ~
  std_ulogic_vector<9> 	suv;	
  std_ulogic_vector<9>	vu;
  std_ulogic_vector<9>	vx;
  std_ulogic_vector<9>	v0;
  std_ulogic_vector<9>	v1;
  std_ulogic_vector<9>	vz;
  std_ulogic_vector<9>	vw;
  std_ulogic_vector<9>	vl;
  std_ulogic_vector<9>	vh;
  std_ulogic_vector<9>	vd;
  std_ulogic_vector<9>	bang;

  suv = "UX01ZWLH-";

  vu = "UUUUUUUUU" & suv;
  vx = "XXXXXXXXX" & suv;
  v0 = "000000000" & suv;
  v1 = "111111111" & suv;
  vz = "ZZZZZZZZZ" & suv;
  vw = "WWWWWWWWW" & suv;
  vl = "LLLLLLLLL" & suv;
  vh = "HHHHHHHHH" & suv;
  vd = "---------" & suv;
   
  cout	<< "\n+-------------------------+"
  << "\n| AND (&) | X | 0 | 1 | Z |" 
  << "\n+-------------------------+"
  << "\n|    X    | " << vx[7] << " | " << vx[6] << " | "
  << vx[5] << " | " << vx[4] << " | "
  << "\n+-------------------------+"
  << "\n|    0    | " << v0[7] << " | " << v0[6] << " | "
  << v0[5] << " | " << v0[4] << " | " 
  << "\n+-------------------------+"
  << "\n|    1    | " << v1[7] << " | " << v1[6] << " | "
  << v1[5] << " | " << v1[4] << " | "
  << "\n+-------------------------+"
  << "\n|    Z    | " << vz[7] << " | " << vz[6] << " | "
  << vz[5] << " | " << vz[4] << " | "
  << "\n+-------------------------+"
  << endl;

  vu = "UUUUUUUUU" | suv;
  vx = "XXXXXXXXX" | suv;
  v0 = "000000000" | suv;
  v1 = "111111111" | suv;
  vz = "ZZZZZZZZZ" | suv;
  vw = "WWWWWWWWW" | suv;
  vl = "LLLLLLLLL" | suv;
  vh = "HHHHHHHHH" | suv;
  vd = "---------" | suv;
  
  cout  << "\n+-------------------------+"
  << "\n| OR  (|) | X | 0 | 1 | Z |"
  << "\n+-------------------------+"
  << "\n|    X    | " << vx[7] << " | " << vx[6] << " | "
  << vx[5] << " | " << vx[4] << " | "
  << "\n+-------------------------+"
  << "\n|    0    | " << v0[7] << " | " << v0[6] << " | "
  << v0[5] << " | " << v0[4] << " | "
  << "\n+-------------------------+"
  << "\n|    1    | " << v1[7] << " | " << v1[6] << " | "
  << v1[5] << " | " << v1[4] << " | "
  << "\n+-------------------------+"
  << "\n|    Z    | " << vz[7] << " | " << vz[6] << " | "
  << vz[5] << " | " << vz[4] << " | "
  << "\n+-------------------------+"
  << endl;

  vu = "UUUUUUUUU" ^ suv;
  vx = "XXXXXXXXX" ^ suv;
  v0 = "000000000" ^ suv;
  v1 = "111111111" ^ suv;
  vz = "ZZZZZZZZZ" ^ suv;
  vw = "WWWWWWWWW" ^ suv;
  vl = "LLLLLLLLL" ^ suv;
  vh = "HHHHHHHHH" ^ suv;
  vd = "---------" ^ suv;

  cout  << "\n+-------------------------+"
  << "\n| XOR (^) | X | 0 | 1 | Z |"
  << "\n+-------------------------+"
  << "\n|    X    | " << vx[7] << " | " << vx[6] << " | "
  << vx[5] << " | " << vx[4] << " | "
  << "\n+-------------------------+"
  << "\n|    0    | " << v0[7] << " | " << v0[6] << " | "
  << v0[5] << " | " << v0[4] << " | "
  << "\n+-------------------------+"
  << "\n|    1    | " << v1[7] << " | " << v1[6] << " | "
  << v1[5] << " | " << v1[4] << " | "
  << "\n+-------------------------+"
  << "\n|    Z    | " << vz[7] << " | " << vz[6] << " | "
  << vz[5] << " | " << vz[4] << " | "
  << "\n+-------------------------+"
  << endl;
 
  bang = ~suv;
   
  cout	<< "\n+-------------------------+"
  << "\n| NOT (~) | X | 0 | 1 | Z |" 
  << "\n+-------------------------+"
  << "\n|         | " << bang[7] << " | " << bang[6] << " | "
  << bang[5] << " | " << bang[4] << " | " 
  << "\n+-------------------------+"
  << endl;
  return 0;
}
