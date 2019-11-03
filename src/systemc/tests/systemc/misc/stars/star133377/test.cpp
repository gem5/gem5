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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>

int
sc_main(int, char**)
{
  { // sc_int works fine
    sc_int<8> a;
    sc_int<16> x;
    a=sc_bigint<8>("0b11101011");
    
    x=(a,a); 
    cout << "sc_int     (1) " << sc_bigint<16>(x).to_string(SC_BIN) << endl;
    
    x=(a.range(7,0), a.range(3,0), a.range(3,0));
    cout << "sc_int     (2)                 "
         << sc_bigint<16>(x).to_string(SC_BIN) << endl;
  }
  { // sc_uint works fine
    sc_uint<8> a;
    sc_uint<16> x;
    a=sc_biguint<8>("0b11101011");
    
    x=(a,a); 
    cout << "sc_uint    (1) " << sc_biguint<16>(x).to_string(SC_BIN) << endl;
    
    x=(a.range(7,0), a.range(3,0), a.range(3,0));
    cout << "sc_uint    (2)                 "
         << sc_biguint<16>(x).to_string(SC_BIN) << endl;
  }
  {
    sc_bigint<8> a;
    sc_bigint<16> x;
    a="0b11101011";

    // compile error (SC6.1): 
    // Error: Overloading ambiguity between "operator,(const sc_uint_base&, const sc_uint_base&)" 
    //   and "operator,(const sc_int_base&, const sc_int_base&)".
    // Error: Overloading ambiguity between "sc_bigint<16>::operator=(const sc_uint_base&)" 
    //   and "sc_bigint<16>::operator=(long long)".
    // ----
    // x=(a,a);
    // cout << "sc_bigint  (1) " << x.to_string(SC_BIN) << endl;

    // runtime error: concat yields wrong result
    // returned value is 1111111111111011
    // but should be     1110101110111011
    x=(a.range(7,0), a.range(3,0), a.range(3,0));
    cout << "sc_bigint  (2)                 "<< x.to_string(SC_BIN) << endl;
  }
  {
    sc_biguint<8> a;
    sc_biguint<16> x;
    a="0b11101011";

    // compile error (SC6.1): 
    // Error: Overloading ambiguity between "operator,(const sc_uint_base&, const sc_uint_base&)" 
    //   and "operator,(const sc_int_base&, const sc_int_base&)".
    // Error: Overloading ambiguity between "sc_bigint<16>::operator=(const sc_uint_base&)" 
    //   and "sc_bigint<16>::operator=(long long)".
    // ----
    // x=(a,a);
    // cout << "sc_biguint (1) " << x.to_string(SC_BIN) << endl;

    // runtime error: concat yields wrong result
    // returned value is 0000000000001011
    // but should be     1110101110111011
    x=(a.range(7,0), a.range(3,0), a.range(3,0));
    cout << "sc_biguint (2)                 " << x.to_string(SC_BIN) << endl;
  }
  { // sc_bv works fine
    sc_bv<8> a;
    sc_bv<16> x;
    a="11101011";
    
    x=(a,a); 
    cout << "sc_bv      (1) " << x.to_string() << endl;
    
    x=(a.range(7,0), a.range(3,0), a.range(3,0));
    cout << "sc_bv      (2)                 " << x.to_string() << endl;
  }
  { // sc_lv works fine
    sc_lv<8> a;
    sc_lv<16> x;
    a="11101011";
    
    x=(a,a); 
    cout << "sc_lv      (1) " << x.to_string() << endl;
    
    x=(a.range(7,0), a.range(3,0), a.range(3,0));
    cout << "sc_lv      (2)                 " << x.to_string() << endl;
  }
  return 0;
}
