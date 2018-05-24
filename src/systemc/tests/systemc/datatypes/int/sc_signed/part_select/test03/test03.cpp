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

  test03 -- test for unsigned data values

  Original Author: 

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int sc_main(int argc, char** argv) { 
    sc_int<16> a, tmp; 
    sc_bigint<16> biga, bigtmp; 
	sc_biguint<6> bigu6;
	sc_biguint<16> bigu16;
 
     a = 97; // 0b1100001 
     biga = 97; 
	 bigu6 = 97;
  
  tmp = a.range( 5,0 ); // results in 33 - 0b100001, not sign extended 
   bigtmp = biga.range( 5,0 ); // results in 65505, sign extended 
   cout << hex << tmp << " " << bigtmp << endl;
    
  tmp = biga.range( 5,0 ); // results in 33 - 0b100001, not sign extended 
   bigtmp = a.range( 5,0 ); // results in 65505, sign extended 
   cout << hex << tmp << " " << bigtmp << endl;
    
	cout << hex << a.range(5,0) << " " << biga.range(5,0) << endl;

	bigtmp = bigu6;
	cout << hex << bigtmp << endl;

	bigu16 = biga.range(5,0);
	cout << bigu16 << endl;

	bigu16 = a.range(5,0);
	cout << hex << bigu16 << endl;

    sc_start(1, SC_NS); 
    return 0; 
}
