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

  test1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>
#include "types2.h"

void types2::entry(){

  int     tmp1_uns_lv;
  sc_lv<8>  tmp2_uns_lv;
  sc_uint<8>        tmp1_uns;
  sc_uint<8>       tmp2_uns;
  sc_int<9>       temp_output1;
  sc_int<8>       temp_output2;
  sc_lv<9>       output1;
  int       output2;

  b_new_struct test;
  a_new_struct test1;
  // reset_loop
  out_valid.write(0);

  wait();
  while (true) { 
    test1 = port_in.read();
    while(test1.in_valid == (sc_logic)'0') {
      wait();
      test1 = port_in.read();
     }
    wait();

    //cout << "Starting execution" << endl;

    //reading the inputs
    tmp1_uns_lv = test1.in_value1;
    //cout << "read inputs" << endl;
    tmp2_uns_lv = test1.in_value2;
    //cout << "read inputs" << endl;
    tmp1_uns = tmp1_uns_lv;
    tmp2_uns = tmp2_uns_lv;
    
    //execute simple operations
    tmp1_uns++;
    tmp2_uns--;
    tmp1_uns_lv = tmp1_uns;

  
    output1 = temp_output1 = tmp1_uns + tmp2_uns;
    output2 = temp_output2 = tmp1_uns - tmp2_uns;

    wait();

    // write outputs

    test.out_value1 = "000000000";
    test.out_value2 = tmp1_uns_lv;
    port_out.write(test);
  
    out_valid.write(1);
    wait();

    out_valid.write(0);
    wait();
    test.out_value1 = output1;
    test.out_value2 = output2;

    port_out.write(test);
    out_valid.write(1);
    wait(); 


    out_valid.write(0);
    wait();

}
}
// EOF

