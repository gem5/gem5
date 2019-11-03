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

  xor_1.cpp -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-30

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "xor_1.h"

void xor_1::entry(){

  signed int      tmp1;
  unsigned int    tmp2;
  sc_lv<8>        tmp3;
  sc_lv<8>        tmp3_tmp;
  sc_bigint<8>    tmp4;
  sc_biguint<8>   tmp5;

  // reset_loop
  if (reset.read() == true) {
    out_valid.write(false);
    wait();
  } else wait();

  //
  // main loop
  //
  //
  while(1) {
    while(in_valid.read()==false) wait();
    wait();

    //reading the inputs
    tmp1 = in_value1.read();
    tmp2 = in_value2.read();
    tmp3 = in_value3.read();
    tmp4 = in_value4.read();
    tmp5 = in_value5.read();

    //execute simple operations
    tmp3_tmp = 0x0f;
    tmp1 = tmp1 ^ 0x0f ^ 0x12;
    tmp2 = tmp2 ^ 0x0f ^ 0x13 ;
    tmp3 = tmp3 ^ tmp3_tmp;
    tmp4 = tmp4 ^ 0x0f ^ 0x14 ;
    tmp5 = tmp5 ^ 0x0f ^ 0x15 ;
    wait();

    // write outputs
    out_value1.write(tmp1);
    out_value2.write(tmp2);
    out_value3.write(tmp3);
    out_value4.write(tmp4);
    out_value5.write(tmp5);
    out_valid.write(true);
    wait();
    out_valid.write(false);
    wait();

    //execute simple operations
    tmp3_tmp = 0x03;
    tmp1 ^= 0x03;
    tmp2 ^= 0x03;
    tmp3 ^= tmp3_tmp;
    tmp4 ^= 0x03;
    tmp5 ^= 0x03;
    wait();

    // write outputs
    out_value1.write(tmp1);
    out_value2.write(tmp2);
    out_value3.write(tmp3);
    out_value4.write(tmp4);
    out_value5.write(tmp5);
    out_valid.write(true);
    wait();
    out_valid.write(false);
    wait();
  }
}

// EOF

