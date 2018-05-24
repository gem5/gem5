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

  datatypes.cpp -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-30

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "datatypes.h"

void datatypes::entry()

{
  sc_bigint<8>    tmp1;
  sc_bigint<8>    tmp1r;
  sc_biguint<8>   tmp2;
  sc_biguint<8>   tmp2r;
  long            tmp3;
  long            tmp3r;
  int             tmp4;
  int             tmp4r;
  short           tmp5;
  short           tmp5r;
  char            tmp6;
  char            tmp6r;
  bool            tmp7;
  bool            tmp7r;
  sc_bv<4>        tmp8;
  sc_bv<4>        tmp8r;
  sc_lv<4>        tmp9;
  sc_lv<4>        tmp9r;

// define 1 dimensional array
   int  tmpa[2];
   char tmpb[2];

// reset_loop
  if (reset.read() == true) {
    out_valid.write(false);
    out_ack.write(false);
    wait();
  } else wait();

//
// main loop
//
// initialization of sc_array

   tmpa[0] = 12;
   tmpa[1] = 127;
   tmpb[1] = 'G';


 while(1) {
    while(in_valid.read()==false) wait();

    //reading the inputs
    tmp1 = in_value1.read();
    tmp2 = in_value2.read();
    tmp3 = in_value3.read();
    tmp4 = in_value4.read();
    tmp5 = in_value5.read();
    tmp6 = in_value6.read();
    tmpb[0] = in_value7.read();
    tmp7 = in_value8.read();
    tmp8 = in_value9.read();
    tmp9 = in_value10.read();

    out_ack.write(true);

    //execute mixed data type not operations

    // signed(8) <- ~ unsigned(8)
    tmp1r = ~ tmp2;
    // unsigned(8) <- ~ long
    tmp2r = ~ tmp3;
    // long <- ~ char
    tmp3r = ~ tmp6;
    // int <- ~ short
    tmp4r = ~ tmp5;
    // short <- ~ int
    tmp5r = ~ tmp4;
    // char <- ~ char_array[0]
    // tmp6r = ~ tmp8[0];
    tmp6r = ~ tmp8[0].to_bool();
    // bool <- ! bool;
    tmp7r = !tmp7;
    // sc_bool_vector(4) <- ~ sc_logic_vector(4)
    tmp8r = ~ tmp9;
    //  sc_logic_vector(4) <- ~ sc_bool_vector(4)
    tmp9r = ~ tmp9;

    //write outputs
    out_value1.write(tmp1r);
    out_value2.write(tmp2r);
    out_value3.write(tmp3r);
    out_value4.write(tmp4r);
    out_value5.write(tmp5r);
    out_value6.write(tmp6r);
    out_value7.write(tmp7r);
    out_value8.write(tmp8r);
    out_value9.write(tmp9r);   

    out_valid.write(true);
    wait();
    out_ack.write(false);
    out_valid.write(false);

 }

} // End


