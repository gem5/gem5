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

  if_test.cpp -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-22

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "if_test.h"

void if_test::entry(){

  sc_biguint<4>   tmp1;
  sc_bigint<4>    tmp2;
  sc_lv<4>        tmp3;
  sc_bv<4>        tmp4;
  int             tmp5;

  // reset_loop
    if (reset.read() == true) {
      out_value1.write(0);
      out_value2.write(0);
      out_value3.write(0);
      out_value4.write(0);
      out_value5.write(0);
      out_valid.write(false);
      wait();
    } else wait();

  //
  // main loop
  //
  while(1) {
    do { wait(); } while  (in_valid == false);

    //reading inputs
    tmp1 = in_value1.read();
    tmp2 = in_value2.read();
    tmp3 = in_value3.read();
    tmp4 = in_value4.read();
    tmp5 = in_value5.read();

    //execution
    if (tmp1 == 4) {
	out_value1.write(3);
    } else if (tmp1 == 3) {
	out_value1.write(2);
    } else if (tmp1 == 2) {
	out_value1.write(1);
    } else {
	out_value1.write(tmp1);
    };
    wait();

    if (tmp2 < 6 ) {
      out_value2.write(0);
      wait();
    } else  {
      out_value2.write(tmp2);
      wait();
    };

    if (tmp3 == "0000" ) {
      out_value3.write(1);
      wait();
      wait();
    } else {
      out_value3.write(tmp3);
      wait();
    };

    if (tmp4 != "0001" ) {
      out_value4.write(tmp4);
    };
    wait();

    out_value5.write((tmp5>=6)?0:tmp5);
    wait();

    out_valid.write(true);
    wait();
    out_valid.write(false);

  }
}

// EOF

