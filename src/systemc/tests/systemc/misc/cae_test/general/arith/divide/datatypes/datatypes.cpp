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

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-12-10

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "datatypes.h"

void datatypes::entry(){

  sc_biguint<2>     tmp1;
  sc_bigint<2>      tmp2;
  sc_biguint<3>     tmp3;
  sc_bigint<3>      tmp4;
  sc_biguint<2>     tmp1r;
  sc_bigint<2>      tmp2r;
  sc_biguint<3>     tmp3r;
  sc_bigint<3>      tmp4r;

  // reset_loop
    out_valid.write(false);
    out_ack.write(false);
    wait();

  //
  // main loop
  //

  while(1) {
    //input handshake
    while(in_valid.read()==false) wait();

    //reading the inputs
    tmp1 = in_value1.read();
    tmp2 = in_value2.read();
    tmp3 = in_value3.read();
    tmp4 = in_value4.read();

    // input handshake
    out_ack.write(true);   

    //execute datatypes operations
    // unsigned(2) <- signed(3)/unsigned(2)
    tmp1r = tmp4 / tmp1;
    // signed(2) <-  unsigned(2)/signed(3)
    tmp2r = tmp1 / tmp4;
    // unsigned(3) <- unsigned(3)/unsigned(2)
    tmp3r = tmp3 / tmp1;
    // signed(3) <- signed(3)/signed(2)
    tmp4r = tmp4 / tmp2;

    // write outputs
    out_value1.write(tmp1r);
    out_value2.write(tmp2r);
    out_value3.write(tmp3r);
    out_value4.write(tmp4r);

    //output handshake
    out_valid.write(true);
    wait();

    //input handshake
    out_ack.write(false);
 
    //output handshake
    out_valid.write(false);
    wait();
  }
}

// EOF


