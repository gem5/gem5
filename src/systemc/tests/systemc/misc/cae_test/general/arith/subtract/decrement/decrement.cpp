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

  decrement.cpp -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-14

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "decrement.h"

void decrement::entry(){
  
  #define ONE 1
  const int       eins = 1;
  int             tmp1;
  sc_bigint<4>    tmp2;

  // reset_loop
  if (reset.read() == true) {
    out_valid.write(false);
    out_ack.write(false);
    wait();
  } else wait();

  //
  // main loop
  //
    
 while(1) {
    while(in_valid.read()==false) wait();
    wait();

    //reading the inputs
    tmp1 = in_value1.read();
    tmp2 = in_value2.read();

    //execute simple operations
    tmp1 = tmp1 - 1;
    tmp1 = tmp1 - ONE;
    tmp1 = tmp1 - eins;
    tmp1--;
    tmp2 = tmp2 - 1;
    tmp2 = tmp2 - ONE;
    tmp2 = tmp2 - eins;
    tmp2--;

    out_ack.write(true);

    // write outputs
    out_value1.write(tmp1);
    out_value2.write(tmp2);

    out_valid.write(true);
    wait();
    out_ack.write(false);
    out_valid.write(false);
    wait();
  }
}

// EOF

