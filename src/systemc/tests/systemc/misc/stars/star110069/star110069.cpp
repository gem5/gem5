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

  star110069.cpp -- 

  Original Author: Stan Liao, Synopsys, Inc., 2000-09-19

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>
#include "mem0.h"
 
void mem0::entry(){


  unsigned int tmp1;
  unsigned int tmp2;
int test[16];
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
    tmp1 = in_value1.read().to_uint();
    tmp2 = in_value2.read().to_uint();

    wait();
    wait();
    tmp2 = memory[tmp1];
    cout << "memory content " << tmp2 << endl;
    // write outputs
    out_value1.write( sc_bv<8>( tmp1 ) );
    out_value2.write( sc_bv<8>( tmp2 ) );
    out_valid.write(true);
    wait();
    out_valid.write(false);
    wait();
  }
}

// EOF

