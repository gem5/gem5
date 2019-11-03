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

  star103765.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>
#include "test.h"

void test::reset_loop() {
  sc_uint<8> tmp;
  unsigned int i, j;
  
  wait();
  
  done = 0;
  dato = 0;
  tmp = 0;
  
  wait();
  operational_loop: while(1 != 0) {
    wait();
    
    block1 : for(i = 0; i < 1; i++) {
      tmp = tmp + 1;
      dato = tmp;
      wait();

      tmp = tmp + 1;
      if(tmp < 5) {
        break;
      } else {
        continue;
      }
      tmp = tmp + 1; // should never get here
    }
    wait();
    done_loop : while(1) {
      dato = tmp;
      done = 1;
      wait();
    }
  }
}

