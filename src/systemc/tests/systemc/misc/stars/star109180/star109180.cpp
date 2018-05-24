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

  star109180.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"
#include "regfile.h"

void test::reset_loop() {
  sc_uint<14> cell[2];
  sc_uint<14> cell1[2];

  sc_uint<14> dat0, dat3;
  sc_uint<14> out0, out3;
  
  dato.write(0);
  dat0 = dat3 = out0 = out3 = 0;
  ready.write(sc_logic('0'));
  done.write(sc_logic('0'));
  
  wait();
  while (1) {
    ready.write(sc_logic('1'));
    wait();

    ready.write(sc_logic('0'));
    dat0 = dati.read();
    wait();
    
    dat3 = dati.read();

    cell[0] = dat0;

    cell1[1] = dat3;

    out0 = cell[0];
    out3 = cell1[1];
    
    dato.write(out0);
    done.write(sc_logic('1'));
    wait();

    dato.write(out3);
    done.write(sc_logic('0'));
    wait();

  }
}
