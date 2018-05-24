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

  star110668.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"
typedef struct tstruct {
  sc_int<16> val1;
  sc_int<16> val2;
} dummy_struct;
SC_MODULE(design) {
  sc_in_clk    clock;
  sc_in <sc_int<8> > input;
  sc_out<sc_int<8> > output;
  void write_in_fifo();
  SC_CTOR(design) {
    SC_CTHREAD(write_in_fifo, clock.pos() );
  }  
};
void design :: write_in_fifo() {
  sc_int<8> data_in;
  dummy_struct foofoo;
 write_loop: while (1) {
   data_in = input.read();
   foofoo.val1.range(0,7) = data_in;
   output.write(foofoo.val2.range(8,15));
   wait();
 }
}
