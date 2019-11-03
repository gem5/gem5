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

  test01.cpp -- 

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_signal_resolved_port constructors

#include "systemc.h"


#define WRITE(a) \
    cout << a.name() << " (" << a.kind() << ")" << endl


SC_MODULE( mod_a)
{
  sc_signal_resolved sig_1;
  sc_signal_resolved sig_2;
  sc_signal_resolved sig_3;
  sc_signal_resolved sig_4;

  sc_in_resolved in_1;
  sc_in_resolved in_2;
  sc_in_resolved in_3;
  sc_in_resolved in_4;
  sc_in_resolved in_5;
  sc_in_resolved in_6;
  sc_inout_resolved in_out1;
  sc_inout_resolved in_out2;
  sc_inout_resolved in_out3;
  sc_inout_resolved in_out4;
  sc_inout_resolved in_out5;
  sc_inout_resolved in_out6;
  
  SC_CTOR( mod_a ):in_1(), in_2("in_2"), in_4("in_4"),
    in_out4("in_out4"), in_out5(), in_out6("in_out6") 
    {
     in_3(sig_1);
     in_4(sig_2);
     in_out3(sig_3);
     in_out4(sig_4);
     WRITE(in_1);
     WRITE(in_2);
     WRITE(in_3);
     WRITE(in_4);
     WRITE(in_5);
     WRITE(in_6);
     WRITE(in_out1);
     WRITE(in_out2);
     WRITE(in_out3);
     WRITE(in_out4);
     WRITE(in_out5);
     WRITE(in_out6);
    }
};

SC_MODULE(mod_b)
{
  mod_a a;
  sc_in_resolved input_1;
  sc_in_resolved input_2;
  sc_in_resolved input_3;
  sc_in_resolved input_4;


  SC_CTOR( mod_b ):a("a"), input_2("input_2"), input_4("input_4")
 {
   input_1(a.in_5);
   input_2(a.in_6);
   input_3(a.in_out1);
   input_4(a.in_out2);
   WRITE(input_1);
   WRITE(input_2);
   WRITE(input_3);
   WRITE(input_4);
  }


};

int sc_main(int, char* []){

  mod_b b("b");
 
  return 0;
}
