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

  test03.cpp -- 

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_port constructors for sc_signal_in(inout)_if interface 

#include "systemc.h"


SC_MODULE( mod_b )
{

  sc_port<sc_signal_in_if<bool>,1> input_1;
  sc_port<sc_signal_in_if<sc_logic>,1> input_2;
  sc_port<sc_signal_in_if<bool>,1> input_3;
  sc_port<sc_signal_in_if<sc_logic>,1> input_4;
  sc_port<sc_signal_inout_if<bool>,1> inout_1;
  sc_port<sc_signal_inout_if<sc_logic>,1> inout_2;
  sc_port<sc_signal_inout_if<bool>,1> inout_3;
  sc_port<sc_signal_inout_if<sc_logic>,1> inout_4;
 
  
  SC_CTOR( mod_b )
   { }
};

SC_MODULE( mod_c )
{
  mod_b b; 

  sc_signal<bool> sig1;
  sc_signal<bool> sig2;
  sc_signal<sc_logic> sig3;
  sc_signal<sc_logic> sig4;

  sc_port<sc_signal_in_if<bool>,1> in1;
  sc_port<sc_signal_in_if<sc_logic>,1> in2;
  sc_port<sc_signal_inout_if<bool>,1> inout1;
  sc_port<sc_signal_inout_if<sc_logic>,1> inout2;
  sc_port<sc_signal_in_if<bool>,1> in3;
  sc_port<sc_signal_in_if<sc_logic>,1> in4;
  sc_port<sc_signal_inout_if<bool>,1> inout3;
  sc_port<sc_signal_inout_if<sc_logic>,1> inout4;
  sc_port<sc_signal_in_if<bool>,1> in5;
  sc_port<sc_signal_in_if<sc_logic>,1> in6;
  sc_port<sc_signal_inout_if<bool>,1> inout5;
  sc_port<sc_signal_inout_if<sc_logic>,1> inout6;


  SC_CTOR( mod_c )
    : b("b"), 
      sig1("sig_1"),sig2("sig_2"), sig3("sig_3"), sig4("sig_4"),
      in1( "in_1", sig1 ), in2( "in_2", sig3 ), inout1( "inout_1", sig2),
      inout2( "inout_2", sig4), 
      in3("in_3", b.input_1), in4("in_4", b.input_2),
      inout3("inout_3", b.inout_1), inout4("inout_4", b.inout_2),
      in5(b.input_3), in6(b.input_4), inout5(b.inout_3), 
      inout6(b.inout_4)
  {}
};


#define WRITE(a) \
    cout << a.name() << " (" << a.kind() << ")" << endl


int sc_main(int, char* []){

  mod_c c("c");
  WRITE(c.sig1);
  WRITE(c.sig2);
  WRITE(c.sig3);
  WRITE(c.sig4);
  WRITE(c.in1);
  WRITE(c.in2);
  WRITE(c.in3);
  WRITE(c.in4);
  WRITE(c.in5);
  WRITE(c.in6);
  WRITE(c.inout1);
  WRITE(c.inout2);
  WRITE(c.inout3);
  WRITE(c.inout4);
  WRITE(c.inout5);
  WRITE(c.inout6);

  return 0;
}
