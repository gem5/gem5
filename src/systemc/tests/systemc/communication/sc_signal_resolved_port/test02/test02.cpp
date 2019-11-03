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

  test02.cpp -- 

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_inout_resolved::operator=

#include "systemc.h"


SC_MODULE( mod_a)
{
  sc_inout_resolved in_out1;
  sc_inout_resolved in_out2;
  sc_inout_resolved in_out3;
  sc_inout_resolved in_out4;
  sc_port<sc_signal_inout_if<sc_logic>,1> in_out5;
  sc_port<sc_signal_in_if<sc_logic>,1> in_1;

  sc_in<bool> clk;

  void main_action()
    {
      
      sc_logic m;
      m = 'Z';

      while(1){
      wait();
      cout<< m<<"   ";
      in_out1 = m;
      cout<<in_out1->read()<<"   ";
      in_out2 = in_out1;
      cout<<in_out2->read()<<"   ";
      in_out3 = in_1;
      cout<<in_out3->read()<<"   ";
      in_out4 = in_out5;
      cout<<in_out4->read()<<endl;
      }
    }
 
  SC_CTOR( mod_a ) 
    {
      SC_THREAD(main_action)
	sensitive << clk.pos();
     }
};

int sc_main(int, char*[])
{
  sc_clock clk("clk", 5, SC_NS);
  mod_a a("a");
  sc_signal_resolved sig1;
  sc_signal_resolved sig2;
  sc_signal_resolved sig3;
  sc_signal<sc_logic> sig4;
  sc_signal_resolved sig5;
  sc_signal<sc_logic> sig6;

  a.clk(clk);
  a.in_out1(sig1);
  a.in_out2(sig2);
  a.in_out3(sig3);
  a.in_1(sig4);
  a.in_out4(sig5);
  a.in_out5(sig6);

  sc_start(15, SC_NS);
  return 0;
}
