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

  test04.cpp -- 

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_signal_resolved::operator

#include "systemc.h"

SC_MODULE( mod_a )
{
  sc_in<bool> clk;

  sc_signal_resolved sig1;
  sc_signal_resolved sig2;

  void main_action(){
    sc_logic data('1');
    sig1.write(data);
    int i = 1;

    while(1){
      wait();
      cout<<i<<". cycle\n";
      cout<<sig1.read()<<endl;
      sig2 = sig1;
      cout<<sig2.read()<<endl;
      i = i+1;
    }
  }

  SC_CTOR( mod_a ):sig1("res_sig1"), sig2("res_sig2")
       {
        SC_THREAD(main_action);
        sensitive << clk.pos();
       }
};


int
sc_main( int, char*[] )
{
    sc_clock clk("clk",5, SC_NS);
    mod_a a( "a" );
    a.clk( clk );
    cout<<a.sig1.name()<<endl;
    cout<<a.sig2.name()<<endl;
    sc_start(10, SC_NS);

    return 0;
}
