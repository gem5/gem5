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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifdef RTL
#include "design_rtl.h"
#else
#include "design.h"
#endif

int sc_main(int ac, char *av[])
{
  // Signals
  sc_signal<bool>        clk;
  sc_signal<sc_bv<9> >  out_a0, out_a1;
  sc_signal<int>         count;

  fun c_block("fun_block");
  c_block.clk(clk);
  c_block.count(count);
  c_block.out_a0(out_a0);
  c_block.out_a1(out_a1);

  sc_start(0, SC_NS);

  count = 0;

  for(int i = 0; i < 10; i++){
      clk.write(1);
      sc_start( 5, SC_NS );
      clk.write(0);
      sc_start( 5, SC_NS );      
      count = count + 1;
      cout << "constants " << out_a0 << "\t" << out_a1 << endl;
  }
 
  return 0;
}
