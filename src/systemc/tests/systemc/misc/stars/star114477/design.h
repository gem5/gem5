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

  design.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

struct fun:sc_module
{
  sc_in<bool>        clk;
  sc_in<int>         count;
  sc_bv<9>           x,y,z;
  sc_out<sc_bv<9> >  out_a0, out_a1;
  
  SC_CTOR(fun) {    
    SC_METHOD(entry1);
    sensitive << clk;
    SC_METHOD(entry2);
    sensitive << clk;
}
  
  void entry1();
  void entry2();
};
    
void fun::entry1()
{ 
  z = 0;      
  z[0] = x[1]&y[2];
  z[0] = x[1]|y[2];
  z[0] = x[1]^y[2];
  out_a0 = z;
  out_a1 = x;
}


void fun::entry2()
{ 
  if (count%3==0) {
    x = 120;
    y = 10;
  }
  else {
    x = 44;
    y = 5;
  }
}
