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

// test of ports binding in hierarchical model

#include "systemc.h"

SC_MODULE( mod_a )
{

  sc_in<int> in;  
  sc_out<int> out;

  SC_CTOR( mod_a )
  { }
};

SC_MODULE( mod_b )
{

  sc_in<int> in;  
  sc_out<int> out;

  SC_CTOR( mod_b )
  { }
};
 
// parent model
SC_MODULE( mod_c )
{

  sc_in<int> input;  
  sc_out<int> output;
  sc_signal<int> buf;
  mod_a module_a;
  mod_b module_b;
  
  SC_CTOR( mod_c ):
    module_a("module_a"),
    module_b("module_b")
  {

    module_a.in(input);
    module_a.out(buf);
    module_b.in(buf);
    module_b.out(output);

   }
};


int
sc_main( int, char*[] )
{
  mod_c c("c");
  cout << "binding of models to parent model is completed\n";
  return 0;
}
