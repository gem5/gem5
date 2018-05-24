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

  sc_main.cpp -- 

  Original Author: Ray Ryan, Mentor Graphics, 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE(sctop) {
  SC_CTOR(sctop);
};

#ifdef MTI_INTEGRATION
  SC_MODULE_EXPORT(sctop);
#endif

sctop::sctop(sc_module_name name) : sc_module(name)
{
	sc_bigint<96> bigword("0x999888fffeeedddcccbbbaaa");

    cout << " bw:" << bigword.to_string(SC_HEX) 
        << "\n f0:"  << bigword.range( 11,  0).to_string(SC_HEX)
        << "\n f1:"  << bigword.range( 23, 12).to_string(SC_HEX)
        << "\n f2:"  << bigword.range( 35, 24).to_string(SC_HEX)
        << "\n f3:"  << bigword.range( 47, 36).to_string(SC_HEX)
        << "\n f4:"  << bigword.range( 59, 48).to_string(SC_HEX)
        << "\n f5:"  << bigword.range( 71, 60).to_string(SC_HEX)
        << "\n f6:"  << bigword.range( 83, 72).to_string(SC_HEX)
        << "\n f7:"  << bigword.range( 95, 84).to_string(SC_HEX)
        << endl;

}                                                                                                                      
int sc_main(int argc, char** argv) {
	sctop top("top"); 
	sc_start(1, SC_NS);
	return 0;
}


