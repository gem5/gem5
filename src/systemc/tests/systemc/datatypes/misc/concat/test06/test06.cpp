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

  test06 -- test for high order bit concatenation bug.

  Original Author: 

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int sc_main(int argc, char** argv) { 
  sc_bigint<2>   bi2; 
  sc_bigint<28>  bi28; 
  sc_bigint<30>  bi30; 
  sc_bigint<37>  bi37; 
  sc_bigint<39>  bi39; 
  sc_bigint<100> bi100; 
  sc_bigint<102> bi102; 
  sc_bigint<139> bi139; 
  sc_uint<39>    bu39;

  bi2 = 0; 
  bi28 = -1; 
  bi37 = -1; 
  cout << bi2.to_string(SC_BIN) << endl 
       << bi37.to_string(SC_BIN) << endl;
  cout << endl;

  bi30 = (bi2,bi28); 
  cout << bi30.to_string(SC_BIN) << endl;
  bi30 = (bi28,bi2);
  cout << bi30.to_string(SC_BIN) << endl;
  cout << endl;

  bi39 = (bi2,bi37); 
  cout << bi39.to_string(SC_BIN) << endl;
  bi39 = (bi37,bi2);
  cout << bi39.to_string(SC_BIN) << endl;
  cout << endl;

  bu39 = (bi2,bi37);
  cout << bu39.to_string(SC_BIN) << endl;
  bu39 = (bi37,bi2);
  cout << bu39.to_string(SC_BIN) << endl;
  cout << endl;

  bi100 = -1;
  bi102 = (bi100,bi2);
  cout << bi102.to_string(SC_BIN) << endl;
  bi102 = (bi2,bi100);
  cout << bi102.to_string(SC_BIN) << endl;
  cout << endl;

  bi39 = 0;
  bi139 = (bi39,bi100);
  cout << bi139.to_string(SC_BIN) << endl;
  bi139 = (bi100,bi39);
  cout << bi139.to_string(SC_BIN) << endl;
  cout << endl;

  bu39 = 0;
  bi139 = (bi100,bu39);
  cout << bi139.to_string(SC_BIN) << endl;
  bi139 = (bu39,bi100);
  cout << bi139.to_string(SC_BIN) << endl;
  cout << endl;

  sc_start(1, SC_NS); 
  return 0; 
}
