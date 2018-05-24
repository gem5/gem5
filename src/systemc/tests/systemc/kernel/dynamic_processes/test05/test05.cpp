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

  Original Author: John Rose, Cadence Design Systems, Inc., 2004-02-15

 *****************************************************************************/

// tests sc_spawn() in start_of_simulation()

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test05.cpp,v $
// Revision 1.2  2011/02/01 17:17:40  acg
//  Andy Goodrich: update of copyright notice, added visible CVS logging.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc.h"
SC_MODULE(mod)
{
  void start_of_simulation()
  {
    cout << "spawn thetest" << endl;
    sc_spawn(sc_bind(&mod::thetest, this));
  }
  void thetest()
  {
    cout << "in thetest()" << endl;
    wait(10,SC_NS);
    sc_stop();
  }
  SC_CTOR(mod) {
  }
};

int sc_main(int, char**) {
  mod m1("m1");
  sc_start();
  return 0;
}
