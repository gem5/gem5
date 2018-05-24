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

  Original Author: Bishnupriya Bhattacharya, Cadence Design Systems, Inc., 2004-03-10

 *****************************************************************************/

// tests parent spawning process dying before child spawned process.

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test06.cpp,v $
// Revision 1.2  2011/02/01 17:17:40  acg
//  Andy Goodrich: update of copyright notice, added visible CVS logging.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>

void p3() {
  cerr << sc_time_stamp() << ":entering p3" << endl;
  wait(30, SC_NS);
  cerr << sc_time_stamp() << ":exiting p3" << endl;
}

void p2() {
  cerr << sc_time_stamp() << ":entering p2, spawning p3" << endl;
  sc_spawn(sc_bind(&p3));
  wait(20, SC_NS);
  cerr << sc_time_stamp() << ":exiting p2" << endl;
}

void p1() {
  cerr << sc_time_stamp() << ":entering p1, spawning p2" << endl;
  sc_spawn(sc_bind(&p2));
  wait(10, SC_NS);
  cerr << sc_time_stamp() << ":exiting p1" << endl;
}

void p0() {
  cerr << sc_time_stamp() << ":entering p0, spawning p1" << endl;
  sc_spawn(sc_bind(&p1));
  cerr << sc_time_stamp() << ":exiting p0" << endl;
}

int sc_main(int, char**) {

  sc_start(10, SC_NS);
  p0();
  sc_start(50, SC_NS);

  return 0;
}
