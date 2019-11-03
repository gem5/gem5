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

  testbench.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename testbench.cc */
/* This is the implementation file for synchronous process `testbench' */

#include "testbench.h"

void testbench::entry()
{
  char buf[BUFSIZ];
  s.write(true);
  r.write(false);
  wait();
  sprintf(buf, "SR=%x%x QQ'=%x%x", true, false, q.read(), qp.read());
  cout << buf << endl;
  s.write(false);
  r.write(true);
  wait();
  sprintf(buf, "SR=%x%x QQ'=%x%x", false, true, q.read(), qp.read());
  cout << buf << endl;
  s.write(false);
  r.write(false);
  wait();
  sprintf(buf, "SR=%x%x QQ'=%x%x", false, false, q.read(), qp.read());
  cout << buf << endl;
  s.write(true);
  r.write(true);
  wait();
  sprintf(buf, "SR=%x%x QQ'=%x%x", true, true, q.read(), qp.read());
  cout << buf << endl;
  sc_stop();
} // end of entry function

