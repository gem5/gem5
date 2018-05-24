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

  proc1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename proc1.cc */
/* This is the implementation file for synchronous process `proc1' */

#include "systemc.h"
#include "proc1.h"

void proc1::entry()
{
  data_ready.write(false);
  wait();
  cout << "Ready \t = False" << endl;

  while(true) {
    data_ready.write(true);
    do { wait(); } while (data_ack != true);
    cout << "Ack \t = True" << endl;

    data_ready.write(false);
    do { wait(); } while (data_ack != false);
    cout << "Ack \t = False" << endl;
  }
} // end of entry function

