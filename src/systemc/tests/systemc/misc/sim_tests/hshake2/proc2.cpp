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

  proc2.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename proc2.cc */
/* This is the implementation file for synchronous process `proc2' */

#include "proc2.h"

void proc2::entry()
{
  int i;

  data_ack.write(false);
  wait();
  
  while (true) {
    do { wait(); } while (data_ready == false);
    i = data.read();
    cout << "Proc2: Received data = " << i << " at time " << 
      sc_time_stamp() << endl;
    if (i > 12) {
      sc_stop();
    }
    data_ack.write(true);
    wait();
    do { wait(); } while (data_ready == true);
    data_ack.write(false);
  }
} // end of entry function

