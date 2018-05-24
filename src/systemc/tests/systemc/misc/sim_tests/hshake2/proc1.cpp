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

#include "proc1.h"

void proc1::entry()
{
  int i = 10;

  data_ready.write(false);
  wait();

  while(true) {
    cout << endl << "Initiating Transfer" << endl;
    data_ready.write(true);
    data.write(i++);
    wait();
    cout << "Proc1: Data Ready has value = " << data_ready.read() 
	 << " at time " << sc_time_stamp() << endl;
    cout << "Proc1: Data Ack has value = " << data_ack.read() 
	 << " at same time" << endl;
    do { wait(); } while (data_ack == false);
    cout << "Proc1: Data Ack Received at time " << sc_time_stamp() << 
      endl;
    data_ready.write(false);
    wait();
    cout << "Proc1: Data Ready has value = " << data_ready.read() 
	 << " at time " << sc_time_stamp() << endl;
    cout << "Transfer complete" << endl;
  }
} // end of entry function

