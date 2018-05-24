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

  ram.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename ram.cc */
/* This is the implementation file for asynchronous process `ram' */

#include "ram.h"

void ram::entry()
{
  int address;

  while (true) {
    do { wait(); } while (cs != true); 
    address = addr.read().to_int();
    if (we.read() == true) { // Write operation
      wait(wait_cycles-1);
      memory[address] = datain.read().to_int();
    }
    else { // Read operation
      if (wait_cycles > 2)
	wait(wait_cycles-2); // Introduce delay needed
      dataout.write(memory[address]);
      wait();
    }    
  }
} // end of entry function

