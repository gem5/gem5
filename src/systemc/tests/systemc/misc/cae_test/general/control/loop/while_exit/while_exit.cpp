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

  while_exit.cpp -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-30

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "while_exit.h"

#define max 10

void while_exit::entry()
{

  int  i, inp_tmp;

  // reset_loop
  if (reset.read()==true) {
    result.write(0);
    out_valid.write(false);
    wait();
  } else wait(); 

  //----------
  // main loop
  //----------
  while(1) {

    // read inputs
    while (in_valid.read()==false) wait();

    // execution of while loop with exit after write statement 
    out_valid.write(true);
    i=1;
    wait();
    while (i<=max) {
      inp_tmp = in_value.read();
      result.write(inp_tmp); 
      if (inp_tmp==7) {
	wait();
	break;
      } else {
	i++;
	wait();
      };
    };
    out_valid.write(false);
    wait(6);

    // execution of do loop with continues
    out_valid.write(true);
    i=0;
    wait();
    do {
      i++;
      inp_tmp = in_value.read();
      if (i==8) {
	wait();
	continue;
      } else if (in_value.read()<5) {
	wait();
	continue;
      } else {
	result.write(inp_tmp);
	wait();
      }
    } while (i<=max);
    out_valid.write(false);
    wait(5);

    // execution of do loop with exit after before statement
    out_valid.write(true);
    i=0;
    wait();
    do {
      i++;
      inp_tmp = in_value.read();
      if (inp_tmp==7) {
	wait();
	break;
      } else {
	result.write(inp_tmp);
	wait();
      };
    } while (i<=max);
    out_valid.write(false);
    wait();

  }
}

// EOF

