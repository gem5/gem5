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

  fsmr.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* File containing functionality of the FSM recognizer */

#include "fsmr.h"

void fsm_recognizer::entry()
{
  char c;
  int state = 0;
  bool out;

  while (true) {
    do { wait(); } while (data_ready != true);
    c = input_char.read();
    cout << c << flush;

    switch(state) {
    case 0: 
      if (c == pattern[0]) state = 1;
      else state = 0;
      out = false;
      break;
    case 1:
      if (c == pattern[1]) state = 2;
      else state = 0;
      out = false;
      break;
    case 2:
      if (c == pattern[2]) state = 3;
      else state = 0;
      out = false;
      break;
    case 3:
      if (c == pattern[3]) out = true;
      else out = false;
      state = 0;
      break;
    default:
      cout << "Error: FSM in bad state." << endl;
      break;
    }

    found.write(out);
    wait(); // for writing the found signal
    found.write(false); // reset the found signal 
  }
}
