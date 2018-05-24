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

  stimulus.cpp -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-30

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "stimulus.h"

void stimulus::entry() {

  int i, j;

  // sending some reset values
  reset.write(true);
  in_valid.write(false);
  in_value.write(0);
  wait();
  reset.write(false);
  wait(5);
  for(i=0; i<3; i++){
    in_valid.write(true);
    for(j=0; j<=10; j++) {
      in_value.write(j);
      cout << "Stimuli1 : in_valid = true in_value " << j << " at "
	   << sc_time_stamp() << endl;
      wait();
    };
    in_valid.write(false);
    wait(4);
    for(j=0; j<=10; j++) {
      in_value.write(j);
      cout << "Stimuli2 : in_valid = true in_value " << j << " at "
	   << sc_time_stamp() << endl;
      wait();
    };
    in_valid.write(false);
    wait(4);
    for(j=0; j<=10; j++) {
      in_value.write(j);
      cout << "Stimuli3 : in_valid = true in_value " << j << " at "
	   << sc_time_stamp() << endl;
      wait();
    };
    wait(10);
  };

  wait(15);
  sc_stop();
}

// EOF
