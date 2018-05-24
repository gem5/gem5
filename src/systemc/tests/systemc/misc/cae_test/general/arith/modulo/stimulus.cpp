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

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-05-13

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "stimulus.h"

void stimulus::entry() {
  int             send_value1    = 254;
  sc_signed       send_value2(4);
  sc_unsigned     send_value3(4);
  sc_signed       send_value4(8);
  sc_unsigned     send_value5(8);


  reset.write(true);
  out_valid.write(false);
  send_value2 = 14;
  send_value3 = 14;
  send_value4 = 254;
  send_value5 = 254;
  out_stimulus1.write(0);
  out_stimulus3.write(0);
  out_stimulus3.write(0);
  out_stimulus4.write(0);
  out_stimulus5.write(0);
  wait(3);
  reset.write(false);
  while(true){
    wait(15);
    out_stimulus1.write( send_value1 );
    out_stimulus2.write( send_value2 );
    out_stimulus3.write( send_value3 );
    out_stimulus4.write( send_value4 );
    out_stimulus5.write( send_value5 );
    out_valid.write( true );
    cout << "Stimuli : " << send_value1 << " "
	 << send_value2 << " "
	 << send_value3 << " "
	 << send_value4 << " "
	 << send_value5 << " " << " at "
         << sc_time_stamp() << endl;
    send_value1--;
    send_value2 = send_value2-1;
    send_value3 = send_value3-1;
    send_value4 = send_value4-1;
    send_value5 = send_value5-1;
    wait();
    out_valid.write( false );
  }
}

// EOF
