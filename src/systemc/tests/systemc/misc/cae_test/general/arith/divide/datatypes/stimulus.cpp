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

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-10-01

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "stimulus.h"

void stimulus::entry() {

    reset.write(true);
    wait();
    reset.write(false);

    sc_unsigned tmp1(2);
    sc_signed tmp2(2);
    sc_unsigned tmp3(3);
    sc_signed tmp4(3);
    sc_unsigned zero_2(2);
    sc_unsigned zero_3(3);

    zero_3 = "000";
    zero_2 = "00";
    tmp1 = "01";
    tmp2 = "10";
    tmp3 = "010";
    tmp4 = "011";


    while(true){
       // handshake
       out_valid.write(true); 
       // write stimuli    
       out_value1.write(tmp1);
       out_value2.write(tmp2);
       out_value3.write(tmp3);
       out_value4.write(tmp4);
       cout << "Stimuli: "<< tmp1 << " " << tmp2 << " " << tmp3 << " " << tmp4 << endl;
       // update stimuli
       tmp1 = tmp1 + 1;
       if (tmp1 == zero_2) tmp1 = "01";
       tmp2 = tmp2 + 1;
       if (tmp2 == zero_2) tmp2 = "01";
       tmp3 = tmp3 + 1;
       if (tmp3 == zero_3) tmp3 = "001";
       tmp4 = tmp4 + 1;
       if (tmp4 == zero_3) tmp4 = "001";
       // handshake
       do { wait(); } while (in_ack==false);
       out_valid.write(false);
       wait();
    }
}
// EOF
