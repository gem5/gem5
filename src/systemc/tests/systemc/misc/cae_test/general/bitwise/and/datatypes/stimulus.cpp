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

    reset.write(true);
    wait();
    reset.write(false);

    sc_signed       tmp1(8);
    sc_signed       tmp2(8);
    long            tmp3;
    int             tmp4;
    short           tmp5;
    char            tmp6;
    char            tmp7;
    bool            tmp8;
    sc_unsigned     tmp9(4);
    sc_unsigned     tmp10(4);

    tmp1 = "0b01010101";
    tmp2 = "0b00000010";
    tmp3 = 12345678;
    tmp4 = -123456;
    tmp5 = 20000;
    tmp6 = '$'; 
    tmp7 = 'A';
    tmp8 = "0";
    tmp9 = "0b0001";
    tmp10 = "0b0010";

    while(true){
       out_valid.write(true);
       out_value1.write(tmp1);
       out_value2.write(tmp2);
       out_value3.write(tmp3);
       out_value4.write(tmp4);
       out_value5.write(tmp5);
       out_value6.write(tmp6);
       out_value7.write(tmp7);
       out_value8.write(tmp8);
       out_value9.write(tmp9);
       out_value10.write(tmp10);
       cout << "Stimuli: " << tmp1 << " " << tmp2 << " " << tmp3 << " " << tmp4 << " "
	    << tmp5 << " " << tmp6 << " " << tmp7 << " " << tmp8 << " " << tmp9  << " " << tmp10 <<endl;
       tmp1 = tmp1 + 2;
       tmp2 = tmp2 + 1;
       tmp3 = tmp3 + 5;
       tmp4 = tmp4 + 3;
       tmp5 = tmp5 + 6;
       tmp7 = tmp7 + 1;
       tmp8 = !tmp8;
       tmp9 = tmp9 + 1;
       tmp10 = tmp10 + 1;
       do { wait(); } while (in_ack==false);
       out_valid.write(false);
       wait();
    }
}
// EOF
