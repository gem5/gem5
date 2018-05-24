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

  test3.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>
#include "arraytypes.h"

void arraytypes::entry(){

  char    tmp1_uns_lv[4][4];
  arr_struct1  a;
  arr_struct2  tmp2_uns_lv;

  // reset_loop
  out_valid.write(sc_bit(0));

  wait();
  while (true) { 
  while(in_valid.read() == "0") wait();
    wait();
    a = in_value1.read();
     for (int i = 0; i < 4; i++) {
       for (int j = 0; j < 4; j++) {
	 tmp1_uns_lv[i][j] = a.a[i][j];
       }
     }

     for (int i = 0; i < 4; i++) 
       for (int j = 0; j < 4; j++) 
	 tmp2_uns_lv.b[i][j] = tmp1_uns_lv[j][i];
     out_value1.write (tmp2_uns_lv);

    wait();

    out_valid.write( sc_bit(1));
    wait(); 


    out_valid.write( sc_bit(0));
    wait();

}
}
// EOF

