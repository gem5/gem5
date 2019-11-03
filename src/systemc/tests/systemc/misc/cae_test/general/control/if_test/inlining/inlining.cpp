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

  inlining.cpp -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-22

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "inlining.h"

// list of defines
// #define MAXI(a,b) ((a)>(b)?(a):(b))
#define MAXI(a,b) ( (a) > (b) ? sc_biguint<4>( a ) : sc_biguint<4>( b ) )
#define clockedge wait()
#define my_print(a) cout << #a << " my print " <<  a << endl
#define my_if(a, b, c)   if (a < 6 ) { \
                           b = 0;      \
                         } else  {     \
                           b = c;      \
                         };

void inlining::entry(){

  sc_biguint<4>   tmp1;
  sc_biguint<4>    tmp2;
  sc_lv<4>        tmp3;
  sc_bv<4>        tmp4;

  // reset_loop
    if (reset.read() == true) {
      out_value1.write(0);
      out_value2.write(0);
      out_valid.write(false);
      clockedge;
    } else clockedge;

  //
  // main loop
  //
  while(1) {
    do { wait(); } while  (in_valid == false);

    //reading inputs
    tmp1 = in_value1.read();
    tmp2 = in_value2.read();
    tmp3 = in_value3.read();
    tmp4 = in_value4.read();

    //execution
    ++tmp1;
    out_value1.write(MAXI(tmp1, tmp2));
    my_print(tmp1);
    my_print(tmp2);
    clockedge;

    my_if(tmp2, tmp3, tmp4);
    out_value2.write(tmp4);
    out_valid.write(true);
    clockedge;
    out_valid.write(false);

  }
}

// EOF

