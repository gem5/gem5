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

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-30

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "inlining.h"

// list of defines
#define clock(a) wait(a)
#define intu4(a) sc_biguint<4>  a;
#define vec4(a) sc_lv<4>  a;
#define my_case(a, b, c)   switch (a) {     \
                             case 0:       \
                             case 1:       \
                             case 2:       \
                             case 3:       \
                               b = 0;      \
                               break;      \
                             default :     \
                               b = c;      \
                               break;      \
                           };
#define my_wait_case(a, b, c)  switch (a) { \
                                 case 0:   \
                                 case 1:   \
                                 case 2:   \
                                 case 3:   \
                                   b = 0;  \
                                   wait(); \
                                   break;  \
                                 default : \
                                   b = c;  \
                                   wait(); \
                                   break;  \
                               };  

void inlining::entry(){

  int tmp1;
  int tmp2;
  vec4(tmp3);
  sc_bv<4>  tmp4;

  // reset_loop
    if (reset.read() == true) {
      out_value1.write(0);
      out_value2.write(0);
      out_valid.write(false);
      clock(1);
    } else clock(1);

  //
  // main loop
  //
  while(1) {
    do { wait(); } while  (in_valid == false);

    //reading inputs
    tmp1 = in_value1.read().to_int();
    tmp2 = in_value2.read().to_int();
    tmp3 = in_value3.read();
    tmp4 = in_value4.read();

    //execution
    my_wait_case(tmp1, tmp3, tmp4); 
    out_value1.write(tmp3);
    clock(1);

    my_case(tmp2, tmp3, tmp4); 
    out_value2.write(tmp4);
    out_valid.write(true);
    clock(1);
    out_valid.write(false);

  }
}

// EOF

