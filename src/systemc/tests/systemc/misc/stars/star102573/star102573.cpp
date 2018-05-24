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

  star102573.cpp -- 

  Original Author: Preeti Panda, Synopsys, Inc., 2000-08-09

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>
#include "for_nest.h"

#define max  5
#define max1 4
#define max2 3
#define max3 2

void for_nest::entry()
{

  sc_signed tmp2(2);
  sc_signed tmp4(4);
  sc_signed tmp8(8);
  sc_signed tmp6(6);

  int i, j,  inp_tmp;

  result.write(0);
  out_valid.write(false);
  wait();

  main_loop:while(1) {

    while (in_valid.read()==false) wait();
    out_valid.write(true);
    wait();
    /* unrolled loop inside rolled loop */
    loop1:for (i=1; i<=max; i++)  {
      tmp2    = in_value.read();
      inp_tmp = tmp2.to_int() +  i ;
      loop2:for (j=1; j<=max1; j++)
	     inp_tmp = inp_tmp - tmp2.to_int() ;
      result.write(inp_tmp);
      wait();
    };
    out_valid.write(false);
    wait();
    out_valid.write(true);
    wait();

    inp_tmp = in_value.read();
    wait();
    /* unrolled loop inside unrolled loop */
    loop3:for (i=1; i<=max2; i++)  {
      inp_tmp += i ;
      loop4:for (j=1; j<=max3; j++) {
	inp_tmp -= j ;
      }
    };
    result.write(inp_tmp);
    out_valid.write(false);
    wait();
    out_valid.write(true);
    wait();

  }
}

