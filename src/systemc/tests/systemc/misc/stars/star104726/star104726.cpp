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

  star104726.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "quant.h"

	
void quant::do_quant()
{
   Coeff8x8 c;
   Coeff8x8 fuv;
   COEFF    ff;
   int      u,v;
  
  while(true) {

    ready.write(true);
    data_out_ready.write(false);
    do { wait(); } while (start==false);
    ready.write(false);

    c = data_in.read();

     // quantization
      for( v=0; v<8; v++ ) {
        for( u=0; u<8; u++ ) {
          ff = (c.get(v,u)<<1) / (COEFF)(coeff_quant[v][u]);
          fuv.put(v,u,(ff<0 ? ff : ff+1) >> 1);
        }
      }
  
     data_out.write(fuv);

    data_out_ready.write(true);
    do { wait(); } while (data_ok==false);

  }
}


