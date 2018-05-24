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

  select.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

#define WIDTH 32
#define COUNT 10000

int sc_main( int ac, char* av[] )
{
  sc_int<WIDTH> Bx,By;

  for(unsigned int i = 0; i < WIDTH; i++){
    cout << "i = " << i << endl;
    for(unsigned int j=0; j < COUNT; j++ ){
      
      /*    ( By.range(WIDTH-1, i+1), By.range(i,0) ) = Bx;
	    sc_assert( By == Bx );
	    */
      
      Bx = j;
      By.range(WIDTH-1,i) = Bx.range(WIDTH-1,i);
      if(i >= 1)
	By.range(i-1,0) = Bx.range(i-1,0);
      sc_assert( Bx == By );
    }
  }

  return 0;
}
