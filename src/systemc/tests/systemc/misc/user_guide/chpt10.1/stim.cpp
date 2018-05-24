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

  stim.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename stim.cc */
/* This is the interface file for synchronous process `stim' */

#include "stim.h"

void stim::entry()
{
  // Variable definition
  sc_unsigned	a_tmp(data_width);
  sc_unsigned	b_tmp(data_width);
  bool		c_tmp;
  int 		i, j;

  // Reset handler
  reset.write(0);
  wait(2);
   
  reset.write(1);
  wait();
  cout  << sc_time_stamp() << "\t : "
        << "RESET off \t...by stim" << endl;

  // Stimulus Generation
  c_tmp = 0;
  for (i=0; i<16; i++) {
    for (j=0; j<16; j++) {     
      a_tmp = j;
      b_tmp = i;
      c_tmp = !c_tmp;

      a.write(a_tmp);	
      b.write(b_tmp);	
      cin.write(c_tmp);

      ready.write(1);
      // wait();
      do { wait(); } while (done != 1); 

      ready.write(0);
      // do { wait(); } while (done == 1); 
      wait();
    }
  }
 
  sc_stop(); 
}
