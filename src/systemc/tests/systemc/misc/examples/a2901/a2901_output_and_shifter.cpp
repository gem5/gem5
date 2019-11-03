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

  a2901_output_and_shifter.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "a2901_output_and_shifter.h"

void
a2901_output_and_shifter::entry()
{
    bool i8, i7, f0, f3, q0, q3;
    sc_uint<3> i86;
    int4 z4;

    z4  = 0x0;
    i86 = I.read().range(8,6);
    i8  = I.read()[8];
    i7  = I.read()[7];
    f0  = F.read()[0];
    f3  = F.read()[3];
    q0  = Q.read()[0];
    q3  = Q.read()[3];

    Y.write( ( ( i86 == 0x2 ) && ( OEbar.read() == 0x0)) ? (uint64)A.read() :
	     (!( i86 == 0x2 ) && ( OEbar.read() == 0x0)) ? 
			(uint64)F.read() : (uint64)z4); 

    t_RAM0 .write( (( i8 == 0x1) && ( i7 == 0x0 )) ? f0 : 0x0);             
    t_RAM3 .write( (( i8 == 0x1) && ( i7 == 0x1 )) ? f3 : 0x0);           
    t_Q3   .write( (( i8 == 0x1) && ( i7 == 0x1))  ? q3 : 0x0);         
    t_Q0   .write( (( i8 == 0x1) && ( i7 == 0x0))  ? q0 : 0x0);         
}
