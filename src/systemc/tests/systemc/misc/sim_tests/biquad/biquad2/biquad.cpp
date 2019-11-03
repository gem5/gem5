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

  biquad.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename biquad.cc */
/* This is the implementation file for synchronous process `biquad' */

#include "biquad.h"

void biquad::entry()
{
  float Acc; // Accumulator
  float sample; // Input sample

  if ((bool) reset == true) {
    Del[0] = Del[1] = Del[2] = Del[3] = 0.0;
    out.write(0.0);
    wait();
  }
  
  while (true) {
    sample = in.read();
    Acc = Cte[0] * sample;
    for (int i = 0; i < 4; i++) {
      Acc += Cte[i+1] * Del[i];
    }
    Acc = Acc / 1024.0;
    Del[1] = Del[0]; Del[0] = sample;
    Del[3] = Del[2]; Del[2] = Acc;
    out.write(Acc);
    wait();
  }
  
} // end of entry function
