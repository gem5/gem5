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

  a2901_alu_inputs.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "a2901_alu_inputs.h"

void
a2901_alu_inputs::entry()
{
    int4 Av;
    int4 B;

    Av = RAM[Aadd.read()];
    B  = RAM[Badd.read()];
    A.write(Av);

    switch((int)(I.read().range(2,0))) {
    case 0x0:
    case 0x1:  
      RE.write(Av);
      break;
    case 0x2:
    case 0x3:
    case 0x4:
      RE.write(0x0);
      break;
    default:
      RE.write(D.read());
    }

    switch((int)(I.read().range(2,0))) {
    case 0x4:
    case 0x5:  
      S.write(Av);
      break;
    case 0x1:
    case 0x3:
      S.write(B);
      break;
    case 0x7:
      S.write(0x0);
      break;
    default:
      S.write(Q.read());
    }
}
