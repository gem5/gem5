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

  star107460.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// If one assigns a sc_lv vector containing X or Z to
// sc_bv the error message tells you that this is not
// possible, but doesn't tell you where(which variable
// at least) this happened. In a large program such a 
// message is impossible to trace

#include "systemc.h"

int sc_main(int, char**)
{
  sc_lv<8> lv("00X0100");
  sc_bv<8> bv;

  cout << "\nbv = "<<bv <<"\nlv = "<<lv;
  bv = lv;
  cout << "\nbv = "<<bv <<"\nlv = "<<lv<<"\n";
  
  return 0;
}
