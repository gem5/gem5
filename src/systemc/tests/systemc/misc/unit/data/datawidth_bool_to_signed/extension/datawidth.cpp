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

  datawidth.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

                /*******************************************/
                /* Implementation Filename:  datawidth.cc  */
                /*******************************************/
 
#include "datawidth.h"
 
void
datawidth::entry()
{
  bool_vector6  tmp_a;
  bool_vector6  tmp_b;
  bool_vector9  tmp_result;

  while (true) {
    
    // HANDSHAKING
    do { wait(); } while (ready != 1);

    // COMPUTATION
    tmp_a = in1.read();
    tmp_b = in2.read();
    tmp_result = tmp_a.to_int() + tmp_b.to_int();

    // WRITE OUTPUT
    result.write(tmp_result);		// result = in1 + in2
    wait();
  }
}
