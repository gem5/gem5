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

  stimgen.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

                /*******************************************/
                /* Implementation Filename:  stimgen.cc  */
                /*******************************************/
 
#include "stimgen.h"
 
void
stimgen::entry()
{
  bool_vector8	d;

//  in1.write("0101_1001");
  in1.write("01011001");
  wait(2);

  cout << "IN1 = "  << in1.read() << endl;
  cout << "O1 = "   << o1.read() 
       << "  O2 = " << o2.read() << endl;
  cout << "O3 = "   << o3.read() << endl;
  cout << "O4 = "   << o4.read() << endl;
  cout << "O5 = "   << o5.read() << endl;

  sc_stop();
}
