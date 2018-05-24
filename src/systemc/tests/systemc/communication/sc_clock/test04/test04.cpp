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

  test04.cpp -- 

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_clock::time_stamp()

#include "systemc.h"


int
sc_main( int, char*[] )
{
    sc_time t1( 8, SC_NS );
    sc_time t2( 2, SC_NS );
    
    sc_clock c1( "c1", t1, 0.1, t2 );
    sc_start(t1);
    cout << "current time is: ";  
    cout << c1.time_stamp();
    cout << endl;    

    sc_start(t2);
    cout << "now, the current time is: ";  
    cout << c1.time_stamp();
    cout << endl;    

    return 0;
}
