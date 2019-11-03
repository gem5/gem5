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

  test01.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-03-22
                   Ucar Aziz, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems, 10 Aug 05
  Description of Modification: Rewrite to use sc_process_handle::kind().

 *****************************************************************************/

// test of sc_process_b::kind()

#include "systemc.h"

SC_MODULE( mod_a )
{
    void main_action()
    {
	cout << "main action ";
	cout << sc_get_current_process_b()->kind() << endl;
    }

    SC_CTOR( mod_a )
    {
	SC_METHOD( main_action );
    }
};

int
sc_main( int, char*[] )
{
    mod_a a( "a" );

    sc_start( 5, SC_NS );   

    return 0;
}
