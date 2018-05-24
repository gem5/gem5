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

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_attribute - general test

#include "systemc.h"

int
sc_main( int, char*[] )
{
    sc_clock clk;

    sc_attribute<int> a1( "a1", 42 );
    sc_attribute<std::string> a2( "a2", "foobar" );

    clk.add_attribute( a1 );
    clk.add_attribute( a2 );

    sc_attr_base* p = clk.get_attribute( "a1" );
    cout << p->name() << endl;
    sc_attribute<int>* pi = dynamic_cast<sc_attribute<int>*>( p );
    if( pi != 0 ) {
        cout << pi->value << endl;
    }
    sc_attribute<std::string>* ps = dynamic_cast<sc_attribute<std::string>*>( p );
    if( ps != 0 ) {
        cout << ps->value << endl;
    }

    const sc_attr_cltn& attrs = clk.attr_cltn();
    sc_attr_cltn::const_iterator it = attrs.begin();
    for( ; it != attrs.end(); ++ it ) {
        cout << (*it)->name() << endl;
    }

    clk.remove_attribute( "a1" );
    clk.remove_attribute( "a2" );

    return 0;
}
