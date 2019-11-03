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

  test02.cpp --

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_attribute

#include "systemc.h"

int
sc_main( int, char*[] )
{
  sc_clock clk;
  sc_attr_cltn att_cltn;
  sc_attr_cltn att_cln(att_cltn);
  sc_attribute<std::string> a1( "a1", "clock" );
  sc_attribute<std::string> a2( a1 );
  sc_attribute<int> a3( "a3", 24 );
  sc_attribute<std::string> a4( "a4", "casio" );
  sc_attribute<int> a5("a5");

  cout << a2.name() << endl;
  cout << a2.value << endl;

  clk.add_attribute( a2 );

  sc_attr_base* p = clk.get_attribute("a1");
  cout << p->name() << endl;
  sc_attribute<std::string>* pi = dynamic_cast<sc_attribute<std::string>*>( p );
   if( pi != 0 ) {
        cout << pi->value << endl;
    }

   cout << endl;
   cout << "added attributes to the attribute collection class" << endl;

   if(att_cltn.push_back(&a1) == true) {
     cout << a1.name() << "   ";
     cout << a1.value << endl;
   }

   if(att_cltn.push_back(&a3) == true) {
     cout << a3.name() << "   ";
     cout << a3.value << endl;
   }

   if(att_cltn.push_back(&a4) == true) {
     cout << a4.name() << "   ";
     cout << a4.value << endl << endl;
   }

   cout << "size of the class\n";
   cout << att_cltn.size()<< endl<< endl;

   cout << "looking whether the name is in collection class\n";
   sc_attr_base *pr = att_cltn.operator []("a3");
   if(pr != 0)
     cout << pr->name() << " exists" << endl;
   else
     cout << "a3 does not exist\n";

   sc_attr_base *pm = att_cltn.operator []("a5");
   if(pm != 0)
     cout << pm->name() << " exists" << endl;
   else
     cout << "a5 does not exist\n" << endl;

   cout << "removing names \n";
   sc_attr_base *pk = att_cltn.remove("a1");
   if(pk != 0)
     cout << "a1 is removed \n";
   else
     cout << "a1 does not exist\n";
   sc_attr_base *pn = att_cltn.remove("a5");
   if(pn != 0)
     cout << "a5 is removed \n";
   else
     cout << "a5 does not exist\n";

  return 0;
}
