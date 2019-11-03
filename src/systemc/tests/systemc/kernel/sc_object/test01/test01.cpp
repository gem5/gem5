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

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

//test of attributes 
#include "systemc.h"

SC_MODULE( mod_a )
{
  

  SC_CTOR( mod_a )
    {}
};
    

int
sc_main( int, char*[] )
{
   mod_a module("module");
   mod_a module_1();

   sc_attr_base a1("a1");
   sc_attr_base a2("a2");
   sc_attr_base a3("a3");
   
  module.add_attribute(a1);
  module.add_attribute(a2);
  module.add_attribute(a3);

  cout<<"attributes of object:\n";
  sc_attr_cltn& att_cltn = module.attr_cltn();
  sc_attr_cltn::const_iterator it = att_cltn.begin();
    for( ; it != att_cltn.end(); ++ it ) {
        cout << (*it)->name() << endl;
    }

  cout<<endl;
  module.dump(cout);
  module.dump();
  cout<<endl;

  module.print();

  cout<<endl<<endl<<"Module base name: "<<module.basename()<<endl;
  cout<<"number of attributes: "<< module.num_attributes()<<endl;
  cout<<"after removing attribute a1\n";
  module.remove_attribute("a1");
  cout<<"number of attributes: "<< module.num_attributes()<<endl;
  cout<<"removing all attributes\n";
  module.remove_all_attributes( );
  cout<<"number of attributes: "<< module.num_attributes()<<endl;
  cout<<endl;

  return 0;
}
