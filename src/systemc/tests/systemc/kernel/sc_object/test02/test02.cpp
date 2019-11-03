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

  Original Author: Andy Goodrich, Forte Design Systems, Inc., 2005-12-11

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// Test automatic object name generation.

#include "systemc.h"

class Name : public sc_object {
  public:
  	Name(const char* name) : sc_object(name)
	{}
};
  	
class NoName : public sc_object {
  public:
  	NoName() : sc_object()
	{}
};
  	
SC_MODULE(DUT)
{
	SC_CTOR(DUT)
	{
	}
};
int sc_main(int argc, char* argv[])
{
	Name            name(0);
	Name            name1("");
	NoName          no_name;

	cout << name.name() << endl;
	cout << name1.name() << endl;
	cout << no_name.name() << endl;
	cout << "Program completed" << endl;
	return 0;
}
