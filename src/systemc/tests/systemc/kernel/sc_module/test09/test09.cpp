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

  test09.cpp -- Test derivation from and to sc_module instances.

  Original Author: Andy Goodrich, Forte Design Systemc, Inc. 2003-10-01

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

// sc_module ---> non-sc_module

SC_MODULE(ModuleBase) 
{
  public:
    SC_CTOR(ModuleBase)
    {
    }
	void base_method()
	{
		cout << sc_time_stamp() << ": ModuleBase" << endl;
	}
	sc_in_clk m_clk;
};

class NonModuleDerived : public ModuleBase
{
  public:
	SC_HAS_PROCESS(NonModuleDerived);
	NonModuleDerived(sc_module_name name_) : ModuleBase(name_)
	{
		SC_METHOD(base_method)
		sensitive << m_clk;
		SC_METHOD(derived_method)
		sensitive << m_clk;
	}
	void derived_method()
	{
		cout << sc_time_stamp() << ": NonModuleDerived" << endl;
	}
};

// non-sc_module ---> sc_module

class NonModuleBase
{
  public:
	sc_in_clk m_clk;
};

SC_MODULE(ModuleDerived), public NonModuleBase
{
	SC_CTOR(ModuleDerived) : NonModuleBase()
	{
		SC_METHOD(derived_method)
		sensitive << m_clk;
	}
	void derived_method()
	{
		cout << sc_time_stamp() << ": ModuleDerived" << endl;
	}
};

int sc_main(int argc, char* argv[])
{
	sc_clock clock;
	NonModuleDerived  non_derived("nonderived");
	ModuleDerived     derived("derived");
	non_derived.m_clk(clock);
	derived.m_clk(clock);

	sc_start(20, SC_NS);
	return 0;
}



