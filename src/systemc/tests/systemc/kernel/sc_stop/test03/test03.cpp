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

  test03.cpp -- 

  Original Author: Andy Goodrich, Forte Design Systems

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: 
  Description of Modification: 

 *****************************************************************************/

#include "systemc.h"

SC_MODULE(X)
{
	SC_CTOR(X)
	{
		cout << "Creating able..." << endl;
		SC_THREAD(able);
		sensitive << immediate_event;
		cout << "Creating baker..." << endl;
		SC_THREAD(baker);
	}

	void able()
	{
		wait();
		cout << "able: " << sc_time_stamp() << endl;
	}
	void baker()
	{
		cout << "baker: " << sc_time_stamp()
			 << ": issuing sc_stop()" <<  endl;
		sc_stop();
		immediate_event.notify();
	}
	sc_event immediate_event;
};

int sc_main(int argc, char* argv[] )
{
	X        x("x");

	//sc_set_stop_mode(SC_STOP_IMMEDIATE);
	sc_start(100, SC_NS);
	return 0;
}


