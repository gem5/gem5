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

  test13.cpp -- 

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of next_trigger() for static sensitivity

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_event e1;
    sc_event e2;
 
    void write( const char* msg )
    {
        cout <<"simulation time" << ":" << sc_time_stamp()
             << " " << msg << endl;
     
    }

    void sender()
    {
      write( "sender - e2" );
      e2.notify(10, SC_NS );
      next_trigger();
    }


    void receiver()
    {
        next_trigger();
	write( "receiver - e1" );
        e1.notify(SC_ZERO_TIME );
    }
 

    SC_CTOR( mod_a )
    {
        SC_METHOD( sender );
        sensitive << e1;
        SC_METHOD( receiver );
        sensitive << e2;
    }
};


int
sc_main( int, char*[] )
{
    mod_a a( "a" );
    cout<<"sender notifies e2 after 10 ns, receiver e1 at zero time" << endl;
    sc_start(50,SC_NS);

    return 0;
}
