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

      Name, Affiliation, Date: Bishnupriya Bhattacharya, Cadence Design Systems,
                               September 5, 2003
  Description of Modification: - set stop mode to SC_STOP_IMMEDIATE;
			       - add more output printout to make sure that
				 the immdiate event notification of "e_b" after 
				 issuing sc_stop() does not make the 
				 sensitive process "thread_a" run

 *****************************************************************************/

// test of immediate event notification

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_event e_a;
    sc_event e_b;

    int n;

    void thread_a()
    {
        wait( SC_ZERO_TIME );
        n = 0;
        while( true ) {
            n ++;
            e_a.notify();
            wait( e_b );
	    cout << "Triggered by event e_b" << endl;
        }
    }

    void thread_b()
    {
        while( true ) {
            wait( e_a );
            cout << sc_delta_count() << " " << n << endl;
            if( n == 20 ) {
                cout << "Issuing sc_stop() " << endl;
                sc_stop();
            }
            e_b.notify();
        }
    }

    SC_CTOR( mod_a )
    {
        SC_THREAD( thread_a );
        SC_THREAD( thread_b );
    }
};

int
sc_main( int, char*[] )
{
    mod_a a( "a" );

    sc_set_stop_mode(SC_STOP_IMMEDIATE);
    sc_start();

    return 0;
}
