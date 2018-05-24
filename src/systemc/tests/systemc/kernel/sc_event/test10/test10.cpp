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

  test10.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of wait() for dynamic sensitivity

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_event e1;
    sc_event e2;
    sc_event e3;
    sc_event e_ack;

    void write( const char* msg )
    {
        cout << sc_delta_count() << ":" << sc_time_stamp()
             << " " << msg << "\n";
    }

    void sender()
    {
        // wait one delta cycle
        wait( SC_ZERO_TIME );

        while( true ) {
            e1.notify();
            e2.notify( SC_ZERO_TIME );
            e3.notify( 2, SC_NS );
            timed_out() ? write( "sender - timed out" )
                        : write( "sender" );
            wait( 3, SC_NS, e_ack );
            e2.cancel();
            e3.cancel();
        }
    }

    void receiver()
    {
        sc_time t1( 1, SC_NS );

        while( true ) {

            // test wait(e)
            cout << "*** wait(e)\n";

            wait( e1 );
            write( "receiver - e1" );
            e_ack.notify();
            wait( e2 );
            write( "receiver - e2" );
            e_ack.notify();
            wait( e3 );
            write( "receiver - e3" );
            e_ack.notify();

            // test wait(or_list)
            cout << "*** wait(or_list)\n";

            wait( e1 | e1 | e1 );
            write( "receiver - e1 | e1 | e1" );
            e_ack.notify();
            wait( e2 | e2 | e2 );
            write( "receiver - e2 | e2 | e2" );
            e_ack.notify();
            wait( e3 | e3 | e3 );
            write( "receiver - e3 | e3 | e3" );
            e_ack.notify();
            wait( e1 | e2 | e3 );
            write( "receiver - e1 | e2 | e3" );
            e_ack.notify();
            wait( e3 | e2 | e1 );
            write( "receiver - e3 | e2 | e1" );
            e_ack.notify();

            // test wait(and_list)
            cout << "*** wait(and_list)\n";

            wait( e1 & e1 & e1 );
            write( "receiver - e1 & e1 & e1" );
            e_ack.notify();
            wait( e2 & e2 & e2 );
            write( "receiver - e2 & e2 & e2" );
            e_ack.notify();
            wait( e3 & e3 & e3 );
            write( "receiver - e3 & e3 & e3" );
            e_ack.notify();
            wait( e1 & e2 & e3 );
            write( "receiver - e1 & e2 & e3" );
            e_ack.notify();
            wait( e3 & e2 & e1 );
            write( "receiver - e3 & e2 & e1" );

            // test wait(t)
            cout << "*** wait(t)\n";

            wait( 0, SC_NS );
            write( "receiver - 0 ns" );
            wait( 1, SC_NS );
            write( "receiver - 1 ns" );

            e_ack.notify();

            // test wait(t,e)
            cout << "*** wait(t,e)\n";

            wait( 1, SC_NS, e1 );
            timed_out() ? write( "receiver - 1 ns | e1 - timed out" )
                        : write( "receiver - 1 ns | e1" );
            e_ack.notify();
            wait( t1, e2 );
            timed_out() ? write( "receiver - 1 ns | e2 - timed out" )
                        : write( "receiver - 1 ns | e2" );
            e_ack.notify();
            wait( 1, SC_NS, e3 );
            timed_out() ? write( "receiver - 1 ns | e3 - timed out" )
                        : write( "receiver - 1 ns | e3" );
            e_ack.notify();

            // test wait(t,or_list)
            cout << "*** wait(t,or_list)\n";

            wait( t1, e1 | e2 | e3 );
            timed_out() ? write( "receiver - 1 ns | e1 | e2 | e3 - timed out" )
                        : write( "receiver - 1 ns | e1 | e2 | e3" );
            e_ack.notify();

            // test wait(t,and_list)
            cout << "*** wait(t,and_list)\n";

            wait( t1, e1 & e2 & e3 );
            timed_out() ? write( "receiver - 1 ns | e1 & e2 & e3 - timed out" )
                        : write( "receiver - 1 ns | e1 & e2 & e3" );

            sc_stop();
            write( "receiver - stop" );
            wait( SC_ZERO_TIME );
        }
    }

    SC_CTOR( mod_a )
    {
        SC_THREAD( sender );
        SC_THREAD( receiver );
    }
};

int
sc_main( int, char*[] )
{
    mod_a a( "a" );

    sc_start();

    return 0;
}
