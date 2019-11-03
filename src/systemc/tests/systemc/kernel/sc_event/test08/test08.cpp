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

  test08.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_event's methods; with four different receivers

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_event e;

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

            // test cancel()
            cout << "*** cancel()\n";

            // immediate notification
            e.notify();
            write( "sender - immediate" );
            wait( SC_ZERO_TIME );

            // immediate notification -- canceled (no effect)
            e.notify();
            write( "sender - immediate" );
            e.cancel();
            write( "sender - canceled" );
            wait( SC_ZERO_TIME );

            // delta notification
            e.notify( SC_ZERO_TIME );
            write( "sender - delta" );
            wait( SC_ZERO_TIME );
            wait( SC_ZERO_TIME );

            // delta notification -- canceled
            e.notify( SC_ZERO_TIME );
            write( "sender - delta" );
            e.cancel();
            write( "sender - canceled" );
            wait( SC_ZERO_TIME );
            wait( SC_ZERO_TIME );

            // timed notification
            e.notify( 1, SC_NS );
            write( "sender - timed 1 ns" );
            wait( 1, SC_NS );
            wait( SC_ZERO_TIME );

            // timed notification -- canceled
            e.notify( 1, SC_NS );
            write( "sender - timed 1 ns" );
            e.cancel();
            write( "sender - canceled" );
            wait( 1, SC_NS );
            wait( SC_ZERO_TIME );

            // timed notifiation -- canceled
            e.notify( 2, SC_NS );
            write( "sender - timed 2 ns" );
            wait( 1, SC_NS );
            e.cancel();
            write( "sender - canceled" );
            wait( 1, SC_NS );
            wait( SC_ZERO_TIME );

            // test notify() -- the exception test is in test03.cpp
            cout << "*** notify()\n";

            // delta notification -- made immediate
            e.notify( SC_ZERO_TIME );
            write( "sender - delta" );
            e.notify();
            write( "sender - immediate" );
            wait( SC_ZERO_TIME );
            wait( SC_ZERO_TIME );

            // timed notification -- made immediate
            e.notify( 1, SC_NS );
            write( "sender - timed 1 ns" );
            e.notify();
            write( "sender - immediate" );
            wait( 1, SC_NS );
            wait( SC_ZERO_TIME );

            // timed notification -- made immediate
            e.notify( 2, SC_NS );
            write( "sender - timed 2 ns" );
            wait( 1, SC_NS );
            e.notify();
            write( "sender - immediate" );
            wait( 1, SC_NS );
            wait( SC_ZERO_TIME );

            // test notify(t)
            cout << "*** notify(t)\n";

            e.notify( SC_ZERO_TIME );
            write( "sender - delta" );
            e.notify( 1, SC_NS );
            write( "sender - timed 1 ns" );
            wait( 1, SC_NS );
            wait( SC_ZERO_TIME );

            e.notify( 1, SC_NS );
            write( "sender - timed 1 ns" );
            e.notify( SC_ZERO_TIME );
            write( "sender - delta" );
            wait( 1, SC_NS );
            wait( SC_ZERO_TIME );

            e.notify( 2, SC_NS );
            write( "sender - timed 2 ns" );
            e.notify( 1, SC_NS );
            write( "sender - timed 1 ns" );
            wait( 2, SC_NS );
            wait( SC_ZERO_TIME );

            e.notify( 1, SC_NS );
            write( "sender - timed 1 ns" );
            e.notify( 2, SC_NS );
            write( "sender - timed 2 ns" );
            wait( 2, SC_NS );
            wait( SC_ZERO_TIME );

            sc_stop();
            write( "sender - stop" );
            wait( SC_ZERO_TIME );
        }
    }

    bool receiver_static_method_first;
    bool receiver_dynamic_method_first;

    void receiver_static_method()
    {
        if( receiver_static_method_first ) {
            receiver_static_method_first = false;
            return;
        }
        write( "receiver_static_method" );
    }

    void receiver_dynamic_method()
    {
        next_trigger( e );
        if( receiver_dynamic_method_first ) {
            receiver_dynamic_method_first = false;
            return;
        }
        write( "receiver_dynamic_method" );
    }

    void receiver_static_thread()
    {
        while( true ) {
            wait();
            write( "receiver_static_thread" );
        }
    }

    void receiver_dynamic_thread()
    {
        while( true ) {
            wait( e );
            write( "receiver_dynamic_thread" );
        }
    }

    SC_CTOR( mod_a )
    {
        SC_THREAD( sender );

        SC_METHOD( receiver_static_method );
        sensitive << e;
        receiver_static_method_first = true;

        SC_METHOD( receiver_dynamic_method );
        receiver_dynamic_method_first = true;

        SC_THREAD( receiver_static_thread );
        sensitive << e;

        SC_THREAD( receiver_dynamic_thread );
    }
};

int
sc_main( int, char*[] )
{
    mod_a a( "a" );

    sc_start();

    return 0;
}
