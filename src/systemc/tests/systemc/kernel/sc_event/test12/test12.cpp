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

  test12.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of next_trigger() for dynamic sensitivity; last call counts

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

    bool sender_first;

    void sender()
    {
        next_trigger( SC_ZERO_TIME );
        if( sender_first ) {
            sender_first = false;
            return;
        }

        e2.cancel();
        e3.cancel();
        e1.notify();
        e2.notify( SC_ZERO_TIME );
        e3.notify( 2, SC_NS );
        timed_out() ? write( "sender - timed out" )
                    : write( "sender" );
        next_trigger( 3, SC_NS, e_ack );
    }

    int receiver_state;

    void receiver()
    {
        sc_time t1( 1, SC_NS );

        switch( receiver_state ) {
	case 0:
	    // test next_trigger(e)
	    cout << "*** next_trigger(e)\n";

	    next_trigger( e1 );
	    break;
	case 1:
	    write( "receiver - e1" );
	    e_ack.notify();
	    next_trigger( e1 );
	    next_trigger( e2 );
	    break;
	case 2:
	    write( "receiver - e2" );
	    e_ack.notify();
	    next_trigger( e2 );
	    next_trigger( e3 );
	    break;
	case 3:
	    write( "receiver - e3" );
	    e_ack.notify();

	    // test next_trigger(or_list)
	    cout << "*** next_trigger(or_list)\n";

	    next_trigger( e3 );
	    next_trigger( e1 | e1 | e1 );
	    break;
	case 4:
	    write( "receiver - e1 | e1 | e1" );
	    e_ack.notify();
	    next_trigger( e1 | e1 | e1 );
	    next_trigger( e2 | e2 | e2 );
	    break;
	case 5:
	    write( "receiver - e2 | e2 | e2" );
	    e_ack.notify();
	    next_trigger( e2 | e2 | e2 );
	    next_trigger( e3 | e3 | e3 );
	    break;
	case 6:
	    write( "receiver - e3 | e3 | e3" );
	    e_ack.notify();
	    next_trigger( e3 | e3 | e3 );
	    next_trigger( e1 | e2 | e3 );
	    break;
	case 7:
	    write( "receiver - e1 | e2 | e3" );
	    e_ack.notify();
	    next_trigger( e1 | e2 | e3 );
	    next_trigger( e3 | e2 | e1 );
	    break;
	case 8:
	    write( "receiver - e3 | e2 | e1" );
	    e_ack.notify();

	    // test next_trigger(and_list)
	    cout << "*** next_trigger(and_list)\n";

	    next_trigger( e3 | e2 | e1 );
	    next_trigger( e1 & e1 & e1 );
	    break;
	case 9:
	    write( "receiver - e1 & e1 & e1" );
	    e_ack.notify();
	    next_trigger( e1 & e1 & e1 );
	    next_trigger( e2 & e2 & e2 );
	    break;
	case 10:
	    write( "receiver - e2 & e2 & e2" );
	    e_ack.notify();
	    next_trigger( e2 & e2 & e2 );
	    next_trigger( e3 & e3 & e3 );
	    break;
	case 11:
	    write( "receiver - e3 & e3 & e3" );
	    e_ack.notify();
	    next_trigger( e3 & e3 & e3 );
	    next_trigger( e1 & e2 & e3 );
	    break;
	case 12:
	    write( "receiver - e1 & e2 & e3" );
	    e_ack.notify();
	    next_trigger( e1 & e2 & e3 );
	    next_trigger( e3 & e2 & e1 );
	    break;
	case 13:
	    write( "receiver - e3 & e2 & e1" );

	    // test next_trigger(t)
	    cout << "*** next_trigger(t)\n";

	    next_trigger( e3 & e2 & e1 );
	    next_trigger( 0, SC_NS );
	    break;
	case 14:
	    write( "receiver - 0 ns" );
	    next_trigger( 0, SC_NS );
	    next_trigger( 1, SC_NS );
	    break;
	case 15:
	    write( "receiver - 1 ns" );

	    e_ack.notify();

	    // test next_trigger(t,e)
	    cout << "*** next_trigger(t,e)\n";

	    next_trigger( 1, SC_NS );
	    next_trigger( 1, SC_NS, e1 );
	    break;
	case 16:
	    timed_out() ? write( "receiver - 1 ns | e1 - timed out" )
		        : write( "receiver - 1 ns | e1" );
	    e_ack.notify();
	    next_trigger( 1, SC_NS, e1 );
	    next_trigger( t1, e2 );
	    break;
	case 17:
	    timed_out() ? write( "receiver - 1 ns | e2 - timed out" )
                        : write( "receiver - 1 ns | e2" );
	    e_ack.notify();
	    next_trigger( t1, e2 );
	    next_trigger( 1, SC_NS, e3 );
	    break;
	case 18:
	    timed_out() ? write( "receiver - 1 ns | e3 - timed out" )
		        : write( "receiver - 1 ns | e3" );
	    e_ack.notify();

	    // test next_trigger(t,or_list)
	    cout << "*** next_trigger(t,or_list)\n";

	    next_trigger( 1, SC_NS, e3 );
	    next_trigger( t1, e1 | e2 | e3 );
	    break;
	case 19:
	    timed_out() ? write( "receiver - 1 ns | e1 | e2 | e3 - timed out" )
                        : write( "receiver - 1 ns | e1 | e2 | e3" );
	    e_ack.notify();

	    // test next_trigger(t,and_list)
	    cout << "*** next_trigger(t,and_list)\n";

	    next_trigger( t1, e1 | e2 | e3 );
	    next_trigger( t1, e1 & e2 & e3 );
	    break;
	case 20:
	    timed_out() ? write( "receiver - 1 ns | e1 & e2 & e3 - timed out" )
                        : write( "receiver - 1 ns | e1 & e2 & e3" );

	    sc_stop();
	    write( "receiver - stop" );
	    next_trigger( t1, e1 & e2 & e3 );
	    next_trigger( SC_ZERO_TIME );
	    break;
	default:
	    sc_assert( false );
        }
	receiver_state ++;
    }

    SC_CTOR( mod_a )
    {
        SC_METHOD( sender );
        sender_first = true;
        SC_METHOD( receiver );
        receiver_state = 0;
    }
};

int
sc_main( int, char*[] )
{
    mod_a a( "a" );

    sc_start();

    return 0;
}
