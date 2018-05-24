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

  test04.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-03-23

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_fifo events

#include "systemc.h"

#define W_INFO(msg) \
    cout << sc_time_stamp() << "," << sc_delta_count() \
         << ": writer: " << msg << endl;

#define R_INFO(msg) \
    cout << sc_time_stamp() << "," << sc_delta_count() \
         << ": reader: " << msg << endl;

SC_MODULE( writer )
{
    // port(s)
    sc_fifo_out<int> out;

    // process(es)
    void main_action()
    {
        int val = 0;
        while( true ) {
            wait( 10, SC_NS ); // wait for 10 ns
            W_INFO( "blocking write" );
            for( int i = 0; i < 20; i ++ ) {
                out.write( val ++ ); // blocking write
            }
            wait( 10, SC_NS );
            W_INFO( out.num_free() << " free spaces" );
            W_INFO( "non-blocking write" );
            for( int i = 0; i < 20; i ++ ) {
                while( ! out.nb_write( val ++ ) ) { // non-blocking write
                    W_INFO( "waiting" );
                    wait( out.data_read_event() );
                    W_INFO( "data read event" );
                }
            }
        }
    }

    SC_CTOR( writer )
    {
        SC_THREAD( main_action );
    }
};

SC_MODULE( reader )
{
    // port(s)
    sc_fifo_in<int> in;

    // process(es)
    void main_action()
    {
        int val;
        while( true ) {
            wait( 10, SC_NS ); // wait for 10 ns
            R_INFO( "blocking read 1" );
            for( int i = 0; i < 15; i ++ ) {
                in.read( val ); // blocking read
                R_INFO( val );
            }
            wait( 10, SC_NS );
            R_INFO( in.num_available() << " available samples" );
            R_INFO( "blocking read 2" );
            for( int i = 0; i < 15; i ++ ) {
                val = in.read(); // blocking read
                R_INFO( val );
            }
            wait( 10, SC_NS );
            R_INFO( in.num_available() << " available samples" );
            R_INFO( "non-blocking read" );
            for( int i = 0; i < 15; i ++ ) {
                while( ! in.nb_read( val ) ) { // non-blocking read
                    R_INFO( "waiting" );
                    wait( in.data_written_event() );
                    R_INFO( "data written event" );
                }
                R_INFO( val );
            }
        }
    }

    SC_CTOR( reader )
    {
        SC_THREAD( main_action );
    }
};

int sc_main( int, char*[] )
{
    // sc_clock c;

    // declare channel(s)
    sc_fifo<int> fifo( 10 );

    // instantiate block(s) and connect to channel(s)
    writer w( "writer" );
    reader r( "reader" );

    w.out( fifo );
    r.in( fifo );

    // run the simulation
    sc_start( 100, SC_NS );

    return 0;
}
