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

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of immediate notification check in the update phase

#include "systemc.h"

class my_signal
: public sc_signal<int>
{
public:

    my_signal()
        : sc_signal<int>()
        {}

protected:

    virtual void update()
        {
            if( m_new_val != m_cur_val ) {
                m_cur_val = m_new_val;
                ((sc_event&)value_changed_event()).notify(); // immediate notification!?
            }
        }
};

int
sc_main( int, char*[] )
{
    my_signal sig;

    sig.write( 1 );

    sc_start();

    return 0;
}
