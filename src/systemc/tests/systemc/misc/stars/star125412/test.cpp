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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
I believe there is a bug in SystemC 2.0 beta 1 which
causes simulation to end prematurely in certain cases.
Specifically, if an event notification occurs at a particular
time but no processes are ready to run at that time,
simulation will end. However, there may still be processes
that need to be run at future times.

Below I've included an example which demonstrates the bug,
and below that I've provided a proposed fix for the bug.

// This SystemC 2.0 example shows the bug...
*/

#include <systemc.h>

class top : public sc_module
{
   public:
     SC_HAS_PROCESS(top);

     top(sc_module_name name) : sc_module(name)
     {
         SC_THREAD(main);
     }

     void main()
     {
         sc_event e;

         // comment out the following line to see bug go away

         e.notify(55, SC_NS);

         for (int i = 0; i < 10; i++)
         {
           wait(10, SC_NS);
           cout << "main thread at time " << sc_time_stamp() << endl;
         }
     }
};

int sc_main (int argc , char *argv[])
{
   top top1("Top1");
   sc_start();
   return 0;
}

/*
/////// End of example

The proposed fix for this in SystemC 2.0 beta 1 is:

Edit src/sysc/kernel/sc_simcontext.cpp

Lines 801-802, which read:

// no more runnable processes
break;

Should be changed to read:

// no runnable processes at current time, but maybe in future
t = next_time();
continue;

The same changes should be applied at lines 845-846
Then rebuild and reinstall the SystemC library.


___________________________________________________
Stuart Swan, Senior Architect
System Level Design Group, Cadence Design Systems
Building 11
2670 Seely Avenue
San Jose, CA 95134
Phone: +1 408 895 4579
Email: stuart@cadence.com
___________________________________________________

*/
