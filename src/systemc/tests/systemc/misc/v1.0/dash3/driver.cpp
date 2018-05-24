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
 
  driver.cpp -- Implementation of the driver.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: driver.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:26:24  acg
// systemc_tests-2.3
//
// Revision 1.4  2006/01/24 21:05:54  acg
//  Andy Goodrich: replacement of deprecated features with their non-deprecated
//  counterparts.
//
// Revision 1.3  2006/01/19 00:48:19  acg
// Andy Goodrich: Changes for the fact signal write checking is enabled.
//
// Revision 1.2  2006/01/18 00:23:50  acg
// Change over from SC_NO_WRITE_CHECK to sc_write_check_enable() call.
//

#define SC_NO_WRITE_CHECK
#include "systemc.h"
#include "const.h"
#include "driver.h"

// Driver's output actions.
void
driver_mod::driver_out_proc()
{
  wait();

  //  while (true) { 
  cout << "Driver is up @ " << sc_time_stamp() << endl;  

  // Car is at rest.
  speed_set = 0;
  reset = false;
  start = false;
  wait();
  
  cout << "Driver started the car @ " << sc_time_stamp() << endl;
  cout << "Driver set the speed to 40 km/h @ " << sc_time_stamp() << endl;
  start = true;
  speed_set = 40;
  wait();
  
  cout << "Driver set the speed to 120 km/h @ " << sc_time_stamp() << endl;
  speed_set = 120;
  wait();
  
  cout << "Driver reset the partial distance odometer @ " 
       << sc_time_stamp() << endl;
  cout << "Driver set the speed to 60 km/h @ " << sc_time_stamp() << endl;
  reset = true;
  speed_set = 60;
  wait();
  
  cout << "Driver set the speed to 40 km/h @ " << sc_time_stamp() << endl;
  speed_set = 40;
  wait();
  
  cout << "Driver stopped the car @ " << sc_time_stamp() << endl;
  sc_stop();
  //  }
}

// Driver's input actions.
void
driver_mod::driver_in_proc()
{
  if (speed.event()) {
    cout << "Current speed displayed = " 
         << speed << " km/h @ " << sc_time_stamp() << endl;
  }

  if (angle.event()) {
    cout << "Current speedometer angle = " << angle
         << " degrees @ " << sc_time_stamp() << endl;
  }

  if (total.event()) {
    cout << "Current total distance displayed = " 
         << total << " km @ " << sc_time_stamp() << endl;  
  }

  if (partial.event()) {
    cout << "Current partial distance displayed = " 
         << partial << " km @ " << sc_time_stamp() << endl;  
  }
}

// End of file
