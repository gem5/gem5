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

  define.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#define CLOCK_PERIOD 100
#define TB_CLOCK_PERIOD 50
#define DUTY_CYCLE 0.5
#define EVENT_TIME 50
#define TEST_TIME 50

#define long_wait wait(10)
#define single_cycle wait(2)
#define set_value(var,val) wait(); var = val; wait()
#define test_value(actual, expected) \
	wait (); if (expected != actual) \
                cout << "Mismatch. Expected: " << expected \
	        << ". Actual: " << actual << endl; \
	wait ()
#define test_value_now(actual, expected) \
     if (expected != actual) cout << "Mismatch. Expected: " << expected \
	        << ". Actual: " << actual << endl;

