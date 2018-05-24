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

  test_sem.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "test_sem.h"

TestSem::TestSem( sc_module_name )
        :sem_1(5)
{
	SC_THREAD(body_1);
	sensitive << clk.pos();
    SC_THREAD(body_2);
	sensitive << clk.pos();
}

void TestSem::body_1()
{
    unsigned int loop_counter=0;
    char buf[BUFSIZ];
    
    while (loop_counter++<10 && !sem_1.wait())
    {
        sprintf(buf, "time %f => thread1 : took semaphore %d times\n",
                sc_time_stamp().to_double(), loop_counter);
        cout << buf << flush;
}

    sprintf(buf, "time %f => thread1 : value of semaphore = %d\n",
            sc_time_stamp().to_double(), sem_1.get_value());
    cout << buf << flush;
    
    sc_stop();
}

void TestSem::body_2()
{
    unsigned int loop_counter=0;
    char buf[BUFSIZ];

    do
    {
        wait(2);
        sem_1.post();
        sprintf(buf, "time %f => thread2 : posted semaphore 1\n",
                sc_time_stamp().to_double());
        cout << buf << flush;
    }
    while (loop_counter++ < 5);

    wait(100);
}
