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

  op_queue.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename op_queue.cc */
/* This is the implementation file for synchronous process `op_queue' */

#include "op_queue.h"

// Queue management. Assuming a circular queue.

void op_queue::entry()
{
  bool to_pop;
  int tail, head;
  bool queue_empty = true;
  bool queue_full = false;

  out = 0.0;

  head = 0;
  tail = 0;
  while (true) {
    if (!queue_full) {
      queue[tail] = in.read();
      tail = (tail + 1) % queue_size;
      queue_empty = false;
      if (tail == head) queue_full = true;
    }
    else {
      cout << "Warning: Data is being lost because queue is full" << endl;
    }

    to_pop = pop.read();
    if (to_pop) {
      if (!queue_empty) {
	out.write(queue[head]);
	head = (head + 1) % queue_size;
	queue_full = false;
	if (head == tail) queue_empty = true;
      }
      else {
	cout << "Warning: No data in queue to be popped" << endl;
      }
    }

    wait();
  }
    
} // end of entry function
