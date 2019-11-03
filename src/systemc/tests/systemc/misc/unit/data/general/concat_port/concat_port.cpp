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

  concat_port.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

                /*********************************************/
                /* Implementation Filename:  concat_port.cc  */
                /*********************************************/
 
#include "concat_port.h"
 
void
concat_port::entry()
{
  bool_vector8	tmp_a;
  bool_vector8	tmp_b;

  while (true) {

    done.write(0);
    do { wait(); } while (ready != 1);

    tmp_a = a.read();
    tmp_b = b.read();

    switch (mode.read()) {

      case 0: 	c.write( (tmp_a.range(7,4), tmp_b.range(7,4)) );
       		d.write( (tmp_a, tmp_b) );
    		break;

      case 1: 	c.write( (tmp_a.range(0,7)) );
       		d.write( (tmp_a.range(0,7), tmp_b.range(0,7)) );
    		break;

      case 2: 	c.write( (tmp_a[0], tmp_b.range(1,4), tmp_a.range(7,5)) );
       		d.write( (tmp_a.range(7,4), tmp_b.range(7,4), 
		        tmp_a.range(3,0), tmp_b.range(3,0)) );
    		break;

      case 3: 	c.write( ("1", tmp_b.range(2,0), "0", tmp_a.range(2,0)) );
       		d.write( ("11", tmp_a.range(6,0), tmp_b.range(6,0)) );
    		break;

      default:	cout << "Error: Mode " << mode.read()
		     << " does not exist!" << endl; 
		break;

    }
    done.write(1);
    wait();
  }
}
