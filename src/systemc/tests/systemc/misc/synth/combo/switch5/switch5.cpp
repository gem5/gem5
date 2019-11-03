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

  switch5.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"
unsigned short
select( unsigned a, unsigned b, unsigned c )
{
    sc_unsigned x(7);
    sc_unsigned y(9);

    switch ((c >> 2) & 3) {
    case 0:
        x = a + b;
        return x.to_uint();
    case 1:
        x = a - b;
        break;
    case 2:
        x = (a >> 16) + (b << 16);
        return x.to_uint();
    case 3:
        x = (a << 16) - (b >> 16);
        break;
    }

    y = 2 * x;
    return y.to_uint();
}

int
sc_main( int, char** argv )   
{
    return 0;
}
