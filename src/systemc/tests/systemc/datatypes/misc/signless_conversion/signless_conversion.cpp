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

  signless_conversions.cpp -- Test Casts Of Sign-Less Values

  Original Author: Andy Goodrich, Forte Design Systems, 7 Apr 2005

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

#define DISP(exp) \
{ \
	cout << #exp << " = " << exp << endl; \
}

#define DISPLAY(var) \
{ \
	DISP( sc_int_base(var) ) \
	DISP( sc_uint_base(var) ) \
	DISP( sc_signed(var) ) \
	DISP( sc_unsigned(var) ) \
    cout << endl; \
}

int
sc_main( int argc, char* argv[] )
{
	sc_bigint<6>  bi;
	sc_biguint<6> bui;
    sc_bv<4>      bv;
	sc_int<6>     i;
    sc_lv<4>      lv;
	sc_int<6>     ui;

	bi =  10;
	bui = 10;
	bv =  "1010";
	i =   10;
	lv =  "1010";
	ui =  10;

    DISPLAY(bi(3,0))	
    DISPLAY(bui(3,0))	
    DISPLAY(bv)	
    DISPLAY(i(3,0))	
    DISPLAY(lv)	
    DISPLAY(ui(3,0))	

    DISPLAY((bi(3,2),i(1,0)))	
    DISPLAY((bi(3,2),ui(1,0)))	
    DISPLAY((bi(3,2),bui(1,0)))	
    DISPLAY((bi(3,2),bi(1,0)))	

    DISPLAY((bui(3,2),i(1,0)))	
    DISPLAY((bui(3,2),ui(1,0)))	
    DISPLAY((bui(3,2),bui(1,0)))	
    DISPLAY((bui(3,2),bi(1,0)))	

    DISPLAY((i(3,2),i(1,0)))	
    DISPLAY((i(3,2),ui(1,0)))	
    DISPLAY((i(3,2),bui(1,0)))	
    DISPLAY((i(3,2),bi(1,0)))	

    DISPLAY((ui(3,2),i(1,0)))	
    DISPLAY((ui(3,2),ui(1,0)))	
    DISPLAY((ui(3,2),bui(1,0)))	
    DISPLAY((ui(3,2),bi(1,0)))	

    return 0;
}
