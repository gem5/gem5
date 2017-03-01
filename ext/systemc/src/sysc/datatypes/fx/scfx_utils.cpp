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

  scfx_utils.cpp - 

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: scfx_utils.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include "sysc/datatypes/fx/scfx_utils.h"


namespace sc_dt
{

void
scfx_tc2csd( scfx_string& s, int w_prefix )
{
    if( w_prefix != 0 ) {
	SC_ASSERT_( s[0] == '0' && s[1] == 'c' &&
		    s[2] == 's' && s[3] == 'd', "invalid prefix" );
    }

    scfx_string csd;

    // copy bits from 's' into 'csd'; skip prefix, point, and exponent
    int i = 0;
    int j = (w_prefix != 0 ? 4 : 0);
    while( s[j] )
    {
	if( s[j] == '0' || s[j] == '1' )
	    csd[i ++] = s[j];
	else if( s[j] != '.' )
	    break;
	++ j;
    }
    csd[i] = '\0';

    // convert 'csd' from two's complement to csd
    -- i;
    while( i >= 0 )
    {
	if( csd[i] == '0' )
	    -- i;
	else
	{
	    if( i > 0 && csd[i - 1] == '0' )
		-- i;
	    else if( i == 0 )
		csd[i --] = '-';
	    else
	    {   // i > 0 && csd[i - 1] == '1'
		csd[i --] = '-';
		while( i >= 0 && csd[i] == '1' )
		    csd[i --] = '0';
		if( i > 0 )
		    csd[i] = '1';
		else if( i == 0 )
		    csd[i --] = '1';
	    }
	}
    }

    // copy bits from 'csd' back into 's'
    i = 0;
    j = (w_prefix != 0 ? 4 : 0);
    while( csd[i] )
    {
	if( s[j] == '.' )
	    ++ j;
	s[j ++] = csd[i ++];
    }
}


void
scfx_csd2tc( scfx_string& csd )
{
    SC_ASSERT_( csd[0] == '0' && csd[1] == 'c' &&
		csd[2] == 's' && csd[3] == 'd', "invalid prefix" );

    scfx_string s;

    // copy bits from 'csd' into 's'; skip prefix, point, and exponent
    int i = 0;
    s[i ++] = '0';
    int j = 4;
    while( csd[j] )
    {
	if( csd[j] == '-' || csd[j] == '0' || csd[j] == '1' )
	    s[i ++] = csd[j];
	else if( csd[j] != '.' )
	    break;
	++ j;
    }
    s[i] = '\0';

    // convert 's' from csd to two's complement
    int len = i;
    i = 1;
    while( i < len )
    {
        while( i < len && s[i] != '-' )
	    i ++;
	if( i < len )
	{
	    j = i ++;
	    s[j --] = '1';
	    while( j >= 0 && s[j] == '0' )
	        s[j --] = '1';
	    if( j >= 0 )
	        s[j] = '0';
	}
    }

    // copy bits from 's' back into 'csd'
    j = csd.length();
    csd[j + 1] = '\0';
    while( j > 4 )
    {
	csd[j] = csd[j - 1];
	-- j;
    }
        
    i = 0;
    j = 4;
    while( s[i] )
    {
	if( csd[j] == '.' )
	    ++ j;
	csd[j ++] = s[i ++];
    }
}

} // namespace sc_dt


// Taf!
