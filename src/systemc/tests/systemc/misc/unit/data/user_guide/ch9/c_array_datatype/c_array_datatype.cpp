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

  c_array_datatype.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Main file for "C array" data type */ 

#include "systemc.h"

int sc_main(int ac, char *av[])
{

// 1. DECLARATION SYNTAX
  int			a[4] = { 0, 23, -534, 23423 };  
  long			b[4] = { 0, 23, -534, 23423 };  
  short			c[4] = { 0, 23, -534, 23423 };  
  char			d[9] = { 'U', 'X', '0', '1', 'Z', 'W', 'L', 'H', '-' };

  unsigned int		e[4] = { 0, 23, 534, 23423 };  
  unsigned long		f[4] = { 0, 23, 534, 23423 };  
  unsigned short	g[4] = { 0, 23, 534, 23423 };  
  unsigned char		h[9] = { 'U', 'X', '0', '1', 'Z', 'W', 'L', 'H', '-' };

  float			i[4] = { 0, 4.89, -345.6778, 543222.898394322 };  
  double		j[4] = { 0, 4.89, -345.6778, 543222.898394322 };  
//  type 'long double' is non-portable
//  long double		k[4] = { 0, 4.89, -345.6778, 543222.898394322 };  

  bool			l[4] = { 0, 1, true, false };  

  sc_logic m[9] = { sc_logic('U'), sc_logic('X'), sc_logic('0'), sc_logic('1'),
    sc_logic('Z'), sc_logic('W'), sc_logic('L'), sc_logic('H'), sc_logic('-') };


// 2. TYPE CONVERSION

  // No type conversion because assignment of arrays is illegal

// 3. OPERATORS
//    Supported operators:      []

  cout.precision(15);
  cout	<< "\nINT \t\t"	 
		<< a[0] << "\t" << a[1] << "\t" << a[2] << "\t" << a[3] 
      	<< "\nLONG \t\t" 
		<< b[0] << "\t" << b[1] << "\t" << b[2] << "\t" << b[3] 
      	<< "\nSHORT \t\t" 
		<< c[0] << "\t" << c[1] << "\t" << c[2] << "\t" << c[3] 
      	<< "\nCHAR \t\t" 
		<< d[0] << "\t" << d[1] << "\t" << d[2] << "\t" 
		<< d[3] << "\t" << d[4] << "\t" << d[5] << "\t" 
		<< d[6] << "\t" << d[7] << "\t" << d[8]
      	<< "\n\nUNSIGNED INT \t"	 
		<< e[0] << "\t" << e[1] << "\t" << e[2] << "\t" << e[3] 
      	<< "\nUNSIGNED LONG \t" 
		<< f[0] << "\t" << f[1] << "\t" << f[2] << "\t" << f[3] 
      	<< "\nUNSIGNED SHORT \t" 
		<< g[0] << "\t" << g[1] << "\t" << g[2] << "\t" << g[3] 
      	<< "\nUNSIGNED CHAR \t" 
		<< h[0] << "\t" << h[1] << "\t" << h[2] << "\t" 
		<< h[3] << "\t" << h[4] << "\t" << h[5] << "\t" 
		<< h[6] << "\t" << h[7] << "\t" << h[8]
      	<< "\n\nFLOAT \t\t" 
		<< i[0] << "\t" << i[1] << "\t" << i[2] << "\t" << i[3] 
      	<< "\nDOUBLE \t\t" 
		<< j[0] << "\t" << j[1] << "\t" << j[2] << "\t" << j[3] 
//  type 'long double' is non-portable
//      	<< "\nLONG DOUBLE \t" 
//		<< k[0] << "\t" << k[1] << "\t" << k[2] << "\t" << k[3] 
      	<< "\n\nBOOL \t\t" 
		<< l[0] << "\t" << l[1] << "\t" << l[2] << "\t" << l[3] 
      	<< "\nSTD_ULOGIC \t" 
		<< m[0] << "\t" << m[1] << "\t" << m[2] << "\t" 
		<< m[3] << "\t" << m[4] << "\t" << m[5] << "\t" 
		<< m[6] << "\t" << m[7] << "\t" << m[8]
      	<< endl;
   return 0;
}
