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

  sc_nbexterns.h -- External functions for both sc_signed and sc_unsigned
                    classes. These functions work on two parameters u and
                    v, and copy the result to the first parameter u. This
                    is also the reason that they are suffixed with _on_help.
 
                    The vec_* functions are called through either these
                    functions or those in sc_nbfriends.cpp. The functions in
                    sc_nbfriends.cpp perform their work on two inputs u and v,
                    and return the result object.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_nbexterns.h,v $
// Revision 1.2  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#ifndef SC_NBEXTERNS_H
#define SC_NBEXTERNS_H


#include "sysc/datatypes/int/sc_nbutils.h"


namespace sc_dt
{

extern 
void add_on_help(small_type &us, 
                 int unb, int und, sc_digit *ud, 
                 small_type vs, 
                 int vnb, int vnd, const sc_digit *vd);

extern 
void mul_on_help_signed(small_type &us,
                        int unb, int und, sc_digit *ud, 
                        int vnb, int vnd, const sc_digit *vd);

void div_on_help_signed(small_type &us,
                        int unb, int und, sc_digit *ud, 
                        int vnb, int vnd, const sc_digit *vd);

extern 
void mod_on_help_signed(small_type &us,
                        int unb, int und, sc_digit *ud, 
                        int vnb, int vnd, const sc_digit *vd);

extern 
void mul_on_help_unsigned(small_type &us,
                          int unb, int und, sc_digit *ud, 
                          int vnb, int vnd, const sc_digit *vd);

void div_on_help_unsigned(small_type &us,
                          int unb, int und, sc_digit *ud, 
                          int vnb, int vnd, const sc_digit *vd);

extern 
void mod_on_help_unsigned(small_type &us,
                          int unb, int und, sc_digit *ud, 
                          int vnb, int vnd, const sc_digit *vd);

extern 
void and_on_help(small_type us, 
                 int unb, int und, sc_digit *ud, 
                 small_type vs,
                 int vnb, int vnd, const sc_digit *vd);

extern 
void or_on_help(small_type us, 
                int unb, int und, sc_digit *ud, 
                small_type vs,
                int vnb, int vnd, const sc_digit *vd);

extern 
void xor_on_help(small_type us, 
                 int unb, int und, sc_digit *ud, 
                 small_type vs,
                 int vnb, int vnd, const sc_digit *vd);

} // namespace sc_dt


#endif
