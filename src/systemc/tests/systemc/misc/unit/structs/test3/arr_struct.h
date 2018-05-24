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

  arr_struct.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


struct arr_struct1 {
  char  a[4][4];

 
};

inline
bool
operator == ( const arr_struct1&, const arr_struct1& )
{
    // NOT IMPLEMENTED
    return false;
}

inline
void
sc_trace( sc_trace_file*, const arr_struct1&, const std::string& )
{
    // NOT IMPLEMENTED
}


struct arr_struct2 {

  sc_uint<8> b[4][4];


};

inline
bool
operator == ( const arr_struct2&, const arr_struct2& )
{
    // NOT IMPLEMENTED
    return false;
}

inline
void
sc_trace( sc_trace_file*, const arr_struct2&, const std::string& )
{
    // NOT IMPLEMENTED
}
