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

  new_struct.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef test_struct
#define test_struct

struct a_new_struct {
  char              in_value1;     
  char              in_value2;     
  sc_logic              in_valid;    

  inline bool operator == (const a_new_struct & rhs) const
  {
    return (rhs.in_value1 == in_value1 && rhs.in_value2 == in_value2
	    && rhs.in_valid == in_valid);
}
};

inline
void
sc_trace( sc_trace_file*, const a_new_struct&, const std::string& )
{
    // NOT IMPLEMENTED
}


struct b_new_struct {
  sc_lv<9>                    out_value1;    
  int                    out_value2;    

inline bool operator == (const b_new_struct & rhs) const
  {
    return (rhs.out_value1 == out_value1 && rhs.out_value2 == out_value2);
}
};

inline
void
sc_trace( sc_trace_file*, const b_new_struct&, const std::string& )
{
    // NOT IMPLEMENTED
}


#endif
