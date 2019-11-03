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

  sc_bit_ids.h -- Report ids for the datatypes/bit code.

  Original Author: Martin Janssen, Synopsys, Inc., 2002-01-17

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:
    
 *****************************************************************************/

// $Log: sc_bit_ids.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.5  2006/01/25 00:31:15  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.4  2006/01/24 20:50:55  acg
// Andy Goodrich: added warnings indicating that sc_bit is deprecated and that
// the C bool data type should be used in its place.
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SC_BIT_IDS_H
#define SC_BIT_IDS_H


#include "sysc/utils/sc_report.h"


// ----------------------------------------------------------------------------
//  Report ids (datatypes/bit)
//
//  Report ids in the range of 200-299.
// ----------------------------------------------------------------------------

#ifndef SC_DEFINE_MESSAGE
#define SC_DEFINE_MESSAGE(id,unused1,unused2) \
    namespace sc_core { extern const char id[]; }
namespace sc_core {
    extern const char SC_ID_REGISTER_ID_FAILED_[]; // in sc_report_handler.cpp
}
#endif


SC_DEFINE_MESSAGE( SC_ID_LENGTH_MISMATCH_, 200,
		"length mismatch in bit/logic vector assignment" )
SC_DEFINE_MESSAGE( SC_ID_INCOMPATIBLE_TYPES_, 201,
		"incompatible types" )
SC_DEFINE_MESSAGE( SC_ID_CANNOT_CONVERT_, 202,
		"cannot perform conversion" )
SC_DEFINE_MESSAGE( SC_ID_INCOMPATIBLE_VECTORS_, 203,
		"incompatible vectors" )
SC_DEFINE_MESSAGE( SC_ID_VALUE_NOT_VALID_, 204,
		"value is not valid" )
SC_DEFINE_MESSAGE( SC_ID_ZERO_LENGTH_,     205,
		"zero length" )
SC_DEFINE_MESSAGE( SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, 206,
		"vector contains 4-value logic" )  
SC_DEFINE_MESSAGE( SC_ID_SC_BV_CANNOT_CONTAIN_X_AND_Z_, 207,
		"sc_bv cannot contain values X and Z" )
SC_DEFINE_MESSAGE( SC_ID_VECTOR_TOO_LONG_,  208,
		"vector is too long: truncated" )  
SC_DEFINE_MESSAGE( SC_ID_VECTOR_TOO_SHORT_, 209,
		"vector is too short: 0-padded" )  
SC_DEFINE_MESSAGE( SC_ID_WRONG_VALUE_, 210,
		"wrong value" )
SC_DEFINE_MESSAGE( SC_ID_LOGIC_Z_TO_BOOL_, 211,
		"sc_logic value 'Z' cannot be converted to bool" )
SC_DEFINE_MESSAGE( SC_ID_LOGIC_X_TO_BOOL_, 212,
		"sc_logic value 'X' cannot be converted to bool" )

#endif

// Taf!
