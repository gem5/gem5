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

  sc_utils_ids.h -- Report ids for the utils code.

  Original Author: Martin Janssen, Synopsys, Inc., 2002-01-17

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef SC_UTILS_IDS_H
#define SC_UTILS_IDS_H

// ----------------------------------------------------------------------------
//  Report ids (utils)
//
//  Report ids in the range of 800-899.
// ----------------------------------------------------------------------------

#ifndef SC_DEFINE_MESSAGE
#define SC_DEFINE_MESSAGE(id,unused1,unused2) \
    namespace sc_core { extern const char id[]; }
namespace sc_core {
    extern const char SC_ID_REGISTER_ID_FAILED_[]; // in sc_report_handler.cpp
}
#endif

SC_DEFINE_MESSAGE(SC_ID_STRING_TOO_LONG_,
		  801, "string is too long")
SC_DEFINE_MESSAGE(SC_ID_FRONT_ON_EMPTY_LIST_,
		  802, "attempt to take front() on an empty list")
SC_DEFINE_MESSAGE(SC_ID_BACK_ON_EMPTY_LIST_,
		  803, "attempt to take back() on an empty list")
SC_DEFINE_MESSAGE(SC_ID_IEEE_1666_DEPRECATION_,
		  804, "/IEEE_Std_1666/deprecated" )
SC_DEFINE_MESSAGE(SC_ID_VECTOR_INIT_CALLED_TWICE_,
                  805, "sc_vector::init has already been called" )
SC_DEFINE_MESSAGE(SC_ID_VECTOR_INIT_INVALID_CONTEXT_,
                  806, "sc_vector::init called from invalid object context" )
SC_DEFINE_MESSAGE(SC_ID_VECTOR_BIND_EMPTY_,
                  807, "sc_vector::bind called with empty range" )
SC_DEFINE_MESSAGE(SC_ID_VECTOR_NONOBJECT_ELEMENTS_,
                  808, "sc_vector::get_elements called for element type "
                       "not derived from sc_object" )

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:
    
  Alex Riesen, Synopsys, Inc., 2003-02-02
  ported to SystemC 2.1 exception reporting.
    
 *****************************************************************************/

// $Log: sc_utils_ids.h,v $
// Revision 1.5  2011/08/26 20:46:20  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.3  2011/02/14 17:54:25  acg
//  Andy Goodrich: Philipp's addition of early bind checks.
//
// Revision 1.2  2010/12/07 20:10:19  acg
// Andy Goodrich: messages for new sc_vector class.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.6  2006/01/25 00:31:27  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.5  2006/01/24 22:01:35  acg
//  Andy Goodrich: consolidated all IEEE 1666 compliance messages to use the
//  SC_ID_IEEE_1666_DEPRECATION_ message type.
//
// Revision 1.4  2006/01/24 20:53:41  acg
// Andy Goodrich: added warnings indicating that use of integer ids in reports
// is deprecated. Added tracing/sc_trace_ids.h to message list.
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.
//

#endif

// Taf!
