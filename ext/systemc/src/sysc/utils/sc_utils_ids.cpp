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

  sc_utils_ids.cpp -- Report ids for the utils code.

  Original Author: Martin Janssen, Synopsys, Inc., 2002-01-17

  CHANGE LOG AT END OF FILE
 *****************************************************************************/


#include <cstdlib>
#include <cstring>

#include "sysc/utils/sc_report.h"


namespace sc_core {
#define SC_DEFINE_MESSAGE(id,unused,text) extern const char id[] = text;
#include "sysc/utils/sc_utils_ids.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/communication/sc_communication_ids.h"
#include "sysc/datatypes/bit/sc_bit_ids.h"
#include "sysc/datatypes/fx/sc_fx_ids.h"
#include "sysc/datatypes/int/sc_int_ids.h"
#include "sysc/tracing/sc_tracing_ids.h"
#undef SC_DEFINE_MESSAGE


static sc_msg_def texts[] = {
#define SC_DEFINE_MESSAGE(id,n,unused) \
    { (id), 0u, {0u}, 0u, {0u}, 0u, 0u, {0u}, 0, n },

#undef SC_UTILS_IDS_H
#include "sysc/utils/sc_utils_ids.h"

#undef SC_KERNEL_IDS_H
#include "sysc/kernel/sc_kernel_ids.h"

#undef SC_COMMUNICATION_IDS_H
#include "sysc/communication/sc_communication_ids.h"

#undef SC_BIT_IDS_H
#include "sysc/datatypes/bit/sc_bit_ids.h"

#undef SC_FX_IDS_H
#include "sysc/datatypes/fx/sc_fx_ids.h"

#undef SC_INT_IDS_H
#include "sysc/datatypes/int/sc_int_ids.h"

#undef SC_TRACING_IDS_H
#include "sysc/tracing/sc_tracing_ids.h"

#undef SC_DEFINE_MESSAGE
};
static sc_report_handler::msg_def_items items = {
    texts, sizeof(texts)/sizeof(*texts), false, 0
};

static
int initialize()
{
    sc_report_handler::add_static_msg_types(&items);

    // PROCESS ANY ENVIRONMENTAL OVERRIDES:

    const char* deprecation_warn = std::getenv("SC_DEPRECATION_WARNINGS");
    if ( (deprecation_warn!=0) && !std::strcmp(deprecation_warn,"DISABLE") )
    {
        sc_report_handler::set_actions( SC_ID_IEEE_1666_DEPRECATION_
                                      , SC_DO_NOTHING);
    }
    return 42;
}

static int forty_two = initialize();

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

  Alex Riesen, Synopsys, Inc., 2003-02-02
  ported to SystemC 2.1 exception reporting.
    
 *****************************************************************************/


// $Log: sc_utils_ids.cpp,v $
// Revision 1.5  2011/08/26 20:46:19  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.3  2009/02/28 00:27:57  acg
//  Andy Goodrich: includes for C++ library types to keep compiler happy.
//
// Revision 1.2  2008/05/20 20:43:21  acg
//  Andy Goodrich: Added includes <cstdlib> and <cstring> to pick up their
//  declarations.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.6  2006/01/31 21:42:07  acg
//  Andy Goodrich: Added checks for SC_DEPRECATED_WARNINGS being defined as
//  DISABLED. If so, we turn off the /IEEE_Std_1666/deprecated message group.
//
// Revision 1.5  2006/01/24 21:59:59  acg
//  Andy Goodrich: removed sc_trace_ids.h since its only message has been
//  replaced by SC_ID_IEEE_1666_DEPRECATION_ message.
//
// Revision 1.4  2006/01/24 20:53:41  acg
// Andy Goodrich: added warnings indicating that use of integer ids in reports
// is deprecated. Added tracing/sc_trace_ids.h to message list.
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// Taf!
