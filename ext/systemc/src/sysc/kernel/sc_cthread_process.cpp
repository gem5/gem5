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

  sc_cthread_process.cpp -- Clocked thread implementation.

  Original Author: Andy Goodrich, Forte Design Systems, 4 August 2005
               

 CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/kernel/sc_cthread_process.h"
#include "sysc/kernel/sc_simcontext_int.h"

namespace sc_core {

//------------------------------------------------------------------------------
//"sc_cthread_process::dont_initialize"
//
// This virtual method sets the initialization switch for this object instance.
//------------------------------------------------------------------------------
void sc_cthread_process::dont_initialize( bool /* dont */ )
{
    SC_REPORT_WARNING( SC_ID_DONT_INITIALIZE_, 0 );
}

//------------------------------------------------------------------------------
//"sc_cthread_process::sc_cthread_process"
//
// This is the object instance constructor for this class.
//------------------------------------------------------------------------------
sc_cthread_process::sc_cthread_process( const char* name_p, 
    bool free_host, SC_ENTRY_FUNC method_p, 
    sc_process_host* host_p, const sc_spawn_options* opt_p 
):
    sc_thread_process(name_p, free_host, method_p, host_p, opt_p)
{
    m_dont_init = true;
    m_process_kind = SC_CTHREAD_PROC_;
}

//------------------------------------------------------------------------------
//"sc_cthread_process::~sc_cthread_process"
//
// This is the object instance constructor for this class.
//------------------------------------------------------------------------------
sc_cthread_process::~sc_cthread_process()
{
}

} // namespace sc_core 

// $Log: sc_cthread_process.cpp,v $
// Revision 1.11  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.10  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
//
// Revision 1.9  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.8  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.7  2011/02/11 13:25:24  acg
//  Andy Goodrich: Philipp A. Hartmann's changes:
//    (1) Removal of SC_CTHREAD method overloads.
//    (2) New exception processing code.
//
// Revision 1.6  2011/02/01 21:00:35  acg
//  Andy Goodrich: removed throw_reset as it is now handled by parent
//  sc_thread_process::throw_reset().
//
// Revision 1.5  2011/01/18 20:10:44  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.4  2009/07/28 01:10:53  acg
//  Andy Goodrich: updates for 2.3 release candidate.
//
// Revision 1.3  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.2  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.6  2006/04/20 17:08:16  acg
//  Andy Goodrich: 3.0 style process changes.
//
// Revision 1.5  2006/04/11 23:13:20  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.4  2006/01/24 20:49:04  acg
// Andy Goodrich: changes to remove the use of deprecated features within the
// simulator, and to issue warning messages when deprecated features are used.
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//
