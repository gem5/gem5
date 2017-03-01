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

  sc_module_name.cpp -- An object used to help manage object names 
                        and hierarchy

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#include <cstdlib>

#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_module.h"
#include "sysc/kernel/sc_module_name.h"
#include "sysc/kernel/sc_object_manager.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/utils/sc_iostream.h"

namespace sc_core {

sc_module_name::sc_module_name( const char* name_ )
: m_name( name_ ),
  m_module_p( 0 ),
  m_next( 0 ),
  m_simc( sc_get_curr_simcontext() ),
  m_pushed( true )
{
    m_simc->get_object_manager()->push_module_name( this );
}

sc_module_name::sc_module_name( const sc_module_name& name_ )
: m_name( name_.m_name ),
  m_module_p( 0 ),
  m_next( 0 ),
  m_simc( name_.m_simc ),
  m_pushed( false )
{}

sc_module_name::~sc_module_name()
{
    if( m_pushed ) {
        sc_module_name* smn = m_simc->get_object_manager()->pop_module_name();
        if( this != smn ) {
            SC_REPORT_ERROR( SC_ID_SC_MODULE_NAME_USE_, 0 );
        }
	if ( m_module_p ) m_module_p->end_module();
    }
}

sc_module_name::operator const char*() const
{
    return m_name;
}

} // namespace sc_core

// $Log: sc_module_name.cpp,v $
// Revision 1.5  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.3  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/03/14 23:56:58  acg
//   Andy Goodrich: This fixes a bug when an exception is thrown in
//   sc_module::sc_module() for a dynamically allocated sc_module
//   object. We are calling sc_module::end_module() on a module that has
//   already been deleted. The scenario runs like this:
//
//   a) the sc_module constructor is entered
//   b) the exception is thrown
//   c) the exception processor deletes the storage for the sc_module
//   d) the stack is unrolled causing the sc_module_name instance to be deleted
//   e) ~sc_module_name() calls end_module() with its pointer to the sc_module
//   f) because the sc_module has been deleted its storage is corrupted,
//      either by linking it to a free space chain, or by reuse of some sort
//   g) the m_simc field is garbage
//   h) the m_object_manager field is also garbage
//   i) an exception occurs
//
//   This does not happen for automatic sc_module instances since the
//   storage for the module is not reclaimed its just part of the stack.
//
//   I am fixing this by having the destructor for sc_module clear the
//   module pointer in its sc_module_name instance. That cuts things at
//   step (e) above, since the pointer will be null if the module has
//   already been deleted. To make sure the module stack is okay, I call
//   end-module() in ~sc_module in the case where there is an
//   sc_module_name pointer lying around.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//
