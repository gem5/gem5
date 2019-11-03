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

  sc_name_gen.cpp -- Unique name generator.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_name_gen.h"
#include "sysc/utils/sc_iostream.h"

#if defined(_MSC_VER) && _MSC_VER >= 1310
// "C4351: new behavior: elements of array will be default initialized"
#pragma warning(disable: 4351)
#endif

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_name_gen
//
//  Unique name generator class.
// ----------------------------------------------------------------------------

sc_name_gen::sc_name_gen() : m_unique_name_map(), m_unique_name()
{}

sc_name_gen::~sc_name_gen()
{
    sc_strhash<int*>::iterator it( m_unique_name_map );
    for( ; ! it.empty(); it ++ ) {
	delete it.contents();
    }
    m_unique_name_map.erase();
}


// to generate unique names for objects in an MT-Safe way

const char*
sc_name_gen::gen_unique_name( const char* basename_, bool preserve_first )
{
    if( basename_ == 0 ) {
	SC_REPORT_ERROR( SC_ID_GEN_UNIQUE_NAME_, 0 );
    }
    int* c = m_unique_name_map[basename_];
    if( c == 0 ) {
	c = new int( 0 );
	m_unique_name_map.insert( CCAST<char*>( basename_ ), c );
	if (preserve_first) {
	    std::sprintf( m_unique_name, "%s", basename_ );
	} else {    
            std::sprintf( m_unique_name, "%s_%d", basename_, *c );
        }
    } else {
        std::sprintf( m_unique_name, "%s_%d", basename_, ++ (*c) );
    }
    return m_unique_name;
}

} // namespace sc_core

// $Log: sc_name_gen.cpp,v $
// Revision 1.6  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.5  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
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
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//

// Taf!
