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

  sc_attribute.cpp -- Attribute classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

 CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/kernel/sc_attribute.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_attr_base
//
//  Attribute base class.
// ----------------------------------------------------------------------------

// constructors

sc_attr_base::sc_attr_base( const std::string& name_ )
: m_name( name_ )
{}

sc_attr_base::sc_attr_base( const sc_attr_base& a )
: m_name( a.m_name )
{}


// destructor (does nothing)

sc_attr_base::~sc_attr_base()
{}


// get the name
const std::string&
sc_attr_base::name() const
{
    return m_name;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_attr_cltn
//
//  Attribute collection class. Stores pointers to attributes.
//  Note: iterate over the collection by using iterators.
// ----------------------------------------------------------------------------

// constructors

sc_attr_cltn::sc_attr_cltn() : m_cltn()
{}

sc_attr_cltn::sc_attr_cltn( const sc_attr_cltn& a )
: m_cltn( a.m_cltn )
{}


// destructor
sc_attr_cltn::~sc_attr_cltn()
{
    remove_all();
}


// add attribute to the collection.
// returns 'true' if the name of the attribute is unique,
// returns 'false' otherwise (attribute is not added).

bool
sc_attr_cltn::push_back( sc_attr_base* attribute_ )
{
    if( attribute_ == 0 ) {
	return false;
    }
    for( int i = m_cltn.size() - 1; i >= 0; -- i ) {
	if( attribute_->name() == m_cltn[i]->name() ) {
	    return false;
	}
    }
    m_cltn.push_back( attribute_ );
    return true;
}


// get attribute by name.
// returns pointer to attribute, or 0 if name does not exist.

sc_attr_base*
sc_attr_cltn::operator [] ( const std::string& name_ )
{
    for( int i = m_cltn.size() - 1; i >= 0; -- i ) {
	if( name_ == m_cltn[i]->name() ) {
	    return m_cltn[i];
	}
    }
    return 0;
}

const sc_attr_base*
sc_attr_cltn::operator [] ( const std::string& name_ ) const
{
    for( int i = m_cltn.size() - 1; i >= 0; -- i ) {
	if( name_ == m_cltn[i]->name() ) {
	    return m_cltn[i];
	}
    }
    return 0;
}


// remove attribute by name.
// returns pointer to attribute, or 0 if name does not exist.

sc_attr_base*
sc_attr_cltn::remove( const std::string& name_ )
{
    for( int i = m_cltn.size() - 1; i >= 0; -- i ) {
	if( name_ == m_cltn[i]->name() ) {
	    sc_attr_base* attribute = m_cltn[i];
	    std::swap( m_cltn[i], m_cltn.back() );
	    m_cltn.pop_back();
	    return attribute;
	}
    }
    return 0;
}


// remove all attributes

void
sc_attr_cltn::remove_all()
{
    m_cltn.clear();
}

} // namespace sc_core

// $Log: sc_attribute.cpp,v $
// Revision 1.7  2011/08/26 20:46:08  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.6  2011/08/24 22:05:50  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.5  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.4  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.3  2010/07/22 20:02:33  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.2  2008/05/22 17:06:24  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/01/16 22:27:08  acg
// Test of $Log comment.
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.

// Taf!
