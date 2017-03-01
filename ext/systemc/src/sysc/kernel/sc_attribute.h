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

  sc_attribute.h -- Attribute classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_ATTRIBUTE_H
#define SC_ATTRIBUTE_H

#include <string>
#include <vector>

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_attr_base
//
//  Attribute base class.
// ----------------------------------------------------------------------------

class sc_attr_base
{
public:

    // constructors
    sc_attr_base( const std::string& name_ );
    sc_attr_base( const sc_attr_base& );

    // destructor (does nothing)
    virtual ~sc_attr_base();

    // get the name
    const std::string& name() const;

private:

    std::string m_name;

private:

    // disabled
    sc_attr_base();
    sc_attr_base& operator = ( const sc_attr_base& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_attr_cltn
//
//  Attribute collection class. Stores pointers to attributes.
//  Note: iterate over the collection by using iterators.
// ----------------------------------------------------------------------------

class sc_attr_cltn
{
public:

    // typedefs
    typedef sc_attr_base*                          elem_type;
    typedef std::vector<elem_type>::iterator       iterator;
    typedef std::vector<elem_type>::const_iterator const_iterator;

    // constructors
    sc_attr_cltn();
    sc_attr_cltn( const sc_attr_cltn& );

    // destructor
    ~sc_attr_cltn();

    // add attribute to the collection.
    // returns 'true' if the name of the attribute is unique,
    // returns 'false' otherwise (attribute is not added).
    bool push_back( sc_attr_base* );

    // get attribute by name.
    // returns pointer to attribute, or 0 if name does not exist.
          sc_attr_base* operator [] ( const std::string& name_ );
    const sc_attr_base* operator [] ( const std::string& name_ ) const;

    // remove attribute by name.
    // returns pointer to attribute, or 0 if name does not exist.
    sc_attr_base* remove( const std::string& name_ );

    // remove all attributes
    void remove_all();

    // get the size of the collection
    int size() const
        { return m_cltn.size(); }

    // get the begin iterator
    iterator begin()
        { return m_cltn.begin(); }
    const_iterator begin() const
        { return m_cltn.begin(); }

    // get the end iterator
    iterator end()
        { return m_cltn.end(); }
    const_iterator end() const
        { return m_cltn.end(); }

private:
    std::vector<sc_attr_base*> m_cltn;

private:

    // disabled
    sc_attr_cltn& operator = ( const sc_attr_cltn& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_attribute<T>
//
//  Attribute class.
//  Note: T must have a default constructor and copy constructor.
// ----------------------------------------------------------------------------

template <class T>
class sc_attribute
: public sc_attr_base
{
public:

    // constructors

    sc_attribute( const std::string& name_ )
        : sc_attr_base( name_ ), value()
        {}

    sc_attribute( const std::string& name_, const T& value_ )
        : sc_attr_base( name_ ), value( value_ )
        {}

    sc_attribute( const sc_attribute<T>& a )
        : sc_attr_base( a.name() ), value( a.value )
        {}


    // destructor (does nothing)

    virtual ~sc_attribute()
        {}

public:

    // public data member; for easy access
    T value;

private:

    // disabled
    sc_attribute();
    sc_attribute<T>& operator = ( const sc_attribute<T>& );
};

} // namespace sc_core

// $Log: sc_attribute.h,v $
// Revision 1.6  2011/08/26 20:46:08  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
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
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//

#endif

// Taf!
