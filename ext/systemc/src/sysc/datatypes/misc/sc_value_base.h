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

  sc_value_base.h -- Base class for SystemC bit values.

  Original Author: Andy Goodrich, Forte Design Systems

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_value_base.h,v $
// Revision 1.4  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.3  2011/08/24 22:05:48  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.2  2011/06/28 21:23:04  acg
//  Andy Goodrich: merging of SCV tree.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:54:01  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SC_VALUE_BASE_H
#define SC_VALUE_BASE_H


#include "sysc/datatypes/int/sc_nbdefs.h"

namespace sc_dt
{

class sc_signed;
class sc_unsigned;

// ----------------------------------------------------------------------------
//  CLASS : sc_value_base
//
//  Abstract base class of all SystemC native variables. It provides
//  support for concatenation operations via a set of virtual methods. 
//  A general description of the methods appear with their default 
//  definitions in sc_object.cpp.
// ----------------------------------------------------------------------------

class sc_value_base 
{
    friend class sc_concatref;
  private: 
    virtual void concat_clear_data( bool to_ones=false );
    virtual bool concat_get_ctrl( sc_digit* dst_p, int low_i ) const;
    virtual bool concat_get_data( sc_digit* dst_p, int low_i ) const;
    virtual uint64 concat_get_uint64() const;
    virtual int concat_length(bool* xz_present_p=0) const;
    virtual void concat_set( int64 src, int low_i );
    virtual void concat_set( const sc_signed& src, int low_i );
    virtual void concat_set( const sc_unsigned& src, int low_i );
    virtual void concat_set( uint64 src, int low_i );
  public:
    virtual ~sc_value_base() {}
};


// ----------------------------------------------------------------------------
//  CLASS : sc_generic_base
//
//  Proxy class for user-defined value classes and other classes that 
//  are defined outside of SystemC. 
//  The class is utilized as a base class for the arbitrary class:
//
//       class my_class : public sc_generic_base<my_class>
//
//  The purpose of the class is to allow to_XXXX methods defined within that 
//  class so that assignments and casts from the arbitrary class to native 
//  SystemC types are possible. To interact correctly with the SystemC library 
//  the class derived from sc_generic_base must implement the following 
//  methods: 
//    (1) uint64 to_uint64() const
//    (2) int64  to_int64() const
//    (3) void to_sc_unsigned( sc_unsigned& ) const
//    (4) void to_sc_signed( sc_signed& ) const
// ----------------------------------------------------------------------------
template< class T >
class sc_generic_base {
  public: 
    inline const T* operator-> () const
    {
        return (const T*)this;
    }
    inline T* operator-> () 
    {
        return (T*)this;
    }
};

} // namespace sc_dt

#endif
