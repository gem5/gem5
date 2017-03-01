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

  sc_vector.cpp - A vector of named (SystemC) objects (modules, ports, channels)

  Original Author: Philipp A. Hartmann, OFFIS

  CHANGE LOG AT END OF FILE
 *****************************************************************************/


#include "sc_vector.h"

#include "sysc/utils/sc_hash.h"
#include "sysc/utils/sc_list.h"
#include "sysc/utils/sc_utils_ids.h"

#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_object_manager.h"

#include <sstream>

namespace sc_core {

sc_vector_base::sc_vector_base()
  : sc_object( sc_gen_unique_name("vector") )
  , vec_()
  , objs_vec_()
{}

std::vector< sc_object* > const &
sc_vector_base::get_elements() const
{
  if( !objs_vec_ )
    objs_vec_ = new std::vector< sc_object* >;

  if( objs_vec_->size() || !size() )
    return *objs_vec_;

  objs_vec_->reserve( size() );
  for( const_iterator it=begin(); it != end(); ++it )
    if( sc_object* obj = object_cast(*it) )
      objs_vec_->push_back( obj );

  return *objs_vec_;
}

sc_object*
sc_vector_base::implicit_cast( ... ) const
{
  SC_REPORT_ERROR( SC_ID_VECTOR_NONOBJECT_ELEMENTS_, name() );
  return NULL;
}

void
sc_vector_base::check_index( size_type i ) const
{
  if( i>=size() )
  {
    std::stringstream str;
    str << name()
        << "[" << i << "] >= size() = " << size();
    SC_REPORT_ERROR( SC_ID_OUT_OF_BOUNDS_, str.str().c_str() );
  }
}

bool
sc_vector_base::check_init( size_type n ) const
{
  if ( !n )
    return false;

  if( size() ) // already filled
  {
    std::stringstream str;
    str << name()
        << ", size=" << size()
        << ", requested size=" << n;
    SC_REPORT_ERROR( SC_ID_VECTOR_INIT_CALLED_TWICE_
                   , str.str().c_str() );
    return false;
  }

  sc_simcontext* simc = simcontext();
  sc_assert( simc == sc_get_curr_simcontext() );

  sc_object* parent_p = simc->active_object();
  if( parent_p != get_parent_object() )
  {
    std::stringstream str;
    str << name() << ": expected "
        << ( get_parent_object()
              ? get_parent_object()->name() : "<top-level>" )
        << ", got "
        << ( parent_p ? parent_p->name() : "<top-level>" );

    SC_REPORT_ERROR( SC_ID_VECTOR_INIT_INVALID_CONTEXT_
                   , str.str().c_str() );
    return false;
  }

  return true;
}

void
sc_vector_base::report_empty_bind( const char* kind_, bool dst_empty_ ) const
{
  std::stringstream str;

  str << "target `" << name() << "' "
      << "(" << kind_ << ") ";

  if( !size() ) {
    str << "not initialised yet";
  } else if ( dst_empty_ ) {
    str << "empty range given";
  } else {
    str << "empty destination range given";
  }

  SC_REPORT_WARNING( SC_ID_VECTOR_BIND_EMPTY_, str.str().c_str() );
}

std::string
sc_vector_base::make_name( const char* prefix, size_type /* idx */ )
{
  // TODO: How to handle name clashes due to interleaved vector
  //       creation and init()?
  //       sc_vector< foo > v1, v2;
  //       v1.name() == "vector", v2.name() == "vector_0"
  //       v1.init( 1 ); -> v1[0].name() == "vector_0" -> clash
  return sc_gen_unique_name( prefix );
}

} // namespace sc_core

// $Log: sc_vector.cpp,v $
// Revision 1.6  2011/08/26 20:46:20  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.5  2011/04/01 22:35:19  acg
//  Andy Goodrich: spelling fix.
//
// Revision 1.4  2011/03/28 13:03:09  acg
//  Andy Goodrich: Philipp's latest update.
//
// Revision 1.3  2011/03/23 16:16:28  acg
//  Philipp A. Hartmann: rebase implementation on void*
//      - supports virtual inheritance from sc_object again
//      - build up get_elements result on demand
//      - still requires element type to be derived from sc_object
//
// Revision 1.2  2011/02/14 17:54:25  acg
//  Andy Goodrich: Philipp's addition of early bind checks.
//
// Revision 1.1  2011/02/13 21:54:14  acg
//  Andy Goodrich: turn on embedding of cvs log records.

// Taf!
