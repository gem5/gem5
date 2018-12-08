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

#include "tlm_core/tlm_2/tlm_generic_payload/tlm_phase.h"
#include "sysc/utils/sc_string_view.h"
#include "sysc/utils/sc_typeindex.h"
#include "sysc/utils/sc_report.h"
#include <cstring>
#include <map>

using sc_core::sc_string_view;
using sc_core::sc_type_index;

namespace tlm {
/* anonymous */ namespace {

struct tlm_phase_registry
{
  typedef unsigned int key_type;

  static tlm_phase_registry& instance()
    { static tlm_phase_registry inst; return inst; }

  unsigned int register_phase(sc_type_index type, sc_string_view name)
  {
    type_map::const_iterator it = ids_.find( type );

    if( name.empty() ) {
      SC_REPORT_FATAL( sc_core::SC_ID_INTERNAL_ERROR_,
                       "unexpected empty tlm_phase name" );
      return UNINITIALIZED_PHASE;
    }

    if( it == ids_.end() ) { // new phase - generate/store ID and name
      type_map::value_type v( type, static_cast<key_type>(names_.size()) );
      names_.push_back( name_table::value_type(name.data(), name.size()) );
      ids_.insert( v );
      return v.second;
    }

    if( names_[it->second] != name ) {
      SC_REPORT_FATAL( sc_core::SC_ID_INTERNAL_ERROR_,
                       "tlm_phase registration failed: duplicate type info" );
      sc_core::sc_abort();
    }
    return it->second;
  }

  const char* get_name( key_type id ) const
  {
    sc_assert( id < names_.size() );
    return names_[id].c_str();
  }

private:
  typedef std::map<sc_type_index, key_type> type_map;
  typedef std::vector<std::string> name_table;

  type_map   ids_;
  name_table names_;

  tlm_phase_registry()
    : names_( END_RESP+1 )
  {
#   define BUILTIN_PHASE(phase) \
      names_[phase] = #phase

    BUILTIN_PHASE( UNINITIALIZED_PHASE );
    BUILTIN_PHASE( BEGIN_REQ );
    BUILTIN_PHASE( END_REQ );
    BUILTIN_PHASE( BEGIN_RESP );
    BUILTIN_PHASE( END_RESP );
  }

}; // class tlm_phase_registry

} // anonymous namespace

tlm_phase::tlm_phase( unsigned int id )
  : m_id(id)
{
  // TODO: validate id?
  // TODO: add deprecation warning?
}

tlm_phase::tlm_phase( const std::type_info& type, const char* name )
  : m_id( tlm_phase_registry::instance().register_phase(type, name) )
{}

const char*
tlm_phase::get_name() const
{
  return tlm_phase_registry::instance().get_name( m_id );
}

} // namespace tlm
// Taf!
