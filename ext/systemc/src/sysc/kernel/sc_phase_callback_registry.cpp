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

  sc_phase_callback_registry.cpp -- Implementation of phase callback registry

  Original Author: Philipp A. Hartmann, OFFIS, 2013-02-15

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#include "sysc/kernel/sc_object.h"
#include "sysc/kernel/sc_phase_callback_registry.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/utils/sc_report.h"

#include <algorithm>
#include <functional>

namespace sc_core {

#if SC_HAS_PHASE_CALLBACKS_

sc_phase_callback_registry::sc_phase_callback_registry( sc_simcontext& simc )
  : m_simc( &simc )
#if 0
  , m_cb_eval_vec()
#endif
  , m_cb_update_vec()
  , m_cb_timestep_vec()
{}

sc_phase_callback_registry::~sc_phase_callback_registry()
{}

static const sc_phase_callback_registry::mask_type
  SC_PHASE_CALLBACK_MASK = SC_STATUS_ANY;

namespace /* anonymous */ {

struct entry_match
  : std::unary_function< sc_phase_callback_registry::entry, bool >
{
    typedef sc_phase_callback_registry::cb_type* ref_type;

    explicit
    entry_match( ref_type ref )
      : ref_(ref)
    {}

    result_type operator()( argument_type e )
        { return e.target == ref_; }
private:
    sc_phase_callback_registry::cb_type * ref_;

}; // entry_match

template< typename T >
inline void
erase_remove( std::vector<T>* vec, T const& t )
{
    vec->erase( std::remove( vec->begin(), vec->end(), t ) );
}

}  // namespace anonymous


sc_phase_callback_registry::mask_type
sc_phase_callback_registry::validate_mask( cb_type& cb
                                         , mask_type m
                                         , bool warn = false )
{
    if( SC_UNLIKELY_(m & ~SC_PHASE_CALLBACK_MASK) )
    {
        if( warn )
        {
            std::stringstream ss;
            ss << cb.name() << ": invalid phase callback mask: "
               << (sc_status)m;
            SC_REPORT_WARNING( SC_ID_PHASE_CALLBACK_REGISTER_
                             , ss.str().c_str() );
        }
        m &= SC_PHASE_CALLBACK_MASK;
    }

    mask_type check_mask;

    // elaboration callbacks
    check_mask = ( SC_ELABORATION
                 | SC_BEFORE_END_OF_ELABORATION
                 | SC_END_OF_ELABORATION );
    if( SC_UNLIKELY_( (m & check_mask ) && m_simc->elaboration_done() ) )
    {
        if( warn )
        {
            std::stringstream ss;
            ss << cb.name() << ": elaboration done\n\t "
               << (sc_status)( m & check_mask ) << " callback(s) ignored";
            SC_REPORT_WARNING(SC_ID_PHASE_CALLBACK_REGISTER_
                             , ss.str().c_str() );
        }
        m &= ~check_mask;
    }

    check_mask = (SC_BEFORE_END_OF_ELABORATION | SC_END_OF_ELABORATION);
    if( SC_UNLIKELY_(m & SC_ELABORATION) )
    {
        if( warn )
        {
            std::stringstream ss;
            ss << cb.name() << ": " << SC_ELABORATION
               << ":\n\t substituted by " << (sc_status)(check_mask);
            SC_REPORT_WARNING( SC_ID_PHASE_CALLBACK_REGISTER_
                             , ss.str().c_str() );
        }
        m &= ~SC_ELABORATION;
        m |= check_mask;
    }

    check_mask = ( SC_END_OF_INITIALIZATION
#if 0
                 | SC_END_OF_EVALUATION
#endif
                 | SC_END_OF_UPDATE
                 | SC_BEFORE_TIMESTEP );
    if( SC_UNLIKELY_(m & SC_RUNNING) )
    {
        if( warn )
        {
            std::stringstream ss;
            ss << cb.name() << ": " << SC_RUNNING
               << ":\n\t substituted by " << (sc_status)(check_mask);
            SC_REPORT_WARNING( SC_ID_PHASE_CALLBACK_REGISTER_
                             , ss.str().c_str() );
        }
        m &= ~SC_RUNNING;
        m |= check_mask;
    }
    return m;
}


sc_phase_callback_registry::mask_type
sc_phase_callback_registry::register_callback( cb_type& cb, mask_type m )
{
    storage_type::iterator it =
      find_if( m_cb_vec.begin(), m_cb_vec.end(), entry_match(&cb) );

    m = validate_mask(cb, m, /* warn */ true );

    mask_type diff_mask = m;
    mask_type new_mask  = m;

    if( it != m_cb_vec.end() ) // update existing entry
    {
        // update masks
        new_mask   =  (*it).mask | m;
        diff_mask  = ~(*it).mask & m;
        (*it).mask = new_mask;
    }
    else // new entry
    {
        if( !m ) // empty, do nothing
            return SC_UNITIALIZED;

        entry new_entry = { &cb, new_mask };
        m_cb_vec.push_back( new_entry );
    }

    // add to callback shortcut sets
#if 0
    if( diff_mask & SC_END_OF_EVALUATION )
        m_cb_eval_vec.push_back( &cb );
#endif
    if( diff_mask & SC_END_OF_UPDATE )
        m_cb_update_vec.push_back( &cb );
    if( diff_mask & SC_BEFORE_TIMESTEP )
        m_cb_timestep_vec.push_back( &cb );

    return new_mask;
}


sc_phase_callback_registry::mask_type
sc_phase_callback_registry::unregister_callback( cb_type& cb, mask_type m )
{
    storage_type::iterator it =
      find_if( m_cb_vec.begin(), m_cb_vec.end(), entry_match(&cb) );

    m = validate_mask(cb, m);

    mask_type diff_mask = m;
    mask_type new_mask  = m;

    if( it == m_cb_vec.end() ) { // not registered
        return SC_UNITIALIZED;
    }

    // update masks
    new_mask   = (*it).mask & ~m;
    diff_mask  = (*it).mask & m;
    (*it).mask = new_mask;

    if( !new_mask )
        m_cb_vec.erase(it);

    // drop from callback shortcut sets
#if 0
    if( diff_mask & SC_END_OF_EVALUATION )
        erase_remove( &m_cb_eval_vec, &cb );
#endif
    if( diff_mask & SC_END_OF_UPDATE )
        erase_remove( &m_cb_update_vec, &cb );
    if( diff_mask & SC_BEFORE_TIMESTEP )
        erase_remove( &m_cb_timestep_vec, &cb );

    return new_mask;
}


// generic implementation (for non-critical callbacks)
//  - also restores hierarchy around callback object
void
sc_phase_callback_registry::do_callback( sc_status s ) const
{
    typedef storage_type::const_iterator it_type;
    storage_type const & vec = m_cb_vec;

    for(it_type it = vec.begin(), end = vec.end(); it != end; ++it) {
        if( s & it->mask ) {
            sc_object::hierarchy_scope scope(it->target);
            it->target->do_simulation_phase_callback();
        }
    }
}

#else // ! SC_HAS_PHASE_CALLBACKS_

sc_phase_callback_registry::sc_phase_callback_registry( sc_simcontext& ){}
sc_phase_callback_registry::~sc_phase_callback_registry(){}

static inline void
warn_phase_callbacks( sc_core::sc_object const& obj )
{
    static bool warned = false;
    if (!warned)
    {
        std::stringstream ss;
        ss << obj.name() << ".\n"
           << "Please recompile SystemC with "
              "\"SC_ENABLE_SIMULATION_PHASE_CALLBACKS\" defined.";
        SC_REPORT_WARNING( SC_ID_PHASE_CALLBACKS_UNSUPPORTED_
                         , ss.str().c_str() );
    }
}

sc_phase_callback_registry::mask_type
sc_phase_callback_registry::register_callback( cb_type& cb, mask_type )
{
    warn_phase_callbacks( cb );
    return SC_UNITIALIZED;
}

sc_phase_callback_registry::mask_type
sc_phase_callback_registry::unregister_callback( cb_type& cb, mask_type )
{
    warn_phase_callbacks( cb );
    return SC_UNITIALIZED;
}

#endif // ! SC_HAS_PHASE_CALLBACKS_

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// Taf!
