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

  sc_phase_callback_registry.h -- Definition of the simulation phase callbacks

  The most critical functions are defined inline in this file.  Only active,
  if SC_ENABLE_SIMULATION_PHASE_CALLBACKS[_TRACING] is defined during the
  SystemC library build.

  Original Author: Philipp A. Hartmann, OFFIS, 2013-02-15

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef SC_PHASE_CALLBACK_REGISTRY_H_INCLUDED_
#define SC_PHASE_CALLBACK_REGISTRY_H_INCLUDED_

#if defined( SC_ENABLE_SIMULATION_PHASE_CALLBACKS ) \
 || defined( SC_ENABLE_SIMULATION_PHASE_CALLBACKS_TRACING )
#  define SC_HAS_PHASE_CALLBACKS_ 1
#else
#  define SC_HAS_PHASE_CALLBACKS_ 0
#endif

#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_object_int.h"
#include "sysc/kernel/sc_status.h"

#include <vector>

namespace sc_core {

class sc_simcontext;
class sc_object;

class sc_phase_callback_registry
{
public:
    typedef sc_phase_callback_registry this_type;
    typedef sc_object                  cb_type;
    typedef cb_type::phase_cb_mask     mask_type;

    struct entry
    {
        cb_type*  target;
        mask_type mask;
    };

    friend class sc_simcontext;
    friend class sc_object;

private: // interface completely internal

    explicit
    sc_phase_callback_registry( sc_simcontext& simc );

    ~sc_phase_callback_registry();

    // --- callback forwarders

    bool construction_done()   const; //< returns false
    void elaboration_done()    const;
    void initialization_done() const;
    void start_simulation()    const;

    void evaluation_done()     const;
    void update_done()         const;
    void before_timestep()     const;

    void simulation_paused()   const;
    void simulation_stopped()  const;
    void simulation_done()     const;


    // --- callback registration and implementation

    mask_type register_callback( cb_type&, mask_type mask );
    mask_type unregister_callback( cb_type&, mask_type mask );

    // generic caller
    void do_callback( sc_status ) const;

private:
    typedef std::vector<entry>    storage_type;
    typedef std::vector<cb_type*> single_storage_type;

#if SC_HAS_PHASE_CALLBACKS_

    // set and restore simulation status
    struct scoped_status
    {
        scoped_status( sc_status& ref, sc_status s )
          : ref_(ref), prev_(ref) { ref_ = s;}
        ~scoped_status() { ref_ = prev_; }
    private:
        sc_status& ref_;
        sc_status  prev_;
    }; // scoped_status

    mask_type validate_mask( cb_type&, mask_type, bool warn );

private:

    sc_simcontext*        m_simc;
    storage_type          m_cb_vec;            // all callbacks
#if 0
    single_storage_type   m_cb_eval_vec;     //  - eval cb shortcut
#endif
    single_storage_type   m_cb_update_vec;   //  - update cb shortcut
    single_storage_type   m_cb_timestep_vec; //  - timestep cb shortcut

#endif // SC_HAS_PHASE_CALLBACKS_

private:
    // disabled
    sc_phase_callback_registry( const this_type& );
    this_type& operator=(const this_type&);

}; // sc_phase_callback_registry


// -------------------- callback implementations --------------------
//                   (empty, if feature is disabled)

inline bool
sc_phase_callback_registry::construction_done() const
{
#if SC_HAS_PHASE_CALLBACKS_
    do_callback( SC_BEFORE_END_OF_ELABORATION );
#endif
    return false;
}

inline void
sc_phase_callback_registry::elaboration_done() const
{
#if SC_HAS_PHASE_CALLBACKS_
    do_callback( SC_END_OF_ELABORATION );
#endif
}

inline void
sc_phase_callback_registry::start_simulation() const
{
#if SC_HAS_PHASE_CALLBACKS_
    do_callback( SC_START_OF_SIMULATION );
#endif
}

inline void
sc_phase_callback_registry::initialization_done() const
{
#if SC_HAS_PHASE_CALLBACKS_
    scoped_status scope( m_simc->m_simulation_status
                       , SC_END_OF_INITIALIZATION );

    do_callback( SC_END_OF_INITIALIZATION );
#endif
}

inline void
sc_phase_callback_registry::simulation_paused() const
{
#if SC_HAS_PHASE_CALLBACKS_
    do_callback( SC_PAUSED );
#endif
}

inline void
sc_phase_callback_registry::simulation_stopped() const
{
#if SC_HAS_PHASE_CALLBACKS_
    do_callback( SC_STOPPED );
#endif
}

inline void
sc_phase_callback_registry::simulation_done() const
{
#if SC_HAS_PHASE_CALLBACKS_
    do_callback( SC_END_OF_SIMULATION );
#endif
}

// -------------- specialized callback implementations --------------

#if 0
inline void
sc_phase_callback_registry::evaluation_done() const
{
#if SC_HAS_PHASE_CALLBACKS_

    if( !m_cb_eval_vec.size() ) return;

    typedef single_storage_type::const_iterator it_type;
    single_storage_type const & vec = m_cb_eval_vec;

    scoped_status scope( m_simc->m_simulation_status
                       , SC_END_OF_EVALUATION );

    for(it_type it = vec.begin(), end = vec.end(); it != end; ++it)
        (*it)->do_simulation_phase_callback();
#endif
}
#endif

inline void
sc_phase_callback_registry::update_done() const
{
#if SC_HAS_PHASE_CALLBACKS_

    if( !m_cb_update_vec.size() ) return;

    typedef single_storage_type::const_iterator it_type;
    single_storage_type const & vec = m_cb_update_vec;

    scoped_status scope( m_simc->m_simulation_status
                       , SC_END_OF_UPDATE );

    for(it_type it = vec.begin(), end = vec.end(); it != end; ++it)
        (*it)->do_simulation_phase_callback();
#endif
}

inline void
sc_phase_callback_registry::before_timestep() const
{
#if SC_HAS_PHASE_CALLBACKS_

    if( !m_cb_timestep_vec.size() ) return;

    typedef single_storage_type::const_iterator it_type;
    single_storage_type const & vec = m_cb_timestep_vec;

    scoped_status scope( m_simc->m_simulation_status
                       , SC_BEFORE_TIMESTEP );

    for(it_type it = vec.begin(), end = vec.end(); it != end; ++it)
        (*it)->do_simulation_phase_callback();
#endif
}

// ------------------------------------------------------------------

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
#endif /* SC_PHASE_CALLBACK_REGISTRY_H_INCLUDED_ */
// Taf!

