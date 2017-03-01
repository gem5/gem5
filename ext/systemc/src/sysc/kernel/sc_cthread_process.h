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

  sc_cthread_process.h -- Clocked thread declarations

  Original Author: Andy Goodrich, Forte Design Systems, 4 August 2005
               

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#if !defined(sc_cthread_process_h_INCLUDED)
#define sc_cthread_process_h_INCLUDED

#include "sysc/kernel/sc_thread_process.h"

namespace sc_core {

// friend function declarations:

void halt( sc_simcontext* );
void wait( int, sc_simcontext* );


//==============================================================================
// sc_cthread_process -
//
//==============================================================================
class sc_cthread_process : public sc_thread_process {

    friend class sc_module;
    friend class sc_process_handle;
    friend class sc_process_table;
    friend class sc_thread_process;
    friend class sc_simcontext;

    friend void sc_cthread_cor_fn( void* );

    friend void halt( sc_simcontext* );
    friend void wait( int, sc_simcontext* );

  public:
    sc_cthread_process( const char* name_p, bool free_host,
        SC_ENTRY_FUNC method_p, sc_process_host* host_p, 
        const sc_spawn_options* opt_p );

    virtual void dont_initialize( bool dont );
    virtual const char* kind() const
        { return "sc_cthread_process"; }

private:

    sc_cthread_process( const char*   nm,
            SC_ENTRY_FUNC fn,
            sc_process_host*    host );

    // may not be deleted manually (called from sc_process_b)
    virtual ~sc_cthread_process();

    bool eval_watchlist();
    bool eval_watchlist_curr_level();

    void wait_halt();

};

//------------------------------------------------------------------------------
//"sc_cthread_process::wait_halt"
//
//------------------------------------------------------------------------------
inline void sc_cthread_process::wait_halt()
{
    m_wait_cycle_n = 0;
    suspend_me();
    throw sc_halt();
}

} // namespace sc_core 

// $Log: sc_cthread_process.h,v $
// Revision 1.8  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.7  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.6  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.5  2011/02/11 13:25:24  acg
//  Andy Goodrich: Philipp A. Hartmann's changes:
//    (1) Removal of SC_CTHREAD method overloads.
//    (2) New exception processing code.
//
// Revision 1.4  2011/02/01 21:01:41  acg
//  Andy Goodrich: removed throw_reset() as it is now handled by the parent
//  method sc_thread_process::throw_reset().
//
// Revision 1.3  2011/01/18 20:10:44  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.2  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.6  2006/05/08 17:57:13  acg
//  Andy Goodrich: Added David Long's forward declarations for friend functions
//  to keep the Microsoft C++ compiler happy.
//
// Revision 1.5  2006/04/20 17:08:16  acg
//  Andy Goodrich: 3.0 style process changes.
//
// Revision 1.4  2006/04/11 23:13:20  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//

#endif // !defined(sc_cthread_process_h_INCLUDED)
