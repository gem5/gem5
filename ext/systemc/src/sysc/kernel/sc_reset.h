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

  sc_reset.h -- Process reset support.

  Original Author: Andy Goodrich, Forte Design Systems, 17 June 2003

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#if !defined(sc_reset_h_INCLUDED)
#define sc_reset_h_INCLUDED

#include "sysc/communication/sc_writer_policy.h"

namespace sc_core {

// FORWARD CLASS REFERENCES:

template<typename DATA> class sc_signal_in_if;
template<typename IF, sc_writer_policy POL> class sc_signal;
template<typename DATA> class sc_in;
template<typename DATA> class sc_inout;
template<typename DATA> class sc_out;
template<typename SOURCE> class sc_spawn_reset;
class sc_reset;
class sc_process_b;

//==============================================================================
// CLASS sc_reset_target - RESET ENTRY FOR AN sc_process_b TARGET
//
// This class describes a reset condition associated with an sc_process_b
// instance. 
//==============================================================================
class sc_reset_target {
  public:
    bool          m_async;     // true asynchronous reset, false synchronous.
    bool          m_level;     // level for reset.
    sc_process_b* m_process_p; // process this reset entry is for.
};

inline std::ostream& operator << ( std::ostream& os, 
                                   const sc_reset_target& target )
{
    os << "[";
    os << target.m_async << ",";
    os << target.m_level << ",";
    os << target.m_process_p << ",";
    return os;
}

//==============================================================================
// CLASS sc_reset - RESET INFORMATION FOR A RESET SIGNAL
//
// See the top of sc_reset.cpp for an explaination of how the reset mechanism
// is implemented.
//==============================================================================
class sc_reset {
    friend class sc_cthread_process;
    friend class sc_method_process; 
    friend class sc_module; 
    friend class sc_process_b;
    friend class sc_signal<bool, SC_ONE_WRITER>;
    friend class sc_signal<bool, SC_MANY_WRITERS>;
    friend class sc_signal<bool, SC_UNCHECKED_WRITERS>;
    friend class sc_simcontext;
    template<typename SOURCE> friend class sc_spawn_reset;
    friend class sc_thread_process; 

  protected:
    static void reconcile_resets();
    static void 
	reset_signal_is(bool async, const sc_signal_in_if<bool>& iface, 
	                bool level);
    static void 
	reset_signal_is( bool async, const sc_in<bool>& iface, bool level);
    static void 
	reset_signal_is( bool async, const sc_inout<bool>& iface, bool level);
    static void 
	reset_signal_is( bool async, const sc_out<bool>& iface, bool level);

  protected:
    sc_reset( const sc_signal_in_if<bool>* iface_p ) :
        m_iface_p(iface_p), m_targets() {}
    void notify_processes();
    void remove_process( sc_process_b* );

  protected:
    const sc_signal_in_if<bool>*  m_iface_p;  // Interface to read.
    std::vector<sc_reset_target>  m_targets;  // List of processes to reset.

  private: // disabled
    sc_reset( const sc_reset& );
    const sc_reset& operator = ( const sc_reset& );
};

// $Log: sc_reset.h,v $
// Revision 1.11  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.10  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.9  2011/04/08 22:38:30  acg
//  Andy Goodrich: added comment pointing to the description of how the
//  reset mechanism works that is in sc_reset.cpp.
//
// Revision 1.8  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.7  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.6  2011/01/06 18:00:32  acg
//  Andy Goodrich: Removed commented out code.
//
// Revision 1.5  2010/12/07 20:09:14  acg
// Andy Goodrich: removed sc_signal signatures since already have sc_signal_in_if signatures.
//
// Revision 1.4  2010/11/20 17:10:57  acg
//  Andy Goodrich: reset processing changes for new IEEE 1666 standard.
//
// Revision 1.3  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.6  2006/12/02 20:58:19  acg
//  Andy Goodrich: updates from 2.2 for IEEE 1666 support.
//
// Revision 1.4  2006/04/11 23:13:21  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.

} // namespace sc_core

#endif // !defined(sc_reset_h_INCLUDED)
