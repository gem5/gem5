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

  sc_reset.cpp -- Support for reset.

  Original Author: Andy Goodrich, Forte Design Systems

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_reset.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/communication/sc_signal.h"
#include "sysc/communication/sc_signal_ports.h"


// THE SYSTEMC PROOF OF CONCEPT SIMULATOR RESET SIGNAL IMPLEMENTATION:
//
// (1) An instance of the sc_reset class is attached to each sc_signal<bool> 
//     that is used as a reset signal.
//
// (2) Each process that is senstive to a reset signal will be registered in the
//     sc_reset class attached to that reset signal.
//
// (3) When a change in the value of a reset signal occurs it invokes the
//     notify_processes() method of its sc_reset object instance. The 
//     notify_processes() method will call the reset_changed() method of each
//     process that is registered with it to inform the process that 
//     state of the reset signal has changed.
//
// (4) A process may have multiple reset signals, so counters are kept for the 
//     number of active asynchronous, and synchronous, reset signals that are
//     active. Those counters are incremented and decremented in the process'
//     reset_changed() method.
//
// (5) When a process' semantics() method is called the current reset state is
//     checked, and a reset sequence is initiated if the process is in reset.
//     This will occur every time an SC_METHOD is dispatched. SC_CTHREAD and
//     and SC_THREAD instances, only go through the semantics() method they
//     initially start up. So the reset check  is duplicated in the suspend_me()
//     method, the tail of which will execute each time the thread is 
//     dispatched.

namespace sc_core {

class sc_reset_finder;
static sc_reset_finder* reset_finder_q=0;  // Q of reset finders to reconcile.

//==============================================================================
// sc_reset_finder - place holder class for a port reset signal until it is
//                   bound and an interface class is available. When the port
//                   has been bound the information in this class will be used
//                   to initialize its sc_reset object instance.
//==============================================================================
class sc_reset_finder {
    friend class sc_reset;
  public:
    sc_reset_finder( bool async, const sc_in<bool>* port_p, bool level, 
        sc_process_b* target_p);
    sc_reset_finder( bool async, const sc_inout<bool>* port_p, bool level, 
        sc_process_b* target_p);
    sc_reset_finder( bool async, const sc_out<bool>* port_p, bool level, 
        sc_process_b* target_p);

  protected:
    bool                   m_async;     // True if asynchronous reset.
    bool                   m_level;     // Level for reset.
    sc_reset_finder*       m_next_p;    // Next reset finder in list.
    const sc_in<bool>*     m_in_p;      // Port for which reset is needed.
    const sc_inout<bool>*  m_inout_p;   // Port for which reset is needed.
    const sc_out<bool>*    m_out_p;     // Port for which reset is needed.
    sc_process_b*          m_target_p;  // Process to reset.

  private: // disabled
    sc_reset_finder( const sc_reset_finder& );
    const sc_reset_finder& operator = ( const sc_reset_finder& );
};

inline sc_reset_finder::sc_reset_finder(
    bool async, const sc_in<bool>* port_p, bool level, sc_process_b* target_p) :
    m_async(async), m_level(level), m_next_p(0), m_in_p(port_p), m_inout_p(0), 
    m_out_p(0), m_target_p(target_p)
{   
    m_next_p = reset_finder_q;
    reset_finder_q = this;
}

inline sc_reset_finder::sc_reset_finder(
    bool async, const sc_inout<bool>* port_p, bool level, sc_process_b* target_p
) : 
    m_async(async), m_level(level), m_next_p(0), m_in_p(0), m_inout_p(port_p), 
    m_out_p(0), m_target_p(target_p)
{   
    m_next_p = reset_finder_q;
    reset_finder_q = this;
}

inline sc_reset_finder::sc_reset_finder(
    bool async, const sc_out<bool>* port_p, bool level, sc_process_b* target_p
) : 
    m_async(async), m_level(level), m_next_p(0), m_in_p(0), m_inout_p(0),
    m_out_p(port_p), m_target_p(target_p)
{   
    m_next_p = reset_finder_q;
    reset_finder_q = this;
}


//------------------------------------------------------------------------------
//"sc_reset::notify_processes"
//
// Notify processes that there is a change in the reset signal value.
//------------------------------------------------------------------------------
void sc_reset::notify_processes()
{
    bool                                    active;       // true if reset is active.
    sc_reset_target*                        entry_p;      // reset entry processing.
    std::vector<sc_reset_target>::size_type process_i;    // index of process resetting.
    std::vector<sc_reset_target>::size_type process_n;    // # of processes to reset.
    bool                                    value;        // value of our signal.

    value = m_iface_p->read();
    process_n = m_targets.size();
    for ( process_i = 0; process_i < process_n; process_i++ )
    {
        entry_p = &m_targets[process_i];
	active = ( entry_p->m_level == value );
	entry_p->m_process_p->reset_changed( entry_p->m_async, active );
    }
}


//------------------------------------------------------------------------------
//"sc_reset::reconcile_resets"
//
// This static method processes the sc_reset_finders to establish the actual
// reset connections.
//
// Notes:
//   (1) If reset is asserted we tell the process that it is in reset.
//------------------------------------------------------------------------------
void sc_reset::reconcile_resets()
{
    const sc_signal_in_if<bool>*  iface_p;      // Interface to reset signal.
    sc_reset_finder*              next_p;       // Next finder to process.
    sc_reset_finder*              now_p;        // Finder currently processing.
    sc_reset_target               reset_target; // Target's reset entry.
    sc_reset*                     reset_p;      // Reset object to use.

    for ( now_p = reset_finder_q; now_p; now_p = next_p )
    {
        next_p = now_p->m_next_p;
        if ( now_p->m_in_p )
        {
            iface_p = DCAST<const sc_signal_in_if<bool>*>(
                now_p->m_in_p->get_interface());
        }
        else if ( now_p->m_inout_p )
        {
            iface_p = DCAST<const sc_signal_in_if<bool>*>(
                now_p->m_inout_p->get_interface());
        }
        else
        {
            iface_p = DCAST<const sc_signal_in_if<bool>*>(
                now_p->m_out_p->get_interface());
        }
        assert( iface_p != 0 );
        reset_p = iface_p->is_reset();
	now_p->m_target_p->m_resets.push_back(reset_p);
	reset_target.m_async = now_p->m_async;
	reset_target.m_level = now_p->m_level;
	reset_target.m_process_p = now_p->m_target_p;
	reset_p->m_targets.push_back(reset_target);
	if ( iface_p->read() == now_p->m_level ) // see note 1 above
	    now_p->m_target_p->initially_in_reset( now_p->m_async );
        delete now_p;
    }
}


//------------------------------------------------------------------------------
//"sc_reset::remove_process"
//
// This method removes the supplied process from the list of processes that
// should be notified when there is a change in the value of the reset signal.
//
// Arguments:
//     process_p -> process to be removed.
//------------------------------------------------------------------------------
void sc_reset::remove_process( sc_process_b* process_p )
{
    int process_i; // Index of process resetting.
    int process_n; // # of processes to reset.

    process_n = m_targets.size();
    for ( process_i = 0; process_i < process_n; )
    {
        if ( m_targets[process_i].m_process_p == process_p )
        {
            m_targets[process_i] = m_targets[process_n-1];
	    process_n--;
            m_targets.resize(process_n);
        }
	else
	{
	    process_i++;
	}
    }
}

//------------------------------------------------------------------------------
//"sc_reset::reset_signal_is - ports"
//
// These overloads of the reset_signal_is() method will register the active
// process with the sc_reset object instance associated with the supplied port.
// If the port does not yet have a pointer to its sc_signal<bool> instance it
// will create an sc_reset_finder class object instance that will be used
// to set the process' reset information when the port has been bound.
//
// Arguments:
//     async = true if the reset signal is asynchronous, false if not.
//     port  = port for sc_signal<bool> that will provide the reset signal.
//     level = level at which reset is active, either true or false.
//------------------------------------------------------------------------------
void sc_reset::reset_signal_is( bool async, const sc_in<bool>& port, bool level)
{
    const sc_signal_in_if<bool>* iface_p;
    sc_process_b*                process_p;
    
    process_p = (sc_process_b*)sc_get_current_process_handle();
    assert( process_p );
    process_p->m_has_reset_signal = true;
    switch ( process_p->proc_kind() )
    {
      case SC_THREAD_PROC_:
      case SC_METHOD_PROC_:
      case SC_CTHREAD_PROC_:
        iface_p = DCAST<const sc_signal_in_if<bool>*>(port.get_interface());
        if ( iface_p )
            reset_signal_is( async, *iface_p, level );
        else
            new sc_reset_finder( async, &port, level, process_p );
        break;
      default:
        SC_REPORT_ERROR(SC_ID_UNKNOWN_PROCESS_TYPE_, process_p->name());
        break;
    }
}

void sc_reset::reset_signal_is(
    bool async, const sc_inout<bool>& port, bool level )
{
    const sc_signal_in_if<bool>* iface_p;
    sc_process_b*                process_p;
    
    process_p = (sc_process_b*)sc_get_current_process_handle();
    assert( process_p );
    process_p->m_has_reset_signal = true;
    switch ( process_p->proc_kind() )
    {
      case SC_THREAD_PROC_:
      case SC_METHOD_PROC_:
      case SC_CTHREAD_PROC_:
        iface_p = DCAST<const sc_signal_in_if<bool>*>(port.get_interface());
        if ( iface_p )
            reset_signal_is( async, *iface_p, level );
        else
            new sc_reset_finder( async, &port, level, process_p );
        break;
      default:
        SC_REPORT_ERROR(SC_ID_UNKNOWN_PROCESS_TYPE_, process_p->name());
        break;
    }
}

void sc_reset::reset_signal_is(
    bool async, const sc_out<bool>& port, bool level )
{
    const sc_signal_in_if<bool>* iface_p;
    sc_process_b*                process_p;
    
    process_p = (sc_process_b*)sc_get_current_process_handle();
    assert( process_p );
    process_p->m_has_reset_signal = true;
    switch ( process_p->proc_kind() )
    {
      case SC_THREAD_PROC_:
      case SC_METHOD_PROC_:
      case SC_CTHREAD_PROC_:
        iface_p = DCAST<const sc_signal_in_if<bool>*>(port.get_interface());
        if ( iface_p )
            reset_signal_is( async, *iface_p, level );
        else
            new sc_reset_finder( async, &port, level, process_p );
        break;
      default:
        SC_REPORT_ERROR(SC_ID_UNKNOWN_PROCESS_TYPE_, process_p->name());
        break;
    }
}

//------------------------------------------------------------------------------
//"sc_reset::reset_signal_is"
//
// This static method will register the active process instance as being
// reset by the sc_signal<bool> whose interface has been supplied. If no
// sc_reset object instance has been attached to the sc_signal<bool> yet, it
// will be created and attached. The active process instance is pushed into
// the list of processes that the sc_reset object instance should notify if
// the value of the reset signal changes.
//
// Arguments:
//     async = true if the reset signal is asynchronous, false if not.
//     iface = interface for the reset signal.
//     level = is the level at which reset is active, either true or false.
// Notes:
//   (1) If reset is asserted we tell the process that it is in reset
//       initially.
//------------------------------------------------------------------------------
void sc_reset::reset_signal_is( 
    bool async, const sc_signal_in_if<bool>& iface, bool level )
{
    sc_process_b*   process_p;    // process adding reset for.
    sc_reset_target reset_target; // entry to build for the process.
    sc_reset*       reset_p;      // reset object.

    process_p = sc_process_b::last_created_process_base();
    assert( process_p );
    process_p->m_has_reset_signal = true;
    switch ( process_p->proc_kind() )
    {
      case SC_METHOD_PROC_:
      case SC_CTHREAD_PROC_:
      case SC_THREAD_PROC_:
	reset_p = iface.is_reset();
	process_p->m_resets.push_back(reset_p);
        reset_target.m_async = async; 
	reset_target.m_level = level;
	reset_target.m_process_p = process_p;
	reset_p->m_targets.push_back(reset_target);
	if ( iface.read() == level ) process_p->initially_in_reset( async );
        break;
      default:
        SC_REPORT_ERROR(SC_ID_UNKNOWN_PROCESS_TYPE_, process_p->name());
        break;
    }
}

} // namespace sc_core

// $Log: sc_reset.cpp,v $
// Revision 1.16  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.15  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.14  2011/04/08 22:37:34  acg
//  Andy Goodrich: documentation of the reset mechanism and additional
//  documentation of methods. Removal of check for SC_METHODs in
//  sc_reset_signal_is() that should not have been there.
//
// Revision 1.13  2011/03/20 15:13:01  acg
//  Andy Goodrich: set the reset flag for async_reset_signal_is to catch
//  the suspend() corner case.
//
// Revision 1.12  2011/03/20 13:43:23  acg
//  Andy Goodrich: added async_signal_is() plus suspend() as a corner case.
//
// Revision 1.11  2011/03/06 19:57:11  acg
//  Andy Goodrich: refinements for the illegal suspend - synchronous reset
//  interaction.
//
// Revision 1.10  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.9  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.8  2011/02/01 21:08:26  acg
//  Andy Goodrich: new multiple reset support.
//
// Revision 1.7  2011/01/06 18:04:38  acg
//  Andy Goodrich: removed commented out code.
//
// Revision 1.6  2010/12/07 20:09:13  acg
// Andy Goodrich: removed sc_signal overloads since already have sc_signal_in_if overloads.
//
// Revision 1.5  2010/11/20 17:10:56  acg
//  Andy Goodrich: reset processing changes for new IEEE 1666 standard.
//
// Revision 1.4  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.3  2009/03/12 22:59:58  acg
//  Andy Goodrich: updates for 2.4 stuff.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.7  2006/12/02 20:58:19  acg
//  Andy Goodrich: updates from 2.2 for IEEE 1666 support.
//
// Revision 1.5  2006/04/11 23:13:21  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.4  2006/01/24 20:49:05  acg
// Andy Goodrich: changes to remove the use of deprecated features within the
// simulator, and to issue warning messages when deprecated features are used.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//
