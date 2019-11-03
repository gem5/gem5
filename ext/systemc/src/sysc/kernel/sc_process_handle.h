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

  sc_process_handle.h -- Process access support.

  Original Author: Andy Goodrich, Forte Design Systems, 17 June 2003
               

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

// $Log: sc_process_handle.h,v $
// Revision 1.21  2011/08/26 21:54:04  acg
//  Torsten Maehne: Simplify use of dynamic_cast<> for initializing m_target.
//
// Revision 1.20  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//

#if !defined(sc_process_handle_h_INCLUDED)
#define sc_process_handle_h_INCLUDED

#include "sysc/kernel/sc_module.h"

namespace sc_core {

// forward operator declarations:

class sc_process_handle;
bool 
operator == ( const sc_process_handle& left, const sc_process_handle& right );
bool 
operator != ( const sc_process_handle& left, const sc_process_handle& right );
bool 
operator < ( const sc_process_handle& left, const sc_process_handle& right );



//=============================================================================
// CLASS sc_process_handle
//
// This class provides access to an sc_process_b object instance in a
// manner which allows some persistence after the deletion of the actual 
// process.
//=============================================================================
class sc_simcontext;
class sc_process_handle {
    typedef sc_process_handle this_type;

    friend bool operator == ( const this_type& left, const this_type& right );
    friend bool operator != ( const this_type& left, const this_type& right );
    friend bool operator < ( const this_type& left, const this_type& right );
    friend class sc_object;
    friend class sc_join;
    friend class sc_module;
    friend class sc_reset;
    friend class sc_sensitive;
    friend class sc_sensitive_pos;
    friend class sc_sensitive_neg;
    friend class sc_thread_process;

  public:
    inline sc_process_handle();
    inline explicit sc_process_handle( sc_object* object_p );
    inline explicit sc_process_handle( sc_process_b* process_p );
    inline sc_process_handle( const sc_process_handle& orig );
    inline ~sc_process_handle();
    inline sc_process_handle& operator = ( sc_process_handle src );
    inline void swap( sc_process_handle& other );

  public:
    inline void disable(
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline bool dynamic() const;
    inline void enable(
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline const std::vector<sc_event*>& get_child_events() const;
    inline const std::vector<sc_object*>& get_child_objects() const;
    inline sc_object* get_parent_object() const;
    inline sc_object* get_process_object() const;
    inline bool is_unwinding() const;
    inline void kill( 
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline const char* name() const;
    inline sc_curr_proc_kind proc_kind() const;
    inline void reset( 
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline sc_event& reset_event() const;
    inline void resume(
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline void suspend(
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline void sync_reset_off( 
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline void sync_reset_on( 
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline sc_event& terminated_event();
    inline bool terminated() const;
    template<typename EXCEPT>
    inline void throw_it( const EXCEPT& exception,
        sc_descendant_inclusion_info descendants=SC_NO_DESCENDANTS );
    inline bool valid() const;

  public: // implementation specific methods:
    inline std::string dump_state() const;

  protected:
    inline bool dont_initialize() const 
        { return m_target_p ? m_target_p->dont_initialize() : false; }
    inline void dont_initialize( bool dont );

  public:
    operator sc_process_b* () 
        { return m_target_p; }
    operator sc_cthread_handle ();
    operator sc_method_handle ();
    operator sc_thread_handle ();  

  protected:
    sc_process_b* m_target_p;   // Target for this object instance.

  protected:
    static std::vector<sc_event*>  empty_event_vector;  // If m_target_p == 0.
    static std::vector<sc_object*> empty_object_vector; // If m_target_p == 0.
    static sc_event                non_event;           // If m_target_p == 0.
};

inline bool operator == ( 
    const sc_process_handle& left, const sc_process_handle& right )
{
    return (left.m_target_p != 0) && (right.m_target_p != 0) &&
        (left.m_target_p == right.m_target_p);
}

inline bool operator != ( 
    const sc_process_handle& left, const sc_process_handle& right )
{
    return (left.m_target_p == 0) || (right.m_target_p == 0) ||
        (left.m_target_p != right.m_target_p);
}

inline bool operator < (
    const sc_process_handle& left, const sc_process_handle& right )
{
    return left.m_target_p < right.m_target_p;
}

//------------------------------------------------------------------------------
//"sc_process_handle::sc_process_handle - non-pointer constructor"
//
// This version of the object instance constructor for this class creates
// an object instance whose target needs to be supplied via an assignment.
//------------------------------------------------------------------------------
inline sc_process_handle::sc_process_handle() : m_target_p(0)
{
}

//------------------------------------------------------------------------------
//"sc_process_handle::sc_process_handle - pointer constructor"
//
// This version of the object instance constructor for this class creates
// an object instance whose target is the supplied sc_object instance.
// The supplied sc_object must in fact be an sc_process_b instance.
//     object_p -> sc_object instance this is handle for.
//------------------------------------------------------------------------------
inline sc_process_handle::sc_process_handle( sc_object* object_p ) :
    m_target_p(DCAST<sc_process_b*>(object_p))
{
    if ( m_target_p ) m_target_p->reference_increment();
}

//------------------------------------------------------------------------------
//"sc_process_handle::sc_process_handle - pointer constructor"
//
// This version of the object instance constructor for this class creates
// an object instance whose target is the supplied sc_process_b instance.
// This saves a dynamic cast compared to the sc_object* case.
//     process_p -> process instance this is handle for.
//------------------------------------------------------------------------------
inline sc_process_handle::sc_process_handle( sc_process_b* process_p ) :
  m_target_p(process_p)
{
  if ( m_target_p ) m_target_p->reference_increment();
}

//------------------------------------------------------------------------------
//"sc_process_handle::sc_process_handle - copy constructor"
//
// This version of the object instance constructor for this class provides
// the copy constructor for the class. It clones the supplied original
// handle and increments the references to its target.
//     orig = sc_process_handle object instance to be copied from.
//------------------------------------------------------------------------------
inline sc_process_handle::sc_process_handle( const sc_process_handle& orig ) :
    m_target_p(orig.m_target_p)
{
    if ( m_target_p ) m_target_p->reference_increment();
}


//------------------------------------------------------------------------------
//"sc_process_handle::operator ="
//
// This assignment operator signature is call by value rather than reference.
// This means that an sc_process_handle instance will be created and the
// target for that instance will be incremented before the assignment is done.
// The assignment is done using the swap() method, which simply swaps the
// targets of 'orig' and this object instance. We don't need to increment
// the reference count for our new target since that was done when 'orig'
// was created. Our old target's reference count will be decremented when
// 'orig' is deleted.
//     orig = sc_process_handle object instance to be copied from.
// Result is a reference for this object instance.
//------------------------------------------------------------------------------
inline sc_process_handle& 
sc_process_handle::operator = ( sc_process_handle orig )
{
    swap( orig );
    return *this;
}


//------------------------------------------------------------------------------
//"sc_process_handle::~sc_process_handle"
//
// This is the object instance destructor for this class. It decrements
// the reference count for its target.
//------------------------------------------------------------------------------
inline sc_process_handle::~sc_process_handle()
{
    if ( m_target_p ) m_target_p->reference_decrement();
}

//------------------------------------------------------------------------------
//"sc_process_handle::inline methods"
//
// These are short inline methods.
//------------------------------------------------------------------------------

// disable this object instance's target.

inline void sc_process_handle::disable(sc_descendant_inclusion_info descendants)
{
    if ( m_target_p ) 
        m_target_p->disable_process(descendants);
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "disable()");
}

// call dont_initialize() on this object instance's target.

inline void sc_process_handle::dont_initialize( bool dont )
{
    if ( m_target_p ) 
        m_target_p->dont_initialize( dont );
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "dont_initialize()");
}

// dump the status of this object instance's target:

inline std::string sc_process_handle::dump_state() const
{
    return m_target_p ? m_target_p->dump_state() : std::string("NO TARGET");
}

// return whether this object instance's target is dynamic or not.

inline bool sc_process_handle::dynamic() const
{
    return  m_target_p ?  m_target_p->dynamic() : false;
}

// enable this object instance's target.

inline void sc_process_handle::enable(sc_descendant_inclusion_info descendants)
{
    if ( m_target_p ) 
        m_target_p->enable_process(descendants);
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "enable()");
}

// return the child objects for this object instance's target.

inline 
const std::vector<sc_event*>& sc_process_handle::get_child_events() const
{
    return m_target_p ?  m_target_p->get_child_events() : empty_event_vector;
}

// return the child objects for this object instance's target.

inline 
const std::vector<sc_object*>& sc_process_handle::get_child_objects() const
{
    return m_target_p ?  m_target_p->get_child_objects() : empty_object_vector;
}

// return the parent object for this object instance's target.

inline sc_object* sc_process_handle::get_parent_object() const
{
    return m_target_p ?  m_target_p->get_parent_object() : NULL;
}

// return this object instance's target.

inline sc_object* sc_process_handle::get_process_object() const
{
    return m_target_p;
}

// return whether this object instance is unwinding or not.

inline bool sc_process_handle::is_unwinding() const
{
    if ( m_target_p )
        return m_target_p->is_unwinding();
    else {
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "is_unwinding()");
        return false;
    }
}

// kill this object instance's target.

inline void sc_process_handle::kill( sc_descendant_inclusion_info descendants )
{
    if ( m_target_p ) 
        m_target_p->kill_process( descendants );
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "kill()");
}

// return the name of this object instance's target.

inline const char* sc_process_handle::name() const
{
    return  m_target_p ? m_target_p->name() : ""; 
}

// return the process kind for this object instance's target.

inline sc_curr_proc_kind sc_process_handle::proc_kind() const
{
    return m_target_p ?  m_target_p->proc_kind() : SC_NO_PROC_;
}

// reset this object instance's target.

inline void sc_process_handle::reset( sc_descendant_inclusion_info descendants )
{
    if ( m_target_p ) 
        m_target_p->reset_process( sc_process_b::reset_asynchronous,
	                           descendants );
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "reset()");
}

// return the reset event for this object instance's target.

inline sc_event& sc_process_handle::reset_event() const
{
    if ( m_target_p ) 
        return m_target_p->reset_event();
    else
    {
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "reset()");
        return sc_process_handle::non_event;
    }
}

// resume this object instance's target.

inline void sc_process_handle::resume(sc_descendant_inclusion_info descendants)
{
    if ( m_target_p ) 
        m_target_p->resume_process(descendants);
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "resume()");
}

// suspend this object instance's target.

inline void sc_process_handle::suspend(sc_descendant_inclusion_info descendants)
{
    if ( m_target_p ) 
        m_target_p->suspend_process(descendants);
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "suspend()");
}

// swap targets of this process handle with the supplied one.

inline void sc_process_handle::swap( sc_process_handle& other )
{
    sc_process_b* tmp = m_target_p;
    m_target_p = other.m_target_p;
    other.m_target_p = tmp;
}

// turn sync_reset off for this object instance's target.

inline void sc_process_handle::sync_reset_off(
    sc_descendant_inclusion_info descendants)
{
    if ( m_target_p ) 
        m_target_p->reset_process( sc_process_b::reset_synchronous_off, 
	                           descendants);
    else
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "sync_reset_off()");
}

// turn sync_reset on for this object instance's target.

inline void sc_process_handle::sync_reset_on(
    sc_descendant_inclusion_info descendants)
{
    if ( m_target_p ) 
    {
        m_target_p->reset_process(sc_process_b::reset_synchronous_on, 
            descendants);
    }
    else
    {
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "sync_reset_on()");
    }
}

// terminate this object instance's target.

inline bool sc_process_handle::terminated() const
{
    return m_target_p ?  m_target_p->terminated() : false;
}

// return the termination event for this object instance's target.

inline sc_event& sc_process_handle::terminated_event()
{
    if ( m_target_p )
        return m_target_p->terminated_event();
    else
    {
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "terminated_event()");
        return sc_process_handle::non_event;
    }
}

// return true if this object instance has a target, false it not.

inline bool sc_process_handle::valid() const
{
    return m_target_p ? true : false;
}

//------------------------------------------------------------------------------
//"sc_process_handle::sc_throw_it"
//
// This method throws the supplied exception to the process whose handle this
// object instance is, and optionally to the process' descendants. Once the
// exception is thrown the currently executed process will suspend to allow
// the exception to be propagated. Once the propagation has occurred the
// current process will be resumed.
//
// Notes:
//   (1) We allocate the helper function on the stack, see the description of
//       sc_throw_it<EXCEPT>, in sc_process.h, for why.
//
// Arguments:
//    exception = exception to be thrown
//    descendants = indication of whether descendant processes should also
//                  receive the throw.
//------------------------------------------------------------------------------
template<typename EXCEPT>
inline void sc_process_handle::throw_it( const EXCEPT& exception,
    sc_descendant_inclusion_info descendants)
{
    sc_throw_it<EXCEPT> helper(exception);  // helper to throw the exception.

    if ( !m_target_p ) 
    {
        SC_REPORT_WARNING( SC_ID_EMPTY_PROCESS_HANDLE_, "throw_it()");
	return;
    }
    m_target_p->throw_user(helper, descendants);
}


//------------------------------------------------------------------------------
//"sc_process_b::last_created_process_handle"
//
// This method returns the kind of this process.
//------------------------------------------------------------------------------
inline sc_process_handle sc_process_b::last_created_process_handle()
{
    return sc_process_handle(m_last_created_process_p);
}

inline sc_process_handle sc_get_last_created_process_handle()
{
    return sc_process_b::last_created_process_handle();
}

} // namespace sc_core

// Revision 1.19  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.18  2011/04/01 22:08:26  acg
//  Andy Goodrich: remove unused variable.
//
// Revision 1.17  2011/03/12 21:07:51  acg
//  Andy Goodrich: changes to kernel generated event support.
//
// Revision 1.16  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.15  2011/02/17 19:53:03  acg
//  Andy Goodrich: changed dump_status() to dump_state() with new signature.
//
// Revision 1.14  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.13  2011/02/13 21:33:30  acg
//  Andy Goodrich: added dump_status() to allow the dumping of the status
//  of a process handle's target.
//
// Revision 1.12  2011/02/01 23:01:53  acg
//  Andy Goodrich: removed dead code.
//
// Revision 1.11  2011/02/01 21:07:36  acg
//  Andy Goodrich: defering of run queue manipulations to the
//  sc_thread_process::throw_it() method.
//
// Revision 1.10  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.9  2011/01/20 16:52:20  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.8  2011/01/19 23:21:50  acg
//  Andy Goodrich: changes for IEEE 1666 2011
//
// Revision 1.7  2011/01/18 20:10:45  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.6  2010/07/30 05:21:22  acg
//  Andy Goodrich: release 2.3 fixes.
//
// Revision 1.5  2010/07/22 20:02:33  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.4  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.3  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.2  2007/09/20 20:32:35  acg
//  Andy Goodrich: changes to the semantics of throw_it() to match the
//  specification. A call to throw_it() will immediately suspend the calling
//  thread until all the throwees have executed. At that point the calling
//  thread will be restarted before the execution of any other threads.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.7  2006/05/08 17:58:24  acg
// Andy Goodrich: added David Long's forward declarations for friend
//   functions, methods, and operators to keep the Microsoft compiler happy.
//
// Revision 1.6  2006/04/20 17:08:17  acg
//  Andy Goodrich: 3.0 style process changes.
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

#endif // !defined(sc_spawn_h_INCLUDED)
