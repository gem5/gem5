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

/*******************************************************************************

  sc_runnable_int.h -- For inline definitions of some utility functions.
                       DO NOT EXPORT THIS INCLUDE FILE. Include this file
                       after "sc_process_int.h" so that we can get the base
                       class right.

  Original Author: Bishnupriya Bhattacharya , Cadence Design, 28th July, 2003

 CHANGE LOG AT THE END OF THE FILE
 ******************************************************************************/

#ifndef SC_RUNNABLE_INT_H
#define SC_RUNNABLE_INT_H


#include "sysc/kernel/sc_runnable.h"
#include "sysc/kernel/sc_method_process.h"
#include "sysc/kernel/sc_thread_process.h"

// DEBUGGING MACROS:
//
// DEBUG_MSG(NAME,P,MSG)
//     MSG  = message to print
//     NAME = name that must match the process for the message to print, or
//            null if the message should be printed unconditionally.
//     P    = pointer to process message is for, or NULL in which case the
//            message will not print.
#if 0
#   define DEBUG_NAME ""
#   define DEBUG_MSG(NAME,P,MSG) \
    { \
        if ( P && ( (strlen(NAME)==0) || !strcmp(NAME,P->name())) ) \
          std::cout << "**** " << sc_time_stamp() << " ("  \
	            << sc_get_current_process_name() << "): " << MSG \
		    << " - " << P->name() << std::endl; \
    }
#else
#   define DEBUG_MSG(NAME,P,MSG) 
#endif

namespace sc_core {

// The values below are used to indicate when a queue is empty. A non-zero     
// non-legal pointer value is used for this so that a zero value in the
// m_execute_p field of an sc_process_b instance can be used to indicate      
// that is has not been queued for run. (If we did not use a non-zero
// queue empty indicator then a sc_process_b instance that was queued
// twice in a row might end up on the queue twice if it were the first
// one that was queued!)

#define SC_NO_METHODS ((sc_method_handle)0xdb)
#define SC_NO_THREADS ((sc_thread_handle)0xdb)


//------------------------------------------------------------------------------
//"sc_runnable::dump"
//
// This method dumps the contents of this object instance.
//------------------------------------------------------------------------------
inline void sc_runnable::dump() const
{
    // Dump the thread queues:

    std::cout << "thread pop queue: " << std::endl;
    for ( sc_thread_handle p = m_threads_pop; p != SC_NO_THREADS; 
          p = p->next_runnable() )
    {
        std::cout << "    " << p << std::endl;
    }

    std::cout << "thread push queue: " << std::endl;
    for ( sc_thread_handle p = m_threads_push_head->next_runnable(); 
          p != SC_NO_THREADS; p = p->next_runnable() )
    {
        std::cout << "    " << p << std::endl;
    }
}

//------------------------------------------------------------------------------
//"sc_runnable::execute_method_next"
//
// This method pushes the the supplied method to execute as the next process.
// This is done by pushing it onto the front of the m_methods_pop.
//     method_h -> method process to add to the queue.
//------------------------------------------------------------------------------
inline void sc_runnable::execute_method_next( sc_method_handle method_h )
{
    DEBUG_MSG(DEBUG_NAME,method_h,"pushing this method to execute next");
    method_h->set_next_runnable( m_methods_pop );
    m_methods_pop = method_h;
}

//------------------------------------------------------------------------------
//"sc_runnable::execute_thread_next"
//
// This method pushes the the supplied thread to execute as the next process.
// This is done by pushing it onto the front of the m_threads_pop.
//     thread_h -> thread process to add to the queue.
//------------------------------------------------------------------------------
inline void sc_runnable::execute_thread_next( sc_thread_handle thread_h )
{
    DEBUG_MSG(DEBUG_NAME,thread_h,"pushing this thread to execute next");
    thread_h->set_next_runnable( m_threads_pop );
    m_threads_pop = thread_h;
}

//------------------------------------------------------------------------------
//"sc_runnable::init"
//
// This method initializes this object instance. Note we allocate the queue
// heads if necessary. This is done here rather than in the constructor for
// this class to eliminate CTOR processing errors with gcc.
//------------------------------------------------------------------------------
inline void sc_runnable::init()
{
    m_methods_pop = SC_NO_METHODS;
    if ( !m_methods_push_head )
    {
        m_methods_push_head = new sc_method_process("methods_push_head", true, 
	                                           (SC_ENTRY_FUNC)0, 0, 0);
        m_methods_push_head->dont_initialize(true);
	m_methods_push_head->detach();
    }
    m_methods_push_tail = m_methods_push_head;
    m_methods_push_head->set_next_runnable(SC_NO_METHODS);

    m_threads_pop = SC_NO_THREADS;
    if ( !m_threads_push_head )
    {
        m_threads_push_head = new sc_thread_process("threads_push_head", true, 
	                                            (SC_ENTRY_FUNC)0, 0, 0);
        m_threads_push_head->dont_initialize(true);
	m_threads_push_head->detach();
    }
    m_threads_push_head->set_next_runnable(SC_NO_THREADS);
    m_threads_push_tail = m_threads_push_head;
}


//------------------------------------------------------------------------------
//"sc_runnable::is_empty"
//
// This method returns true if the push queue is empty, or false if not.
//------------------------------------------------------------------------------
inline bool sc_runnable::is_empty() const
{
    return m_methods_push_head->next_runnable() == SC_NO_METHODS && 
           m_methods_pop == SC_NO_METHODS &&
	   m_threads_push_head->next_runnable() == SC_NO_THREADS &&
	   m_threads_pop == SC_NO_THREADS;
}


//------------------------------------------------------------------------------
//"sc_runnable::is_initialized"
//
// This method returns true if the push queue is already initialized.
//------------------------------------------------------------------------------
inline bool sc_runnable::is_initialized() const
{
    return m_methods_push_head && m_threads_push_head;
}


//------------------------------------------------------------------------------
//"sc_runnable::push_back_method"
//
// This method pushes the supplied method process onto the back of the queue of 
// runnable method processes.
//     method_h -> method process to add to the queue.
//------------------------------------------------------------------------------
inline void sc_runnable::push_back_method( sc_method_handle method_h )
{
    // assert( method_h->next_runnable() == 0 ); // Can't queue twice.
    DEBUG_MSG(DEBUG_NAME,method_h,"pushing back method");
    method_h->set_next_runnable(SC_NO_METHODS);
    m_methods_push_tail->set_next_runnable(method_h);
    m_methods_push_tail = method_h;
}


//------------------------------------------------------------------------------
//"sc_runnable::push_back_thread"
//
// This method pushes the supplied thread process onto the back of the queue of 
// runnable thread processes.
//     thread_h -> thread process to add to the queue.
//------------------------------------------------------------------------------
inline void sc_runnable::push_back_thread( sc_thread_handle thread_h )
{
    // assert( thread_h->next_runnable() == 0 ); // Can't queue twice.
    DEBUG_MSG(DEBUG_NAME,thread_h,"pushing back thread");
    thread_h->set_next_runnable(SC_NO_THREADS);
    m_threads_push_tail->set_next_runnable(thread_h);
    m_threads_push_tail = thread_h;
}


//------------------------------------------------------------------------------
//"sc_runnable::push_front_method"
//
// This method pushes the supplied method process onto the front of the queue of
// runnable method processes. If the queue is empty the process is the tail 
// also.
//     method_h -> method process to add to the queue.
//------------------------------------------------------------------------------
inline void sc_runnable::push_front_method( sc_method_handle method_h )
{
    // assert( method_h->next_runnable() == 0 ); // Can't queue twice.
    DEBUG_MSG(DEBUG_NAME,method_h,"pushing front method");
    method_h->set_next_runnable(m_methods_push_head->next_runnable());
    if ( m_methods_push_tail == m_methods_push_head ) // Empty queue.
    {
        m_methods_push_tail->set_next_runnable(method_h);
	m_methods_push_tail = method_h;
    }
    else                                               // Non-empty queue.
    {
	m_methods_push_head->set_next_runnable(method_h);
    }
}


//------------------------------------------------------------------------------
//"sc_runnable::push_front_thread"
//
// This method pushes the supplied thread process onto the front of the queue of
// runnable thread processes. If the queue is empty the process is the tail 
// also.
//     thread_h -> thread process to add to the queue.
//------------------------------------------------------------------------------
inline void sc_runnable::push_front_thread( sc_thread_handle thread_h )
{
    // assert( thread_h->next_runnable() == 0 ); // Can't queue twice.
    DEBUG_MSG(DEBUG_NAME,thread_h,"pushing front thread");
    thread_h->set_next_runnable(m_threads_push_head->next_runnable());
    if ( m_threads_push_tail == m_threads_push_head ) // Empty queue.
    {
        m_threads_push_tail->set_next_runnable(thread_h);
	m_threads_push_tail = thread_h;
    }
    else                                               // Non-empty queue.
    {
	m_threads_push_head->set_next_runnable(thread_h);
    }
}

//------------------------------------------------------------------------------
//"sc_runnable::pop_method"
//
// This method pops the next method process to be executed, or returns a null 
// if no method processes are available for execution.
//------------------------------------------------------------------------------
inline sc_method_handle sc_runnable::pop_method()
{
    sc_method_handle result_p;

    result_p = m_methods_pop;
    if ( result_p != SC_NO_METHODS ) 
    {
        m_methods_pop = result_p->next_runnable();
        result_p->set_next_runnable(0);
    }
    else
    {
	result_p = 0;
    }
    DEBUG_MSG(DEBUG_NAME,result_p,"popping method");
    return result_p;

}

//------------------------------------------------------------------------------
//"sc_runnable::pop_thread"
//
// This method pops the next thread process to be executed, or returns a null 
// if no thread processes are available for execution.
//------------------------------------------------------------------------------
inline sc_thread_handle sc_runnable::pop_thread()
{
    sc_thread_handle result_p;

    result_p = m_threads_pop;
    if ( result_p != SC_NO_THREADS ) 
    {
        m_threads_pop = result_p->next_runnable();
        result_p->set_next_runnable(0);
    }
    else
    {
	    result_p = 0;
    }
    DEBUG_MSG(DEBUG_NAME,result_p,"popping thread for execution");
    return result_p;
}


//------------------------------------------------------------------------------
//"sc_runnable::remove_method"
//
// This method removes the supplied method process from the push queue if it is
// present. Note we clear the method's next pointer so that it may be queued 
// again.
//     remove_p -> method process to remove from the run queue.
//------------------------------------------------------------------------------
inline void sc_runnable::remove_method( sc_method_handle remove_p )
{
    sc_method_handle now_p;     // Method now checking.
    sc_method_handle prior_p;   // Method prior to now_p.

    // Don't try to remove things if we have not been initialized.

    if ( !is_initialized() ) return;

    // Search the push queue:

    prior_p = m_methods_push_head;
    for ( now_p = m_methods_push_head; now_p!= SC_NO_METHODS; 
	    now_p = now_p->next_runnable() )
    {
        if ( remove_p == now_p )
        {
            prior_p->set_next_runnable( now_p->next_runnable() );
            if (now_p == m_methods_push_tail) {
                m_methods_push_tail = prior_p;
            }
            now_p->set_next_runnable(0);
	    DEBUG_MSG(DEBUG_NAME,now_p,"removing method from push queue");
            return;
        }
        prior_p = now_p;
    }

    // Search the pop queue:

    prior_p = NULL;
    for ( now_p = m_methods_pop; now_p != SC_NO_METHODS; 
	  now_p = now_p->next_runnable() )
    {
        if ( remove_p == now_p )
        {
	    if ( prior_p )
		prior_p->set_next_runnable( now_p->next_runnable() );
	    else
	        m_methods_pop = now_p->next_runnable();
            now_p->set_next_runnable(0);
	    DEBUG_MSG(DEBUG_NAME,now_p,"removing method from pop queue");
            return;
        }
        prior_p = now_p;
    }
}


//------------------------------------------------------------------------------
//"sc_runnable::remove_thread"
//
// This method removes the supplied thread process from the push or pop
// queue if it is present. Note we clear the thread's next pointer so that it 
// may be queued again.
//     remove_p -> thread process to remove from the run queue.
//------------------------------------------------------------------------------
inline void sc_runnable::remove_thread( sc_thread_handle remove_p )
{
    sc_thread_handle now_p;     // Thread now checking.
    sc_thread_handle prior_p;   // Thread prior to now_p.

    // Don't try to remove things if we have not been initialized.

    if ( !is_initialized() ) return;

    // Search the push queue:

    prior_p = m_threads_push_head;
    for ( now_p = m_threads_push_head; now_p != SC_NO_THREADS; 
	  now_p = now_p->next_runnable() )
    {
        if ( remove_p == now_p )
        {
            prior_p->set_next_runnable( now_p->next_runnable() );
            if (now_p == m_threads_push_tail) {
                m_threads_push_tail = prior_p;
            }
            now_p->set_next_runnable(0);
	    DEBUG_MSG(DEBUG_NAME,now_p,"removing thread from push queue");
            return;
        }
        prior_p = now_p;
    }

    // Search the pop queue:

    prior_p = NULL;
    for ( now_p = m_threads_pop; now_p != SC_NO_THREADS; 
	  now_p = now_p->next_runnable() )
    {
        if ( remove_p == now_p )
        {
	    if ( prior_p )
		prior_p->set_next_runnable( now_p->next_runnable() );
	    else
	        m_threads_pop = now_p->next_runnable();
            now_p->set_next_runnable(0);
	    DEBUG_MSG(DEBUG_NAME,now_p,"removing thread from pop queue");
            return;
        }
        prior_p = now_p;
    }
}

//------------------------------------------------------------------------------
//"sc_runnable::sc_runnable"
//
// This is the object instance constructor for this class.
//------------------------------------------------------------------------------
inline sc_runnable::sc_runnable() : 
   m_methods_push_head(0), m_methods_push_tail(0), m_methods_pop(SC_NO_METHODS),
   m_threads_push_head(0), m_threads_push_tail(0), m_threads_pop(SC_NO_THREADS)
{}

//------------------------------------------------------------------------------
//"sc_runnable::~sc_runnable"
//
// This is the object instance destructor for this class.
//------------------------------------------------------------------------------
inline sc_runnable::~sc_runnable()
{
    delete m_methods_push_head;
    delete m_threads_push_head;
}


//------------------------------------------------------------------------------
//"sc_runnable::toggle_methods"
//
// This method moves the methods push queue to the pop queue and zeros the push
// queue. This will only be done if the pop queue is presently empty.
//------------------------------------------------------------------------------
inline void sc_runnable::toggle_methods()
{
    if ( m_methods_pop == SC_NO_METHODS )
    {
	m_methods_pop = m_methods_push_head->next_runnable();
	m_methods_push_head->set_next_runnable(SC_NO_METHODS);
	m_methods_push_tail = m_methods_push_head;
    }
}


//------------------------------------------------------------------------------
//"sc_runnable::toggle_threads"
//
// This method moves the threads push queue to the pop queue and zeros the push
// queue. This will only be done if the pop queue is presently empty.
//------------------------------------------------------------------------------
inline void sc_runnable::toggle_threads()
{
    if ( m_threads_pop == SC_NO_THREADS )
    {
	m_threads_pop = m_threads_push_head->next_runnable();
	m_threads_push_head->set_next_runnable(SC_NO_THREADS);
	m_threads_push_tail = m_threads_push_head;
    }
}

#undef SC_NO_METHODS
#undef SC_NO_THREADS
#undef DEBUG_MSG

} // namespace sc_core


/*******************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.
      Andy Goodrich, Forte Design Systems, 2 September 2003
      Changed queue heads to instances to eliminate the checks for null heads.

 ******************************************************************************/

// $Log: sc_runnable_int.h,v $
// Revision 1.19  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.18  2011/08/07 19:08:04  acg
//  Andy Goodrich: moved logs to end of file so line number synching works
//  better between versions.
//
// Revision 1.17  2011/04/13 02:45:11  acg
//  Andy Goodrich: eliminated warning message that occurred if the DEBUG_MSG
//  macro was used.
//
// Revision 1.16  2011/04/10 22:18:23  acg
//  Andy Goodrich: debugging message clean up.
//
// Revision 1.15  2011/04/08 18:26:07  acg
//  Andy Goodrich: added execute_method_next() to handle method dispatch
//   for asynchronous notifications that occur outside the evaluation phase.
//
// Revision 1.14  2011/04/01 21:31:10  acg
//  Andy Goodrich: turn off diagnostic messages by default.
//
// Revision 1.13  2011/04/01 21:30:02  acg
//  Andy Goodrich: inserted conditional displays for queue manipulations.
//
// Revision 1.12  2011/03/30 00:01:34  acg
//  Philip A. Hartmann: change break to return in remove_method() to short
//  circuit the search the way remove_thread() works.
//
// Revision 1.11  2011/03/28 13:02:52  acg
//  Andy Goodrich: Changes for disable() interactions.
//
// Revision 1.10  2011/03/06 15:58:17  acg
//  Andy Goodrich: formatting changes.
//
// Revision 1.9  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.8  2011/02/13 21:47:38  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.7  2011/02/02 06:37:03  acg
//  Andy Goodrich: removed toggle() method since it is no longer used.
//
// Revision 1.6  2011/02/01 21:09:13  acg
//  Andy Goodrich: addition of toggle_methods() and toggle_threads() calls.
//
// Revision 1.5  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.4  2010/07/22 20:02:33  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.3  2009/02/28 00:26:58  acg
//  Andy Goodrich: changed boost name space to sc_boost to allow use with
//  full boost library applications.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/04/20 17:08:17  acg
//  Andy Goodrich: 3.0 style process changes.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//

#endif // SC_RUNNABLE_INT_H

// Taf!
