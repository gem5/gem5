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

  sc_join.h -- Join Process Synchronization Definition

  Original Author: Andy Goodrich, Forte Design Systems, 5 May 2003

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

// $Log: sc_join.h,v $
// Revision 1.8  2011/08/26 21:45:00  acg
//  Andy Goodrich: fix internal event naming.
//
// Revision 1.7  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//

#ifndef SC_JOIN_H
#define SC_JOIN_H

#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_wait.h"

namespace sc_core {

//==============================================================================
// CLASS sc_join
//
// This class provides a way of waiting for a set of threads to complete their
// execution. The threads whose completion is to be monitored are registered,
// and upon their completion an event notification will occur.
//==============================================================================
class sc_join : public sc_process_monitor {
    friend class sc_process_b;
    friend class sc_process_handle;
  public:
    sc_join();
    void add_process( sc_process_handle process_h );
    inline int process_count();
    virtual void signal(sc_thread_handle thread_p, int type);
    inline void wait();
    inline void wait_clocked();

  protected: 
    void add_process( sc_process_b* process_p );

  protected:
    sc_event m_join_event;  // Event to notify when all threads have reported.
    int      m_threads_n;   // # of threads still need to wait for.
};

int sc_join::process_count() { return m_threads_n; }

// suspend a thread that does not have a sensitivity list:

inline void sc_join::wait() { ::sc_core::wait(m_join_event); }

// suspend a thread that has a sensitivity list:

inline void sc_join::wait_clocked()
{
    do { ::sc_core::wait(); } while (m_threads_n != 0);
}

#define SC_CJOIN \
    }; \
    sc_core::sc_join           join; \
    for ( unsigned int i = 0; \
        i < sizeof(forkees)/sizeof(sc_core::sc_process_handle); \
        i++ ) \
        join.add_process(forkees[i]); \
    join.wait_clocked(); \
}

#define SC_FORK \
{ \
    sc_core::sc_process_handle forkees[] = {

#define SC_JOIN \
    }; \
    sc_core::sc_join           join; \
    for ( unsigned int i = 0; \
        i < sizeof(forkees)/sizeof(sc_core::sc_process_handle); \
        i++ ) \
        join.add_process(forkees[i]); \
    join.wait(); \
}

} // namespace sc_core

// Revision 1.6  2011/08/24 22:05:50  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.5  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.4  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.3  2009/07/28 01:10:53  acg
//  Andy Goodrich: updates for 2.3 release candidate.
//
// Revision 1.2  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.5  2006/04/28 21:38:27  acg
//  Andy Goodrich: fixed loop constraint that was using sizeof(sc_thread_handle)
//  rather than sizeof(sc_process_handle).
//
// Revision 1.4  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.

#endif // SC_JOIN_H
