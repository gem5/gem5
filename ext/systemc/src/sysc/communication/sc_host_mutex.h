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

  sc_host_mutex.h -- A "real" mutex for the underlying host system

  Original Author: Philipp A. Hartmann, OFFIS

 CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_HOST_MUTEX_H_INCLUDED_
#define SC_HOST_MUTEX_H_INCLUDED_

#ifndef SC_INCLUDE_WINDOWS_H
#  define SC_INCLUDE_WINDOWS_H // include Windows.h, if needed
#endif
#include "sysc/kernel/sc_cmnhdr.h"
#include "sysc/communication/sc_mutex_if.h"

#if defined(WIN32) || defined(_WIN32)

#define SC_MTX_TYPE_ CRITICAL_SECTION

#define SC_MTX_INIT_( Mutex ) \
    InitializeCriticalSection( &(Mutex) )
#define SC_MTX_LOCK_( Mutex ) \
    EnterCriticalSection( &(Mutex) )
#define SC_MTX_UNLOCK_( Mutex ) \
    LeaveCriticalSection( &(Mutex) )
#define SC_MTX_TRYLOCK_( Mutex ) \
    ( TryEnterCriticalSection( &(Mutex) ) != 0 )
#define SC_MTX_DESTROY_( Mutex ) \
    DeleteCriticalSection( &(Mutex) )

#else // use pthread mutex

#include <pthread.h>
#define SC_MTX_TYPE_ pthread_mutex_t

#if defined(__hpux)
#  define SC_PTHREAD_NULL_ cma_c_null
#else // !defined(__hpux)
#  define SC_PTHREAD_NULL_ NULL
#endif

#define SC_MTX_INIT_( Mutex ) \
    pthread_mutex_init( &(Mutex), SC_PTHREAD_NULL_ )
#define SC_MTX_LOCK_( Mutex ) \
    pthread_mutex_lock( &(Mutex) )
#define SC_MTX_UNLOCK_( Mutex ) \
    pthread_mutex_unlock( &(Mutex) )

#ifdef _XOPEN_SOURCE
#   define SC_MTX_TRYLOCK_( Mutex ) \
       ( pthread_mutex_trylock( &(Mutex) ) == 0 )
#else // no try_lock available
#   define SC_MTX_TRYLOCK_( Mutex ) \
       ( false ) 
#endif

#define SC_MTX_DESTROY_( Mutex ) \
    pthread_mutex_destroy( &(Mutex) )

#endif

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_host_mutex
//
//   The sc_host_mutex class, wrapping an OS mutex on the simulation host
// ----------------------------------------------------------------------------

class sc_host_mutex : public sc_mutex_if
{
    typedef SC_MTX_TYPE_ underlying_type;
public:

    // constructors and destructor

    sc_host_mutex()
	{ SC_MTX_INIT_(m_mtx); }
    virtual ~sc_host_mutex()
	{ SC_MTX_DESTROY_(m_mtx); }


    // interface methods

    // blocks until mutex could be locked
    virtual int lock()
	{ SC_MTX_LOCK_(m_mtx); return 0; }

    // returns -1 if mutex could not be locked
    virtual int trylock()
	{ return SC_MTX_TRYLOCK_(m_mtx) ? 0 : -1; }

    // should return -1 if mutex was not locked by caller,
    // but is not yet able to check this
    virtual int unlock()
	{ SC_MTX_UNLOCK_(m_mtx); return 0; }

private:
    underlying_type m_mtx;
};

} // namespace sc_core

#undef SC_MTX_TYPE_
#undef SC_MTX_INIT_
#undef SC_MTX_DESTROY_
#undef SC_MTX_LOCK_
#undef SC_MTX_TRYLOCK_
#undef SC_MTX_UNLOCK_
#undef SC_PTHREAD_NULL_

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:
 *****************************************************************************/
//$Log: sc_host_mutex.h,v $
//Revision 1.4  2011/08/30 21:53:23  acg
// Jerome Cornet: include window.h for gnu c in windows case.
//
//Revision 1.3  2011/08/07 19:08:01  acg
// Andy Goodrich: moved logs to end of file so line number synching works
// better between versions.
//
//Revision 1.2  2011/06/25 17:08:38  acg
// Andy Goodrich: Jerome Cornet's changes to use libtool to build the
// library.
//
//Revision 1.1  2011/04/18 19:04:11  acg
// Philipp A. Hartmann: first check in of file.
//

#endif

// Taf!
