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

  sc_fxnum_observer.h -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_fxnum_observer.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_FX_SC_FXNUM_OBSERVER_HH__
#define __SYSTEMC_EXT_DT_FX_SC_FXNUM_OBSERVER_HH__

#include "sc_fxdefs.hh"

namespace sc_dt
{

// classes defined in this module
class sc_fxnum_observer;
class sc_fxnum_fast_observer;

// forward class declarations
class sc_fxnum;
class sc_fxnum_fast;

#ifdef SC_ENABLE_OBSERVERS

#define SC_FXNUM_OBSERVER_CONSTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxnum_observer *, construct)
#define SC_FXNUM_OBSERVER_DESTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxnum_observer *, destruct)
#define SC_FXNUM_OBSERVER_READ_(object) \
    SC_OBSERVER_(object, sc_fxnum_observer *, read)
#define SC_FXNUM_OBSERVER_WRITE_(object) \
    SC_OBSERVER_(object, sc_fxnum_observer *, write)
#define SC_FXNUM_OBSERVER_DEFAULT_ \
    SC_OBSERVER_DEFAULT_(sc_fxnum_observer)

#define SC_FXNUM_FAST_OBSERVER_CONSTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxnum_fast_observer *, construct)
#define SC_FXNUM_FAST_OBSERVER_DESTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxnum_fast_observer *, destruct)
#define SC_FXNUM_FAST_OBSERVER_READ_(object) \
    SC_OBSERVER_(object, sc_fxnum_fast_observer *, read)
#define SC_FXNUM_FAST_OBSERVER_WRITE_(object) \
    SC_OBSERVER_(object, sc_fxnum_fast_observer *, write)
#define SC_FXNUM_FAST_OBSERVER_DEFAULT_ \
    SC_OBSERVER_DEFAULT_(sc_fxnum_fast_observer)

#else

#define SC_FXNUM_OBSERVER_CONSTRUCT_(object)
#define SC_FXNUM_OBSERVER_DESTRUCT_(object)
#define SC_FXNUM_OBSERVER_READ_(object)
#define SC_FXNUM_OBSERVER_WRITE_(object)
#define SC_FXNUM_OBSERVER_DEFAULT_

#define SC_FXNUM_FAST_OBSERVER_CONSTRUCT_(object)
#define SC_FXNUM_FAST_OBSERVER_DESTRUCT_(object)
#define SC_FXNUM_FAST_OBSERVER_READ_(object)
#define SC_FXNUM_FAST_OBSERVER_WRITE_(object)
#define SC_FXNUM_FAST_OBSERVER_DEFAULT_

#endif


// ----------------------------------------------------------------------------
//  CLASS : sc_fxnum_observer
//
//  Abstract base class for fixed-point types observers; arbitrary precision.
// ----------------------------------------------------------------------------

class sc_fxnum_observer
{
  protected:
    sc_fxnum_observer() {}
    virtual ~sc_fxnum_observer() {}

  public:
    virtual void construct(const sc_fxnum &);
    virtual void destruct(const sc_fxnum &);
    virtual void read(const sc_fxnum &);
    virtual void write(const sc_fxnum &);

    static sc_fxnum_observer *(* default_observer)();
};


// ----------------------------------------------------------------------------
//  CLASS : sc_fxnum_fast_observer
//
//  Abstract base class for fixed-point types observers; limited precision.
// ----------------------------------------------------------------------------

class sc_fxnum_fast_observer
{
  protected:
    sc_fxnum_fast_observer() {}
    virtual ~sc_fxnum_fast_observer() {}

  public:
    virtual void construct(const sc_fxnum_fast &);
    virtual void destruct(const sc_fxnum_fast &);
    virtual void read(const sc_fxnum_fast &);
    virtual void write(const sc_fxnum_fast &);

    static sc_fxnum_fast_observer *(* default_observer) ();
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
//  CLASS : sc_fxnum_observer
//
//  Abstract base class for fixed-point types observers; arbitrary precision.
// ----------------------------------------------------------------------------

inline void sc_fxnum_observer::construct(const sc_fxnum &) {}
inline void sc_fxnum_observer::destruct(const sc_fxnum &) {}
inline void sc_fxnum_observer::read(const sc_fxnum &) {}
inline void sc_fxnum_observer::write(const sc_fxnum &) {}


// ----------------------------------------------------------------------------
//  CLASS : sc_fxnum_fast_observer
//
//  Abstract base class for fixed-point types observers; limited precision.
// ----------------------------------------------------------------------------

inline void sc_fxnum_fast_observer::construct(const sc_fxnum_fast &) {}
inline void sc_fxnum_fast_observer::destruct(const sc_fxnum_fast &) {}
inline void sc_fxnum_fast_observer::read(const sc_fxnum_fast &) {}
inline void sc_fxnum_fast_observer::write(const sc_fxnum_fast &) {}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_FX_SC_FXNUM_OBSERVER_HH__
