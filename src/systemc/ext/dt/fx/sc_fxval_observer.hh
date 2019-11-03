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

  sc_fxval_observer.h -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_fxval_observer.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_FX_SC_FXVAL_OBSERVER_HH__
#define __SYSTEMC_EXT_DT_FX_SC_FXVAL_OBSERVER_HH__

#include "sc_fxdefs.hh"

namespace sc_dt
{

// classes defined in this module
class sc_fxval_observer;
class sc_fxval_fast_observer;

// forward class declarations
class sc_fxval;
class sc_fxval_fast;

#ifdef SC_ENABLE_OBSERVERS

#define SC_FXVAL_OBSERVER_CONSTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxval_observer *, construct)
#define SC_FXVAL_OBSERVER_DESTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxval_observer *, destruct)
#define SC_FXVAL_OBSERVER_READ_(object) \
    SC_OBSERVER_(object, sc_fxval_observer *, read)
#define SC_FXVAL_OBSERVER_WRITE_(object) \
    SC_OBSERVER_(object, sc_fxval_observer *, write)
#define SC_FXVAL_OBSERVER_DEFAULT_ \
    SC_OBSERVER_DEFAULT_(sc_fxval_observer)

#define SC_FXVAL_FAST_OBSERVER_CONSTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxval_fast_observer *, construct)
#define SC_FXVAL_FAST_OBSERVER_DESTRUCT_(object) \
    SC_OBSERVER_(object, sc_fxval_fast_observer *, destruct)
#define SC_FXVAL_FAST_OBSERVER_READ_(object) \
    SC_OBSERVER_(object, sc_fxval_fast_observer *, read)
#define SC_FXVAL_FAST_OBSERVER_WRITE_(object) \
    SC_OBSERVER_(object, sc_fxval_fast_observer *, write)
#define SC_FXVAL_FAST_OBSERVER_DEFAULT_ \
    SC_OBSERVER_DEFAULT_(sc_fxval_fast_observer)

#else

#define SC_FXVAL_OBSERVER_CONSTRUCT_(object)
#define SC_FXVAL_OBSERVER_DESTRUCT_(object)
#define SC_FXVAL_OBSERVER_READ_(object)
#define SC_FXVAL_OBSERVER_WRITE_(object)
#define SC_FXVAL_OBSERVER_DEFAULT_

#define SC_FXVAL_FAST_OBSERVER_CONSTRUCT_(object)
#define SC_FXVAL_FAST_OBSERVER_DESTRUCT_(object)
#define SC_FXVAL_FAST_OBSERVER_READ_(object)
#define SC_FXVAL_FAST_OBSERVER_WRITE_(object)
#define SC_FXVAL_FAST_OBSERVER_DEFAULT_

#endif


// ----------------------------------------------------------------------------
//  CLASS : sc_fxval_observer
//
//  Abstract base class for fixed-point value type observers;
//  arbitrary precision.
// ----------------------------------------------------------------------------

class sc_fxval_observer
{
  protected:
    sc_fxval_observer() {}
    virtual ~sc_fxval_observer() {}
  public:
    virtual void construct(const sc_fxval &);
    virtual void destruct(const sc_fxval &);
    virtual void read(const sc_fxval &);
    virtual void write(const sc_fxval &);

    static sc_fxval_observer *(*default_observer) ();
};


// ----------------------------------------------------------------------------
//  CLASS : sc_fxval_fast_observer
//
//  Abstract base class for fixed-point value type observers;
//  limited precision.
// ----------------------------------------------------------------------------

class sc_fxval_fast_observer
{
  protected:
    sc_fxval_fast_observer() {}
    virtual ~sc_fxval_fast_observer() {}

  public:
    virtual void construct(const sc_fxval_fast &);
    virtual void destruct(const sc_fxval_fast &);
    virtual void read(const sc_fxval_fast &);
    virtual void write(const sc_fxval_fast &);

    static sc_fxval_fast_observer *(*default_observer) ();
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
//  CLASS : sc_fxval_observer
//
//  Abstract base class for fixed-point value type observers;
//  arbitrary precision.
// ----------------------------------------------------------------------------

inline void sc_fxval_observer::construct(const sc_fxval &) {}
inline void sc_fxval_observer::destruct(const sc_fxval &) {}
inline void sc_fxval_observer::read(const sc_fxval &) {}
inline void sc_fxval_observer::write(const sc_fxval &) {}


// ----------------------------------------------------------------------------
//  CLASS : sc_fxval_fast_observer
//
//  Abstract base class for fixed-point value type observers;
//  limited precision.
// ----------------------------------------------------------------------------

inline void sc_fxval_fast_observer::construct(const sc_fxval_fast &) {}
inline void sc_fxval_fast_observer::destruct(const sc_fxval_fast &) {}
inline void sc_fxval_fast_observer::read(const sc_fxval_fast &) {}
inline void sc_fxval_fast_observer::write(const sc_fxval_fast &) {}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_FX_SC_FXVAL_OBSERVER_HH__
