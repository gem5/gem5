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

  sc_context.h -

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_context.h,v $
// Revision 1.2  2011/08/24 22:05:43  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.5  2006/05/26 20:36:52  acg
//  Andy Goodrich: added a using for sc_core::default_ptr_hash_fn to keep HP
//  aCC happy.
//
// Revision 1.4  2006/03/21 00:00:31  acg
//   Andy Goodrich: changed name of sc_get_current_process_base() to be
//   sc_get_current_process_b() since its returning an sc_process_b instance.
//
// Revision 1.3  2006/01/13 18:53:57  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_FX_SC_CONTEXT_HH__
#define __SYSTEMC_EXT_DT_FX_SC_CONTEXT_HH__

#include <map>

#include "../../core/sc_process_handle.hh"
#include "../../utils/sc_report_handler.hh"
#include "messages.hh"

namespace sc_dt
{

// classes defined in this module
class sc_without_context;
template <class T>
class sc_global;
template <class T>
class sc_context;


// ----------------------------------------------------------------------------
//  CLASS : sc_without_context
//
//  Empty class that is used for its type only.
// ----------------------------------------------------------------------------

class sc_without_context {};


// ----------------------------------------------------------------------------
//  TEMPLATE CLASS : sc_global
//
//  Template global variable class; singleton; co-routine safe.
// ----------------------------------------------------------------------------

template <class T>
class sc_global
{
    sc_global();
    void update();

  public:
    static sc_global<T>* instance();
    const T*& value_ptr();

  private:
    static sc_global<T> *m_instance;

    std::map<void *, const T *> m_map;
    void *m_proc; // context (current process or NULL)
    const T *m_value_ptr;
};


// ----------------------------------------------------------------------------
//  ENUM : sc_context_begin
//
//  Enumeration of context begin options.
// ----------------------------------------------------------------------------

enum sc_context_begin
{
    SC_NOW,
    SC_LATER
};


// ----------------------------------------------------------------------------
//  CLASS : sc_context
//
//  Template context class; co-routine safe.
// ----------------------------------------------------------------------------

template <class T>
class sc_context
{
    // disabled
    sc_context(const sc_context<T> &);
    void *operator new(std::size_t);

  public:
    explicit sc_context(const T &, sc_context_begin=SC_NOW);
    ~sc_context();

    void begin();
    void end();

    static const T &default_value();
    const T &value() const;

  private:
    sc_context &operator = (const sc_context &) /* = delete */;

    const T m_value;
    const T *&m_def_value_ptr;
    const T *m_old_value_ptr;
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
//  TEMPLATE CLASS : sc_global
//
//  Template global variable class; singleton; co-routine safe.
// ----------------------------------------------------------------------------

template <class T>
sc_global<T> *sc_global<T>::m_instance = 0;

template <class T>
inline sc_global<T>::sc_global() : m_map(),
    // use &m_instance as unique "non-process" key (NULL denotes 'sc_main'
    // context)
    m_proc(&m_instance), m_value_ptr(0)
{}


template <class T>
inline void
sc_global<T>::update()
{
    void *p = (::sc_gem5::Process *)sc_core::sc_get_current_process_handle();
    if (p != m_proc) {
        const T *vp = m_map[p];
        if (vp == 0) {
            vp = new T(sc_without_context());
            m_map.emplace(p, vp);
        }
        m_proc = p;
        m_value_ptr = vp;
    }
}


template <class T>
inline sc_global<T> *
sc_global<T>::instance()
{
    if (m_instance == 0) {
        m_instance = new sc_global<T>;
    }
    return m_instance;
}


template <class T>
inline const T *&
sc_global<T>::value_ptr()
{
    update();
    return m_value_ptr;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_context
//
//  Template context class; co-routine safe.
// ----------------------------------------------------------------------------

template <class T>
inline sc_context<T>::sc_context(const T &value_, sc_context_begin begin_) :
        m_value(value_),
        m_def_value_ptr(sc_global<T>::instance()->value_ptr()),
        m_old_value_ptr(0)
{
    if (begin_ == SC_NOW) {
        m_old_value_ptr = m_def_value_ptr;
        m_def_value_ptr = &m_value;
    }
}

template <class T>
inline sc_context<T>::~sc_context()
{
    if (m_old_value_ptr != 0) {
        m_def_value_ptr = m_old_value_ptr;
        m_old_value_ptr = 0;
    }
}


template <class T>
inline void
sc_context<T>::begin()
{
    if (m_old_value_ptr == 0) {
        m_old_value_ptr = m_def_value_ptr;
        m_def_value_ptr = &m_value;
    } else {
        SC_REPORT_ERROR(sc_core::SC_ID_CONTEXT_BEGIN_FAILED_, 0);
    }
}

template <class T>
inline void
sc_context<T>::end()
{
    if (m_old_value_ptr != 0) {
        m_def_value_ptr = m_old_value_ptr;
        m_old_value_ptr = 0;
    } else {
        SC_REPORT_ERROR(sc_core::SC_ID_CONTEXT_END_FAILED_, 0);
    }
}


template <class T>
inline const T &
sc_context<T>::default_value()
{
    return *sc_global<T>::instance()->value_ptr();
}

template <class T>
inline const T &
sc_context<T>::value() const
{
    return m_value;
}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_FX_SC_CONTEXT_HH__
