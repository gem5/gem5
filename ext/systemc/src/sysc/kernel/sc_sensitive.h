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

  sc_sensitive.h -- Sensitivity classes. Requires "sc_process.h"
  for declarations of sc_method_handle, &.c.

  Original Author: Stan Y. Liao, Synopsys, Inc.
                   Martin Janssen, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_SENSITIVE_H
#define SC_SENSITIVE_H

#include "sysc/kernel/sc_process.h"

namespace sc_dt
{
    class sc_logic;
}

namespace sc_core {

class sc_process_handle;
class sc_event;
class sc_event_finder;
class sc_interface;
class sc_module;
class sc_port_base;
template <class T> class sc_in;
template <class T> class sc_inout;
template <class T> class sc_signal_in_if;


// ----------------------------------------------------------------------------
//  CLASS : sc_sensitive
//
//  Static sensitivity class for events.
// ----------------------------------------------------------------------------

class sc_sensitive
{
    friend class sc_module;

public:

    // typedefs
    typedef sc_signal_in_if<bool>            in_if_b_type;
    typedef sc_signal_in_if<sc_dt::sc_logic> in_if_l_type;
    typedef sc_in<bool>                      in_port_b_type;
    typedef sc_in<sc_dt::sc_logic>           in_port_l_type;
    typedef sc_inout<bool>                   inout_port_b_type;
    typedef sc_inout<sc_dt::sc_logic>        inout_port_l_type;

private:

    // constructor
    explicit sc_sensitive( sc_module* );

    // destructor
    ~sc_sensitive();

public:

    // changing between process handles
    sc_sensitive& operator << ( sc_process_handle );
#if 0
    sc_sensitive& operator << ( sc_method_handle );
    sc_sensitive& operator << ( sc_thread_handle );
#endif // 0

    sc_sensitive& operator () ( const sc_event& );
    sc_sensitive& operator () ( const sc_interface& );
    sc_sensitive& operator () ( const sc_port_base& );
    sc_sensitive& operator () ( sc_event_finder& );

    sc_sensitive& operator << ( const sc_event& );
    sc_sensitive& operator << ( const sc_interface& );
    sc_sensitive& operator << ( const sc_port_base& );
    sc_sensitive& operator << ( sc_event_finder& );

    sc_sensitive& operator () ( sc_cthread_handle, sc_event_finder& );
    sc_sensitive& operator () ( sc_cthread_handle, const in_if_b_type& );
    sc_sensitive& operator () ( sc_cthread_handle, const in_if_l_type& );
    sc_sensitive& operator () ( sc_cthread_handle, const in_port_b_type& );
    sc_sensitive& operator () ( sc_cthread_handle, const in_port_l_type& );
    sc_sensitive& operator () ( sc_cthread_handle, const inout_port_b_type& );
    sc_sensitive& operator () ( sc_cthread_handle, const inout_port_l_type& );

    static void make_static_sensitivity( sc_process_b*, const sc_event& );
    static void make_static_sensitivity( sc_process_b*, const sc_interface& );
    static void make_static_sensitivity( sc_process_b*, const sc_port_base&);
    static void make_static_sensitivity( sc_process_b*, sc_event_finder& );

    void reset();

private:

    sc_module*                                m_module;
    enum { SC_NONE_, SC_METHOD_, SC_THREAD_ } m_mode;
    sc_process_b*                          m_handle;

private:

    // disabled

    sc_sensitive();
    sc_sensitive( const sc_sensitive& );
    sc_sensitive& operator = ( const sc_sensitive& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_sensitive_pos
//
//  Static sensitivity class for positive edge events.
// ----------------------------------------------------------------------------

class sc_sensitive_pos
{
    friend class sc_module;

public:

    // typedefs
    typedef sc_signal_in_if<bool>            in_if_b_type;
    typedef sc_signal_in_if<sc_dt::sc_logic> in_if_l_type;
    typedef sc_in<bool>                      in_port_b_type;
    typedef sc_in<sc_dt::sc_logic>           in_port_l_type;
    typedef sc_inout<bool>                   inout_port_b_type;
    typedef sc_inout<sc_dt::sc_logic>        inout_port_l_type;

private:

    // constructor
    explicit sc_sensitive_pos( sc_module* );

    // destructor
    ~sc_sensitive_pos();

public:

    // changing between process handles
    sc_sensitive_pos& operator << ( sc_process_handle );
    sc_sensitive_pos& operator << ( sc_method_handle );
    sc_sensitive_pos& operator << ( sc_thread_handle );

    sc_sensitive_pos& operator () ( const in_if_b_type& );
    sc_sensitive_pos& operator () ( const in_if_l_type& );
    sc_sensitive_pos& operator () ( const in_port_b_type& );
    sc_sensitive_pos& operator () ( const in_port_l_type& );
    sc_sensitive_pos& operator () ( const inout_port_b_type& );
    sc_sensitive_pos& operator () ( const inout_port_l_type& );

    sc_sensitive_pos& operator << ( const in_if_b_type& );
    sc_sensitive_pos& operator << ( const in_if_l_type& );
    sc_sensitive_pos& operator << ( const in_port_b_type& );
    sc_sensitive_pos& operator << ( const in_port_l_type& );
    sc_sensitive_pos& operator << ( const inout_port_b_type& );
    sc_sensitive_pos& operator << ( const inout_port_l_type& );

    void reset();

private:

    sc_module*                                m_module;
    enum { SC_NONE_, SC_METHOD_, SC_THREAD_ } m_mode;
    sc_process_b*                          m_handle;

private:

    // disabled
    sc_sensitive_pos();
    sc_sensitive_pos( const sc_sensitive_pos& );
    sc_sensitive_pos& operator = ( const sc_sensitive_pos& );
};


// ----------------------------------------------------------------------------
//  CLASS : sc_sensitive_neg
//
//  Static sensitivity class for negative edge events.
// ----------------------------------------------------------------------------

class sc_sensitive_neg
{
    friend class sc_module;

public:

    // typedefs
    typedef sc_signal_in_if<bool>            in_if_b_type;
    typedef sc_signal_in_if<sc_dt::sc_logic> in_if_l_type;
    typedef sc_in<bool>                      in_port_b_type;
    typedef sc_in<sc_dt::sc_logic>           in_port_l_type;
    typedef sc_inout<bool>                   inout_port_b_type;
    typedef sc_inout<sc_dt::sc_logic>        inout_port_l_type;

private:

    // constructor
    explicit sc_sensitive_neg( sc_module* );

    // destructor
    ~sc_sensitive_neg();

public:

    // changing between process handles
    sc_sensitive_neg& operator << ( sc_process_handle );
    sc_sensitive_neg& operator << ( sc_method_handle );
    sc_sensitive_neg& operator << ( sc_thread_handle );

    sc_sensitive_neg& operator () ( const in_if_b_type& );
    sc_sensitive_neg& operator () ( const in_if_l_type& );
    sc_sensitive_neg& operator () ( const in_port_b_type& );
    sc_sensitive_neg& operator () ( const in_port_l_type& );
    sc_sensitive_neg& operator () ( const inout_port_b_type& );
    sc_sensitive_neg& operator () ( const inout_port_l_type& );

    sc_sensitive_neg& operator << ( const in_if_b_type& );
    sc_sensitive_neg& operator << ( const in_if_l_type& );
    sc_sensitive_neg& operator << ( const in_port_b_type& );
    sc_sensitive_neg& operator << ( const in_port_l_type& );
    sc_sensitive_neg& operator << ( const inout_port_b_type& );
    sc_sensitive_neg& operator << ( const inout_port_l_type& );

    void reset();

private:

    sc_module*                                m_module;
    enum { SC_NONE_, SC_METHOD_, SC_THREAD_ } m_mode;
    sc_process_b*                          m_handle;

private:

    // disabled
    sc_sensitive_neg();
    sc_sensitive_neg( const sc_sensitive_neg& );
    sc_sensitive_neg& operator = ( const sc_sensitive_neg& );
};

} // namespace sc_core 

#endif

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Bishnupriya Bhattacharya, Cadence Design Systems,
                               25 August, 2003
  Description of Modification: Add make_static_sensitivity() methods to enable
                               dynamic method process creation with static 
                               sensitivity.

 *****************************************************************************/

// $Log: sc_sensitive.h,v $
// Revision 1.5  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.3  2011/02/13 21:47:38  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/04/11 23:13:21  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.

// Taf!
