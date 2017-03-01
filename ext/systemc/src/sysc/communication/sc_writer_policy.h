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

  sc_writer_policy.h -- The sc_signal<T> writer policy definition

  Original Author: Philipp A: Hartmann, OFFIS

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_WRITER_POLICY_H_INCLUDED_
#define SC_WRITER_POLICY_H_INCLUDED_

#if !defined(SC_DEFAULT_WRITER_POLICY)
#  if defined(SC_NO_WRITE_CHECK)
#    define SC_DEFAULT_WRITER_POLICY SC_UNCHECKED_WRITERS
#  else
#    define SC_DEFAULT_WRITER_POLICY SC_ONE_WRITER
#  endif
#endif

namespace sc_core {

class sc_object;
class sc_port_base;
extern
void
sc_signal_invalid_writer( sc_object* target, sc_object* first_writer,
                          sc_object* second_writer, bool check_delta );

// SIGNAL WRITING POLICIES
//
// Note: if you add a new policy to the enum below you will need to add
// an additional overload of sc_reset::reset_signal_is() for the sc_signal<bool>
// instance. That will require changes to sysc/kernel/sc_reset.cpp and 
// sysc/kernel/sc_reset.h

enum sc_writer_policy
{
  SC_ONE_WRITER        = 0, ///< unique writer (from a unique port)
  SC_MANY_WRITERS      = 1, ///< allow multiple writers (with different ports)
  SC_UNCHECKED_WRITERS = 3  ///< even allow delta cycle conflicts (non-standard)
};

// signal forward declaration
template< typename T, sc_writer_policy POL = SC_DEFAULT_WRITER_POLICY >
class sc_signal;

template< sc_writer_policy >
struct sc_writer_policy_check;

struct sc_writer_policy_nocheck_write
{
  bool check_write( sc_object* /* target */, bool /* value_changed */ )
    { return true; }
  void update(){}
};

struct sc_writer_policy_check_write
{
  bool check_write( sc_object* target, bool value_changed );
  void update(){}
protected:
  sc_writer_policy_check_write( bool check_delta = false )
    : m_check_delta( check_delta ), m_writer_p(NULL) {}
  const bool         m_check_delta;
  sc_object*         m_writer_p;
};

struct sc_writer_policy_check_delta
    : sc_writer_policy_check_write
{

  sc_writer_policy_check_delta()
    : sc_writer_policy_check_write(true) {}

  bool check_write( sc_object* target, bool value_changed )
  {
      if( value_changed )
          return sc_writer_policy_check_write::check_write( target, true );
      return true;
  }

  void update(){ m_writer_p = NULL; }
};

struct sc_writer_policy_nocheck_port
{
  bool check_port( sc_object*, sc_port_base*, bool )
    { return true; }
};

struct sc_writer_policy_check_port
{
  bool check_port( sc_object* target, sc_port_base* port, bool is_output );

protected:
  sc_writer_policy_check_port() : m_output(0) {}
  sc_port_base* m_output;
};

template<>
struct sc_writer_policy_check<SC_ONE_WRITER>
  : sc_writer_policy_check_port
  , sc_writer_policy_check_write
{};

template<>
struct sc_writer_policy_check<SC_MANY_WRITERS>
  : sc_writer_policy_nocheck_port
  , sc_writer_policy_check_delta
{};

template<>
struct sc_writer_policy_check<SC_UNCHECKED_WRITERS>
  : sc_writer_policy_nocheck_port
  , sc_writer_policy_nocheck_write
{};

} // namespace sc_core

#endif

// Taf!
