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

  sc_trace.h - Functions for tracing signals and variables.

  Author: Abhijit Ghosh, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*****************************************************************************

   Acknowledgement: The tracing mechanism is based on the tracing
   mechanism developed at Infineon (formerly Siemens HL). Though this
   code is somewhat different, the basics are identical to what was
   originally contributed by Infineon.  The contribution of Infineon
   in the development of this tracing technology is hereby
   acknowledged.

 *****************************************************************************/

#ifndef SC_TRACE_H
#define SC_TRACE_H

#include <cstdio>

#include "sysc/datatypes/int/sc_nbdefs.h"
#include "sysc/kernel/sc_time.h"

// Some forward declarations
namespace sc_dt
{
    class sc_bit;
    class sc_logic;
    class sc_bv_base;
    class sc_lv_base;
    class sc_signed;
    class sc_unsigned;
    class sc_int_base;
    class sc_uint_base;
    class sc_fxval;
    class sc_fxval_fast;
    class sc_fxnum;
    class sc_fxnum_fast;
}

namespace sc_core {

class sc_time;

template <class T> class sc_signal_in_if;


// Base class for all kinds of trace files. 

class sc_trace_file
{
    friend class sc_simcontext;
    
public:

    // Constructor
    sc_trace_file();
    
    // All functions are pure virtual because they need to be defined by the
    // particular tracing mechanism


#define DECL_TRACE_METHOD_A(tp)                                               \
    virtual void trace( const tp& object,                                     \
			const std::string& name ) = 0;

#define DECL_TRACE_METHOD_B(tp)                                               \
    virtual void trace( const tp& object,                                     \
			const std::string& name,                                     \
			int width ) = 0;


    DECL_TRACE_METHOD_A( bool )
    DECL_TRACE_METHOD_A( sc_dt::sc_bit )
    DECL_TRACE_METHOD_A( sc_dt::sc_logic )

    DECL_TRACE_METHOD_B( unsigned char )
    DECL_TRACE_METHOD_B( unsigned short )
    DECL_TRACE_METHOD_B( unsigned int )
    DECL_TRACE_METHOD_B( unsigned long )
    DECL_TRACE_METHOD_B( char )
    DECL_TRACE_METHOD_B( short )
    DECL_TRACE_METHOD_B( int )
    DECL_TRACE_METHOD_B( long )
    DECL_TRACE_METHOD_B( sc_dt::int64 )
    DECL_TRACE_METHOD_B( sc_dt::uint64 )

    DECL_TRACE_METHOD_A( float )
    DECL_TRACE_METHOD_A( double )
    DECL_TRACE_METHOD_A( sc_dt::sc_int_base )
    DECL_TRACE_METHOD_A( sc_dt::sc_uint_base )
    DECL_TRACE_METHOD_A( sc_dt::sc_signed )
    DECL_TRACE_METHOD_A( sc_dt::sc_unsigned )

    DECL_TRACE_METHOD_A( sc_dt::sc_fxval )
    DECL_TRACE_METHOD_A( sc_dt::sc_fxval_fast )
    DECL_TRACE_METHOD_A( sc_dt::sc_fxnum )
    DECL_TRACE_METHOD_A( sc_dt::sc_fxnum_fast )

    DECL_TRACE_METHOD_A( sc_dt::sc_bv_base )
    DECL_TRACE_METHOD_A( sc_dt::sc_lv_base )


#undef DECL_TRACE_METHOD_A
#undef DECL_TRACE_METHOD_B

    // Trace an enumerated object - where possible output the enumeration
    // literals in the trace file. Enum literals is a null terminated array
    // of null terminated char* literal strings.
    virtual void trace( const unsigned int& object,
			const std::string& name,
			const char** enum_literals ) = 0;

    // Output a comment to the trace file
    virtual void write_comment( const std::string& comment ) = 0;

    // Set the amount of space before next column
    // (For most formats this does nothing)
    virtual void space( int n );

    // Also trace transitions between delta cycles if flag is true.
    virtual void delta_cycles( bool flag );

    // Set time unit.
    virtual void set_time_unit( double v, sc_time_unit tu )=0;

protected:

    // Write trace info for cycle
    virtual void cycle( bool delta_cycle ) = 0;

    // Flush results and close file
    virtual ~sc_trace_file()
	{ /* Intentionally blank */ }
};

/*****************************************************************************/

// Now comes all the SystemC defined tracing functions.
// We define two sc_trace() versions for scalar types; one where the object to
// be traced is passed as a reference and the other where a pointer to the
// tracing object is passed.

#define DECL_TRACE_FUNC_REF_A(tp)     \
void                                  \
sc_trace( sc_trace_file* tf,          \
	  const tp& object,               \
	  const std::string& name );

#define DECL_TRACE_FUNC_PTR_A(tp)     \
void                                  \
sc_trace( sc_trace_file* tf,          \
	  const tp* object,               \
	  const std::string& name );        \

#define DECL_TRACE_FUNC_A(tp)         \
DECL_TRACE_FUNC_REF_A(tp)             \
DECL_TRACE_FUNC_PTR_A(tp)


DECL_TRACE_FUNC_A( sc_dt::sc_bit )
DECL_TRACE_FUNC_A( sc_dt::sc_logic )

DECL_TRACE_FUNC_A( sc_dt::sc_int_base )
DECL_TRACE_FUNC_A( sc_dt::sc_uint_base )
DECL_TRACE_FUNC_A( sc_dt::sc_signed )
DECL_TRACE_FUNC_A( sc_dt::sc_unsigned )

DECL_TRACE_FUNC_REF_A( sc_dt::sc_bv_base )
DECL_TRACE_FUNC_REF_A( sc_dt::sc_lv_base )


#undef DECL_TRACE_FUNC_REF_A
#undef DECL_TRACE_FUNC_PTR_A
#undef DECL_TRACE_FUNC_A


// ----------------------------------------------------------------------------

#define DEFN_TRACE_FUNC_REF_A(tp)                                             \
inline                                                                        \
void                                                                          \
sc_trace( sc_trace_file* tf, const tp& object, const std::string& name ) \
{                                                                             \
    if( tf ) {                                                                \
	tf->trace( object, name );                                            \
    }                                                                         \
}

#define DEFN_TRACE_FUNC_PTR_A(tp)                                             \
inline                                                                        \
void                                                                          \
sc_trace( sc_trace_file* tf, const tp* object, const std::string& name ) \
{                                                                             \
    if( tf ) {                                                                \
	tf->trace( *object, name );                                           \
    }                                                                         \
}

#define DEFN_TRACE_FUNC_A(tp)                                                 \
DEFN_TRACE_FUNC_REF_A(tp)                                                     \
DEFN_TRACE_FUNC_PTR_A(tp)


#define DEFN_TRACE_FUNC_REF_B(tp)                                             \
inline                                                                        \
void                                                                          \
sc_trace( sc_trace_file* tf, const tp& object, const std::string& name,  \
          int width = 8 * sizeof( tp ) )                                      \
{                                                                             \
    if( tf ) {                                                                \
	tf->trace( object, name, width );                                     \
    }                                                                         \
}

#define DEFN_TRACE_FUNC_PTR_B(tp)                                             \
inline                                                                        \
void                                                                          \
sc_trace( sc_trace_file* tf, const tp* object, const std::string& name,  \
          int width = 8 * sizeof( tp ) )                                      \
{                                                                             \
    if( tf ) {                                                                \
	tf->trace( *object, name, width );                                    \
    }                                                                         \
}


#define DEFN_TRACE_FUNC_B(tp)                                                 \
DEFN_TRACE_FUNC_REF_B(tp)                                                     \
DEFN_TRACE_FUNC_PTR_B(tp)


DEFN_TRACE_FUNC_A( bool )
DEFN_TRACE_FUNC_A( float )
DEFN_TRACE_FUNC_A( double )

DEFN_TRACE_FUNC_B( unsigned char )
DEFN_TRACE_FUNC_B( unsigned short )
DEFN_TRACE_FUNC_B( unsigned int )
DEFN_TRACE_FUNC_B( unsigned long )
DEFN_TRACE_FUNC_B( char )
DEFN_TRACE_FUNC_B( short )
DEFN_TRACE_FUNC_B( int )
DEFN_TRACE_FUNC_B( long )
DEFN_TRACE_FUNC_B( sc_dt::int64 )
DEFN_TRACE_FUNC_B( sc_dt::uint64 )


#undef DEFN_TRACE_FUNC_REF_A
#undef DEFN_TRACE_FUNC_PTR_A
#undef DEFN_TRACE_FUNC_A

#undef DEFN_TRACE_FUNC_REF_B
#undef DEFN_TRACE_FUNC_PTR_B
#undef DEFN_TRACE_FUNC_B


template <class T> 
inline
void
sc_trace( sc_trace_file* tf,
	  const sc_signal_in_if<T>& object,
	  const std::string& name )
{
    sc_trace( tf, object.read(), name );
}

template< class T >
inline
void
sc_trace( sc_trace_file* tf,
	  const sc_signal_in_if<T>& object,
	  const char* name )
{
    sc_trace( tf, object.read(), name );
}


// specializations for signals of type char, short, int, long

void sc_trace( sc_trace_file* tf,
	       const sc_signal_in_if<char>& object,
	       const std::string& name,
	       int width );

void sc_trace( sc_trace_file* tf,
	       const sc_signal_in_if<short>& object,
	       const std::string& name,
	       int width );

void sc_trace( sc_trace_file* tf,
	       const sc_signal_in_if<int>& object,
	       const std::string& name,
	       int width );

void sc_trace( sc_trace_file* tf,
	       const sc_signal_in_if<long>& object,
	       const std::string& name,
	       int width );


// 1. non-template function is better than template
// 2. more-specialized template is better than less-specialized
// 3. no partial specialization for template functions


// Trace an enumerated object - where possible output the enumeration literals
// in the trace file. Enum literals is a null terminated array of null
// terminated char* literal strings.

void
sc_trace( sc_trace_file* tf,
	  const unsigned int& object,
	  const std::string& name,
	  const char** enum_literals );


// Dummy function for arbitrary types of value, does nothing

extern void sc_trace( sc_trace_file* tf,
		      const void* object,
		      const std::string& name );


// Turn on/off delta cycle tracing on trace file `tf'.
// Default is to turn on delta cycle tracing.

inline
void
sc_trace_delta_cycles( sc_trace_file* tf, bool on = true )
{
    if( tf ) tf->delta_cycles( on );
}


// Output a comment to the trace file

inline
void
sc_write_comment( sc_trace_file* tf, const std::string& comment )
{
    if( tf ) tf->write_comment( comment );
}


// Equivalent of std::fprintf for trace files!

void tprintf( sc_trace_file* tf,  const char* format, ... );

// ----------------------------------------------------------------------------
// Create VCD file
extern sc_trace_file *sc_create_vcd_trace_file(const char* name);
extern void sc_close_vcd_trace_file( sc_trace_file* tf );


// ----------------------------------------------------------------------------
// Create WIF file
extern sc_trace_file *sc_create_wif_trace_file(const char *name);
extern void sc_close_wif_trace_file( sc_trace_file* tf );

} // namespace sc_core

#endif // SC_TRACE_H
// Taf
