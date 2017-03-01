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

  sc_wif_trace.h - Implementation of WIF tracing.

  Original Author - Abhijit Ghosh, Synopsys, Inc.

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
   code is somewhat different, and significantly enhanced, the basics
   are identical to what was originally contributed by Infineon.
   The contribution of Infineon in the development of this tracing
   technology is hereby acknowledged.

 *****************************************************************************/

/*****************************************************************************

   Instead of creating the binary WIF format, we create the ASCII
   WIF format which can be converted to the binary format using
   a2wif (utility that comes with VSS from Synopsys). This way, 
   a user who does not have Synopsys VSS can still create WIF 
   files, but the files can only be viewed by users who have VSS.

 *****************************************************************************/

#ifndef SC_WIF_TRACE_H
#define SC_WIF_TRACE_H

#include <cstdio>
#include "sysc/datatypes/int/sc_nbdefs.h"
#include "sysc/tracing/sc_trace_file_base.h"

namespace sc_core {

class wif_trace;  // defined in wif_trace.cc
template<class T> class wif_T_trace;

class wif_trace_file
  : public sc_trace_file_base
{
public:
    enum wif_enum {WIF_BIT=0, WIF_MVL=1, WIF_REAL=2, WIF_LAST};

    // Create a wif trace file.
    // `Name' forms the base of the name to which `.awif' is added.
    explicit wif_trace_file(const char *name);

    ~wif_trace_file();

protected:
    // These are all virtual functions in sc_trace_file and
    // they need to be defined here.

    // Trace a boolean object (single bit)
     void trace(const bool& object, const std::string& name);

    // Trace a sc_bit object (single bit)
     void trace(const sc_dt::sc_bit& object, const std::string& name);

    // Trace a sc_logic object (single bit)
     void trace(const sc_dt::sc_logic& object, const std::string& name);
    
    // Trace an unsigned char with the given width
     void trace(const unsigned char& object, const std::string& name, 
	int width);

    // Trace an unsigned short with the given width
     void trace(const unsigned short& object, const std::string& name, 
         int width);

    // Trace an unsigned int with the given width
     void trace(const unsigned int& object, const std::string& name, 
         int width);

    // Trace an unsigned long with the given width
     void trace(const unsigned long& object, const std::string& name, 
         int width);

    // Trace a signed char with the given width
     void trace(const char& object, const std::string& name, int width);

    // Trace a signed short with the given width
     void trace(const short& object, const std::string& name, int width);

    // Trace a signed int with the given width
     void trace(const int& object, const std::string& name, int width);

    // Trace a signed long with the given width
     void trace(const long& object, const std::string& name, int width);
    
    // Trace a signed long long with the given width
     void trace(const sc_dt::int64& object, const std::string& name, 
         int width);
    
    // Trace an usigned long long with the given width
     void trace(const sc_dt::uint64& object, const std::string& name, 
         int width);
    
    // Trace a float
     void trace(const float& object, const std::string& name);

    // Trace a double
     void trace(const double& object, const std::string& name);

    // Trace sc_unsigned
     void trace (const sc_dt::sc_unsigned& object, 
	 const std::string& name);

    // Trace sc_signed
     void trace (const sc_dt::sc_signed& object, 
	 const std::string& name);

    // Trace sc_uint_base
     void trace (const sc_dt::sc_uint_base& object, 
         const std::string& name);

    // Trace sc_int_base
     void trace (const sc_dt::sc_int_base& object, const std::string& name);

    // Trace sc_fxval
    void trace( const sc_dt::sc_fxval& object, const std::string& name );

    // Trace sc_fxval_fast
    void trace( const sc_dt::sc_fxval_fast& object, 
    	const std::string& name );

    // Trace sc_fxnum
    void trace( const sc_dt::sc_fxnum& object, const std::string& name );

    // Trace sc_fxnum_fast
    void trace( const sc_dt::sc_fxnum_fast& object, 
    	const std::string& name );

    template<class T>
    void traceT(const T& object, const std::string& name, wif_enum type)
    {
        if( add_trace_check(name) )
            traces.push_back( new wif_T_trace<T>( object, name
                                                , obtain_name(),type ) );
    }

    // Trace sc_bv_base (sc_bv)
    virtual void trace( const sc_dt::sc_bv_base& object, 
    	const std::string& name );

    // Trace sc_lv_base (sc_lv)
    virtual void trace( const sc_dt::sc_lv_base& object, 
    	const std::string& name );

    // Trace an enumerated object - where possible output the enumeration literals
    // in the trace file. Enum literals is a null terminated array of null
    // terminated char* literal strings.
     void trace(const unsigned& object, const std::string& name, 
         const char** enum_literals);

    // Output a comment to the trace file
     void write_comment(const std::string& comment);

    // Write trace info for cycle.
     void cycle(bool delta_cycle);

private:

#if SC_TRACING_PHASE_CALLBACKS_
    // avoid hidden overload warnings
    virtual void trace( sc_trace_file* ) const { sc_assert(false); }
#endif // SC_TRACING_PHASE_CALLBACKS_

    // Initialize the tracing mechanism
    virtual void do_initialize();

    unsigned wif_name_index;           // Number of variables traced

    unsigned previous_time_units_low;  // Previous time as 64 bit integer
    unsigned previous_time_units_high;
    double   previous_time;            // Previous time as a double

public:
    // Create wif names for each variable
    std::string obtain_name();

    // Array to store the variables traced
    std::vector<wif_trace*> traces;
};

} //  namespace sc_core

#endif // SC_WIF_TRACE_H
// Taf!
