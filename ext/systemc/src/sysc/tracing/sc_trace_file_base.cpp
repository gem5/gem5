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

  sc_trace_file_base.cpp - Shared internal tracing implementation

  Original Author: Philipp A. Hartmann, OFFIS, 2013-11-15

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

/*****************************************************************************

   Acknowledgement: The tracing mechanism is based on the tracing
   mechanism developed at Infineon (formerly Siemens HL). Though this
   code is somewhat different, and significantly enhanced, the basics
   are identical to what was originally contributed by Infineon.  The
   contribution of Infineon in the development of this tracing
   technology is hereby acknowledged.

 *****************************************************************************/

#include <ctime>

#include "sysc/tracing/sc_trace_file_base.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_simcontext_int.h"

#if SC_TRACING_PHASE_CALLBACKS_
#  include "sysc/kernel/sc_object_int.h"
#endif

namespace sc_core {

bool sc_trace_file_base::tracing_initialized_ = false;


sc_trace_file_base::sc_trace_file_base( const char* name, const char* extension )
  : sc_trace_file()
#if SC_TRACING_PHASE_CALLBACKS_
  , sc_object( sc_gen_unique_name("$$$$kernel_tracefile$$$$") )
#endif
  , fp(0)
  , timescale_unit()
  , timescale_set_by_user(false)
  , filename_() 
  , initialized_(false)
  , trace_delta_cycles_(false)
{
    if( !name || !*name ) {
        SC_REPORT_ERROR( SC_ID_TRACING_FOPEN_FAILED_, "no name given" );
        return;
    } else {
        std::stringstream ss;
        ss << name << "." << extension;
        ss.str().swap( filename_ );
    }

#if SC_TRACING_PHASE_CALLBACKS_ == 1
    // remove from hierarchy
    sc_object::detach();
    // register regular (non-delta) callbacks
    sc_object::register_simulation_phase_callback(
        // Note: Usually, one would expect to dump the initial values
        //       of the traced variables at the end of the initialization
        //       phase.  The "non-callback" implementation dumps those
        //       values only after the first delta cycle, though.
        // SC_END_OF_INITIALIZATION |
        SC_BEFORE_TIMESTEP |
        SC_PAUSED | SC_STOPPED
    );
#else // explicitly register with simcontext
    sc_get_curr_simcontext()->add_trace_file( this );
#endif
}

sc_trace_file_base::~sc_trace_file_base()
{
    if( fp )
        fclose(fp);

#if SC_TRACING_PHASE_CALLBACKS_ == 0
    // unregister from simcontext
    sc_get_curr_simcontext()->remove_trace_file( this );
#endif
}

/*****************************************************************************/
// simulation phase callback based trigger
//
//  The tracing updates are triggered
//    (- at the end of the initialization phase [disabled for now])
//    - before an update of the simulation time
//    - before returning to sc_start (via sc_pause() or sc_stop())
//    - after an update phase (if delta cycles need to be traced)
//
#if SC_TRACING_PHASE_CALLBACKS_
void
sc_trace_file_base::simulation_phase_callback()
{
    // delta cycle is traced at the end of an update phase
    cycle( simcontext()->get_status() == SC_END_OF_UPDATE );
}
#endif // SC_TRACING_PHASE_CALLBACKS_

/*****************************************************************************/

bool
sc_trace_file_base::initialize()
{
    if( initialized_ )
      return false;

    initialized_ = true;

    if( !tracing_initialized_ ) {
        tracing_initialized_ = true;
        bool running_regression = ( getenv( "SYSTEMC_REGRESSION" ) != NULL );

        // hide some messages during regression
        if( running_regression ) {
          sc_report_handler::set_actions( SC_ID_TRACING_TIMESCALE_DEFAULT_
                                        , SC_INFO,    SC_DO_NOTHING );
          sc_report_handler::set_actions( SC_ID_TRACING_VCD_DUPLICATE_TIME_
                                        , SC_WARNING, SC_DO_NOTHING );
        }
    }

    // open trace file
    if(!fp) open_fp();

    // setup timescale
    if( !timescale_set_by_user )
    {
        timescale_unit = sc_get_time_resolution().to_seconds();

        std::stringstream ss;
        ss << sc_get_time_resolution() << " (" << filename_ << ")";
        SC_REPORT_INFO( SC_ID_TRACING_TIMESCALE_DEFAULT_
                      , ss.str().c_str() );
    }

    // initialize derived tracing implementation class (VCD/WIF)
    do_initialize();

    return initialized_;
}

void
sc_trace_file_base::open_fp()
{
    sc_assert( !fp );
    fp = fopen( filename(), "w" );
    if( !fp ) {
        SC_REPORT_ERROR( SC_ID_TRACING_FOPEN_FAILED_, filename() );
        std::terminate(); // can't recover from here
    }
}

void
sc_trace_file_base::delta_cycles( bool flag )
{
    trace_delta_cycles_ = flag;
#if SC_TRACING_PHASE_CALLBACKS_
    if( trace_delta_cycles_ ) {
        sc_object::register_simulation_phase_callback( SC_END_OF_UPDATE );
    } else {
        sc_object::unregister_simulation_phase_callback( SC_END_OF_UPDATE );
    }
#endif
}

void
sc_trace_file_base::set_time_unit( double v, sc_time_unit tu )
{
    if( initialized_ )
    {
        std::stringstream ss;
        ss << filename_ << "\n"
           "\tTimescale unit cannot be changed once tracing has begun.\n"
           "\tTo change the scale, create a new trace file.";
        SC_REPORT_ERROR( SC_ID_TRACING_ALREADY_INITIALIZED_
                       , ss.str().c_str() );
        return;
    }

    switch ( tu )
    {
      case SC_FS:  v = v * 1e-15; break;
      case SC_PS:  v = v * 1e-12; break;
      case SC_NS:  v = v * 1e-9;  break;
      case SC_US:  v = v * 1e-6;  break;
      case SC_MS:  v = v * 1e-3;  break;
      case SC_SEC:                break;
      default: {
            std::stringstream ss;
            ss << "unknown time unit:" << tu
               << " (" << filename_ << ")";
            SC_REPORT_WARNING( SC_ID_TRACING_TIMESCALE_UNIT_
                             , ss.str().c_str() );
        }
    }

    timescale_set_by_user = true;
    timescale_unit = v;

    // EMIT ADVISORY MESSAGE ABOUT CHANGE IN TIME SCALE:
    {
      std::stringstream ss;
      ss << sc_time( timescale_unit, SC_SEC )
         << " (" << filename_ << ")";
      SC_REPORT_INFO( SC_ID_TRACING_TIMESCALE_UNIT_, ss.str().c_str() );
    }
}

bool
sc_trace_file_base::add_trace_check( const std::string & name ) const
{
    if( !initialized_ ) return true;

    std::stringstream ss;
    ss << "sc_trace() failed:\n"
         "\tNo traces can be added to "
         "'" << filename_  << "'"
         " once trace recording has started.\n"
         "\tTo add tracing of '" << name << "', create a new trace file.";

    SC_REPORT_ERROR( SC_ID_TRACING_ALREADY_INITIALIZED_
                   , ss.str().c_str() );
    return false;
}

// obtain formatted time string
std::string localtime_string()
{
    char buf[200];
    time_t long_time;
    time(&long_time);
    struct tm* p_tm = localtime(&long_time);
    strftime(buf, 199, "%b %d, %Y       %H:%M:%S", p_tm);
    return buf;
}


} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// Taf!
