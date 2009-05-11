/*
 * Copyright (c) 1999-2005 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
   This file has been modified by Kevin Moore and Dan Nussbaum of the
   Scalable Systems Research Group at Sun Microsystems Laboratories
   (http://research.sun.com/scalable/) to support the Adaptive
   Transactional Memory Test Platform (ATMTP).

   Please send email to atmtp-interest@sun.com with feedback, questions, or
   to request future announcements about ATMTP.

   ----------------------------------------------------------------------

   File modification date: 2008-02-23

   ----------------------------------------------------------------------
*/

/*
 * FileName:  initvar.C
 * Synopsis:  implementation of global variable initialization in simics
 * Author:    cmauer
 * Version:   $Id$
 */

/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/

using namespace std;
#include <string>
#include <map>
#include <stdlib.h>

// Maurice
// extern "C" {
// #include "global.hh"
// #include "simics/api.hh"
// #ifdef SIMICS22X
// #include "configuration_api.hh"
// #endif
// #ifdef SIMICS30
// #include "configuration.hh"
// #endif
// };

#include "mem/ruby/common/Global.hh"

#include "mem/gems_common/ioutil/confio.hh"
#include "mem/gems_common/ioutil/initvar.hh"

/*------------------------------------------------------------------------*/
/* Variable declarations                                                  */
/*------------------------------------------------------------------------*/

// define global "constants" using centralized file
#define PARAM( NAME ) \
   int32  NAME;
#define PARAM_UINT( NAME ) \
   uint32 NAME;
#define PARAM_ULONG( NAME ) \
   uint64 NAME;
#define PARAM_BOOL( NAME ) \
   bool   NAME;
#define PARAM_DOUBLE( NAME ) \
   double NAME;
#define PARAM_STRING( NAME ) \
   char  *NAME;
#define PARAM_ARRAY( PTYPE, NAME, ARRAY_SIZE ) \
   PTYPE  NAME[ARRAY_SIZE];
#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY

/** global initvar object */
initvar_t  *initvar_t::m_inst = NULL;

/*------------------------------------------------------------------------*/
/* Forward declarations                                                   */
/*------------------------------------------------------------------------*/

static attr_value_t initvar_get_attr( void *ptr, void *obj );
static set_error_t  initvar_set_attr( void *ptr, void *obj,
                                      attr_value_t *value );

/*------------------------------------------------------------------------*/
/* Constructor(s) / destructor                                            */
/*------------------------------------------------------------------------*/

//**************************************************************************
initvar_t::initvar_t( const char *name, const char *relativeIncludePath,
                      const char *initializingString,
                      void (*allocate_fn)(void),
                      void (*my_generate_fn)(void) )
{
  m_is_init           = false;
  m_name              = (char *) malloc( sizeof(char)*(strlen( name ) + 2) );
  m_rel_include_path  = (char *) malloc( sizeof(char)*(strlen( relativeIncludePath ) + 2) );
  m_config_filename   = NULL;
  strcpy( m_name, name );
  strcpy( m_rel_include_path, relativeIncludePath );
  m_allocate_f        = allocate_fn;
  m_generate_values_f = my_generate_fn;

  initvar_t::m_inst = this;
  init_config_reader( initializingString );
}

//**************************************************************************
initvar_t::~initvar_t( )
{
#define PARAM( NAME )
#define PARAM_UINT( NAME )
#define PARAM_ULONG( NAME )
#define PARAM_BOOL( NAME )
#define PARAM_DOUBLE( NAME )
#define PARAM_STRING( NAME ) \
   if (NAME != NULL) {       \
      free( NAME );          \
      NAME = NULL;           \
   }
#define PARAM_ARRAY( PTYPE, NAME, ARRAY_SIZE )
#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY
  if (m_name) {
    free( m_name );
  }
  if (m_rel_include_path) {
    free( m_rel_include_path );
  }
  if (m_config_reader) {
    delete m_config_reader;
  }
  if (m_config_filename) {
    delete m_config_filename;
  }
}

//**************************************************************************
void initvar_t::init_config_reader( const char *initString )
{
  int         rc;
  const char *name;

  m_config_reader = new confio_t();
  m_config_reader->setVerbosity( false );

  // Initialize the config reader object to identify each parameter
#define PARAM_UINT   PARAM
#define PARAM_ULONG  PARAM
#define PARAM_BOOL   PARAM
#define PARAM_DOUBLE PARAM
#define PARAM( NAME )                                         \
  name = #NAME;                                               \
  rc = m_config_reader->register_attribute( name,             \
                                     initvar_get_attr, (void *) name,   \
                                     initvar_set_attr, (void *) name );
#define PARAM_STRING( NAME )                                  \
  NAME = NULL;                                                \
  name = #NAME;                                               \
  rc = m_config_reader->register_attribute( name,             \
                                     initvar_get_attr, (void *) name,   \
                                     initvar_set_attr, (void *) name );
#define PARAM_ARRAY( PTYPE, NAME, ARRAY_SIZE )                \
  name = #NAME;                                               \
  rc = m_config_reader->register_attribute( name,             \
                                     initvar_get_attr, (void *) name,   \
                                     initvar_set_attr, (void *) name );

#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY

  // read the default configuration from the embedded text file
  rc = m_config_reader->readConfigurationString( initString );
  (*m_generate_values_f)();
}

/*------------------------------------------------------------------------*/
/* Public methods                                                         */
/*------------------------------------------------------------------------*/

//**************************************************************************
void initvar_t::allocate( void )
{
  if ( confirm_init() ) {
    DEBUG_OUT("error: %s initvar::allocate() called twice\n", m_name);
    return;
  }

  (*m_generate_values_f)();
  (*m_allocate_f)();
  m_is_init = true;
}

//**************************************************************************
void initvar_t::checkInitialization( void )
{
  m_config_reader->checkInitialization();
}

//**************************************************************************
int initvar_t::dispatch_get( void *id, void *obj,
                                      attr_value_t *idx )
{
  const char *command = (const char *) id;
  if ( !confirm_init() ) {
    DEBUG_OUT("error: %s is uninitialized. unable to get \'%s\'\n", m_name, command);
    DEBUG_OUT("     : you must initialize %s with a configuration file first.\n", m_name);
    DEBUG_OUT("     : use the command \'%s0.init\'\n", m_name);

    return 0;
  }

  std::cerr << __FILE__ << "(" << __LINE__ << "): Not implmented." << std::endl;
  return 0;
}


//**************************************************************************
set_error_t initvar_t::dispatch_set( void *id, void *obj,
                                     attr_value_t *val, attr_value_t *idx )
{
  const char *command = (const char *) id;

  // DEBUG_OUT("set attribute: %s\n", command);
  if (!strcmp(command, "init")) {
    if (val->kind == Sim_Val_String) {
      if (!strcmp( val->u.string, "" )) {
        //       update generated values, then allocate
        allocate();
      } else {
        read_config( val->u.string );
        allocate();
      }
      return Sim_Set_Ok;
    } else {
      return Sim_Set_Need_String;
    }
  } else if (!strcmp(command, "readparam")) {
    if (val->kind == Sim_Val_String) {
      read_config( val->u.string );
      return Sim_Set_Ok;
    } else {
      return Sim_Set_Need_String;
    }
  } else if (!strcmp(command, "saveparam")) {
    if (val->kind == Sim_Val_String) {
      FILE *fp = fopen( val->u.string, "w" );
      if (fp == NULL) {
        ERROR_OUT("error: unable to open file: %s\n", val->u.string);
        return Sim_Set_Illegal_Value;
      }
      list_param( fp );
      if (fp != NULL) {
        fclose( fp );
      }
      return Sim_Set_Ok;
    } else {
      ERROR_OUT("error: saveparam given wrong type.\n");
      return Sim_Set_Illegal_Value;
    }
  } else if (!strcmp(command, "param")) {
    if (val->kind == Sim_Val_Integer) {
      list_param( stdout );
      return Sim_Set_Ok;
    } else if ( val->kind == Sim_Val_List &&
                val->u.list.size == 2 &&
                val->u.list.vector[0].kind == Sim_Val_String ) {
      return (set_param( val->u.list.vector[0].u.string,
                         &val->u.list.vector[1] ));
    } else {
      DEBUG_OUT("error: set parameter given wrong type.\n");
      return Sim_Set_Illegal_Value;
    }
  }

  if ( !confirm_init() ) {
    DEBUG_OUT("error: %s is uninitialized. unable to set \'%s\'\n", m_name, id);
    DEBUG_OUT("     : you must initialize %s with a configuration file first.\n", m_name);
    DEBUG_OUT("     : use the command \'%s0.init\'\n", m_name);
    return Sim_Set_Illegal_Value;
  }


  std::cerr << __FILE__ << "(" << __LINE__ << "): Not implmented." << std::endl;
  return Sim_Set_Illegal_Value;
}

/*------------------------------------------------------------------------*/
/* Accessor(s) / mutator(s)                                               */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* Private methods                                                        */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* Static methods                                                         */
/*------------------------------------------------------------------------*/

//**************************************************************************
static attr_value_t initvar_get_attr( void *ptr, void *obj )
{
  const char *name = (const char *) ptr;
  attr_value_t  ret;
  memset( &ret, 0, sizeof(attr_value_t) );

#define PARAM_UINT   PARAM
#define PARAM_ULONG  PARAM
#define PARAM_BOOL   PARAM
#define PARAM( NAME )                       \
  if (!strcmp(name, #NAME)) {               \
    ret.kind = Sim_Val_Integer;             \
    ret.u.integer = NAME;                   \
    return (ret);                           \
  }
#define PARAM_DOUBLE( NAME )                \
  if (!strcmp(name, #NAME)) {               \
    ret.kind = Sim_Val_Floating;            \
    ret.u.floating = NAME;                  \
    return (ret);                           \
  }
#define PARAM_STRING( NAME )                \
  if (!strcmp(name, #NAME)) {               \
    ret.kind = Sim_Val_String;              \
    ret.u.string = NAME;                    \
    return (ret);                           \
  }
#define PARAM_ARRAY( PTYPE, NAME, ARRAY_SIZE )            \
  if (!strcmp(name, #NAME)) {                             \
    ret.kind = Sim_Val_List;                                  \
    ret.u.list.size = ARRAY_SIZE;                         \
    ret.u.list.vector = mallocAttribute( ARRAY_SIZE );    \
    for (int i = 0; i < ARRAY_SIZE; i++) {                \
      ret.u.list.vector[i].u.integer = NAME[i];           \
    }                                                     \
    return (ret);                                         \
  }

#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY

  DEBUG_OUT("error: %s not found.\n", name);
  ret.kind = Sim_Val_Invalid;
  return (ret);
}

//***************************************************************************
static set_error_t initvar_set_attr( void *ptr, void *obj,
                                     attr_value_t *value )
{
  const char *name = (const char *) ptr;

#define PARAM_UINT   PARAM
#define PARAM_ULONG  PARAM
#define PARAM( NAME )                           \
  if (!strcmp(name, #NAME)) {                   \
    if ( value->kind != Sim_Val_Integer ) {     \
      ERROR_OUT("error: %s is not an integer\n", name );\
      return Sim_Set_Need_Integer;              \
    }                                           \
    NAME = value->u.integer;                    \
    return Sim_Set_Ok;                          \
  }
#define PARAM_BOOL( NAME )                     \
  if (!strcmp(name, #NAME)) {                  \
    if ( value->kind != Sim_Val_String ) {     \
      ERROR_OUT("error: %s is not an bool string\n", name );\
      return Sim_Set_Need_String;              \
    }                                          \
    if (!strcmp(value->u.string, "true")) {    \
      NAME = true;                             \
    } else if (!strcmp(value->u.string, "false")) { \
      NAME = false;                            \
    } else {                                   \
      ERROR_OUT("error: value %s for %s is not an bool string (set to false)\n", value->u.string, name );\
      NAME = false;                            \
    }                                          \
    return Sim_Set_Ok;                         \
  }
#define PARAM_DOUBLE( NAME )                     \
  if (!strcmp(name, #NAME)) {                    \
    if ( value->kind != Sim_Val_String ) {     \
      ERROR_OUT("error: %s is not a float\n", name );\
      return Sim_Set_Need_Floating;              \
    }                                            \
    NAME = atof( value->u.string );              \
    return Sim_Set_Ok;                           \
  }
#define PARAM_STRING( NAME )                    \
  if (!strcmp(name, #NAME)) {                   \
    if ( value->kind != Sim_Val_String ) {      \
      ERROR_OUT("error: %s is not an string\n", name ); \
      return Sim_Set_Need_String;               \
    }                                           \
    if (NAME != NULL) {                         \
      free( NAME );                             \
    }                                           \
    NAME = strdup( value->u.string );           \
    return Sim_Set_Ok;                          \
  }
#define PARAM_ARRAY( PTYPE, NAME, ARRAY_SIZE )            \
  if (!strcmp(name, #NAME)) {                             \
    if ( value->kind != Sim_Val_List ) {                      \
      ERROR_OUT("error: %s is not an list\n", name );     \
      return Sim_Set_Need_List;                               \
    }                                                     \
    if ( value->u.list.size != ARRAY_SIZE ) {             \
      ERROR_OUT("error: %s has %lld elements (should be %d)\n", name, value->u.list.size, ARRAY_SIZE); \
      return Sim_Set_Illegal_Value;                           \
    }                                                     \
    for (int i = 0; i < ARRAY_SIZE; i++) {                \
      NAME[i] = value->u.list.vector[i].u.integer;        \
    }                                                     \
    return Sim_Set_Ok;                                        \
  }

#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY

  ERROR_OUT("error: %s not a parameter\n", name);
  return Sim_Set_Illegal_Value;
}

//***************************************************************************
void initvar_t::read_config( const char *parameterFile )
{
  DEBUG_OUT("read configuration: %s\n", parameterFile );
  m_config_filename = strdup( parameterFile );
  int rc = m_config_reader->readConfiguration( parameterFile,
                                               m_rel_include_path );
  if ( rc < 0 ) {
    ERROR_OUT("fatal error in read configuration: unable to continue.\n");
    exit(1);
  }
  // update generated values
  (*m_generate_values_f)();
}

/** sets one of the parameters */
//**************************************************************************
set_error_t initvar_t::set_param( const char *name, attr_value_t *value )
{

  // [dann 2007-04-04] ATMTP VV
  //
  // HACK ALERT: allow setting REMOVE_SINGLE_CYCLE_DCACHE_FAST_PATH,
  // PROFILE_EXCEPTIONS, PROFILE_XACT, ATMTP_DEBUG_LEVEL and
  // ATMTP_ENABLED after initialization.  This works is because ruby's
  // m_generate_values_f() does nothing -- more particularly, nothing
  // that depends on any of these parameters is pre-calculated
  // anywhere.
  //
  if (strcmp(name, "REMOVE_SINGLE_CYCLE_DCACHE_FAST_PATH") != 0 &&
      strcmp(name, "PROFILE_EXCEPTIONS") != 0 &&
      strcmp(name, "PROFILE_XACT") != 0 &&
      strcmp(name, "ATMTP_DEBUG_LEVEL") != 0 &&
      strcmp(name, "ATMTP_ENABLED") != 0) {
    //
    // [dann 2007-04-04] ATMTP ^^
    if ( confirm_init() ) {
      DEBUG_OUT("error: %s is already initialized.\n", m_name);
      DEBUG_OUT("     : setting parameters after initialization is unsupported\n");
      return (Sim_Set_Illegal_Value);
    }
    // [dann 2007-04-04] ATMTP VV
    //
  }
  //
  // [dann 2007-04-04] ATMTP ^^

  set_error_t result = initvar_set_attr( (void *) name, NULL, value );
  (*m_generate_values_f)();
  return (result);
}

/** print out a list of valid parameters */
//**************************************************************************
void initvar_t::list_param( FILE *fp )
{
    if (!fp)
        fp = stdout;

#define PARAM( NAME )                            \
  fprintf( fp, "%-44.44s: %26d\n", #NAME, NAME );
#define PARAM_UINT( NAME )                              \
  fprintf( fp, "%-44.44s: %26u\n", #NAME, NAME );
#define PARAM_ULONG( NAME )                             \
  fprintf( fp, "%-44.44s: %26llu\n", #NAME, NAME );
#define PARAM_BOOL( NAME )                              \
  if (NAME == true) {                                 \
    fprintf( fp, "%-44.44s: %26.26s\n", #NAME, "true" ); \
  } else {                                            \
    fprintf( fp, "%-44.44s: %26.26s\n", #NAME, "false" );\
  }
#define PARAM_DOUBLE( NAME )                            \
  fprintf( fp, "%-44.44s: %26f\n", #NAME, NAME );
#define PARAM_STRING( NAME )                              \
  if ( NAME != NULL ) {                                   \
    fprintf( fp, "%-44.44s: %26.26s\n", #NAME, NAME );  \
  }
#define PARAM_ARRAY( PTYPE, NAME, ARRAY_SIZE )          \
  fprintf( fp, "%-44.44s: (", #NAME );                \
  for (int i = 0; i < ARRAY_SIZE; i++) {                \
    if ( i != 0 ) {                                     \
      fprintf( fp, ", " );                            \
    }                                                   \
    fprintf( fp, "%d", NAME[i] );                     \
  }                                                     \
  fprintf( fp, ")\n" );

#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY
}

//**************************************************************************
const char *initvar_t::get_config_name( void )
{
  if (m_config_filename == NULL) {
    return "default";
  }
  return m_config_filename;
}

