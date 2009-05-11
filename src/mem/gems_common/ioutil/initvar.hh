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

#ifndef _INCLUDE_H_
#define _INCLUDE_H_

/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* Macro declarations                                                     */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* Class declaration(s)                                                   */
/*------------------------------------------------------------------------*/

/**
* This class deals with initializing the global variables in the object,
* setting the varibles (from the command line), printing the configuration,
* and saving it to a file.
*
* @see     confio_t
* @author  cmauer
* @version $Id$
*/
class initvar_t {
public:
  /**
   * @name Constructor(s) / destructor
   */
  //@{

  /**
   * Constructor: creates object
   * @param name                The name of this object
   * @param relativeIncludePath The relative path to config files
   * @param initializingString  A string (with value pairs) for initialization
   * @param allocate_f        A ptr to the allocate function
   * @param generate_values   A ptr to the generate values function
   * @param my_get_attr       A ptr to the get attribute function
   * @param my_set_attr       A ptr to the set attribute function
   */
  initvar_t( const char *name, const char *relativeIncludePath,
             const char *initializingString,
             void (*allocate_fn)(void),
             void (*my_generate_fn)(void)
           );

  /**
   * Destructor: frees object.
   */
  ~initvar_t();
  //@}

  /**
   * @name Methods
   */
  //@{
  /// calls the allocation routine explicitly (used by the tester)
  void         allocate( void );

  /// checks to see if all vars have been initialized
  void         checkInitialization( void );

  /// list all parameters: to a file (or stdout if file is NULL)
  void         list_param( FILE *fp );

  /// returns the name of the last config file to be read ("default" is none)
  const char  *get_config_name( void );

  /// calls through to the get_attr function, if object is initialized
  int dispatch_get( void *id, void *obj,
                    attr_value_t *idx );

  /** adds initialization attributes, calls through to the set_attr function,
   *  if object is initialized.
   */
  set_error_t  dispatch_set( void *id, void *obj,
                             attr_value_t *val, attr_value_t *idx );
  //@}
  /// (single) instance of the init var object
  static initvar_t *m_inst;

protected:
  ///returns true if the variables are initialized
  bool  confirm_init( void ) {
    return m_is_init;
  }

  ///read a configuration file
  void         read_config( const char *parameterFile );

  /// set a parameter to be a particular value
  set_error_t  set_param( const char *name, attr_value_t *value );

  /// initializes the configuration reader
  void         init_config_reader( const char *initString );

  /// bool value (true if initialized)
  bool       m_is_init;

  /// configuration reader
  confio_t  *m_config_reader;

  /// a pointer to a string (corresponding to this objects name)
  char      *m_name;

  /// a pointer to a string (representing the last config file read)
  char      *m_config_filename;

  /// the relative include path to the configuration files
  char      *m_rel_include_path;

  /// a pointer to the allocation function
  void     (*m_allocate_f)(void);

  /// a pointer to the generate values function
  void     (*m_generate_values_f)(void);

};


/*------------------------------------------------------------------------*/
/* Global variables                                                       */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* Global functions                                                       */
/*------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

///provides a dispatch mechanism that catches a few commands to get variables
attr_value_t initvar_dispatch_get( void *id, void *obj,
                                   attr_value_t *idx );

///provides a dispatch mechanism that catches a few commands to set variables
set_error_t  initvar_dispatch_set( void *id, void *obj,
                                   attr_value_t *val, attr_value_t *idx );

#ifdef __cplusplus
} // extern "C"
#endif

#endif  /* _INCLUDE_H_ */
