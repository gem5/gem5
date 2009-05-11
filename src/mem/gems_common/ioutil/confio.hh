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

#ifndef _CONFIO_H_
#define _CONFIO_H_

/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/

#include "FakeSimicsDataTypes.hh"

/*------------------------------------------------------------------------*/
/* Macro declarations                                                     */
/*------------------------------------------------------------------------*/

/// constant for attribute parsing: a (x) single value
const  attr_kind_t   CONF_ATTR_SINGLE = (attr_kind_t) (Sim_Val_Object + 1);
/// constant for attribute parsing: a (x,y) pair of values
const  attr_kind_t   CONF_ATTR_PAIR   = (attr_kind_t) (Sim_Val_Object + 2);

/*------------------------------------------------------------------------*/
/* Class declaration(s)                                                   */
/*------------------------------------------------------------------------*/

/*
 *  Functions for modifying the micro-architectural configuation of
 *  a class.
 */

/// function for getting the configuration value
typedef  attr_value_t (*get_confio_t)( void *ptr, void *obj );
/// function for setting the configuration value
typedef  set_error_t  (*set_confio_t)( void *ptr, void *obj,
                                       attr_value_t *value );

/// a struture containing the functional callbacks for each conf node
typedef struct confnode {
  get_confio_t get_attr;
  set_confio_t set_attr;
  void        *set_attr_data;
  void        *get_attr_data;
  bool         attr_is_set;
} confnode_t;

/// a mapping from a string to a configuration structure
typedef map<string, confnode_t *> ConfTable;

/**
* Configuration state saving: allows the user to save the micro-architectural
* state in a text file for later runs. This file is also used to set
* globals during simulation.
*
* @author  cmauer
* @version $Id$
*/
class confio_t {

public:


  /**
   * @name Constructor(s) / destructor
   */
  //@{

  /**
   * Constructor: creates object
   */
  confio_t();

  /**
   * Destructor: frees object.
   */
  ~confio_t();
  //@}

  /**
   * @name Methods
   */
  //@{
  //@}

  /**
   * @name Accessor(s) / mutator(s)
   */
  //@{
  /**
   * register a configuration variable with the configuration manager.
   * @param get_attr   A function to get the attribute value
   * @param get_attr_data Void pointer, available to get_attr
   * @param set_attr   A function to set the attribute value
   * @param set_attr_data Void pointer, available to set_attr
   * @return [Description of return value]
   */
  int  register_attribute( const char *name,
                           get_confio_t get_attr, void *get_attr_data,
                           set_confio_t set_attr, void *set_attr_data );

  /**
   * Set verbosity of the configuration
   * @param verbose     True causes more info to be printed out, False doesn't
   */
  void setVerbosity( bool verbose ) {
    m_verbose = verbose;
  }

  /**
   * write a configuration file: e.g. save state
   */
  int  writeConfiguration( const char *outputFilename );

  /**
   * read state from an existing configuration file
   * @param inputFilename          The file to read
   * @param relativeIncludePath    The path to search on 'include' statements
   */
  int  readConfiguration( const char *inputFilename,
                          const char *relativeIncludePath );

  /**
   * read state from a string
   */
  int  readConfigurationString( const char *inputBuffer );

  /**
   * check that each registered configuration is set (reports a warning if
   * they are not.)
   */
  void checkInitialization( void );
  //@}

private:
  /**
   * Apply an attribute list to the configuration table
   */
  int  applyConfiguration( attr_value_t *attr );

  /// configuration table: contains a map from a string -> conf node
  ConfTable     m_table;

  /// if false, nothing is printed under normal operation
  bool          m_verbose;
};

/*------------------------------------------------------------------------*/
/* Global variables                                                       */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* Global functions                                                       */
/*------------------------------------------------------------------------*/

/**
 * Allocates an array of attributes.
 */
attr_value_t *mallocAttribute( uint32 number );

/**
 * Walks an attribute tree, freeing all memory under attr. Does not free
 * attr itself.
 */
void          freeAttribute( attr_value_t *attr );

#endif  /* _CONFIO_H_ */


