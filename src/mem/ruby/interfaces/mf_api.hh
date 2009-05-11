
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* Macro declarations                                                     */
/*------------------------------------------------------------------------*/

#ifndef _MF_MEMORY_API_H_
#define _MF_MEMORY_API_H_

#ifdef SIMICS30
#ifndef pa_t
typedef physical_address_t pa_t;
typedef physical_address_t la_t;
#endif
#endif

/**
 *  Defines types of memory requests
 */
typedef enum OpalMemop {
  OPAL_LOAD,
  OPAL_STORE,
  OPAL_IFETCH,
  OPAL_ATOMIC,
} OpalMemop_t;

/*------------------------------------------------------------------------*/
/* Class declaration(s)                                                   */
/*------------------------------------------------------------------------*/

/**
* structure which provides an interface between ruby and opal.
*/
typedef struct mf_opal_api {
  /**
   * @name Methods
   */
  //@{
  /**
   * notify processor model that data from address address is available at proc
   */
  void (*hitCallback)( int cpuNumber, pa_t phys_address, OpalMemop_t type, int thread );

  /**
   * notify opal that ruby is loaded, or removed
   */
  void (*notifyCallback)( int status );

  /**
   * query for the number of instructions executed on a given processor.
   */
  integer_t (*getInstructionCount)( int cpuNumber );

  // for printing out debug info on crash
  void (*printDebug)();

  /** query Opal for the current time */
  uint64 (*getOpalTime)(int cpuNumber);

  /** For WATTCH power stats */
  // Called whenever L2 is accessed
  void (*incrementL2Access)(int cpuNumber);
  // Called whenever prefetcher is accessed
  void (*incrementPrefetcherAccess)(int cpuNumber, int num_prefetches, int isinstr);

  /* Called whenever there's an L2 miss */
  void (*notifyL2Miss)(int cpuNumber, physical_address_t physicalAddr, OpalMemop_t type, int tagexists);

  //@}
} mf_opal_api_t;

typedef struct mf_ruby_api {
  /**
   * @name Methods
   */
  //@{
  /**
   * Check to see if the system is ready for more requests
   */
  int  (*isReady)( int cpuNumber, la_t logicalAddr, pa_t physicalAddr, OpalMemop_t typeOfRequest, int thread );

  /**
   * Make a 'mandatory' request to the memory hierarchy
   */
  void (*makeRequest)( int cpuNumber, la_t logicalAddr, pa_t physicalAddr,
                       int requestSize, OpalMemop_t typeOfRequest,
                       la_t virtualPC, int isPriv, int thread);

  /**
   * Make a prefetch request to the memory hierarchy
   */
  void (*makePrefetch)( int cpuNumber, la_t logicalAddr, pa_t physicalAddr,
                        int requestSize, OpalMemop_t typeOfRequest,
                        la_t virtualPC, int isPriv, int thread);

  /**
   * Ask the memory hierarchy for 'stale' data that can be used for speculation
   * Returns true (1) if the tag matches, false (0) if not.
   */
  int (*staleDataRequest)( int cpuNumber, pa_t physicalAddr,
                           int requestSize, int8 *buffer );

  /**
   * Advance ruby's cycle time one step
   */
  void (*advanceTime)( void );

  /**
   * Get ruby's cycle time count.
   */
  uint64 (*getTime)( void );

  /** prints Ruby's outstanding request table */
  void (*printProgress)(int cpuNumber);

  /**
   * notify ruby that opal is loaded, or removed
   */
  void (*notifyCallback)( int status );

  // Returns the number of outstanding request
  int (*getNumberOutstanding)(int cpuNumber);

  // Returns the number of outstanding demand requests
  int (*getNumberOutstandingDemand)(int cpuNumber );

  // Returns the number of outstanding prefetch request
  int (*getNumberOutstandingPrefetch)(int cpuNumber );


  //@}
} mf_ruby_api_t;

#endif //_MF_MEMORY_API_H_
