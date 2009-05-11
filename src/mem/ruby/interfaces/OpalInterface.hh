
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

/*
 * $Id$
 *
 * Description:
 *
 */

#ifndef OpalInterface_H
#define OpalInterface_H

/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/

#include "Global.hh"
#include "Driver.hh"
#include "mf_api.hh"
#include "CacheRequestType.hh"

/*------------------------------------------------------------------------*/
/* Class declaration(s)                                                   */
/*------------------------------------------------------------------------*/

class System;
class TransactionInterfaceManager;
class Sequencer;

/**
 * the processor model (opal) calls these OpalInterface APIs to access
 * the memory hierarchy (ruby).
 * @see     pseq_t
 * @author  cmauer
 * @version $Id$
 */
class OpalInterface : public Driver {
public:
  // Constructors
  OpalInterface(System* sys_ptr);

  // Destructor
  // ~OpalInterface();

  integer_t getInstructionCount(int procID) const;
  void hitCallback( NodeID proc, SubBlock& data, CacheRequestType type, int thread );
  void printStats(ostream& out) const;
  void clearStats();
  void printConfig(ostream& out) const;
  void print(ostream& out) const;

  integer_t readPhysicalMemory(int procID, physical_address_t address,
                               int len );

  void writePhysicalMemory( int procID, physical_address_t address,
                            integer_t value, int len );
  uint64 getOpalTime(int procID) const;

  // for WATTCH power
  void incrementL2Access(int procID) const;
  void incrementPrefetcherAccess(int procID, int num_prefetches, int isinstr) const;

  // notifies Opal of an L2 miss
  void notifyL2Miss(int procID, physical_address_t physicalAddr, OpalMemop_t type, int tagexists) const;

  void printDebug();

  /// The static opalinterface instance
  static OpalInterface *inst;

  /// static methods
  static int getNumberOutstanding(int cpuNumber);
  static int getNumberOutstandingDemand(int cpuNumber);
  static int getNumberOutstandingPrefetch( int cpuNumber );

  /* returns true if the sequencer is able to handle more requests.
     This implements "back-pressure" by which the processor knows
     not to issue more requests if the network or cache's limits are reached.
   */
  static int  isReady( int cpuNumber, la_t logicalAddr, pa_t physicalAddr, OpalMemop_t typeOfRequest, int thread );

  /*
  makeRequest performs the coherence transactions necessary to get the
  physical address in the cache with the correct permissions. More than
  one request can be outstanding to ruby, but only one per block address.
  The size of the cache line is defined to Intf_CacheLineSize.
  When a request is finished (e.g. the cache contains physical address),
  ruby calls completedRequest(). No request can be bigger than
  Opal_CacheLineSize. It is illegal to request non-aligned memory
  locations. A request of size 2 must be at an even byte, a size 4 must
  be at a byte address half-word aligned, etc. Requests also can't cross a
  cache-line boundaries.
  */
  static void makeRequest(int cpuNumber, la_t logicalAddr, pa_t physicalAddr,
                          int requestSize, OpalMemop_t typeOfRequest,
                          la_t virtualPC, int isPriv, int thread);

  /* prefetch a given block...
   */
  static void makePrefetch(int cpuNumber, la_t logicalAddr, pa_t physicalAddr,
                           int requestSize, OpalMemop_t typeOfRequest,
                           la_t virtualPC, int isPriv, int thread);

  /*
   * request data from the cache, even if it's state is "Invalid".
   */
  static int staleDataRequest( int cpuNumber, pa_t physicalAddr,
                               int requestSize, int8 *buffer );

  /* notify ruby of opal's status
   */
  static void notify( int status );

  /*
   * advance ruby one cycle
   */
  static void advanceTime( void );

  /*
   * return ruby's cycle count.
   */
  static unsigned long long getTime( void );

  /* prints Ruby's outstanding request table */
  static void printProgress(int cpuNumber);

  /*
   * initialize / install the inter-module interface
   */
  static void installInterface( mf_ruby_api_t *api );

  /*
   * Test if opal is loaded or not
   */
  static bool isOpalLoaded( void );

  /*
   * query opal for its api
   */
  void queryOpalInterface( void );

  /*
   * remove the opal interface (opal is unloaded).
   */
  void removeOpalInterface( void );

  /*
   * set the opal interface (used if stand-alone testing)
   */
  void setOpalInterface( mf_opal_api_t *opal_intf ) {
    m_opal_intf = opal_intf;
  }

  /**
   * Signal an abort
   */
  //void abortCallback(NodeID proc);

private:
  // Private Methods

  // Private copy constructor and assignment operator
  OpalInterface(const OpalInterface& obj);
  OpalInterface& operator=(const OpalInterface& obj);

  // Data Members (m_ prefix)
  mf_opal_api_t  *m_opal_intf;
  Time            m_simicsStartTime;

  static int      s_advance_counter;
};

// Output operator declaration
ostream& operator<<(ostream& out, const OpalInterface& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const OpalInterface& obj)
{
//  obj.print(out);
  out << flush;
  return out;
}

#endif // OpalInterface_H
