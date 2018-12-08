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

#ifndef __TLM_ANALYSIS_PORT_H__
#define __TLM_ANALYSIS_PORT_H__

#include "tlm_core/tlm_1/tlm_analysis/tlm_analysis_if.h"
#include <deque>
#include <algorithm>

namespace tlm {


template < typename T>
class tlm_analysis_port :
  public sc_core::sc_object ,
  public virtual tlm_analysis_if< T >
{
 public:
  tlm_analysis_port() : sc_core::sc_object() {}
  tlm_analysis_port( const char *nm ) : sc_core::sc_object( nm ) {}

  // bind and () work for both interfaces and analysis ports, since
  // analysis ports implement the analysis interface

  virtual void bind( tlm_analysis_if<T> &_if ) {
    m_interfaces.push_back( &_if );
  }

  void operator() ( tlm_analysis_if<T> &_if ) { bind( _if ); }

  virtual bool unbind( tlm_analysis_if<T> &_if ) {

    typename std::deque< tlm_analysis_if<T> * >::iterator i
      = std::remove( m_interfaces.begin(), m_interfaces.end(), &_if );

    if( i != m_interfaces.end() ) {
      m_interfaces.erase(i, m_interfaces.end() );
      return 1;
    }

    return 0;

  }

  void write( const T &t ) {
    typename std::deque< tlm_analysis_if<T> * >::iterator i;

    for( i = m_interfaces.begin();
   i != m_interfaces.end();
   i++ ) {

      (*i)->write( t );

    }

  }

 private:
  std::deque< tlm_analysis_if<T> * > m_interfaces;

};

} // namespace tlm

#endif


