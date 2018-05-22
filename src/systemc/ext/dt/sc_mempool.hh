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

  sc_mempool.h - Memory pools for small objects.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef __SYSTEMC_EXT_DT_SC_MEMPOOL_HH__
#define __SYSTEMC_EXT_DT_SC_MEMPOOL_HH__

namespace sc_core
{

// ----------------------------------------------------------------------------
//  CLASS : sc_mempool
//
//  ...
// ----------------------------------------------------------------------------

class sc_mempool
{
  public:
    static void *allocate(std::size_t sz);
    static void release(void *p, std::size_t sz);
    static void display_statistics();
};

// ----------------------------------------------------------------------------
//  CLASS : sc_mpobject
//
//  ...
// ----------------------------------------------------------------------------

class sc_mpobject
{
  public:
    static void *
    operator new(std::size_t sz)
    {
        return sc_mempool::allocate(sz);
    }

    static void
    operator delete(void *p, std::size_t sz)
    {
        sc_mempool::release(p, sz);
    }

    static void *
    operator new [] (std::size_t sz)
    {
        return sc_mempool::allocate(sz);
    }

    static void
    operator delete [] (void *p, std::size_t sz)
    {
        sc_mempool::release(p, sz);
    }
};

} // namespace sc_core

// $Log: sc_mempool.h,v $
// Revision 1.3  2011/08/26 20:46:18  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.2  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

#endif // __SYSTEMC_EXT_DT_SC_MEMPOOL_HH__
