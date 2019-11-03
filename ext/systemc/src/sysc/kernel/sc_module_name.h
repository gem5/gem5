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

  sc_module_name.h -- An object used to help manage object names 
                      and hierarchy.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

// $Log: sc_module_name.h,v $
// Revision 1.5  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//

#ifndef SC_MODULE_NAME_H
#define SC_MODULE_NAME_H


namespace sc_core {

class sc_module;
class sc_simcontext;


// ----------------------------------------------------------------------------
//  CLASS : sc_module_name
//
//  Module name class.
// ----------------------------------------------------------------------------

class sc_module_name
{
    friend class sc_module;
    friend class sc_object_manager;

public:

    sc_module_name( const char* );
    sc_module_name( const sc_module_name& );

    ~sc_module_name();

    operator const char*() const;

protected:
    inline void clear_module( sc_module* module_p );
    inline void set_module( sc_module* module_p );

private:

    const char*     m_name;
    sc_module*      m_module_p;
    sc_module_name* m_next;
    sc_simcontext*  m_simc;
    bool            m_pushed;

private:

    // disabled
    sc_module_name();
    sc_module_name& operator = ( const sc_module_name& );
};

inline void sc_module_name::clear_module( sc_module* module_p )
{
    assert( m_module_p == module_p );
    m_module_p = 0;
}

inline void sc_module_name::set_module( sc_module* module_p )
{
    m_module_p = module_p;
}

} // namespace sc_core

// Revision 1.4  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.3  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/03/14 23:56:58  acg
//   Andy Goodrich: This fixes a bug when an exception is thrown in
//   sc_module::sc_module() for a dynamically allocated sc_module
//   object. We are calling sc_module::end_module() on a module that has
//   already been deleted. The scenario runs like this:
//
//   a) the sc_module constructor is entered
//   b) the exception is thrown
//   c) the exception processor deletes the storage for the sc_module
//   d) the stack is unrolled causing the sc_module_name instance to be deleted
//   e) ~sc_module_name() calls end_module() with its pointer to the sc_module
//   f) because the sc_module has been deleted its storage is corrupted,
//      either by linking it to a free space chain, or by reuse of some sort
//   g) the m_simc field is garbage
//   h) the m_object_manager field is also garbage
//   i) an exception occurs
//
//   This does not happen for automatic sc_module instances since the
//   storage for the module is not reclaimed its just part of the stack.
//
//   I am fixing this by having the destructor for sc_module clear the
//   module pointer in its sc_module_name instance. That cuts things at
//   step (e) above, since the pointer will be null if the module has
//   already been deleted. To make sure the module stack is okay, I call
//   end-module() in ~sc_module in the case where there is an
//   sc_module_name pointer lying around.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.

#endif
