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

  sc_except.h - Exception classes to be handled by SystemC.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_EXCEPT_H
#define SC_EXCEPT_H

#include <exception>

namespace sc_core {

class sc_simcontext;
class sc_process_b;
class sc_method_process;
class sc_thread_process;
void sc_thread_cor_fn( void* arg );

/*
 *  These classes are intentionally empty. Their raison d'etre is for
 *  the implementation of various SystemC throws.
 */

class sc_user
{
    /*EMPTY*/
public:
    sc_user() {}
    sc_user( const sc_user& ) {}
};

class sc_halt
{
public:
    sc_halt() {}
    sc_halt( const sc_halt& ) {}
};

class sc_kill
{
public:
    sc_kill() {}
    sc_kill( const sc_kill& ) {}
};

class sc_unwind_exception : public std::exception
{
    friend class sc_simcontext;
    friend class sc_process_b;
    friend class sc_method_process;
    friend class sc_thread_process;
    friend void sc_thread_cor_fn( void* arg );

  public:
    virtual bool is_reset() const { return m_is_reset; }
    virtual const char* what() const throw();

  public:

    // enable catch by value
    sc_unwind_exception( const sc_unwind_exception& );
    virtual ~sc_unwind_exception() throw();

  protected:
    explicit
    sc_unwind_exception( sc_process_b* target_p, bool is_reset = false );

    bool active() const;
    void clear()  const;

  private:
    // disabled
    sc_unwind_exception& operator=( const sc_unwind_exception& );

    mutable sc_process_b* m_proc_p;   // used to check, if caught by the kernel
    const   bool          m_is_reset; // true if this is an unwind of a reset

};

inline
sc_unwind_exception::sc_unwind_exception( const sc_unwind_exception& that )
  : std::exception( that )
  , m_proc_p( that.m_proc_p )
  , m_is_reset( that.m_is_reset )
{
    that.m_proc_p = 0; // move to new instance
}

//------------------------------------------------------------------------------
// global exception handling
//------------------------------------------------------------------------------
 
class sc_report;
sc_report* sc_handle_exception();

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Gene Bushuyev. Synopsys, Inc.
  Description of Modification: - Had to add empty public default and copy
                                 constructors to satisfy VC6.0.
    
      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_except.h,v $
// Revision 1.11  2011/08/26 21:40:26  acg
//  Philipp A. Hartmann: fix up sc_unwind_exception copy-ctor.
//
// Revision 1.10  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.9  2011/08/24 22:05:50  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.8  2011/05/09 04:07:48  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
// Revision 1.7  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.6  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.5  2011/02/11 13:25:24  acg
//  Andy Goodrich: Philipp A. Hartmann's changes:
//    (1) Removal of SC_CTHREAD method overloads.
//    (2) New exception processing code.
//
// Revision 1.4  2011/01/18 20:10:44  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.3  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.2  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.

#endif
