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

  sc_export.h -- Base classes of all export classes.

  Original Author: Andy Goodrich, Forte Design Systems
                   Bishnupriya Bhattacharya, Cadence Design Systems

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_EXPORT_H
#define SC_EXPORT_H
#include <typeinfo>

#include "sysc/communication/sc_communication_ids.h"
#include "sysc/communication/sc_interface.h"
#include "sysc/kernel/sc_object.h"

#if ! defined( SC_DISABLE_VIRTUAL_BIND )
#  define SC_VIRTUAL_ virtual
#else
#  define SC_VIRTUAL_ /* non-virtual */
#endif

namespace sc_core {

//=============================================================================
//  CLASS : sc_export_base
//
//  Abstract base class for class sc_export<IF>.
//=============================================================================

class sc_export_base : public sc_object
{
    friend class sc_export_registry;
public:

    // typedefs
    
    typedef sc_export_base this_type;

public:

    virtual       sc_interface* get_interface() = 0;
    virtual       const sc_interface* get_interface() const = 0;

protected:
    
    // constructors

    sc_export_base();
    sc_export_base(const char* name);

    // destructor

    virtual ~sc_export_base();

protected:

    // called when construction is done 
    virtual void before_end_of_elaboration();

    // called when elaboration is done (does nothing by default)
    virtual void end_of_elaboration();

    // called before simulation starts (does nothing by default)
    virtual void start_of_simulation();

    // called after simulation ends (does nothing)
    virtual void end_of_simulation();

    virtual const char* if_typename() const = 0;
 
    // error reporting
    void report_error( const char* id, const char* add_msg = 0) const;

private:

    void construction_done();
    void elaboration_done();
    void start_simulation();
    void simulation_done();

    // disabled
    sc_export_base(const this_type&);
    this_type& operator = (const this_type& );

};

//=============================================================================
//  CLASS : sc_export
//
//  Generic export class for other export classes. This
//  class provides a binding point for access to an interface.
//=============================================================================
template<class IF>
class sc_export : public sc_export_base
{
    typedef sc_export<IF> this_type;

public: // constructors:
    sc_export() : sc_export_base()
    {
	m_interface_p = 0;
    }

    explicit sc_export( const char* name_ ) : sc_export_base(name_)
    {
	m_interface_p = 0;
    }

public: // destructor:
    virtual ~sc_export() 
    {
    }

public: // interface access:

    virtual sc_interface* get_interface() 
    {
	return m_interface_p;
    }

    virtual const sc_interface* get_interface() const
    {
        return m_interface_p;
    }

    const IF* operator -> () const {
        if ( m_interface_p == 0 )
        {
            SC_REPORT_ERROR(SC_ID_SC_EXPORT_HAS_NO_INTERFACE_,name());
        }
        return m_interface_p;
    }

    IF* operator -> () {
        if ( m_interface_p == 0 )
        {
            SC_REPORT_ERROR(SC_ID_SC_EXPORT_HAS_NO_INTERFACE_,name());
        }
        return m_interface_p;
    }

    operator IF& ()
    {
	if ( m_interface_p == 0 )
	{
	    SC_REPORT_ERROR(SC_ID_SC_EXPORT_HAS_NO_INTERFACE_,name());
	}
	return *m_interface_p;
    }
    operator const IF&() const
        { return *const_cast<this_type*>(this); }


public: // binding:
    SC_VIRTUAL_ void bind( IF& interface_ )
    {
    	if ( m_interface_p )
	{
	    SC_REPORT_ERROR(SC_ID_SC_EXPORT_ALREADY_BOUND_,name());
	}
	else
	{
	    m_interface_p = &interface_;
	}
    }

    void operator () ( IF& interface_ )
    {
        this->bind(interface_);
    }

public: // identification:
    virtual const char* kind() const { return "sc_export"; }

protected:
  const char* if_typename() const {
    return typeid( IF ).name();
  }

private: // disabled
    sc_export( const this_type& );
    this_type& operator = ( const this_type& );

protected: // data fields:
    IF* m_interface_p;		// Interface this port provides.
};

// ----------------------------------------------------------------------------
//  CLASS : sc_export_registry
//
//  Registry for all exports.
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

class sc_export_registry
{
    friend class sc_simcontext;

public:

    void insert( sc_export_base* );
    void remove( sc_export_base* );

    int size() const
        { return m_export_vec.size(); }

private:

    // constructor
    explicit sc_export_registry( sc_simcontext& simc_ );

    // destructor
    ~sc_export_registry();

    // called when construction is done
    bool construction_done();

    // called when elaboration is done
    void elaboration_done();

    // called before simulation starts
    void start_simulation();

    // called after simulation ends
    void simulation_done();

private:

    int                          m_construction_done;
    std::vector<sc_export_base*> m_export_vec;
    sc_simcontext*               m_simc;

private:

    // disabled
    sc_export_registry();
    sc_export_registry( const sc_export_registry& );
    sc_export_registry& operator = ( const sc_export_registry& );
};

} // namespace sc_core

#undef SC_VIRTUAL_

// $Log: sc_export.h,v $
// Revision 1.7  2011/08/26 20:45:40  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.6  2011/05/09 04:07:37  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
// Revision 1.5  2011/04/02 00:02:14  acg
//  Philipp A. Hartmann: add const overload for sc_export::operator IF&
//
// Revision 1.4  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.3  2011/02/14 17:50:16  acg
//  Andy Goodrich: testing for sc_port and sc_export instantiations during
//  end of elaboration and issuing appropriate error messages.
//
// Revision 1.2  2011/01/20 16:52:15  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

#endif

// Taf!
