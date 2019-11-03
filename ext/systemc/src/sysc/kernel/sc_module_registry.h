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

  sc_module_registry.h -- Registry for all modules.
                          FOR INTERNAL USE ONLY.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_MODULE_REGISTRY_H
#define SC_MODULE_REGISTRY_H


namespace sc_core {

class sc_module;
class sc_simcontext;


// ----------------------------------------------------------------------------
//  CLASS : sc_module_registry
//
//  Registry for all modules.
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

class sc_module_registry
{
    friend class sc_simcontext;

public:

    void insert( sc_module& );
    void remove( sc_module& );

    int size() const
        { return m_module_vec.size(); }

private:

    // constructor
    explicit sc_module_registry( sc_simcontext& simc_ );

    // destructor
    ~sc_module_registry();

    // called when construction is done
    bool construction_done();

    // called when elaboration is done
    void elaboration_done();

    // called before simulation begins
    void start_simulation();

    // called after simulation ends
    void simulation_done();


private:

    int                     m_construction_done;
    std::vector<sc_module*> m_module_vec;
    sc_simcontext*          m_simc;

private:

    // disabled
    sc_module_registry();
    sc_module_registry( const sc_module_registry& );
    sc_module_registry& operator = ( const sc_module_registry& );
};

} // namespace sc_core

#endif

// $Log: sc_module_registry.h,v $
// Revision 1.6  2011/08/26 20:46:10  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.5  2011/05/09 04:07:49  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
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
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.

// Taf!
