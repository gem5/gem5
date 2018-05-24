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

  test07.cpp -- Test handling of process objects with living descendants

  Original Author: Philipp A. Hartmann, OFFIS

 *****************************************************************************/
/*****************************************************************************
  MODIFICATION LOG - modifiers, enter your name, affiliation, date and  
  changes you are making here.

      Name, Affiliation, Date: 
  Description of Modification: 

 *****************************************************************************/
// $Log: test07.cpp,v $
// Revision 1.2  2011/02/14 17:00:00  acg
//  Andy Goodrich: updated copyright and added cvs logging information inline.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>

void
dump_hierarchy(
    std::vector< sc_object* > const & objs = sc_get_top_level_objects()
  , unsigned level = 0
)
{
  if (!level)
    std::cout << "------ " << "(" << sc_time_stamp() << ")" << " ------"
      << std::endl;

  std::vector<sc_object*>::const_iterator it = objs.begin();
  for( ; it != objs.end(); it++ )
  {
    std::cout << std::string( level + 1, ' ' )
              << (*it)->name() << " (" << (*it)->kind() << ")";

    sc_process_handle h(*it);

    std::cout << ( h.valid() // is it a process? -> print state
                 ? (! h.terminated() ? " (running)" : " (terminated)"  )
                 : "" )
              << std::endl;

    dump_hierarchy( (*it)->get_child_objects(), level+1 );
  }

  if (!level)
    std::cout << "---------------------- " << std::endl;
}

struct my_object : sc_object {
  my_object( const char* name ) : sc_object( name ) {}
  ~my_object()
  {
    std::cout << "+++ " << this->name() << " deleted" << std::endl;
  }
};

SC_MODULE(DUT)
{
  SC_CTOR(DUT)
    : leaf_(0)
  {
    SC_THREAD(parent);
  }

  enum start_options
  {
    no_children      = 0,
    start_child_proc = 1,
    create_child_obj = 2,
    both_children    = 3
  };

  void child( start_options opt ){
    start();

    my_object local( "local" );

    if( opt & create_child_obj )
      leaf_=new my_object("dyn_obj");

    wait( 100, SC_NS );

    if( opt & start_child_proc )
      sc_spawn( sc_bind( &DUT::child, this, no_children )
              , "grandchild" );

    wait( 100, SC_NS );
    end();
  }

  void parent()
  {
    // only parent alive
    wait( 50, SC_NS );
    dump_hierarchy();

    sc_spawn( sc_bind( &DUT::child, this, start_child_proc ), "child0" );
    sc_spawn( sc_bind( &DUT::child, this, both_children ), "child1" );

    // direct children up and running
    wait( 50, SC_NS );
    dump_hierarchy();

    // grandchildren started, child object created
    wait( 100, SC_NS );
    dump_hierarchy();

    // direct children ended (zombies kept)
    wait( 100, SC_NS );
    dump_hierarchy();

    // grandhildren ended (zombie with child object kept)
    wait( 100, SC_NS );
    dump_hierarchy();

    // child object removed, zombies deleted
    delete leaf_; leaf_ = 0;
    wait( 100, SC_NS );
    dump_hierarchy();

    {
      // create another pair of children
      sc_process_handle
        h0 = sc_spawn( sc_bind( &DUT::child, this, start_child_proc ), "child0" ),
        h1 = sc_spawn( sc_bind( &DUT::child, this, start_child_proc ), "child1" );

      wait( 100, SC_NS );
      dump_hierarchy();

      // and kill them, after it has spawned its grandchild
      wait( 50, SC_NS );
      h0.kill();
      h1.kill( SC_INCLUDE_DESCENDANTS );

      std::cout << "+++ kills sent ... "
                << "(" << sc_time_stamp() << ")"
                << std::endl;

      // needed to avoid segfault
      //wait(SC_ZERO_TIME);

    } // drop handles

    wait( 50, SC_NS );
    dump_hierarchy();

    end();
  }

  void start()
  {
    std::cout
      << "+++ "
      << sc_get_current_process_handle().name()
      << " starting "
      << "(" << sc_time_stamp() << ")"
      << std::endl;
  }
  void end()
  {
    std::cout
      << "+++ "
      << sc_get_current_process_handle().name()
      << " exiting "
      << "(" << sc_time_stamp() << ")"
      << std::endl;
  }

  my_object* leaf_;
};


int sc_main( int, char*[] )
{
  DUT dut("dut");
  sc_start(900, SC_NS );
  // everything cleaned up (only module still alive)
  dump_hierarchy();

  return 0;
}
