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

// sc_vector.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: sc_vector.cpp,v $
// Revision 1.3  2011/07/24 16:04:12  acg
//  Philipp A. Hartmann: remove assert() that does not test the correct thing.
//
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// sc_vector

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc>
#include <cstring>

using namespace sc_core;
using std::cout;
using std::endl;
using std::string;

struct Sub: sc_module
{
  sc_inout<int> p;
  
  Sub(sc_module_name)
  {
    SC_THREAD(T);
  }
  
  void T()
  {
    for (;;)
    {
      wait(p.default_event());
    }
  }
  
  SC_HAS_PROCESS(Sub);
};


struct my_module: sc_module
{
  my_module(sc_module_name n, string weird_arg )
  {
    sc_assert( weird_arg == "The Creator" || weird_arg == "WeIrD_aRg" );
  }
};


struct my_object: sc_object
{
  my_object()              : sc_object()  {}
  my_object(const char* c) : sc_object(c) {}
};


struct i_f: virtual sc_interface
{
  virtual void method() = 0;
};


struct Initiator: sc_module
{
  sc_vector<sc_port<i_f> > ports;
  
  Initiator(sc_module_name)
  : ports("ports", 4)
  {
    SC_THREAD(T);
  }
  
  void T()
  {
    for (unsigned int i = 0; i < ports.size(); i++)  // Use method size() with vector
    {
      wait(10, SC_NS);
      ports[i]->method();  // Use operator[] with vector
    }
  }
  SC_HAS_PROCESS(Initiator);
};


struct Initiator1: sc_module
{
  sc_port<i_f> port;
  
  Initiator1(sc_module_name)
  : port("port")
  {
    SC_THREAD(T);
  }
  
  void T()
  {
    wait(10, SC_NS);
    port->method();
  }
  SC_HAS_PROCESS(Initiator1);
};


struct Target: public sc_module, private i_f
{
  sc_export<i_f> xp;
  
  Target(sc_module_name)
  : xp("xp")
  {
    xp.bind( *this );
  }
  
  virtual void method() { 
    cout << "Called method() in " << name() << " at " << sc_time_stamp() << endl;
  }
};


typedef sc_vector<sc_inout<int> > port_type;

struct M: sc_module
{
  port_type ports; // Vector-of-ports
  
  sc_vector<Sub> kids; // Vector-of-modules, each with a port p
  sc_vector<Sub> kids2;
  sc_vector<sc_signal<int> > sigs2;
  sc_vector<sc_signal<int> > sigs3;
  
  int dim;
  
  Initiator*        initiator;  
  sc_vector<Target> targets;
  
  sc_vector<Initiator1> initiator_vec;
  sc_vector<Target>     target_vec;

  
  sc_vector<my_module> my_vec_of_modules;
  
  struct my_module_creator
  {
    my_module_creator( string arg ) : weird_arg(arg) {}
    
    my_module* operator() (const char* name, size_t)
    {
      return new my_module(name, weird_arg );
    }
    string weird_arg;
  };
  
  sc_vector<my_module> my_vec_of_modules2;

  // If creator is not a function object, it could be a function
  my_module* my_module_creator_func( const char* name, size_t i )
  {
    creator_func_called = true;
    return new my_module( name, "WeIrD_aRg" );
  }
  
  bool creator_func_called;


  M(sc_module_name _name, int N)
  : ports("ports", N)
  , kids("kids")  // Construct the vector with name seed "kids"
  , kids2("kids2", 8)
  , sigs2("sigs2", 4)
  , sigs3("sigs3", 4)
  , dim(N)
  , targets("targets", N)
  , initiator_vec("initiator_vec", N)
  , target_vec("target_vec", N)
  , my_vec_of_modules("my_vec_of_modules")
  , creator_func_called(false)
  {
    //vec.init(N);  // Alternative initialization (instead of passing N to ctor)
    kids.init(N);
    
    sc_assert( ports.size() == static_cast<unsigned int>(N) );
    sc_assert( kids.size()  == static_cast<unsigned int>(N) );
   
    // Using vector view to create vector-of-ports
    sc_assemble_vector(kids, &Sub::p).bind( ports ); 
 
    for (unsigned int i = 0; i < kids.size(); i++)
    {
      sc_assert( kids[i].p.get_interface() == ports[i].get_interface() );
    }    
   
    initiator = new Initiator("initiator");
    
    // Using vector view to create vector-of-exports
    initiator->ports.bind( sc_assemble_vector(targets, &Target::xp) ); 
    
    // Double whammy
    sc_assemble_vector(initiator_vec, &Initiator1::port).bind( sc_assemble_vector(target_vec, &Target::xp) );
    
    // sc_vector_view has no public constructors, though it is copyable
    sc_vector_assembly< Initiator1, sc_port<i_f> > assembly = sc_assemble_vector(initiator_vec, &Initiator1::port); 

    sc_assert( &*(assembly.begin()) == &(*initiator_vec.begin()).port );
    // sc_assert( &*(assembly.end())   == &(*initiator_vec.end()).port );
    sc_assert( assembly.size()  == initiator_vec.size() );
    for (unsigned int i = 0; i < assembly.size(); i++)
    {
      sc_assert( &assembly[i]    == &initiator_vec[i].port );
      sc_assert( &assembly.at(i) == &initiator_vec[i].port );
    }

    std::vector<sc_object*> elements;

    // sc_vector_view (aka sc_vector_assembly) acts as a proxy for sc_vector
    // It has begin() end() size() operator[] bind() etc
    
    elements = assembly.get_elements();
    for ( unsigned i = 0; i < elements.size(); i++ )
      if ( elements[i] )
        sc_assert( elements[i] == &initiator_vec[i].port );
       
    elements = ports.get_elements();
    for ( unsigned i = 0; i < elements.size(); i++ )
      if ( elements[i] )
        sc_assert( elements[i] == &ports[i] );

    elements = sc_assemble_vector(initiator_vec, &Initiator1::port).get_elements();
    for ( unsigned i = 0; i < elements.size(); i++ )
      if ( elements[i] )
        sc_assert( elements[i] == &initiator_vec[i].port );
        
    // Additional method to support a vector iterator as an offset
    sc_assemble_vector(kids2, &Sub::p).bind( sigs2.begin(), sigs2.end(), kids2.begin());
    sc_assemble_vector(kids2, &Sub::p).bind( sigs3.begin(), sigs3.end(), kids2.begin() + 4 );

    // Construct elements of sc_vector, passing through ctor arguments to user-defined sc_module     
    my_vec_of_modules.init(N, my_module_creator("The Creator"));
    
    // Alternatively, instead of a function object pass in a function
    my_vec_of_modules2.init(N, sc_bind( &M::my_module_creator_func, this, sc_unnamed::_1, sc_unnamed::_2 ) );

    // Well-formedness check on creator function call
    my_module_creator foo("The Creator");
    const char* nm = "foo";
    unsigned int idx = 0;
    my_module* next = foo( (const char*)nm, (sc_vector<my_module>::size_type)idx );
    char buf[80];
    strcpy(buf, this->name());
    strcat(buf, ".foo");
    sc_assert( strcmp(next->name(), buf) == 0 );
    delete next;
    
    SC_THREAD(T);
  }
  
  void T()
  {
    int j = 0;
    for (int i = 0; i < 10; i++)
    {
      wait(10, SC_NS);
      ports[i % dim].write((j++) % 10);  // Use operator[] with vector
    }
  }
  
  SC_HAS_PROCESS(M);
};

struct Top: sc_module
{
  sc_vector<sc_signal<int> > sigs; // Vector-of-signals
  sc_vector<sc_signal<int> > more_sigs; // Vector-of-signals
  sc_vector<sc_signal<int> > hi_sigs; // Vector-of-signals
  sc_vector<sc_signal<int> > lo_sigs; // Vector-of-signals
  
  M *m1, *m2, *m3;
  
  Top(sc_module_name _name)
  : sigs("sigs", 4)
  , more_sigs("more_sigs", 4)
  , hi_sigs("hi_sigs", 2)
  , lo_sigs("lo_sigs", 2)
  {
    m1 = new M("m1", 4);
    m2 = new M("m2", 4);
    m3 = new M("m3", 4);

    for (int i = 0; i < 4; i++)
      m1->ports[i].bind( sigs[i] );  // Using operator[] with a vector
    
    port_type::iterator it = m2->ports.bind( more_sigs ); // Vector-to-vector bind 
    sc_assert( (it - m2->ports.begin()) == 4 );

    // Bind upper half of ports vector to hi_sigs    
    it = m3->ports.bind( hi_sigs.begin(), hi_sigs.end() ); 
    sc_assert( (it - m3->ports.begin()) == 2 );

    // Bind lower half of ports vector to lo_sigs    
    it = m3->ports.bind( lo_sigs.begin(), lo_sigs.end(), it); 
    sc_assert( (it - m3->ports.begin()) == 4 );
    
    SC_THREAD(T);
    SC_THREAD(T2);
  }
  
  void T()
  {
    sc_event_or_list list;
    for (int i = 0; i < 4; i++)
      list |= sigs[i].default_event();
    for (;;)
    {
      wait(list);
      cout << "Top:" << sigs[0] << sigs[1] << sigs[2] << sigs[3] << endl;
    }
  }


  void T2()
  {
    wait(10, SC_US);

    // Create sc_vector during simulation   
    sc_vector<my_object> vec_obj("vec_obj", 4);
    sc_assert( vec_obj.size() == 4 );
    for (unsigned int i = 0; i < vec_obj.size(); i++)
      cout << "vec_obj[" << i << "].name() = " << vec_obj[i].name() << endl;
    
    sc_object* proc = sc_get_current_process_handle().get_process_object();
    std::vector<sc_object*> children = proc->get_child_objects();

    sc_assert( children.size() == 5 ); // sc_vector itself + 4 X my_object
  }

  SC_HAS_PROCESS(Top);
};


int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  std::vector<sc_object*> children = top.get_child_objects();
  sc_assert( children.size() == 21 ); // sc_vectors themselves are sc_objects
  
  sc_start();
  
  sc_assert( top.m1->creator_func_called );
  sc_assert( top.m2->creator_func_called );
  sc_assert( top.m3->creator_func_called );
  
  cout << endl << "Success" << endl;
  return 0;
}
  
