//----------------------------------------------------------------------
//   Copyright 2009 Cadence Design Systems, Inc.
//   All Rights Reserved Worldwide
//   Copyright 2009 Forte Design Systems, Inc.
//   Copyright 2010 OFFIS Institute for Information technology
// 
// test06: test hierarchical kills
//----------------------------------------------------------------------

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>

SC_MODULE(top) {
public:
  SC_CTOR(top) {
    SC_THREAD(parent);
      sensitive << clk.pos();
      dont_initialize();
  }

  void proc_tree( unsigned depth, unsigned width, bool as_method, bool spawn_only )
  {
    unsigned w = width;
    if (sc_time_stamp() == SC_ZERO_TIME || spawn_only )
      while( depth && w --> 0 )
    {
      sc_spawn_options sp;
      sp.set_sensitivity( &clk.pos() );

      if(as_method) // we are spawned as method, spawn a thread
      {
        sc_spawn( sc_bind( &top::proc_tree, this, depth-1, width, !as_method, false )
                , sc_gen_unique_name("thread"), &sp );
      }
      else // we are spawned as thread, spawn a method
      {
        sp.spawn_method();
        sc_spawn( sc_bind( &top::proc_tree, this, depth-1, width, !as_method, false )
                , sc_gen_unique_name("method"), &sp );
      }
    }

    if(spawn_only) return;

    std::cout << sc_get_current_process_handle().name()
              << " triggered "
                << "(" << sc_time_stamp() << " @ " << sc_delta_count() << ")"
              << std::endl;

    // start thread
    if( !as_method ) thread_loop();
  }

  void thread_loop()
  {
    struct local_ {
       ~local_(){
          std::cout
            << sc_get_current_process_handle().name()
            << " local deleted "
            << "(" << sc_time_stamp() << " @ " << sc_delta_count() << ")"
            << std::endl;
       }
    } l; l=l;

    unsigned rounds = 5;
    while( rounds --> 0 )
    {
      wait();
      std::cout << sc_get_current_process_handle().name()
                << " triggered "
                << "(" << sc_time_stamp() << " @ " << sc_delta_count() << ")"
                << std::endl;
    }
    std::cout << sc_get_current_process_handle().name()
              << " ended "
              << "(" << sc_time_stamp() << " @ " << sc_delta_count() << ")"
              << std::endl;
  }

  void parent()
  {
    proc_tree( 3, 1, true , true );
    proc_tree( 3, 1, false, true );

    wait();

    // copy children (needed, since children may get reordered)
    std::vector< sc_object* > children =
      sc_get_current_process_handle().get_child_objects();

    std::vector< sc_object* >::const_iterator it = children.begin();

    while( it != children.end() )
    {
      sc_process_handle h( *it++ );
      sc_assert( h.valid() );

      std::cout << h.name() << " "
                << "kill requested "
                << "(" << h.get_process_object()->kind() << ") "
                << "(" << sc_time_stamp() << " @ " << sc_delta_count() << ")"
                << std::endl;

      h.kill( SC_INCLUDE_DESCENDANTS );
    }

    wait();

    std::cout << sc_get_current_process_handle().name()
              << " ended "
              << "(" << sc_time_stamp() << " @ " << sc_delta_count() << ")"
              << std::endl;

    wait();
    sc_stop();
    while(true) wait();
  }

  sc_in<bool> clk;
};

int sc_main (int argc, char *argv[])
{
  sc_clock clk("clk", 10, SC_NS, 0.5);
  top t("top");
  t.clk(clk);
  sc_start();
  return 0;
}
