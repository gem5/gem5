#include <systemc.h>

SC_MODULE(Rec) {
  sc_event_queue_port E;

  SC_CTOR(Rec) {
    SC_METHOD(P);
    sensitive << E;
    dont_initialize();
  }
  void P() {
    cout << sc_time_stamp() 
	 << " delta " << sc_delta_count()
	 << ": P awakes\n";
  }
};

SC_MODULE(Sender) {
  sc_in<bool> Clock;
  sc_event_queue_port E;

  SC_CTOR(Sender) {
      SC_METHOD(P);
      sensitive << Clock.pos();
      dont_initialize();
  }
  void P() {
      // trigger in now (2x), now+1ns (2x)
      E->notify( 0, SC_NS );
      E->notify( 0, SC_NS );
      E->notify( 1, SC_NS );
      E->notify( 1, SC_NS );
  }
};

SC_MODULE(xyz) {
  SC_CTOR(xyz) {
      SC_THREAD(P);
  }
  void P() {
      wait(15, SC_NS);
      cout << sc_time_stamp() 
	   << " delta " << sc_delta_count()
	   << ": xyz awakes\n";	  
  }
};

int sc_main (int agrc, char** argv)
{
  sc_event_queue E("E");

  Rec R("Rec");
  R.E(E);

  sc_clock C1 ("C1", 20, SC_NS);
  sc_clock C2 ("C2", 40, SC_NS);

  xyz xyz_obj("xyz");

  // Events at 0ns (2x), 1ns (2x),   20ns (2x), 21ns (2x),   40ns (2x), ...
  Sender S1("S1");
  S1.Clock(C1);
  S1.E(E);

  // Events at 0ns (2x), 1ns (2x),   40ns (2x), 41ns (2x),   80ns (2x), ...
  Sender S2("S2");
  S2.Clock(C2);
  S2.E(E);

  // Events at 3ns, 5ns (2x), 8ns
  sc_start(10, SC_NS);
  E.notify( 5,SC_NS );
  E.notify( 3,SC_NS );
  E.notify( 5,SC_NS );
  E.notify( 8,SC_NS) ;

  // Events would be at 40ns, 43ns (2x), 44ns but all are cancelled
  sc_start(40, SC_NS);
  E.notify( 3, SC_NS );
  E.notify( 3, SC_NS );
  E.notify( 4, SC_NS );
  E.notify( SC_ZERO_TIME );
  E.cancel_all();

  sc_start(40, SC_NS);

  return 0;
}
