#include "systemc.h"

// Interface
class C_if : virtual public sc_interface
{ 
public:
    virtual void run() = 0;
};

// Channel
class C : public C_if, public sc_channel
{ 
public:
    SC_CTOR(C) { }
    virtual void run() 
    { 
        cout << sc_time_stamp() << " In Channel run() " << endl; 
    }
};

// --- D: export channel C through IFP --------
SC_MODULE( D )
{
    sc_export<C_if> IFP;

    SC_CTOR( D ) 
	: IFP("IFP"),   // explicit name
	  m_C("C")
    {                
	IFP( m_C );     // bind sc_export->interface by name
    }
 private:
    C m_C;            // channel
};

// --- E: module with two interface-ports ---
SC_MODULE( E )
{
 private:
    C m_C;
    D m_D;

 public:
    sc_export<C_if> IFP1;
    sc_export<C_if> IFP2;
				 
    SC_CTOR( E ) 		 
	: m_C("C"),
	  m_D("D"),
          IFP1("IFP1")
    {				 
        IFP1( m_C );
	IFP2( m_D.IFP );          // bind sc_export->sc_export by name
        IFP1.get_interface();     // just to see whether it compiles
    }
};

// Module X connected to the channels through E
SC_MODULE( X )
{
    sc_port<C_if> P1;
    sc_port<C_if> P2;
    SC_CTOR(X) {
        SC_THREAD(run);
    }
    void run() {
        wait(10, SC_NS);
	P1->run();
	wait(10, SC_NS);
	P2->run();
    }
};

int sc_main(int argc, char** argv) {
  E the_E("E");
  X the_X("X");
  // port->IFP
  the_X.P1( the_E.IFP1 );
  the_X.P2( the_E.IFP2 );

  sc_start(17, SC_NS);
  the_E.IFP1->run();  // testing the operator-> of sc_export
  sc_start(50, SC_NS);

  return 0;
}
