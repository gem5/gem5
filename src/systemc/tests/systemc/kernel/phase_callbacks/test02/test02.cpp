#include "systemc.h"

class Sig : public sc_prim_channel {
  public:
	virtual void before_end_of_elaboration()
	{
		 cout << "prim_channel: before end of elaboration" << endl;
	}
	virtual void end_of_simulation()
	{
		 cout << "prim_channel: end of simulation" << endl;
	}
	virtual void start_of_simulation()
	{
		 cout << "prim_channel: start of simulation" << endl;
	}
};

SC_MODULE(X)
{
	SC_CTOR(X)
	{
		SC_CTHREAD(y, clk.pos());
	}
	void y()
	{
		wait();
	}
	sc_in_clk clk;
};

int sc_main(int argc, char* argv[])
{
	sc_clock clock;
	Sig      signal;
	X        x("x");

	x.clk(clock);

	if ( sc_start_of_simulation_invoked() ) 
		 cout << __FILE__ << "(" << __LINE__ << "): bad start flag should be false" << endl;
	if ( sc_end_of_simulation_invoked() ) 
		 cout << __FILE__ << "(" << __LINE__ << "): bad end flag should be false" << endl;

	sc_start(2, SC_NS);
	if ( !sc_start_of_simulation_invoked() ) 
		 cout << __FILE__ << "(" << __LINE__ << "): bad start flag should be true" << endl;

	sc_stop(); 
	if ( !sc_end_of_simulation_invoked() ) 
		 cout << __FILE__ << "(" << __LINE__ << "): bad end flag should be true" << endl;

	 cerr << "Program completed" << endl;

    return 0;
}

