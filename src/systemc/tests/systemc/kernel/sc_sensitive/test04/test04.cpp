#include "systemc.h"

SC_MODULE(X)
{
	SC_CTOR(X)
	{
		SC_THREAD(able);
		sensitive << clk.pos();
	}
		
	void able()
	{
		for (;;)
		{
			wait();
			sensitive << clk.posedge_event();
			cout << "able: " << sc_time_stamp() << endl;
		}
	}
	sc_in_clk clk;
};

int sc_main(int argc, char* argv[])
{
	sc_clock clock;
	X        x("x");
	x.clk(clock);

	sc_start(100, SC_NS);
	return 0;
}
