#include "systemc.h"

SC_MODULE(X)
{
	SC_CTOR(X)
	{
		a(b);
	}
	sc_export<sc_signal_inout_if<int> > a;
	sc_export<sc_signal_inout_if<int> > b;
};

int sc_main(int argc, char* argv[])
{
	sc_clock clock;
	X        x("x");

	sc_start(1, SC_NS);
	return 0;
}
