#include "systemc.h"

SC_MODULE(X)
{
	SC_CTOR(X) : a("a")
	{
	}
	sc_export<sc_signal_inout_if<int> > a;
};

int sc_main(int argc, char* argv[])
{
	X        x("x");

	sc_start(1, SC_NS);
	return 0;
}
