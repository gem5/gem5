#include "systemc.h"

SC_MODULE(Y)
{
public:
        sc_in<bool> in;
        SC_CTOR(Y) : in("in")
	{
		SC_METHOD(comb);
		sensitive << in;
		init = 1;
	}
	int             init;
	void comb()
	{
		if ( init )
		{
			init = 0;
			SC_THREAD(dork);
		}
	}
	void dork()
	{
		cout << "dork" << endl;
	}
};

int sc_main(int argc, char* arg[])
{
	sc_clock clock;
	Y        y("y");
        y.in(clock);


	sc_start(10, SC_NS);
	cout << "Program completed" << endl;

    return 0;
}

