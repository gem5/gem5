#include "systemc.h"

SC_MODULE(TB)
{
	SC_CTOR(TB)
	{
		SC_THREAD(exec);
		sensitive << m_clk.pos();
	}
	void exec()
	{
		for (;;)
		{
			wait(2);
			cout << sc_time_stamp() << endl;
			wait(4);
			cout << sc_time_stamp() << endl;
			wait(1);
			cout << sc_time_stamp() << endl;
			wait(1000);
			cout << sc_time_stamp() << endl;
			sc_stop();
		}
	}
	sc_in_clk m_clk;
};

int sc_main( int, char*[] )
{
	sc_clock clock;
    TB       tb("tb");

	tb.m_clk(clock);
	sc_start(2000, SC_NS);

    return 0;
}
