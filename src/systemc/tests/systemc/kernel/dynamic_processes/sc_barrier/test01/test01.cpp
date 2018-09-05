#include "systemc.h"
#include "sc_barrier.h"
using sc_dp::sc_barrier;

SC_MODULE(X)
{
	SC_CTOR(X)
	{
		sc_thread_handle last_thread;

		SC_THREAD(a);
		SC_THREAD(b);
		SC_THREAD(c);

		m_barrier.initialize(3);
	}
	void a()
	{
		wait(5.0, SC_NS);
		m_barrier.wait();
		cout << sc_time_stamp() << " - a" << endl;
	}
	void b()
	{
		wait(11.0, SC_NS);
		m_barrier.wait();
		cout << sc_time_stamp() << " - b" << endl;
	}
	void c()
	{
		m_barrier.wait();
		cout << sc_time_stamp() << " - c" << endl;
	}
	sc_barrier   m_barrier;
};

int sc_main( int argc, char* argv[] )
{
	sc_clock clock;
	X x("x");

	sc_start(1000, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}

