#include "systemc.h"

SC_MODULE(TB)
{
	SC_CTOR(TB)
	{
		SC_METHOD(method)
		sensitive << m_flipper;
		SC_THREAD(thread)
	}
	void method()
	{
	}
	void thread()
	{
		m_flipper = !m_flipper;
		wait(2, SC_NS);
		cout << sc_delta_count() << endl;
		m_flipper = !m_flipper;
		wait(3, SC_NS);
		cout << sc_delta_count() << endl;
		sc_stop();
	}
	sc_signal<bool> m_flipper;
};

int sc_main(int argc, char* argv[])
{
	TB		tb("tb");
	sc_start();
	cout << "Program completed after " << sc_time_stamp() << endl;
	return 0;
}
