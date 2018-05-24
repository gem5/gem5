#include "systemc.h" 

SC_MODULE(TB)
{
	SC_CTOR(TB)
	{
		SC_METHOD(sync);
		sensitive << m_clk.pos();
		SC_METHOD(sync);
		sensitive << m_clk.pos();
	}
	void sync()
	{
		sc_curr_proc_handle cpi = 
			sc_get_curr_simcontext()->get_curr_proc_info();
		cout << sc_time_stamp() << ": " << cpi->process_handle->name() << endl;
	}
	sc_in_clk m_clk;
};

int sc_main(int argc,char **argv) 
{ 
    sc_clock clock;
	TB       tb("tb");

	tb.m_clk(clock);
	sc_start(2, SC_NS);

	cerr << "Program completed" << endl;
    return (0); 
} 
