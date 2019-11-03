#include "systemc.h"

SC_MODULE(READ_LEAF) 
{
    SC_CTOR(READ_LEAF) 
    {
        SC_METHOD(delta);
        sensitive << in;
    }
    void delta()
    {
        cout << "READ_LEAF: change " << (int)in->read() << endl;
    }
    sc_in<sc_uint<8> >     in;
};

SC_MODULE(WRITE_LEAF) 
{
    SC_CTOR(WRITE_LEAF) : out("out"), clk("clk")
    {
                my_export(out);
		SC_METHOD(sync)
		sensitive << clk.pos();
    }
	void sync()
	{
		out = out.read() + 1;
	}
    sc_signal<sc_uint<8> > out;
    sc_export<sc_signal_in_if<sc_uint<8> > >     my_export;
	sc_in_clk			   clk;
};

SC_MODULE(MIDDLE) 
{
    SC_CTOR(MIDDLE) : reader("reader"), writer("writer")
    {
		writer.clk(clk);     // Bind clk going down the module hierarchy.
        my_port(writer.my_export); // Bind my_port coming up the module hierarchy.
        reader.in(my_port);  // Bind my_port going down the module hierarchy.
    }
	sc_in_clk			   clk;
    sc_export<sc_signal_in_if<sc_uint<8> > >     my_port;
	READ_LEAF			   reader;
	WRITE_LEAF			   writer;
};

SC_MODULE(TOP) 
{
    SC_CTOR(TOP) : down("down")
    {
		down.clk(clk);    // Bind clk going down the module hierarchy.
        in(down.my_port); // Bind in coming up the module hierarchy.
        SC_METHOD(delta);
        sensitive << in;
    }
    void delta()
    {
        cout << "TOP: change " << (int)in.read() << endl;
    }
	sc_in_clk			   clk;
    sc_in<sc_uint<8> >     in;
	MIDDLE     			   down;
};

int sc_main(int argc, char* arg[])
{
    sc_clock clock;
    TOP top("top");
	top.clk(clock);

    sc_start(10, SC_NS);
    return 0;
}

