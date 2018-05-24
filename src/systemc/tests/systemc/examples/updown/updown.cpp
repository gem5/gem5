#include "systemc.h"
#include "specialized_signals/scx_signal_int.h"
#include "specialized_signals/scx_signal_signed.h"
#include "specialized_signals/scx_signal_uint.h"
#include "specialized_signals/scx_signal_unsigned.h"

SC_MODULE(up_down)
{
    sc_in_clk             clk;
    sc_in<sc_uint<1> >    up;
    sc_in<sc_uint<1> >    down; 
    sc_in<sc_uint<9> >    data_in;
    sc_inout<sc_uint<1> > parity_out;
    sc_inout<sc_uint<1> > carry_out;
    sc_inout<sc_uint<1> > borrow_out;
    sc_inout<sc_uint<9> > count_out;

    sc_uint<10>           cnt_dn;
    sc_uint<10>           cnt_up;
    sc_uint<9>            count_nxt;
    sc_uint<1>            load;
     
	SC_CTOR(up_down)
	{
		SC_METHOD(run);
		sensitive << clk.pos();
	}

     void run()
	 {
        cnt_dn = count_out - 5;
        cnt_up = count_out + 3;
        
        load = 1;
        switch( (unsigned int )(up,down) ) {
          case(0): 
		    count_nxt = data_in;
            break;
          case(1):
            count_nxt = cnt_dn;
            break;
          case(2):
            count_nxt = cnt_up;
            break;
          case(3):
            load = 0;
            break;
        }

        if( load)  {
                parity_out = count_nxt.xor_reduce();
                carry_out = up&cnt_up[9];
                borrow_out = down&cnt_dn[9];
                count_out = count_nxt;
       }
    }
};


#define UP_DOWN(up, down) up, down

struct stimulus {
   int up;
   int down;
   int data_in;
} s[] = {
	{ UP_DOWN(0, 0), 200 }, /* load 200 */
        { UP_DOWN(1, 0), 0 },   /* inc */
        { UP_DOWN(1, 0), 0 },   /* inc */
        { UP_DOWN(0, 1), 0 },   /* dec */
        { UP_DOWN(0, 1), 0 },   /* dec */
        { UP_DOWN(0, 1), 0 },   /* dec */
	{ UP_DOWN(1, 1), 0 },   /* hold */
	{ UP_DOWN(1, 1), 0 },   /* hold */
	{ UP_DOWN(1, 1), 0 },   /* hold */
	{ UP_DOWN(0, 0), 200 }, /* load 200 */
	{ UP_DOWN(1, 1), 0 },   /* hold */
	{ UP_DOWN(1, 1), 0 },   /* hold */
        { UP_DOWN(0, 1), 0 },   /* dec */
	{ UP_DOWN(0, 0), 2 },   /* load 2 */
        { UP_DOWN(0, 1), 0 },   /* dec */
	{ UP_DOWN(0, 0), 0x1ff},/* load 0x1ff */
        { UP_DOWN(1, 0), 0 },   /* inc */
};


int sc_main(int argc, char* argv[])
{
    sc_signal<sc_uint<1> > borrow_out;
    sc_signal<sc_uint<1> > carry_out;
	sc_clock			   clock;
    sc_signal<sc_uint<9> > count_out;
    sc_signal<sc_uint<9> > data_in;
    sc_signal<sc_uint<1> > down;
    unsigned int           i;
    sc_signal<sc_uint<1> > parity_out;
    sc_signal<sc_uint<1> > up;

    up_down up_down_0("up_down_0");
		up_down_0.borrow_out(borrow_out);
		up_down_0.carry_out(carry_out);
		up_down_0.data_in(data_in);
		up_down_0.clk(clock);
		up_down_0.count_out(count_out);
		up_down_0.down(down);
		up_down_0.parity_out(parity_out);
		up_down_0.up(up);

    printf("%5s %2s %4s %7s %10s %8s %10s %9s\n",
    	   "clock", "up", "down", "data_in", "parity_out",
	   "carry_out", "borrow_out", "count_out");

    for( i = 0; i < sizeof s/sizeof(struct stimulus); i++) {
        up = s[i].up;
        down = s[i].down;
        data_in = s[i].data_in;

		sc_start(1, SC_NS);

        printf("%5d %2d %4d %7d %10d %8d %10d %9d\n",
			(int)sc_time_stamp().to_double()/1000, (int)up, (int)down, 
			(int)data_in, (int)parity_out, (int)carry_out, (int)borrow_out,
			(int)count_out);
    }

    /* get last register values */
    printf("%5d %2d %4d %7d %10d %8d %10d %9d\n",
		(int)sc_time_stamp().to_double()/1000, (int)up, (int)down, 
		(int)data_in, (int)parity_out, (int)carry_out, (int)borrow_out,
		(int)count_out);

	return 0;
}
	
