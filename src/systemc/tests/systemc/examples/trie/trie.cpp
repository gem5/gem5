#include <stdio.h>
#include "systemc.h"

SC_MODULE(lc)
{
	sc_out<bool> DIN_rdy;
	sc_in<bool>  DIN_vld;
#define LOAD_ADDR   0
#define LOAD_DATA   1
#define LOOKUP 2 // really op(1) == 1 and op(0) - don't care
	sc_in<sc_uint<2> >  DIN_op;
	sc_in<sc_uint<32> > DIN_data_in;
	sc_in<bool>  DOUT_rdy;
	sc_out<bool> DOUT_vld;
	sc_out<sc_uint<32> > DOUT_hop;
	sc_in<bool> RSTN;
	sc_in_clk CLK;

	void thread();
	sc_uint<20> extract(sc_uint<32>, sc_uint<7>, sc_uint<5>);
	sc_uint<10> lookup(sc_uint<32>);

	sc_uint<32> M[1024];
	sc_uint<32>  addr;	// latched addr for loading memory

	SC_CTOR(lc){
		SC_CTHREAD(thread, CLK.pos());
		reset_signal_is(RSTN,false);
	}
};

//
// this implements the level-compressed trie
// ip routing algorithm that is proposed in:
//
// "Fast Address Lookup for Internet Routers"
// Stefan Nilsson and Gunnar Karlsson
// Proc IFIP 4th International Conference on BroadBand Communication
// pp. 11-22, 1998
//
// 
void
lc::thread()
{
	if( !RSTN){
		DIN_rdy = 1;
		DOUT_vld = 0;
		wait(1);
	}

	for(;;){
		bool tmp_vld;
		bool tmp_rdy;
		sc_uint<2> op;
		sc_uint<32> data_in;
		sc_uint<32> hop;

                {
                do {
                	tmp_vld = DIN_vld.read();
                        op = DIN_op.read();
                        data_in = DIN_data_in.read();
                        wait(1);
                } while( !tmp_vld);
                }

		if( op == 0){
			// latch address
			addr = data_in;
			wait(1);
		} else if(op == 1){
			// load memory
			M[addr] = data_in;
			wait(1);
		} else if( op[1] == 1){
			// do the ip lookup to next hop
			hop = lookup(data_in);
			{
				DIN_rdy = 0;
				do {
					tmp_rdy = DOUT_rdy.read();
					wait(1);
				} while( !tmp_rdy);
				DOUT_vld = 1;
				DOUT_hop.write(hop);
				DIN_rdy = 1;
				wait(1);
				DOUT_vld = 0;
			}
		}
	}

}

sc_uint<20>
lc::extract(sc_uint<32> x, sc_uint<7> p, sc_uint<5> w)
{
	return( (x>>(p-w+1)) & ~(~0<<w) );
}

#define BRANCH(T)	T.range(31,27)
#define SKIP(T)		T.range(26,20)
#define ADDR(T)		T.range(19,0)
#define LEN(T)		T.range(31,25)
#define NEXT_HOP(T)	T.range(24,15)
#define NEXT_PREFIX(T)	T.range(14,0)

sc_uint<10>
lc::lookup(sc_uint<32> ip)
{
	sc_uint<7> pos;
	sc_uint<5> branch;
	sc_uint<20> addr;
	sc_uint<32> t;
	sc_uint<32> ip_prefix;
	sc_uint<10> next_hop;
	sc_uint<32> mask;

	t = M[0];
	pos = SKIP(t);
	branch = BRANCH(t);
	addr = ADDR(t);
	while(branch != 0){
		addr = addr + extract(ip, pos, branch);
		t = M[addr];
		pos = pos -  (branch + SKIP(t));
		branch = BRANCH(t);
		addr = ADDR(t);
	}


	next_hop = 0;
	for(;;) {
		addr <<= 1;

		ip_prefix = M[addr];
		t = M[addr|1];
		mask = ~0 << (32-LEN(t));
		if( (ip_prefix&mask) == (ip&mask)){
			next_hop = NEXT_HOP(t);
			break;
		} 
		addr = NEXT_PREFIX(t);
		if( addr == 0)
			break;
	}

	return(next_hop);
}


/*
 * hop	prefix(bits 31 .. 0)
 *  1   0000*
 *  2   0001*
 *  3   00101*
 *  4   010*
 *  5   0110*
 *  6   0111*
 *  7   100*
 *  8   101000*
 *  9   101001*
 * 10   10101*
 * 11   10110*
 * 12   10111* 
 * 13   110*
 * 14   11101000*
 * 15   11101001*
 *
 * "not in table" produces a hop of 0
 */

#define TRIE(LN, SK, AD) \
  ((((LN)&0x1f)<<27) | (((SK)&0x7f)<<20) | ((AD)&0xfffff))
#define E(LN, NHP, NPX) \
  ((((LN)&0x7f)<<25) | (((NHP)&0x3ff)<<15) | ((((NPX))&0x7fff)))


#define M_SIZE 52
sc_uint<32> M[M_SIZE] = {
/* TRIE */
/* 00 */ TRIE(3, 31, 1),
/* 01 */ TRIE(1, 0, 9),
/* 02 */ TRIE(0, 2, 2+11),
/* 03 */ TRIE(0, 0, 3+11),
/* 04 */ TRIE(1, 0, 11),
/* 05 */ TRIE(0, 0, 6+11),
/* 06 */ TRIE(2, 0, 13),
/* 07 */ TRIE(0, 0, 12+11), 
/* 08 */ TRIE(1, 4, 17), 
/* 09 */ TRIE(0, 0, 0+11),
/* 10 */ TRIE(0, 0, 1+11), 
/* 11 */ TRIE(0, 0, 4+11),
/* 12 */ TRIE(0, 0, 5+11),
/* 13 */ TRIE(1, 0, 19), 
/* 14 */ TRIE(0, 0, 9+11),
/* 15 */ TRIE(0, 0, 10+11),
/* 16 */ TRIE(0, 0, 11+11),
/* 17 */ TRIE(0, 0, 13+11),
/* 18 */ TRIE(0, 0, 14+11),
/* 19 */ TRIE(0, 0, 7+11),
/* 20 */ TRIE(0, 0, 8+11),
/* 21 */ 0, 			/* pad */
/* BASE + PREFIX */
/* 22 */ 0x00000000, E(4, 1, 0),
/* 24 */ 0x10000000, E(4, 2, 0),
/* 26 */ 0x28000000, E(5, 3, 0),
/* 28 */ 0x40000000, E(3, 4, 0),
/* 31 */ 0x60000000, E(4, 5, 0),
/* 32 */ 0x70000000, E(4, 6, 0),
/* 34 */ 0x80000000, E(3, 7, 0),
/* 36 */ 0xa0000000, E(6, 8, 0),
/* 38 */ 0xa4000000, E(6, 9, 0),
/* 40 */ 0xa8000000, E(5, 10, 0),
/* 42 */ 0xb0000000, E(5, 11, 0),
/* 44 */ 0xb8000000, E(5, 12, 0),
/* 46 */ 0xc0000000, E(3, 13, 0),
/* 48 */ 0xe8000000, E(8, 14, 0),
/* 50 */ 0xe9000000, E(8, 15, 0)
};


struct stimuli {
	unsigned int ip;
	int hop;
} S[] = {
	{ 0xf0000000, 0 },
	{ 0x10000000, 2 }, 
	{ 0x7c000000, 6 }, 
	{ 0x7c001000, 6 }, 
	{ 0x7c000070, 6 }, 
};

int
sc_main(int argc, char *argv[])
{	
	sc_clock clk;
	sc_signal<bool> reset;
 	sc_signal<bool> in_rdy;
        sc_signal<bool> in_vld;
        sc_signal<bool> out_vld;
        sc_signal<bool> out_rdy;
        sc_signal<sc_uint<2> > op;
        sc_signal<sc_uint<32> > data;
        sc_signal<sc_uint<32> > hop;
	int i;
	int m_addr;
	lc *lcp;

	lcp = new lc("lc0");
	(*lcp)(in_rdy, in_vld,
	      op, data,
	      out_rdy, out_vld,
	      hop,
	      reset, clk);

	reset = 0;
	sc_start(2, SC_NS);
	reset = 1;
	sc_start(2, SC_NS);
	out_rdy = 1;

	/*
	 * download the RAM containing
	 * the routing data
	 */
	for(i = 0, m_addr = 0; i < M_SIZE; i++, m_addr++){
		while(!in_rdy) sc_start(1, SC_NS);
		in_vld = 1;
		op.write(LOAD_ADDR);
		data.write(m_addr);
		do { sc_start(1, SC_NS); in_vld = 0; } while(!in_rdy);
		sc_start(1, SC_NS);
		in_vld = 1;
		op.write(LOAD_DATA);
		data.write(M[m_addr]);
		do { sc_start(1, SC_NS); in_vld = 0; } while(!in_rdy);
		sc_start(1, SC_NS);
	}

	/*
	 * apply some ip's and see what
 	 * comes back as next-hops
	 */
	for(i = 0; i < sizeof S/sizeof(struct stimuli); i++){
		while(!in_rdy) sc_start(1, SC_NS);
		in_vld = 1;
		op.write(LOOKUP);
		data.write(S[i].ip);
		do { sc_start(1, SC_NS); in_vld = 0; } while(!out_vld);
		unsigned int h = hop.read();
		if( h != S[i].hop){
			cout << S[i].ip << " should be hop " << S[i].hop
                             << " got hop " << h << endl;
		}
	}
	cout << "program complete" << endl;
        return 0;

}

	

	




