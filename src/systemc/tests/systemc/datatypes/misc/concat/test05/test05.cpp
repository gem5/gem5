// test05.cpp - Concatenation test

#include "systemc.h"
#include "specialized_signals/scx_signal_int.h"
#include "specialized_signals/scx_signal_signed.h"
#include "specialized_signals/scx_signal_uint.h"
#include "specialized_signals/scx_signal_unsigned.h"

#define COMPARE( a, b ) \
{ \
	if ( a != b ) \
	{ \
		cout << __FILE__ << "(" << __LINE__ << "): " << #a << " = " << \
			a << " " << #b << " = " << b << endl; \
	} \
}

int sc_main( int argc, char* argv[] )
{
	sc_clock                  clock;
	sc_bigint<8>              sc_bigint_a;
	sc_bigint<8>              sc_bigint_b;
	sc_biguint<8>             sc_biguint_a;
	sc_biguint<8>             sc_biguint_b;
	sc_int<8>                 sc_int_a;
	sc_int<8>                 sc_int_b;
	sc_uint<8>                sc_uint_a;
	sc_uint<8>                sc_uint_b;
	sc_signal<sc_bigint<8> >  sig_sc_bigint_a;
	sc_signal<sc_bigint<8> >  sig_sc_bigint_b;
	sc_signal<sc_biguint<8> > sig_sc_biguint_a;
	sc_signal<sc_biguint<8> > sig_sc_biguint_b;
	sc_signal<sc_int<8> >     sig_sc_int_a;
	sc_signal<sc_int<8> >     sig_sc_int_b;
	sc_signal<sc_uint<8> >    sig_sc_uint_a;
	sc_signal<sc_uint<8> >    sig_sc_uint_b;

	sc_bigint_a = "10101111";
	sc_biguint_a = "10101111";
	sc_int_a = "10101111";
	sc_uint_a = "10101111";

	( sc_bigint_b, sc_biguint_b, sc_int_b, sc_uint_b ) =
	    ( sc_bigint_a, sc_biguint_a, sc_int_a, sc_uint_a );

	COMPARE( sc_bigint_a, sc_bigint_b )
	COMPARE( sc_biguint_a, sc_biguint_b )
	COMPARE( sc_int_a, sc_int_b )
	COMPARE( sc_uint_a, sc_uint_b )

	( sig_sc_bigint_b, sig_sc_biguint_b, sig_sc_int_b, sig_sc_uint_b ) =
	    ( sc_bigint_a, sc_biguint_a, sc_int_a, sc_uint_a );

	sc_start(1, SC_NS);
	COMPARE( sc_bigint_a, sig_sc_bigint_b )
	COMPARE( sc_biguint_a, sig_sc_biguint_b )
	COMPARE( sc_int_a, sig_sc_int_b )
	COMPARE( sc_uint_a, sig_sc_uint_b )

	sc_bigint_a = 128;
	sc_biguint_a = 64;
	sc_int_a = 128;
	sc_uint_a = 64;

	( sc_bigint_b, sc_biguint_b, sc_int_b, sc_uint_b ) =
	    ( sc_int_a, sc_uint_a, sc_bigint_a, sc_biguint_a );

	COMPARE( sc_bigint_a, sc_bigint_b )
	COMPARE( sc_biguint_a, sc_biguint_b )
	COMPARE( sc_int_a, sc_int_b )
	COMPARE( sc_uint_a, sc_uint_b )

	sig_sc_bigint_a = 87;
	sig_sc_biguint_a = 124;
	sig_sc_int_a = 97;
	sig_sc_uint_a = 72;
	sc_start(1, SC_NS);

	( sig_sc_bigint_b, sig_sc_biguint_b, sig_sc_int_b, sig_sc_uint_b ) =
	    ( sig_sc_bigint_a, sig_sc_biguint_a, sig_sc_int_a, sig_sc_uint_a );
	sc_start(1, SC_NS);
	COMPARE( sig_sc_bigint_a, sig_sc_bigint_b )
	COMPARE( sig_sc_biguint_a, sig_sc_biguint_b )
	COMPARE( sig_sc_int_a, sig_sc_int_b )
	COMPARE( sig_sc_uint_a, sig_sc_uint_b )

	COMPARE( 87, sig_sc_bigint_b )
	COMPARE( 124, sig_sc_biguint_b )
	COMPARE( 97, sig_sc_int_b )
	COMPARE( 72, sig_sc_uint_b )

	cerr << "Program completed" << endl;
	return 0;
}
