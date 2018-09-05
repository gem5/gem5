#include "systemc.h"

inline void flags_value()
{
	ios::fmtflags flags = cout.flags();
	cout << hex << flags << dec << endl;
	if ( sc_io_show_base(cout) ) cout << "showbase" << endl;
}

#define TEST(BASE) \
{ \
	BASE x; \
	cout << endl << #BASE << endl; \
	for ( i = 0; i < 256; i++ ) \
	{ \
		x = i; \
		cout << "   "; \
		cout << std::noshowbase; \
		cout << dec << " d: " << x; \
		cout << oct << " o: " << x; \
		cout << hex << " x: " << x; \
		cout << std::showbase; \
		cout << dec << " d: " << x; \
		cout << oct << " o: " << x; \
		cout << hex << " x: " << x; \
		cout << endl; \
	} \
}
int sc_main(int argc, char* argv[])
{
	int           i;
	sc_biguint<8> x;

	TEST(sc_bigint<8>)
	TEST(sc_biguint<8>)
	TEST(sc_int<8>)
	TEST(sc_uint<8>)
	TEST(sc_lv<8>)
	TEST(sc_bv<8>)
	cout << "Program completed" << endl;

	return 0;
}
