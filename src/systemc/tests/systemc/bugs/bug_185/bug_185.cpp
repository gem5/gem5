// Bug 185 Test - sc_bv(char) constructor.
//
// sc_bv<8> a('1') was yielding an all-zero value rather than all ones.


#include "systemc.h"

int sc_main(int argc, char* argv[])
{
	sc_bv<8>  a('0');
	sc_bv<9>  b('1');
	sc_bv<11> c(false);
	sc_bv<11> d(true);
	sc_bv<11> e(0);
	sc_bv<11> f(1);

	cout << "a = " << a << endl;
	cout << "b = " << b << endl;
	cout << "c = " << c << endl;
	cout << "d = " << d << endl;
	cout << "e = " << e << endl;
	cout << "f = " << f << endl;

	cerr << "Program completed" << endl;
	return 0;
}
