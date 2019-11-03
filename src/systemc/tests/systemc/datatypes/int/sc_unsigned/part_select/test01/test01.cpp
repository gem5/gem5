#define SC_INCLUDE_FX

#include "systemc.h" 
int sc_main(int argc, char** argv) 
{ 
	sc_biguint<3>  big;

	cout << big(3,1) << endl;

	cout << "Program completed" << endl;
	return 0;
} 

