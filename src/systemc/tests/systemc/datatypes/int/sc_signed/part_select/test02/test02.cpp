#define SC_INCLUDE_FX

#include "systemc.h" 
int sc_main(int argc, char** argv) 
{ 
	sc_bigint<3>  big;

	cout << big(1,2) << endl;

	cout << "Program completed" << endl;
	return 0;
} 

