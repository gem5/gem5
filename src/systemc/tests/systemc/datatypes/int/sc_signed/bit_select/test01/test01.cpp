#include "systemc.h" 
int sc_main(int argc, char** argv) 
{ 
	sc_bigint<3>  bi;

	cout << bi[3] << endl;

	cout << "Program completed" << endl;
	return 0;
} 

