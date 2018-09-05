#include "systemc.h"


int main()
{
	char* argv[] = { "0", "1", "2", "3", "4" };
	return sc_elab_and_sim( sizeof(argv)/sizeof(char*), argv );
}

int sc_main(int argc, char* argv[])
{
	if ( argc != sc_argc() ) 
	{
	    cout << "sc_argc mismatch: expected " << argc << " got " << sc_argc()
			<< endl;
	}
	for ( int argi = 0; argi < argc; argi++ )
	{
		if ( strcmp( argv[argi], sc_argv()[argi] ) )
		{
			cout << "sc_argv()[" << argi << "] mismatch: expected: '" 
			     << argv[argi] << "' got: '" << sc_argv()[argi] << "'" << endl;
		}
	}

	cout << "Program completed" << endl;

    return 0;
}

