
#if !defined(SC_USE_SC_STRING_OLD) && !defined(SC_USE_STD_STRING)
#   define SC_USE_SC_STRING_OLD
//#   define SC_USE_STD_STRING
#endif //SC_USE_STRING_*

#include "systemc.h"

int sc_main(int argc, char* argv[])
{
	sc_string s5 = "abc";
	sc_string s6 = s5;

	s6[1] ='X';
	cout << "s5: " << s5 << endl << "s6: " << s6 << endl;
	return 0;
}
