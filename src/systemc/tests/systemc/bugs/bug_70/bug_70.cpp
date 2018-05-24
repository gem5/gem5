// Bug 70 - Problems with part selections on sc_biguint.

#include "systemc.h"
//#include "iomanip.h"

int sc_main(int argc, char* argv[])
{
  sc_biguint< 16 > a, b, c;
  //sc_uint< 16 > a, b ;

  a = 0x5A6C ;
  b = 0 ;
  c = 0 ;

  cout << "a: " << a.to_string(SC_HEX) << endl ;
  cout << "b: " << b.to_string(SC_HEX)  << " - So far so good" << endl ;
  cout << "c: " << c.to_string(SC_HEX)  << " - So far so good" << endl ;

  b(7,0) = a(15,8) ; // Now b should be "0x005A" or ???
  c = a(15,8) ; // Now c should be "0x005A" or ???

  cout << "a: " << a.to_string(SC_HEX) << endl ;
  cout << "b: " << b.to_string(SC_HEX) << endl ;
  cout << "c: " << c.to_string(SC_HEX) << endl ;

  sc_stop() ;
  return 0;
}

