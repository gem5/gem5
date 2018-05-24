#include "systemc.h"

#define TEST(EXP) cout << #EXP << " = " << EXP << endl;
int
sc_main( int argc, char* argv[] )
{
	sc_bigint<8> BI = 0xff;
	sc_int<8> I = BI;
	sc_uint<8> UI = BI;
	sc_biguint<8> UBI = BI;

TEST(BI(3,0).to_int())
TEST(BI(3,0).to_uint())
TEST(BI(3,0).to_int64())
TEST(BI(3,0).to_uint64())
TEST(BI(3,0).to_double())
TEST(I(3,0).to_int())
TEST(I(3,0).to_uint())
TEST(I(3,0).to_int64())
TEST(I(3,0).to_uint64())
TEST(I(3,0).to_double())
TEST(UI(3,0).to_int())
TEST(UI(3,0).to_uint())
TEST(UI(3,0).to_int64())
TEST(UI(3,0).to_uint64())
TEST(UI(3,0).to_double())
TEST(UBI(3,0).to_int())
TEST(UBI(3,0).to_uint())
TEST(UBI(3,0).to_int64())
TEST(UBI(3,0).to_uint64())
TEST(UBI(3,0).to_double())

	return 0;
}
