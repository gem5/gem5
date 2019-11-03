#include "systemc.h"

#define TEST_VAR(var, v) \
    var = v; \
	sprintf(buffer, "%16llx", var.to_uint64()); \
	bv64 = var; \
	cout << #var << " = " << v << " : " << bv64 << " : " << buffer << endl;

#define TEST(v) \
	cout << endl; \
	TEST_VAR(  bu01, v ); \
	TEST_VAR(  bu02, v ); \
	TEST_VAR(  bu03, v ); \
	TEST_VAR(  bu04, v ); \
	TEST_VAR(  bu05, v ); \
	TEST_VAR(  bu06, v ); \
	TEST_VAR(  bu07, v ); \
	TEST_VAR(  bu08, v ); \
	TEST_VAR(  bu09, v ); \
	TEST_VAR(  bu10, v ); \
	TEST_VAR(  bu11, v ); \
	TEST_VAR(  bu12, v ); \
	TEST_VAR(  bu13, v ); \
	TEST_VAR(  bu14, v ); \
	TEST_VAR(  bu15, v ); \
	TEST_VAR(  bu16, v ); \
	TEST_VAR(  bu17, v ); \
	TEST_VAR(  bu18, v ); \
	TEST_VAR(  bu19, v ); \
	TEST_VAR(  bu20, v ); \
	TEST_VAR(  bu21, v ); \
	TEST_VAR(  bu22, v ); \
	TEST_VAR(  bu23, v ); \
	TEST_VAR(  bu24, v ); \
	TEST_VAR(  bu25, v ); \
	TEST_VAR(  bu26, v ); \
	TEST_VAR(  bu27, v ); \
	TEST_VAR(  bu28, v ); \
	TEST_VAR(  bu29, v ); \
	TEST_VAR(  bu30, v ); \
	TEST_VAR(  bu31, v ); \
	TEST_VAR(  bu32, v ); \
	TEST_VAR(  bu33, v ); \
	TEST_VAR(  bu34, v ); \
	TEST_VAR(  bu35, v ); \
	TEST_VAR(  bu36, v ); \
	TEST_VAR(  bu37, v ); \
	TEST_VAR(  bu38, v ); \
	TEST_VAR(  bu39, v ); \
	TEST_VAR(  bu40, v ); \
	TEST_VAR(  bu41, v ); \
	TEST_VAR(  bu42, v ); \
	TEST_VAR(  bu43, v ); \
	TEST_VAR(  bu44, v ); \
	TEST_VAR(  bu45, v ); \
	TEST_VAR(  bu46, v ); \
	TEST_VAR(  bu47, v ); \
	TEST_VAR(  bu48, v ); \
	TEST_VAR(  bu49, v ); \
	TEST_VAR(  bu50, v ); \
	TEST_VAR(  bu51, v ); \
	TEST_VAR(  bu52, v ); \
	TEST_VAR(  bu53, v ); \
	TEST_VAR(  bu54, v ); \
	TEST_VAR(  bu55, v ); \
	TEST_VAR(  bu56, v ); \
	TEST_VAR(  bu57, v ); \
	TEST_VAR(  bu58, v ); \
	TEST_VAR(  bu59, v ); \
	TEST_VAR(  bu60, v ); \
	TEST_VAR(  bu61, v ); \
	TEST_VAR(  bu62, v ); \
	TEST_VAR(  bu63, v ); \
	TEST_VAR(  bu64, v ); 

int sc_main(int argc, char* argv[])
{
	char           buffer[256];
	sc_biguint<1>  bu01;
	sc_biguint<2>  bu02;
	sc_biguint<3>  bu03;
	sc_biguint<4>  bu04;
	sc_biguint<5>  bu05;
	sc_biguint<6>  bu06;
	sc_biguint<7>  bu07;
	sc_biguint<8>  bu08;
	sc_biguint<9>  bu09;
	sc_biguint<10>  bu10;
	sc_biguint<11>  bu11;
	sc_biguint<12>  bu12;
	sc_biguint<13>  bu13;
	sc_biguint<14>  bu14;
	sc_biguint<15>  bu15;
	sc_biguint<16>  bu16;
	sc_biguint<17>  bu17;
	sc_biguint<18>  bu18;
	sc_biguint<19>  bu19;
	sc_biguint<20>  bu20;
	sc_biguint<21>  bu21;
	sc_biguint<22>  bu22;
	sc_biguint<23>  bu23;
	sc_biguint<24>  bu24;
	sc_biguint<25>  bu25;
	sc_biguint<26>  bu26;
	sc_biguint<27>  bu27;
	sc_biguint<28>  bu28;
	sc_biguint<29>  bu29;
	sc_biguint<30>  bu30;
	sc_biguint<31>  bu31;
	sc_biguint<32>  bu32;
	sc_biguint<33>  bu33;
	sc_biguint<34>  bu34;
	sc_biguint<35>  bu35;
	sc_biguint<36>  bu36;
	sc_biguint<37>  bu37;
	sc_biguint<38>  bu38;
	sc_biguint<39>  bu39;
	sc_biguint<40>  bu40;
	sc_biguint<41>  bu41;
	sc_biguint<42>  bu42;
	sc_biguint<43>  bu43;
	sc_biguint<44>  bu44;
	sc_biguint<45>  bu45;
	sc_biguint<46>  bu46;
	sc_biguint<47>  bu47;
	sc_biguint<48>  bu48;
	sc_biguint<49>  bu49;
	sc_biguint<50>  bu50;
	sc_biguint<51>  bu51;
	sc_biguint<52>  bu52;
	sc_biguint<53>  bu53;
	sc_biguint<54>  bu54;
	sc_biguint<55>  bu55;
	sc_biguint<56>  bu56;
	sc_biguint<57>  bu57;
	sc_biguint<58>  bu58;
	sc_biguint<59>  bu59;
	sc_biguint<60>  bu60;
	sc_biguint<61>  bu61;
	sc_biguint<62>  bu62;
	sc_biguint<63>  bu63;
	sc_biguint<64>  bu64;

	sc_bv<64>       bv64;

	int             i;

	char values[]  = "1111"
					 "1110"
					 "1101"
					 "1100"
					 "1011"
					 "1010"
					 "1001"
					 "1000"
					 "0111"
					 "0110"
					 "0101"
					 "0100"
					 "0011"
					 "0010"
					 "0001"
					 "0000";
	TEST("101");

	TEST( "11111110110111001011101010011000" );
	TEST( "1111111111111111111111111111111111111111111111111111111111111111" );

	TEST( values );

	for ( i = 0; i < (int)strlen(values); i++ )
	{
		TEST_VAR( bu64, &values[i]);
	}

	return 0;
}
