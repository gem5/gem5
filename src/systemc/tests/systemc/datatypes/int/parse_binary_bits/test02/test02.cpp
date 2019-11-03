#include "systemc.h"

#define TEST_VAR(var, v) \
    var = v; \
	sprintf(buffer, "%16llx", var.to_int64()); \
	bv64 = var; \
	cout << #var << " = " << v << " : " << bv64 << " : " << buffer << " : " << var << endl;

#define TEST(v) \
	cout << endl; \
	TEST_VAR(  bi01, v ); \
	TEST_VAR(  bi02, v ); \
	TEST_VAR(  bi03, v ); \
	TEST_VAR(  bi04, v ); \
	TEST_VAR(  bi05, v ); \
	TEST_VAR(  bi06, v ); \
	TEST_VAR(  bi07, v ); \
	TEST_VAR(  bi08, v ); \
	TEST_VAR(  bi09, v ); \
	TEST_VAR(  bi10, v ); \
	TEST_VAR(  bi11, v ); \
	TEST_VAR(  bi12, v ); \
	TEST_VAR(  bi13, v ); \
	TEST_VAR(  bi14, v ); \
	TEST_VAR(  bi15, v ); \
	TEST_VAR(  bi16, v ); \
	TEST_VAR(  bi17, v ); \
	TEST_VAR(  bi18, v ); \
	TEST_VAR(  bi19, v ); \
	TEST_VAR(  bi20, v ); \
	TEST_VAR(  bi21, v ); \
	TEST_VAR(  bi22, v ); \
	TEST_VAR(  bi23, v ); \
	TEST_VAR(  bi24, v ); \
	TEST_VAR(  bi25, v ); \
	TEST_VAR(  bi26, v ); \
	TEST_VAR(  bi27, v ); \
	TEST_VAR(  bi28, v ); \
	TEST_VAR(  bi29, v ); \
	TEST_VAR(  bi30, v ); \
	TEST_VAR(  bi31, v ); \
	TEST_VAR(  bi32, v ); \
	TEST_VAR(  bi33, v ); \
	TEST_VAR(  bi34, v ); \
	TEST_VAR(  bi35, v ); \
	TEST_VAR(  bi36, v ); \
	TEST_VAR(  bi37, v ); \
	TEST_VAR(  bi38, v ); \
	TEST_VAR(  bi39, v ); \
	TEST_VAR(  bi40, v ); \
	TEST_VAR(  bi41, v ); \
	TEST_VAR(  bi42, v ); \
	TEST_VAR(  bi43, v ); \
	TEST_VAR(  bi44, v ); \
	TEST_VAR(  bi45, v ); \
	TEST_VAR(  bi46, v ); \
	TEST_VAR(  bi47, v ); \
	TEST_VAR(  bi48, v ); \
	TEST_VAR(  bi49, v ); \
	TEST_VAR(  bi50, v ); \
	TEST_VAR(  bi51, v ); \
	TEST_VAR(  bi52, v ); \
	TEST_VAR(  bi53, v ); \
	TEST_VAR(  bi54, v ); \
	TEST_VAR(  bi55, v ); \
	TEST_VAR(  bi56, v ); \
	TEST_VAR(  bi57, v ); \
	TEST_VAR(  bi58, v ); \
	TEST_VAR(  bi59, v ); \
	TEST_VAR(  bi60, v ); \
	TEST_VAR(  bi61, v ); \
	TEST_VAR(  bi62, v ); \
	TEST_VAR(  bi63, v ); \
	TEST_VAR(  bi64, v ); 

int sc_main(int argc, char* argv[])
{
	char           buffer[256];
	sc_bigint<1>  bi01;
	sc_bigint<2>  bi02;
	sc_bigint<3>  bi03;
	sc_bigint<4>  bi04;
	sc_bigint<5>  bi05;
	sc_bigint<6>  bi06;
	sc_bigint<7>  bi07;
	sc_bigint<8>  bi08;
	sc_bigint<9>  bi09;
	sc_bigint<10>  bi10;
	sc_bigint<11>  bi11;
	sc_bigint<12>  bi12;
	sc_bigint<13>  bi13;
	sc_bigint<14>  bi14;
	sc_bigint<15>  bi15;
	sc_bigint<16>  bi16;
	sc_bigint<17>  bi17;
	sc_bigint<18>  bi18;
	sc_bigint<19>  bi19;
	sc_bigint<20>  bi20;
	sc_bigint<21>  bi21;
	sc_bigint<22>  bi22;
	sc_bigint<23>  bi23;
	sc_bigint<24>  bi24;
	sc_bigint<25>  bi25;
	sc_bigint<26>  bi26;
	sc_bigint<27>  bi27;
	sc_bigint<28>  bi28;
	sc_bigint<29>  bi29;
	sc_bigint<30>  bi30;
	sc_bigint<31>  bi31;
	sc_bigint<32>  bi32;
	sc_bigint<33>  bi33;
	sc_bigint<34>  bi34;
	sc_bigint<35>  bi35;
	sc_bigint<36>  bi36;
	sc_bigint<37>  bi37;
	sc_bigint<38>  bi38;
	sc_bigint<39>  bi39;
	sc_bigint<40>  bi40;
	sc_bigint<41>  bi41;
	sc_bigint<42>  bi42;
	sc_bigint<43>  bi43;
	sc_bigint<44>  bi44;
	sc_bigint<45>  bi45;
	sc_bigint<46>  bi46;
	sc_bigint<47>  bi47;
	sc_bigint<48>  bi48;
	sc_bigint<49>  bi49;
	sc_bigint<50>  bi50;
	sc_bigint<51>  bi51;
	sc_bigint<52>  bi52;
	sc_bigint<53>  bi53;
	sc_bigint<54>  bi54;
	sc_bigint<55>  bi55;
	sc_bigint<56>  bi56;
	sc_bigint<57>  bi57;
	sc_bigint<58>  bi58;
	sc_bigint<59>  bi59;
	sc_bigint<60>  bi60;
	sc_bigint<61>  bi61;
	sc_bigint<62>  bi62;
	sc_bigint<63>  bi63;
	sc_bigint<64>  bi64;

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
		TEST_VAR( bi64, &values[i]);
	}

	return 0;
}
