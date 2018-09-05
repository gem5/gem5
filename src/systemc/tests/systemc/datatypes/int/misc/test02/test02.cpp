#include "systemc.h"


inline void dump(sc_signed& value)
{
	sc_digit* ptr = value.get_raw();
	printf("    %08x_%08x_%08x_%08x\n", 
		(int)ptr[3], (int)ptr[2], (int)ptr[1], (int)ptr[0]);
	cout << "    " << value << endl;
}

inline void dump(sc_unsigned& value)
{
	sc_digit* ptr = value.get_raw();
	printf("    %08x_%08x_%08x_%08x\n", 
		(int)ptr[3], (int)ptr[2], (int)ptr[1], (int)ptr[0]);
	cout << "    " << value << endl;
}

#define PARSE( SUBJECT ) \
{ \
	src_p = SUBJECT; \
	cout << endl << #SUBJECT << ":" << endl; \
	svalue = src_p; \
    dump(svalue); \
	cout << endl; \
	value = src_p; \
    dump(value); \
}
#define TEST(SUBJECT,EXPECTED) \
{ \
    value = SUBJECT; \
	if ( value.to_uint64() != EXPECTED ) \
	{ \
		printf("%s(%d): %s: %llx != %llx\n", \
			__FILE__, __LINE__,  SUBJECT, value.to_uint64(), (uint64)EXPECTED);\
	} \
}

#define TESTs64(SUBJECT,EXPECTED) \
{ \
	svalue64 = SUBJECT; \
	if ( svalue64 != (long long)EXPECTED )  \
	{ \
		printf("%s(%d): %s: %llx != %llx\n", \
			__FILE__, __LINE__,  SUBJECT, value.to_uint64(), (uint64)EXPECTED);\
	}\
}
	
#define TESTu64(SUBJECT,EXPECTED) \
{ \
	uvalue64 = SUBJECT; \
	if ( uvalue64 != EXPECTED )  \
	{ \
		printf("%s(%d): %s: %llx != %llx\n", \
			__FILE__, __LINE__,  SUBJECT, value.to_uint64(), (uint64)EXPECTED);\
	}\
}
	

int sc_main(int, char**)
{
	char               buffer[128];
	unsigned long long expected;
	sc_bigint<120>     svalue;
	sc_int<64>         svalue64;
	sc_uint<64>        uvalue64;
	sc_biguint<120>    value;


	TESTs64("0xusffffffff", 0xffffffff);
	TESTs64("0xusffffffff", 0xffffffff);
	TESTs64("0xusfffffff", 0xfffffff);
	TESTs64("0XUSfedcab876543210", 0xfedcab876543210ull);
	TESTs64("0ous77777777", 077777777);
	TESTs64("0ous77777777", 077777777);
	TESTs64("0Ous7654321076543", 07654321076543ull);
	TESTs64("55555555555555", 55555555555555ull);
	TESTs64("0bus1100110011001100", 0xcccc);
	TESTs64("0bus1111111011011100101110101001100001110110010101000011001000010000",
		  0xfedcba9876543210ll);

	TESTu64("0xusffffffff", 0xffffffff);
	TESTu64("0xusffffffff", 0xffffffff);
	TESTu64("0xusfffffff", 0xfffffff);
	TESTu64("0XUSfedcab876543210", 0xfedcab876543210ull);
	TESTu64("0ous77777777", 077777777);
	TESTu64("0ous77777777", 077777777);
	TESTu64("0Ous7654321076543", 07654321076543ull);
	TESTu64("55555555555555", 55555555555555ull);
	TESTu64("0bus1100110011001100", 0xcccc);
	TESTu64("0bus1111111011011100101110101001100001110110010101000011001000010000",
		  0xfedcba9876543210ll);
	for ( int i = 0; i < 60; i++ )
	{
		for ( int j = 0; j < 16; j++ )
		{
			expected = j;
			expected = expected << i;
			sprintf(buffer, "0Xus%llx", expected);
			TEST(buffer, expected);
			TESTs64(buffer, expected);
			TESTu64(buffer, expected);
			sprintf(buffer, "0ous%llo", expected);
			TEST(buffer, expected);
			TESTs64(buffer, expected);
			TESTu64(buffer, expected);
			sprintf(buffer, "%lld", expected);
			TEST(buffer, expected);
			TESTs64(buffer, expected);
			TESTu64(buffer, expected);
		}
	}
	TEST("0xusffffffff", 0xffffffff);
	TEST("0xusffffffff", 0xffffffff);
	TEST("0xusfffffff", 0xfffffff);
	TEST("0XUSfedcab876543210", 0xfedcab876543210ull);
	TEST("0ous77777777", 077777777);
	TEST("0ous77777777", 077777777);
	TEST("0Ous7654321076543", 07654321076543ull);
	TEST("55555555555555", 55555555555555ull);
	TEST("0bus1100110011001100", 0xcccc);
	TEST("0bus1111111011011100101110101001100001110110010101000011001000010000",
		  0xfedcba9876543210ll);

	cout << "Program completed" << endl;
	return 0;
}
