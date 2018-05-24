#include <systemc.h>

#define USE_TABLE 1

#if DEBUG
void
Dump_R(char *s, sc_uint<8> X[4][4])
{
	printf("%s R\n", s);
	for(int i = 0; i < 4; i++){
		printf("|");
		for(int j = 0; j < 4; j++){
			printf("%02x|", (int)X[i][j]);
		}
		printf("\n");
	}
}

void
Dump_rk(char *s, sc_uint<8> X[11][4][4])
{
	printf("%s rk\n", s);
	for(int i = 0; i < 11; i++){
		printf("%2d: ", (int)i);
		for(int j = 0; j < 4; j++){
			printf("|");
			for(int k = 0; k < 4; k++){
				printf("%02x|", (int)X[i][j][k]);
			}
			printf("   ");
		}
		printf("\n");
	}
}
#endif

SC_MODULE(AES_Base)
{
	SC_CTOR(AES_Base) {}

	void AddRoundKey(sc_uint<8> a[4][4], const sc_uint<8> rk[4][4]);
	sc_uint<8> S_Table(sc_uint<8> a);
	void SchedKey(sc_uint<8> k[4][4], sc_uint<8> W[11][4][4]);
};

sc_uint<8>
AES_Base::S_Table(sc_uint<8> a)
{
	sc_uint<8> t;
#if USE_TABLE
	{
	static sc_uint<8> S[256] = {
 		99, 124, 119, 123, 242, 107, 111, 197,  
		48,   1, 103,  43, 254, 215, 171, 118,
		202, 130, 201, 125, 250,  89,  71, 240, 
		173, 212, 162, 175, 156, 164, 114, 192,
		183, 253, 147,  38,  54,  63, 247, 204,  
		52, 165, 229, 241, 113, 216,  49,  21,
  		4, 199,  35, 195,  24, 150,   5, 154,   
		7,  18, 128, 226, 235,  39, 178, 117,
  		9, 131,  44,  26,  27, 110,  90, 160,  
		82,  59, 214, 179,  41, 227,  47, 132,
 		83, 209,   0, 237,  32, 252, 177,  91, 
		106, 203, 190,  57,  74,  76,  88, 207,
		208, 239, 170, 251,  67,  77,  51, 133,  
		69, 249,   2, 127,  80,  60, 159, 168,
 		81, 163,  64, 143, 146, 157,  56, 245, 
		188, 182, 218,  33,  16, 255, 243, 210,
		205,  12,  19, 236,  95, 151,  68,  23, 
		196, 167, 126,  61, 100,  93,  25, 115,
 		96, 129,  79, 220,  34,  42, 144, 136,  
		70, 238, 184,  20, 222,  94,  11, 219,
		224,  50,  58,  10,  73,   6,  36,  92, 
		194, 211, 172,  98, 145, 149, 228, 121,
		231, 200,  55, 109, 141, 213,  78, 169, 
		108,  86, 244, 234, 101, 122, 174,   8,
		186, 120,  37,  46,  28, 166, 180, 198, 
		232, 221, 116,  31,  75, 189, 139, 138,
		112,  62, 181, 102,  72,   3, 246,  14,  
		97,  53,  87, 185, 134, 193,  29, 158,
		225, 248, 152,  17, 105, 217, 142, 148, 
		155,  30, 135, 233, 206,  85,  40, 223,
		140, 161, 137,  13, 191, 230,  66, 104,  
		65, 153,  45,  15, 176,  84, 187,  22,
	};
	t = S[a];
	}
#else
#define E(A,B) case A: t = B; break
	switch(a){
	E(  0, 99); E(  1,124); E(  2,119); E(  3,123); E(  4,242); E(  5,107); 
	E(  6,111); E(  7,197); E(  8, 48); E(  9,  1); E( 10,103); E( 11, 43); 
	E( 12,254); E( 13,215); E( 14,171); E( 15,118); E( 16,202); E( 17,130); 
	E( 18,201); E( 19,125); E( 20,250); E( 21, 89); E( 22, 71); E( 23,240); 
	E( 24,173); E( 25,212); E( 26,162); E( 27,175); E( 28,156); E( 29,164); 
	E( 30,114); E( 31,192); E( 32,183); E( 33,253); E( 34,147); E( 35, 38); 
	E( 36, 54); E( 37, 63); E( 38,247); E( 39,204); E( 40, 52); E( 41,165); 
	E( 42,229); E( 43,241); E( 44,113); E( 45,216); E( 46, 49); E( 47, 21); 
	E( 48,  4); E( 49,199); E( 50, 35); E( 51,195); E( 52, 24); E( 53,150); 
	E( 54,  5); E( 55,154); E( 56,  7); E( 57, 18); E( 58,128); E( 59,226); 
	E( 60,235); E( 61, 39); E( 62,178); E( 63,117); E( 64,  9); E( 65,131); 
	E( 66, 44); E( 67, 26); E( 68, 27); E( 69,110); E( 70, 90); E( 71,160); 
	E( 72, 82); E( 73, 59); E( 74,214); E( 75,179); E( 76, 41); E( 77,227); 
	E( 78, 47); E( 79,132); E( 80, 83); E( 81,209); E( 82,  0); E( 83,237); 
	E( 84, 32); E( 85,252); E( 86,177); E( 87, 91); E( 88,106); E( 89,203); 
	E( 90,190); E( 91, 57); E( 92, 74); E( 93, 76); E( 94, 88); E( 95,207); 
	E( 96,208); E( 97,239); E( 98,170); E( 99,251); E(100, 67); E(101, 77); 
	E(102, 51); E(103,133); E(104, 69); E(105,249); E(106,  2); E(107,127); 
	E(108, 80); E(109, 60); E(110,159); E(111,168); E(112, 81); E(113,163); 
	E(114, 64); E(115,143); E(116,146); E(117,157); E(118, 56); E(119,245); 
	E(120,188); E(121,182); E(122,218); E(123, 33); E(124, 16); E(125,255); 
	E(126,243); E(127,210); E(128,205); E(129, 12); E(130, 19); E(131,236); 
	E(132, 95); E(133,151); E(134, 68); E(135, 23); E(136,196); E(137,167); 
	E(138,126); E(139, 61); E(140,100); E(141, 93); E(142, 25); E(143,115); 
	E(144, 96); E(145,129); E(146, 79); E(147,220); E(148, 34); E(149, 42); 
	E(150,144); E(151,136); E(152, 70); E(153,238); E(154,184); E(155, 20); 
	E(156,222); E(157, 94); E(158, 11); E(159,219); E(160,224); E(161, 50); 
	E(162, 58); E(163, 10); E(164, 73); E(165,  6); E(166, 36); E(167, 92); 
	E(168,194); E(169,211); E(170,172); E(171, 98); E(172,145); E(173,149); 
	E(174,228); E(175,121); E(176,231); E(177,200); E(178, 55); E(179,109); 
	E(180,141); E(181,213); E(182, 78); E(183,169); E(184,108); E(185, 86); 
	E(186,244); E(187,234); E(188,101); E(189,122); E(190,174); E(191,  8); 
	E(192,186); E(193,120); E(194, 37); E(195, 46); E(196, 28); E(197,166); 
	E(198,180); E(199,198); E(200,232); E(201,221); E(202,116); E(203, 31); 
	E(204, 75); E(205,189); E(206,139); E(207,138); E(208,112); E(209, 62); 
	E(210,181); E(211,102); E(212, 72); E(213,  3); E(214,246); E(215, 14); 
	E(216, 97); E(217, 53); E(218, 87); E(219,185); E(220,134); E(221,193); 
	E(222, 29); E(223,158); E(224,225); E(225,248); E(226,152); E(227, 17); 
	E(228,105); E(229,217); E(230,142); E(231,148); E(232,155); E(233, 30); 
	E(234,135); E(235,233); E(236,206); E(237, 85); E(238, 40); E(239,223); 
	E(240,140); E(241,161); E(242,137); E(243, 13); E(244,191); E(245,230); 
	E(246, 66); E(247,104); E(248, 65); E(249,153); E(250, 45); E(251, 15); 
	E(252,176); E(253, 84); E(254,187); E(255, 22); 
	}
#undef E
#endif
	return(t);
}


void
AES_Base::AddRoundKey(sc_uint<8> a[4][4], const sc_uint<8> rk[4][4])
{
	for( sc_uint<3> i = 0; i < 4; i++){
		for( sc_uint<3> j = 0; j < 4; j++){
			a[i][j] = a[i][j] ^ rk[i][j];
		}
	}
}

void
AES_Base::SchedKey(sc_uint<8> k[4][4], sc_uint<8> W[11][4][4])
{
	sc_uint<3> i, j;
	sc_uint<8> tk[4][4];   
	sc_uint<8> tt;
	sc_uint<4> t;

	for(j = 0; j < 4; j++){
		for(i = 0; i < 4; i++){
			tk[i][j] = k[i][j];
		}
	}

	for(j = 0; j < 4; j++){
		for(i = 0; i < 4; i++){
			W[0][i][j] = tk[i][j];
		}
	}

	for( t = 1; t < 11; t++){
		for(i = 0; i < 4; i++){
			tk[i][0] ^= S_Table(tk[(i+1)&3][3]);
		}

#if USE_TABLE
		{ static sc_uint<8> rcon[11] = { 
			0x0, /* dummy entry to lineup with t's value */
  			0x01, 
			0x02, 
			0x04, 
			0x08, 
  			0x10, 
			0x20, 
			0x40, 
			0x80, 
  			0x1b, 
			0x36 
		};
		tt = rcon[t];
		}
#else 
		switch(t){
  		case  1: tt = 0x01; break; /* 0000 0001 */
		case  2: tt = 0x02; break; /* 0000 0010 */
		case  3: tt = 0x04; break; /* 0000 0100 */
		case  4: tt = 0x08; break; /* 0000 1000 */
  		case  5: tt = 0x10; break; /* 0001 0000 */
		case  6: tt = 0x20; break; /* 0010 0000 */
		case  7: tt = 0x40; break; /* 0100 0000 */
		case  8: tt = 0x80; break; /* 1000 0000 */
  		case  9: tt = 0x1b; break; /* 0001 1011 */
		case 10: tt = 0x36; break; /* 0011 0110 */
		}
#endif

		tk[0][0] ^= tt;

		for(j = 1; j < 4; j++){
			for(i = 0; i < 4; i++){
				tk[i][j] ^= tk[i][j-1];
			}	
		}

		for(j = 0; j < 4; j++){
			for(i = 0; i < 4; i++){
				W[t][i][j] = tk[i][j];
			}
		}
	}		
}

class AES_Decrypt : public AES_Base {
public:
	SC_HAS_PROCESS(AES_Decrypt);
	AES_Decrypt(sc_module_name name,
		sc_clock& pCLK, 
		sc_signal<bool>& pRST_X,
		sc_signal<bool>& pIn_req,
		sc_signal<bool>& pIn_ack,
		sc_signal<bool>& pIn_cmd,
		sc_signal<sc_biguint<128> >& pIn_wire,
		sc_signal<bool>& pOut_req,
		sc_signal<bool>& pOut_ack,
		sc_signal<sc_biguint<128> >& pOut_wire
	) : AES_Base(name)
	{
	    CLK(pCLK); RST_X(pRST_X);
	    In_req(pIn_req); In_ack(pIn_ack); In_cmd(pIn_cmd); In_wire(pIn_wire);
	    Out_req(pOut_req); Out_ack(pOut_ack); Out_wire(pOut_wire);
		SC_CTHREAD(MainThread, this->CLK.pos());
		reset_signal_is(RST_X,false);
	} 

	sc_in_clk CLK;
	sc_in<bool> RST_X;

	sc_in<sc_biguint<128> >  In_wire;
	sc_out<bool>	      In_req;
	sc_in<bool>	      In_ack;
	sc_in<bool>	      In_cmd;

	sc_out<sc_biguint<128> > Out_wire;
	sc_in<bool>	      Out_req;
	sc_out<bool>	      Out_ack;

#define LOAD_KEY 0
#define DECRYPT 1
	sc_uint<1> cmd;		

	sc_uint<8> rk[11][4][4];	
	sc_uint<8> R[4][4];

	void MainThread(void);
	void Reset(void);

	void Input(void);
	void Output(void);

	void Decrypt(void);

	void InvMixColumns(sc_uint<8> a[4][4]);
	void ShiftRows_Right_Rotate(sc_uint<8> a[4][4]);
	void Substitution_Si(sc_uint<8> a[4][4]);
	sc_uint<8> Si_Table(sc_uint<8> a);

	sc_uint<8> dmul9(const sc_uint<8> B);
	sc_uint<8> dmulb(const sc_uint<8> B);
	sc_uint<8> dmuld(const sc_uint<8> B);
	sc_uint<8> dmule(const sc_uint<8> B);
};

void 
AES_Decrypt::Reset(void)
{
	In_req = false;
	Out_ack = false;
}

void 
AES_Decrypt::Input(void)
{
	sc_uint<8> t;
	sc_biguint<128> t_In_wire;

	(void)wait();
	In_req = true;

	do { wait(); } while(!In_ack);

	cmd = In_cmd;
	t_In_wire = In_wire.read();

	/*
	 * no matter whether it's a decrypt
	 * or a key_schedule, get the input pins
	 * into R[][].
	 */
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			for( int k = 0; k < 8; k++){
				t[k] = t_In_wire[(i*4+j)*8+k];
			}
			R[i][j] = t;
		}
	}

	In_req = false;

	//Dump_R("AES_Descrypt::Input R", R);
}

void 
AES_Decrypt::Output(void)
{
	sc_uint<8> t;
	sc_biguint<128> t_Out_wire;

	do { (void)wait(); } while(!Out_req);

	/*
	 * if it's a decrypt, drive R[][] onto output pins
	 */
	if( cmd == DECRYPT){
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				t = R[i][j];
				for( int k = 0; k < 8; k++){
					t_Out_wire[(i*4+j)*8+k] = t[k];
				}
			}
		}
		Out_wire.write(t_Out_wire);

		//Dump_R("AES_Descrypt::Output R", R);
	}

	Out_ack = true;
	(void)wait();
	Out_ack = false;
	
}

void 
AES_Decrypt::MainThread(void)
{
	(void)Reset();

	{
	while(!RST_X)
		(void)wait();
	}

	for(;;){
		(void)Input();

		if( cmd == LOAD_KEY ){
			/* 
			 * take R[][] and expand
			 * it into rk[][][]
			 */
			(void)SchedKey(R, rk);
			(void)wait(3);

			//Dump_rk("AES_Descrypt::MainThread rk", rk);
		} else {
			/*
			 * take R[][] and rk[][][]
			 * and decrypt the data inplace
			 * into R[][]
			 */
			(void)Decrypt();
			(void)wait(12);
			(void)Output();
		}

	}
}

void
AES_Decrypt::Decrypt(void)
{
	(void)AddRoundKey(R,rk[10]);
	//Dump_R("10: AES_Decrypt::AddRoundKey", R);

	(void)Substitution_Si(R);
	//printf("%d: ", 10); Dump_R("AES_Decrypt::Substitution_Si", R);

	(void)ShiftRows_Right_Rotate(R);
	//printf("%d: ", 10); Dump_R("AES_Decrypt::ShiftRows_Right_Rotate", R);

	for(sc_uint<4> i = 9; i > 0; i--){
		(void)AddRoundKey(R, rk[i]);
		//printf("%d: ", (int)i); Dump_R("AES_Decrypt::AddRoundKey", R);

		(void)InvMixColumns(R);
		//printf("%d: ", (int)i); Dump_R("AES_Decrypt::InvMixColums", R);

		(void)Substitution_Si(R);
		//printf("%d: ", (int)i); Dump_R("AES_Decrypt::Substitution_Si", R);

		(void)ShiftRows_Right_Rotate(R);
		//printf("%d: ", (int)i); Dump_R("AES_Decrypt::ShiftRows_Right_Rotate", R);
	}

	(void)AddRoundKey(R, rk[0]);
	//Dump_R("0: AES_Decrypt::AddRoundKey", R);
}

sc_uint<8>
AES_Decrypt::Si_Table(sc_uint<8> a)
{
	sc_uint<8> t;
#if USE_TABLE
	{
	static sc_uint<8> Si[256] = {
 		82,   9, 106, 213,  48,  54, 165,  56, 
		191,  64, 163, 158, 129, 243, 215, 251,
		124, 227,  57, 130, 155,  47, 255, 135,	 
		52, 142,  67,	68, 196, 222, 233, 203,
 		84, 123, 148,	50, 166, 194,  35,  61, 
		238,  76, 149,	11,  66, 250, 195,  78,
  		8,  46, 161, 102,  40, 217,  36, 178, 
		118,  91, 162,	73, 109, 139, 209,  37,
		114, 248, 246, 100, 134, 104, 152,  22, 
		212, 164,  92, 204,  93, 101, 182, 146,
		108, 112,  72,	80, 253, 237, 185, 218,	 
		94,  21,  70,	87, 167, 141, 157, 132,
		144, 216, 171,	 0, 140, 188, 211,  10, 
		247, 228,  88,	 5, 184, 179,  69,   6,
		208,  44,  30, 143, 202,  63,  15,   2, 
		193, 175, 189,	 3,   1,  19, 138, 107,
 		58, 145,  17,	65,  79, 103, 220, 234, 
		151, 242, 207, 206, 240, 180, 230, 115,
		150, 172, 116,	34, 231, 173,  53, 133, 
		226, 249,  55, 232,  28, 117, 223, 110,
 		71, 241,  26, 113,  29,  41, 197, 137, 
		111, 183,  98,	14, 170,  24, 190,  27,
		252,  86,  62,	75, 198, 210, 121,  32, 
		154, 219, 192, 254, 120, 205,  90, 244,
 		31, 221, 168,	51, 136,   7, 199,  49, 
		177,  18,  16,	89,  39, 128, 236,  95,
 		96,  81, 127, 169,  25, 181,  74,  13,	 
		45, 229, 122, 159, 147, 201, 156, 239,
		160, 224,  59,	77, 174,  42, 245, 176, 
		200, 235, 187,	60, 131,  83, 153,  97,
 		23,  43,   4, 126, 186, 119, 214,  38, 
		225, 105,  20,	99,  85,  33,  12, 125,
	};
	t = Si[a];
	}
#else
#define E(A,B) case A: t = B; break
	switch(a){
	E(  0, 82); E(  1,  9); E(  2,106); E(  3,213); E(  4, 48); E(  5, 54); 
	E(  6,165); E(  7, 56); E(  8,191); E(  9, 64); E( 10,163); E( 11,158); 
	E( 12,129); E( 13,243); E( 14,215); E( 15,251); E( 16,124); E( 17,227); 
	E( 18, 57); E( 19,130); E( 20,155); E( 21, 47); E( 22,255); E( 23,135); 
	E( 24, 52); E( 25,142); E( 26, 67); E( 27, 68); E( 28,196); E( 29,222); 
	E( 30,233); E( 31,203); E( 32, 84); E( 33,123); E( 34,148); E( 35, 50); 
	E( 36,166); E( 37,194); E( 38, 35); E( 39, 61); E( 40,238); E( 41, 76); 
	E( 42,149); E( 43, 11); E( 44, 66); E( 45,250); E( 46,195); E( 47, 78); 
	E( 48,  8); E( 49, 46); E( 50,161); E( 51,102); E( 52, 40); E( 53,217); 
	E( 54, 36); E( 55,178); E( 56,118); E( 57, 91); E( 58,162); E( 59, 73); 
	E( 60,109); E( 61,139); E( 62,209); E( 63, 37); E( 64,114); E( 65,248); 
	E( 66,246); E( 67,100); E( 68,134); E( 69,104); E( 70,152); E( 71, 22); 
	E( 72,212); E( 73,164); E( 74, 92); E( 75,204); E( 76, 93); E( 77,101); 
	E( 78,182); E( 79,146); E( 80,108); E( 81,112); E( 82, 72); E( 83, 80); 
	E( 84,253); E( 85,237); E( 86,185); E( 87,218); E( 88, 94); E( 89, 21); 
	E( 90, 70); E( 91, 87); E( 92,167); E( 93,141); E( 94,157); E( 95,132); 
	E( 96,144); E( 97,216); E( 98,171); E( 99,  0); E(100,140); E(101,188); 
	E(102,211); E(103, 10); E(104,247); E(105,228); E(106, 88); E(107,  5); 
	E(108,184); E(109,179); E(110, 69); E(111,  6); E(112,208); E(113, 44); 
	E(114, 30); E(115,143); E(116,202); E(117, 63); E(118, 15); E(119,  2); 
	E(120,193); E(121,175); E(122,189); E(123,  3); E(124,  1); E(125, 19); 
	E(126,138); E(127,107); E(128, 58); E(129,145); E(130, 17); E(131, 65); 
	E(132, 79); E(133,103); E(134,220); E(135,234); E(136,151); E(137,242); 
	E(138,207); E(139,206); E(140,240); E(141,180); E(142,230); E(143,115); 
	E(144,150); E(145,172); E(146,116); E(147, 34); E(148,231); E(149,173); 
	E(150, 53); E(151,133); E(152,226); E(153,249); E(154, 55); E(155,232); 
	E(156, 28); E(157,117); E(158,223); E(159,110); E(160, 71); E(161,241); 
	E(162, 26); E(163,113); E(164, 29); E(165, 41); E(166,197); E(167,137); 
	E(168,111); E(169,183); E(170, 98); E(171, 14); E(172,170); E(173, 24); 
	E(174,190); E(175, 27); E(176,252); E(177, 86); E(178, 62); E(179, 75); 
	E(180,198); E(181,210); E(182,121); E(183, 32); E(184,154); E(185,219); 
	E(186,192); E(187,254); E(188,120); E(189,205); E(190, 90); E(191,244); 
	E(192, 31); E(193,221); E(194,168); E(195, 51); E(196,136); E(197,  7); 
	E(198,199); E(199, 49); E(200,177); E(201, 18); E(202, 16); E(203, 89); 
	E(204, 39); E(205,128); E(206,236); E(207, 95); E(208, 96); E(209, 81); 
	E(210,127); E(211,169); E(212, 25); E(213,181); E(214, 74); E(215, 13); 
	E(216, 45); E(217,229); E(218,122); E(219,159); E(220,147); E(221,201); 
	E(222,156); E(223,239); E(224,160); E(225,224); E(226, 59); E(227, 77); 
	E(228,174); E(229, 42); E(230,245); E(231,176); E(232,200); E(233,235); 
	E(234,187); E(235, 60); E(236,131); E(237, 83); E(238,153); E(239, 97); 
	E(240, 23); E(241, 43); E(242,  4); E(243,126); E(244,186); E(245,119); 
	E(246,214); E(247, 38); E(248,225); E(249,105); E(250, 20); E(251, 99); 
	E(252, 85); E(253, 33); E(254, 12); E(255,125); 
	}
#undef E
#endif
	return(t);
}
void 
AES_Decrypt::Substitution_Si(sc_uint<8> a[4][4])
{
	for(sc_uint<3> j = 0; j < 4; j++){
		for( sc_uint<3> i = 0; i < 4; i++){
			a[i][j] = Si_Table(a[i][j]);
		}
	}
}

/*
 * this routine does a rotate right
 * on each row.  the first is rotated 0,
 * ie, nothing is done.   the second row
 * is roatated by 1, the third row by 2
 * and the finally row by 3.
 * ie:
 *	
 *	row0	row1	row2	row3
 *      0123    0123    0123    0123
 *	||||    ||||    ||||    ||||
 *	0123    3012    2301    1230 
 * 
 * this is equivalent to ShiftRows(1,...)
 * code in the reference C code.   it
 * has been expanded to get rid of the
 * shifts[][][] array used at run time.
 */
void
AES_Decrypt::ShiftRows_Right_Rotate(sc_uint<8> a[4][4])
{
	sc_uint<8> t;

	t = a[1][3];
	a[1][3] = a[1][2];
	a[1][2] = a[1][1];
	a[1][1] = a[1][0];
	a[1][0] = t;

	t = a[2][0];
	a[2][0] = a[2][2];
	a[2][2] = t;
	t = a[2][1];
	a[2][1] = a[2][3];
	a[2][3] = t;

	t = a[3][0];
	a[3][0] = a[3][1];
	a[3][1] = a[3][2];
	a[3][2] = a[3][3];
	a[3][3] = t;

}

void
AES_Decrypt::InvMixColumns(sc_uint<8> a[4][4])
{
	sc_uint<8> t[4][4];

	for(sc_uint<3> j = 0; j < 4; j++){
		for( sc_uint<3> i = 0; i < 4; i++){
			t[i][j] = dmule(a[i][j])
			          ^ dmulb(a[(i + 1) & 3][j])
			          ^ dmuld(a[(i + 2) & 3][j])
			          ^ dmul9(a[(i + 3) & 3][j]);
		}
	}

	for(sc_uint<3> j = 0; j < 4; j++){
		for( sc_uint<3> i = 0; i < 4; i++){
			a[i][j] = t[i][j];
		}
	}
}

sc_uint<8>
AES_Decrypt::dmul9(const sc_uint<8> B)
{
	sc_uint<8> O;

	O[0] = B[5]          ^B[0];
	O[1] = B[5]^B[6]     ^B[1];
	O[2] = B[6]^B[7]     ^B[2];
	O[3] = B[0]^B[7]^B[5]^B[3];
	O[4] = B[1]^B[6]^B[5]^B[4];
	O[5] = B[7]^B[2]^B[6]^B[5];
	O[6] = B[7]^B[3]     ^B[6];
	O[7] = B[4]          ^B[7];

	return(O);
}

sc_uint<8>
AES_Decrypt::dmulb(const sc_uint<8> B)
{
	sc_uint<8> O;

	O[0] = B[5]           ^B[7]      ^B[0];
	O[1] = B[5]^B[6]      ^B[0]^B[7] ^B[1];
	O[2] = B[6]^B[7]      ^B[1]      ^B[2];
	O[3] = B[0]^B[5]      ^B[2]      ^B[3];
	O[4] = B[1]^B[6]^B[5] ^B[3]^B[7] ^B[4];
	O[5] = B[7]^B[2]^B[6] ^B[4]      ^B[5];
	O[6] = B[7]^B[3]      ^B[5]      ^B[6];
	O[7] = B[4]           ^B[6]      ^B[7];

	return(O);
}

sc_uint<8>
AES_Decrypt::dmuld(const sc_uint<8> B)
{
	sc_uint<8> O;

	O[0] = B[5]           ^B[6]      ^B[0];
	O[1] = B[5]           ^B[7]      ^B[1];
	O[2] = B[6]           ^B[0]      ^B[2];
	O[3] = B[0]^B[7]^B[5] ^B[1]^B[6] ^B[3];
	O[4] = B[1]^B[5]      ^B[2]^B[7] ^B[4];
	O[5] = B[2]^B[6]      ^B[3]      ^B[5];
	O[6] = B[7]^B[3]      ^B[4]      ^B[6];
	O[7] = B[4]           ^B[5]      ^B[7];

	return(O);
}

sc_uint<8>
AES_Decrypt::dmule(const sc_uint<8> B)
{
	sc_uint<8> O;

	O[0] = B[5]           ^B[6]      ^B[7];
	O[1] = B[5]           ^B[7]      ^B[0]^B[7];
	O[2] = B[6]           ^B[0]      ^B[1];
	O[3] = B[0]^B[7]^B[5] ^B[1]^B[6] ^B[2]^B[7];
	O[4] = B[1]^B[5]      ^B[2]^B[7] ^B[3]^B[7];
	O[5] = B[2]^B[6]      ^B[3]      ^B[4];
	O[6] = B[7]^B[3]      ^B[4]      ^B[5];
	O[7] = B[4]           ^B[5]      ^B[6];

	return(O);
}

class AES_Encrypt : public AES_Base {
public:
	SC_HAS_PROCESS(AES_Encrypt);
	AES_Encrypt(sc_module_name name,
		    sc_clock& pCLK, 
		    sc_signal<bool>& pRST_X,
		    sc_signal<bool>& pIn_req,
		    sc_signal<bool>& pIn_ack,
		    sc_signal<bool>& pIn_cmd,
		    sc_signal<sc_biguint<128> >& pIn_wire,
		    sc_signal<bool>& pOut_req,
		    sc_signal<bool>& pOut_ack,
		    sc_signal<sc_biguint<128> >& pOut_wire
	) : AES_Base(name)
	{
	    CLK(pCLK); RST_X(pRST_X);
	    In_req(pIn_req); In_ack(pIn_ack); In_cmd(pIn_cmd); In_wire(pIn_wire);
	    Out_req(pOut_req); Out_ack(pOut_ack); Out_wire(pOut_wire);
		SC_CTHREAD(MainThread, this->CLK.pos());
		reset_signal_is(RST_X,false);
	}

	sc_in_clk CLK;
	sc_in<bool> RST_X;

	sc_in<sc_biguint<128> >  In_wire;
	sc_out<bool>	      In_req;
	sc_in<bool>	      In_ack;
	sc_in<bool>	      In_cmd;

	sc_out<sc_biguint<128> > Out_wire;
	sc_in<bool>	      Out_req;
	sc_out<bool>	      Out_ack;

#define LOAD_KEY 0
#define ENCRYPT 1
	sc_uint<1> cmd;		
	sc_uint<8> rk[11][4][4];
	sc_uint<8> R[4][4];
	
	void MainThread(void);
	void Reset(void);

	void Input(void);
	void Output(void);

	void Encrypt(void);

	void MixColumns(sc_uint<8> a[4][4]);
	void ShiftRows_Left_Rotate(sc_uint<8> a[4][4]);
	void Substitution_S(sc_uint<8> a[4][4]);

	sc_uint<8> dmul2(const sc_uint<8> B);
	sc_uint<8> dmul3(const sc_uint<8> B);
};


void 
AES_Encrypt::Reset(void)
{
	In_req = false;
	Out_ack = false;
}

void 
AES_Encrypt::Input(void)
{
	sc_biguint<128> t_In_wire;
	sc_uint<8> t;

	(void)wait();
	In_req = true;

	do { wait(); } while(!In_ack);

	cmd = In_cmd;
	t_In_wire = In_wire.read();

	/*
	 * no matter whether it's a decrypt
	 * or a key_schedule, get the input pins
	 * into R[][].
	 */
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			for( int k = 0; k < 8; k++){
				t[k] = t_In_wire[(i*4+j)*8+k];
			}
			R[i][j] = t;
		}
	}

	In_req = false;

	//Dump_R("AES_Encrypt::Input R", R);
}

void 
AES_Encrypt::Output(void)
{
	sc_biguint<128> t_Out_wire;
	sc_uint<8> t;

	do { (void)wait(); } while(!Out_req);

	/*
	 * if it's a encrypt, drive R[][] onto output pins
	 */
	if( cmd == ENCRYPT){
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				t = R[i][j];
				for( int k = 0; k < 8; k++){
					t_Out_wire[(i*4+j)*8+k] = t[k];
				}
			}
		}

		(void)Out_wire.write(t_Out_wire);

		//Dump_R("AES_Encrypt::Output R", R);
	}

	Out_ack = true;
	wait();
	Out_ack = false;
	
}

void 
AES_Encrypt::MainThread(void)
{
	(void)Reset();

	{
	while(!RST_X)
		(void)wait();
	}

	for(;;){
		(void)Input();

		if( cmd == LOAD_KEY){
			/* 
			 * take R[][] and expand
			 * it into rk[][][]
			 */
			(void)SchedKey(R, rk);
			(void)wait(3);
			//Dump_rk("AES_Encrypt::MainThread rk", rk);
		} else {
			/*
			 * take R[][] and rk[][][]
			 * and encrypt the data inplace
			 * into R[][]
			 */
			(void)Encrypt();
			(void)wait(12);
			(void)Output();
		}
	}
}

void
AES_Encrypt::Encrypt(void)
{
	(void)AddRoundKey(R, rk[0]);
	//Dump_R("0: AES_Encrypt::AddRoundKey", R);

	for(sc_uint<4> i = 1; i < 10; i++){
		(void)Substitution_S(R);
		//printf("%d: ", (int)i); Dump_R("AES_Encrypt::Substitution_S", R);

		(void)ShiftRows_Left_Rotate(R);
		//printf("%d: ", (int)i); Dump_R("AES_Encrypt::ShiftRows_Left_Rotate", R);

		(void)MixColumns(R);
		//printf("%d: ", (int)i); Dump_R("AES_Encrypt::MixColums", R);

		(void)AddRoundKey(R, rk[i]);
		//printf("%d: ", (int)i); Dump_R("AES_Encrypt::AddRoundKey", R);
	}

	(void)Substitution_S(R);
	//printf("%d: ", 10); Dump_R("AES_Encrypt::Substitution_S", R);

	(void)ShiftRows_Left_Rotate(R);
	//printf("%d: ", 10); Dump_R("AES_Encrypt::ShiftRows_Left_Rotate", R);

	(void)AddRoundKey(R,rk[10]);
	//printf("%d: ", 10); Dump_R("AES_Encrypt::AddRoundKey", R);
}


void 
AES_Encrypt::Substitution_S(sc_uint<8> a[4][4])
{
	for(sc_uint<3> j = 0; j < 4; j++){
		for( sc_uint<3> i = 0; i < 4; i++){
			a[i][j] = S_Table(a[i][j]);
		}
	}
}

/*
 * this routine does a rotate left 
 * on each row.  the first is rotated 0,
 * ie, nothing is done.   the second row
 * is roatated by 1, the third row by 2
 * and the finally row by 3.
 * ie:
 *	
 *	row0	row1	row2	row3
 *      0123    0123    0123    0123
 *	||||    ||||    ||||    ||||
 *	0123    1230    2301    3012
 *
 * this is equivalent to ShiftRows(0,...)
 * code in the reference C code.   it
 * has been expanded to get rid of the
 * shifts[][][] array used at run time.
 */
void
AES_Encrypt::ShiftRows_Left_Rotate(sc_uint<8> a[4][4])
{
	sc_uint<8> t;

	t = a[1][0];
	a[1][0] = a[1][1];
	a[1][1] = a[1][2];
	a[1][2] = a[1][3];
	a[1][3] = t;

	t = a[2][0];
	a[2][0] = a[2][2];
	a[2][2] = t;
	t = a[2][1];
	a[2][1] = a[2][3];
	a[2][3] = t;

	t = a[3][3];
	a[3][3] = a[3][2];
	a[3][2] = a[3][1];
	a[3][1] = a[3][0];
	a[3][0] = t;
}

void
AES_Encrypt::MixColumns(sc_uint<8> a[4][4])
{
	sc_uint<8> t[4][4];

	for(sc_uint<3> j = 0; j < 4; j++){
		for( sc_uint<3> i = 0; i < 4; i++){
			t[i][j] = dmul2(a[i][j])
                                ^ dmul3(a[(i + 1) & 3][j])
                                ^ a[(i + 2) & 3][j]
                                ^ a[(i + 3) & 3][j];

		}
	}

	for(sc_uint<3> j = 0; j < 4; j++){
		for( sc_uint<3> i = 0; i < 4; i++){
			a[i][j] = t[i][j];
		}
	}
}



sc_uint<8>
AES_Encrypt::dmul2(const sc_uint<8> B)
{
	sc_uint<8> O;

	O[0] = B[7];
	O[1] = B[0]^B[7];
	O[2] = B[1];
	O[3] = B[2]^B[7];
	O[4] = B[3]^B[7];
	O[5] = B[4];
	O[6] = B[5];
	O[7] = B[6];

	return(O);
}

sc_uint<8>
AES_Encrypt::dmul3(const sc_uint<8> B)
{
	sc_uint<8> O;

	O[0] = B[7]     ^B[0];
	O[1] = B[0]^B[7]^B[1];
	O[2] = B[1]     ^B[2];
	O[3] = B[2]^B[7]^B[3];
	O[4] = B[3]^B[7]^B[4];
	O[5] = B[4]     ^B[5];
	O[6] = B[5]     ^B[6];
	O[7] = B[6]     ^B[7];

	return(O);
}

#define TESTBENCH 1
#if TESTBENCH

sc_biguint<128>
str2biguint(char *s)
{
	static sc_biguint<128> a;
	char str[16];
	int i, c;

	for(i = 0; i < 16 && *s != '\0'; i++, s++)
		str[i] = *s;

	while( i < 16)
		str[i++] = ' ';

	for(i = 0; i < 16; i++){
		c = str[i];
		a[(i*8)+0] = (c&1);
		a[(i*8)+1] = ((c>>1)&1);
		a[(i*8)+2] = ((c>>2)&1);
		a[(i*8)+3] = ((c>>3)&1);
		a[(i*8)+4] = ((c>>4)&1);
		a[(i*8)+5] = ((c>>5)&1);
		a[(i*8)+6] = ((c>>6)&1); 
		a[(i*8)+7] = ((c>>7)&1);
	}

	return(a);
}

char *
biguint2str(sc_biguint<128> a)
{
	static char str[17];
	int i;
	char c;

	str[16] = '\0';

	for( i = 0; i < 128; i += 8){
		c = 0;
		c |= a[i+0] ? 1 : 0;
		c |= a[i+1] ? 1<<1 : 0;
		c |= a[i+2] ? 1<<2 : 0;
		c |= a[i+3] ? 1<<3 : 0;
		c |= a[i+4] ? 1<<4 : 0;
		c |= a[i+5] ? 1<<5 : 0;
		c |= a[i+6] ? 1<<6 : 0;
		c |= a[i+7] ? 1<<7 : 0;
		str[i/8] = c;
	
	}

	return(&str[0]);
}

sc_biguint<128> 
makekey(char *keyMaterial)
{
	int i, j, t, k;
	sc_uint<8> R[4][4];
	sc_uint<8> tt;
	sc_biguint<128> key;

	for(i = 0; i < 128/8; i++) {
		t = keyMaterial[2*i];
		if ((t >= '0') && (t <= '9')) j = (t - '0') << 4;
		else if ((t >= 'a') && (t <= 'f')) j = (t - 'a' + 10) << 4;
		else if ((t >= 'A') && (t <= 'F')) j = (t - 'A' + 10) << 4;
		else abort();

		t = keyMaterial[2*i+1];
		if ((t >= '0') && (t <= '9')) j ^= (t - '0');
		else if ((t >= 'a') && (t <= 'f')) j ^= (t - 'a' + 10);
		else if ((t >= 'A') && (t <= 'F')) j ^= (t - 'A' + 10);
		else abort();

		R[i % 4][i / 4] = j;
	}

	for(i = 0; i < 4; i++){
		for(j = 0; j < 4; j++){
			tt = R[i][j];
			for(k = 0; k < 8; k++){
				key[(i*4+j)*8+k] = tt[k];
			}
		}
	}
	return(key);
}


int
sc_main(int argc, char *argv[])
{
	sc_clock clock;
	sc_signal<bool> reset;
	sc_signal<bool> D_In_req;
	sc_signal<bool> D_In_ack;
	sc_signal<bool> D_In_cmd;
	sc_signal<bool> D_Out_req;
	sc_signal<bool> D_Out_ack;
	sc_signal<sc_biguint<128> > D_In_wire;
	sc_signal<sc_biguint<128> > D_Out_wire;
	sc_signal<bool> E_In_req;
	sc_signal<bool> E_In_ack;
	sc_signal<bool> E_In_cmd;
	sc_signal<bool> E_Out_req;
	sc_signal<bool> E_Out_ack;
	sc_signal<sc_biguint<128> > E_In_wire;
	sc_signal<sc_biguint<128> > E_Out_wire;
	sc_biguint<128> key;
	sc_biguint<128> t;
	char *s;
	char key_string[33] = "deadbeef0123456776543210beefdead";
	char in[17] = "abcdefghijklmnop";
	char *out;
	bool err;
	int i;
	
	AES_Encrypt E("AES_Encrypt",
		clock,
		reset, 
		E_In_req,
		E_In_ack,
		E_In_cmd,
		E_In_wire,
		E_Out_req,
		E_Out_ack,
		E_Out_wire);

	AES_Decrypt D("AES_Decrypt",
		clock,
		reset, 
		D_In_req,
		D_In_ack,
		D_In_cmd,
		D_In_wire,	
		D_Out_req,
		D_Out_ack,
		D_Out_wire);


	key = makekey(key_string);

	reset = 0;
	sc_start(2, SC_NS);
	reset = 1;
	sc_start(2, SC_NS);

	while( !E_In_req) sc_start(1, SC_NS);
	E_In_cmd = LOAD_KEY;
	E_In_ack = 1;
	E_In_wire = key;
	do { sc_start(1, SC_NS); E_In_ack = 0; } while( !E_In_req);

	while( !D_In_req) sc_start(1, SC_NS);
	D_In_cmd = LOAD_KEY;
	D_In_ack = 1;
	D_In_wire = key;
	do { sc_start(1, SC_NS); D_In_ack = 0; } while( !D_In_req);
	

	while( !E_In_req) sc_start(1, SC_NS);
	E_Out_req = 1;
	E_In_cmd = ENCRYPT;
	E_In_ack = 1;
	E_In_wire = str2biguint((char*)"abcdefghijklmnop");
	do { sc_start(1, SC_NS); E_In_ack = 0; } while(!E_Out_ack);
	E_Out_req = 0;
	sc_start(1, SC_NS);

	while( !D_In_req) sc_start(1, SC_NS);
	D_Out_req = 1;
	D_In_cmd = DECRYPT;
	D_In_ack = 1;
	D_In_wire = E_Out_wire;
	do { sc_start(1, SC_NS); D_In_ack = 0; } while(!D_Out_ack);
	out = biguint2str(D_Out_wire);
	D_Out_req = 0;
	sc_start(1, SC_NS);

	err = false;
	for(i = 0; i < 16; i++){
		if( in[i] != out[i]){
			err = true;
			break;
		}
	}

	if( err){
		cout << "mismatch error at index " << i << endl;
		cout << "key:" << key_string << " in:" << in 
                     << " out:" << out << endl;
	} else {
		cout << "program complete" << endl;
	}

	return(0);
}
#endif


