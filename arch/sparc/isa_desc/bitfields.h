////////////////////////////////////////////////////////////////////
//
// Bitfield definitions.
//

// Bitfields are shared liberally between instruction formats, so they are
// simply defined alphabetically

def bitfield A		<29>;
def bitfield CC02	<20>;
def bitfield CC03	<25>;
def bitfield CC04	<11>;
def bitfield CC12	<21>;
def bitfield CC13	<26>;
def bitfield CC14	<12>;
def bitfield CC2	<18>;
def bitfield CMASK	<6:4>;
def bitfield COND2	<28:25>;
def bitfield COND4	<17:14>;
def bitfield D16HI	<21:20>;
def bitfield D16LO	<13:0>;
def bitfield DISP19	<18:0>;
def bitfield DISP22	<21:0>;
def bitfield DISP30	<29:0>;
def bitfield FCN	<29:26>;
def bitfield I		<13>;
def bitfield IMM_ASI	<12:5>;
def bitfield IMM22	<21:0>;
def bitfield MMASK	<3:0>;
def bitfield OP		<31:30>;
def bitfield OP2	<24:22>;
def bitfield OP3	<24:19>;
def bitfield OPF	<13:5>;
def bitfield OPF_CC	<13:11>;
def bitfield OPF_LOW5	<9:5>;
def bitfield OPF_LOW6	<10:5>;
def bitfield P		<19>;
def bitfield RCOND2	<27:25>;
def bitfield RCOND3	<12:10>;
def bitfield RCOND4	<12:10>;
def bitfield RD		<29:25>;
def bitfield RS1	<18:14>;
def bitfield RS2	<4:0>;
def bitfield SHCNT32	<4:0>;
def bitfield SHCNT64	<5:0>;
def bitfield SIMM10	<9:0>;
def bitfield SIMM11	<10:0>;
def bitfield SIMM13	<12:0>;
def bitfield SW_TRAP	<6:0>;
def bitfield X		<12>;
