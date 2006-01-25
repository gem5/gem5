////////////////////////////////////////////////////////////////////
//
// Bitfield definitions.
//

// Universal (format-independent) fields
def bitfield OPCODE_HI  <31:29>;
def bitfield OPCODE_LO  <28:26>;

def bitfield FUNCTION_HI   < 5: 3>;
def bitfield FUNCTION_LO   < 2: 0>;

def bitfield RT	      <20:16>;
def bitfield RT_HI    <20:19>;
def bitfield RT_LO    <18:16>;

def bitfield RS	      <25:21>;
def bitfield RS_HI    <25:24>;
def bitfield RS_LO    <23:21>;

def bitfield MOVCI <>;
def bitfield SRL   <>;
def bitfield SRLV  <>;
def bitfield SA    <>;

def bitfield BSHFL    <>;
def bitfield BSHFL_HI <>;
def bitfield BSHFL_LO <>;

// Integer operate format(s>;
def bitfield INTIMM	<15: 0>; // integer immediate (literal)
def bitfield IMM	<12:12>; // immediate flag
def bitfield INTFUNC	<11: 5>; // function code
def bitfield RD		<15:11>; // dest reg

// Memory format
def signed bitfield MEMDISP <15: 0>; // displacement
def        bitfield MEMFUNC <15: 0>; // function code (same field, unsigned)

// Memory-format jumps
def bitfield JMPFUNC	<15:14>; // function code (disp<15:14>)
def bitfield JMPHINT	<13: 0>; // tgt Icache idx hint (disp<13:0>)

// Branch format
def signed bitfield BRDISP <20: 0>; // displacement

// Floating-point operate format
def bitfield FMT	  <25:21>;
def bitfield FT		  <20:16>;
def bitfield FS		  <15:11>;
def bitfield FD		  <10: 6>;

def bitfield FP_FULLFUNC  <15: 5>; // complete function code
    def bitfield FP_TRAPMODE  <15:13>; // trapping mode
    def bitfield FP_ROUNDMODE <12:11>; // rounding mode
    def bitfield FP_TYPEFUNC  <10: 5>; // type+func: handiest for decoding
        def bitfield FP_SRCTYPE   <10: 9>; // source reg type
        def bitfield FP_SHORTFUNC < 8: 5>; // short function code
        def bitfield FP_SHORTFUNC_TOP2 <8:7>; // top 2 bits of short func code

// PALcode format
def bitfield PALFUNC	<25: 0>; // function code

// EV5 PAL instructions:
// HW_LD/HW_ST
def bitfield HW_LDST_PHYS  <15>; // address is physical
def bitfield HW_LDST_ALT   <14>; // use ALT_MODE IPR
def bitfield HW_LDST_WRTCK <13>; // HW_LD only: fault if no write acc
def bitfield HW_LDST_QUAD  <12>; // size: 0=32b, 1=64b
def bitfield HW_LDST_VPTE  <11>; // HW_LD only: is PTE fetch
def bitfield HW_LDST_LOCK  <10>; // HW_LD only: is load locked
def bitfield HW_LDST_COND  <10>; // HW_ST only: is store conditional
def signed bitfield HW_LDST_DISP  <9:0>; // signed displacement

// HW_REI
def bitfield HW_REI_TYP <15:14>; // type: stalling vs. non-stallingk
def bitfield HW_REI_MBZ <13: 0>; // must be zero

// HW_MTPR/MW_MFPR
def bitfield HW_IPR_IDX <15:0>;	 // IPR index

// M5 instructions
def bitfield M5FUNC <7:0>;
