////////////////////////////////////////////////////////////////////
//
// The actual decoder specification
//

decode OP default Trap::unknown({{illegal_instruction}}) {

        0x0: decode OP2 {
                0x0: Trap::illtrap({{illegal_instruction}});  //ILLTRAP
                0x1: Branch::bpcc({{
                        switch((CC12 << 1) | CC02)
                        {
                                case 1: case 3:
                                        throw illegal_instruction;
                                case 0:
                                        if(passesCondition(xc->regs.MiscRegs.ccrFields.icc, COND2))
                                                ;//branchHere
                                break;
                                case 2:
                                        if(passesCondition(xc->regs.MiscRegs.ccrFields.xcc, COND2))
                                                ;//branchHere
                                break;
                        }
                }});//BPcc
                0x2: Branch::bicc({{
                        if(passesCondition(xc->regs.MiscRegs.ccrFields.icc, COND2))
                                ;//branchHere
                }});//Bicc
                0x3: Branch::bpr({{
                        switch(RCOND)
                        {
                                case 0: case 4:
                                        throw illegal_instruction;
                                case 1:
                                        if(Rs1 == 0) ;//branchHere
                                break;
                                case 2:
                                        if(Rs1 <= 0) ;//branchHere
                                break;
                                case 3:
                                        if(Rs1 < 0) ;//branchHere
                                break;
                                case 5:
                                        if(Rs1 != 0) ;//branchHere
                                break;
                                case 6:
                                        if(Rs1 > 0) ;//branchHere
                                break;
                                case 7:
                                        if(Rs1 >= 0) ;//branchHere
                                break;
                        }
                }});    //BPr
                0x4: IntegerOp::sethi({{Rd = (IMM22 << 10) & 0xFFFFFC00;}});   //SETHI (or NOP if rd == 0 and imm == 0)
                0x5: Trap::fbpfcc({{throw fp_disabled;}}); //FBPfcc
                0x6: Trap::fbfcc({{throw fp_disabled;}});  //FBfcc
        }
        0x1: Branch::call({{
                //branch here
                Rd = xc->pc;
        }});
        0x2: decode OP3 {
                format IntegerOp {
                        0x00: add({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                Rd = Rs1.sdw + val2;
                        }});//ADD
                        0x01: and({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                Rd = Rs1.udw & val2;
                        }});//AND
                        0x02: or({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                Rd = Rs1.udw | val2;
                        }});//OR
                        0x03: xor({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                Rd = Rs1.udw ^ val2;
                        }});//XOR
                        0x04: sub({{
                                INT64 val2 = ~((UINT64)(I ? SIMM13.sdw : Rs2.udw))+1;
                                Rd = Rs1.sdw + val2;
                        }});//SUB
                        0x05: andn({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                Rd = Rs1.udw & ~val2;
                        }});//ANDN
                        0x06: orn({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                Rd = Rs1.udw | ~val2;
                        }});//ORN
                        0x07: xnor({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                Rd = ~(Rs1.udw ^ val2);
                        }});//XNOR
                        0x08: addc({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                INT64 carryin = xc->regs.MiscRegs.ccrfields.iccfields.c;
                                Rd = Rs1.sdw + val2 + carryin;
                        }});//ADDC
                        0x09: mulx({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = Rs1 * val2;
                        }});//MULX
                        0x0A: umul({{
                                UINT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2.udw);
                                Rd = resTemp = Rs1.udw<31:0> * val2<31:0>;
                                xc->regs.MiscRegs.yFields.value = resTemp<63:32>;
                        }});//UMUL
                        0x0B: smul({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                rd.sdw = resTemp = Rs1.sdw<31:0> * val2<31:0>;
                                xc->regs.MiscRegs.yFields.value = resTemp<63:32>;
                        }});//SMUL
                        0x0C: subc({{
                                INT64 val2 = ~((INT64)(I ? SIMM13.sdw : Rs2.sdw))+1;
                                INT64 carryin = xc->regs.MiscRegs.ccrfields.iccfields.c;
                                Rd.sdw = Rs1.sdw + val2 + carryin;
                        }});//SUBC
                        0x0D: udivx({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                if(val2 == 0) throw division_by_zero;
                                Rd.udw = Rs1.udw / val2;
                        }});//UDIVX
                        0x0E: udiv({{
                                UINT32 resTemp, val2 = (I ? SIMM13.sw : Rs2.udw<31:0>);
                                if(val2 == 0) throw division_by_zero;
                                resTemp = (UINT64)((xc->regs.MiscRegs.yFields.value << 32) | Rs1.udw<31:0>) / val2;
                                INT32 overflow = (resTemp<63:32> != 0);
                                if(overflow) rd.udw = resTemp = 0xFFFFFFFF;
                                else rd.udw = resTemp;
                        }});   //UDIV
                        0x0F: sdiv({{
                                INT32 resTemp, val2 = (I ? SIMM13.sw : Rs2.sdw<31:0>);
                                if(val2 == 0) throw division_by_zero;
                                Rd.sdw = resTemp = (INT64)((xc->regs.MiscRegs.yFields.value << 32) | Rs1.sdw<31:0>) / val2;
                                INT32 overflow = (resTemp<63:31> != 0);
                                INT32 underflow = (resTemp<63:> && resTemp<62:31> != 0xFFFFFFFF);
                                if(overflow) rd.udw = resTemp = 0x7FFFFFFF;
                                else if(underflow) rd.udw = resTemp = 0xFFFFFFFF80000000;
                                else rd.udw = resTemp;
                        }});//SDIV
                }
                format IntegerOpCc {
                        0x10: addcc({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1 + val2;}},
                                {{((Rs1 & 0xFFFFFFFF + val2 & 0xFFFFFFFF) >> 31)}},
                                {{Rs1<31:> == val2<31:> && val2<31:> != resTemp<31:>}},
                                {{((Rs1 >> 1) + (val2 >> 1) + (Rs1 & val2 & 0x1))<63:>}},
                                {{Rs1<63:> == val2<63:> && val2<63:> != resTemp<63:>}}
                        );//ADDcc
                        0x11: andcc({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = Rs1 & val2;}}
                        ,{{0}},{{0}},{{0}},{{0}});//ANDcc
                        0x12: orcc({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = Rs1 | val2;}}
                        ,{{0}},{{0}},{{0}},{{0}});//ORcc
                        0x13: xorcc({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = Rs1 ^ val2;}}
                        ,{{0}},{{0}},{{0}},{{0}});//XORcc
                        0x14: subcc({{
                                INT64 resTemp, val2 = (INT64)(I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1 - val2;}},
                                {{((Rs1 & 0xFFFFFFFF + (~val2) & 0xFFFFFFFF + 1) >> 31)}},
                                {{Rs1<31:> != val2<31:> && Rs1<31:> != resTemp<31:>}},
                                {{((Rs1 >> 1) + (~val2) >> 1) + ((Rs1 | ~val2) & 0x1))<63:>}},
                                {{Rs1<63:> != val2<63:> && Rs1<63:> != resTemp<63:>}}
                        );//SUBcc
                        0x15: andncc({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = Rs1 & ~val2;}}
                        ,{{0}},{{0}},{{0}},{{0}});//ANDNcc
                        0x16: orncc({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = Rs1 | ~val2;}}
                        ,{{0}},{{0}},{{0}},{{0}});//ORNcc
                        0x17: xnorcc({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = ~(Rs1 ^ val2);}}
                        ,{{0}},{{0}},{{0}},{{0}});//XNORcc
                        0x18: addccc({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                INT64 carryin = xc->regs.MiscRegs.ccrfields.iccfields.c;
                                Rd = resTemp = Rs1 + val2 + carryin;}},
                                {{((Rs1 & 0xFFFFFFFF + val2 & 0xFFFFFFFF) >> 31 + carryin)}},
                                {{Rs1<31:> == val2<31:> && val2<31:> != resTemp<31:>}},
                                {{((Rs1 >> 1) + (val2 >> 1) + ((Rs1 & val2) | (carryin & (Rs1 | val2)) & 0x1))<63:>}},
                                {{Rs1<63:> == val2<63:> && val2<63:> != resTemp<63:>}}
                        );//ADDCcc
                        0x1A: umulcc({{
                                UINT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1.udw<31:0> * val2<31:0>;
                                xc->regs.MiscRegs.yFields.value = resTemp<63:32>;}}
                        ,{{0}},{{0}},{{0}},{{0}});//UMULcc
                        0x1B: smulcc({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1.sdw<31:0> * val2<31:0>;
                                xc->regs.MiscRegs.yFields.value = resTemp<63:32>;}}
                        ,{{0}},{{0}},{{0}},{{0}});//SMULcc
                        0x1C: subccc({{
                                INT64 resTemp, val2 = (INT64)(I ? SIMM13.sdw : Rs2);
                                INT64 carryin = xc->regs.MiscRegs.ccrfields.iccfields.c;
                                Rd = resTemp = Rs1 + ~(val2 + carryin) + 1;}},
                                {{((Rs1 & 0xFFFFFFFF + (~(val2 + carryin)) & 0xFFFFFFFF + 1) >> 31)}},
                                {{Rs1<31:> != val2<31:> && Rs1<31:> != resTemp<31:>}},
                                {{((Rs1 >> 1) + (~(val2 + carryin)) >> 1) + ((Rs1 | ~(val2+carryin)) & 0x1))<63:>}},
                                {{Rs1<63:> != val2<63:> && Rs1<63:> != resTemp<63:>}}
                        );//SUBCcc
                        0x1D: udivxcc({{
                                UINT64 val2 = (I ? SIMM13.sdw : Rs2.udw);
                                if(val2 == 0) throw division_by_zero;
                                Rd.udw = Rs1.udw / val2;}}
                        ,{{0}},{{0}},{{0}},{{0}});//UDIVXcc
                        0x1E: udivcc({{
                                UINT32 resTemp, val2 = (I ? SIMM13.sw : Rs2.udw<31:0>);
                                if(val2 == 0) throw division_by_zero;
                                resTemp = (UINT64)((xc->regs.MiscRegs.yFields.value << 32) | Rs1.udw<31:0>) / val2;
                                INT32 overflow = (resTemp<63:32> != 0);
                                if(overflow) rd.udw = resTemp = 0xFFFFFFFF;
                                else rd.udw = resTemp;}},
                                {{0}},
                                {{overflow}},
                                {{0}},
                                {{0}}
                        );//UDIVcc
                        0x1F: sdivcc({{
                                INT32 resTemp, val2 = (I ? SIMM13.sw : Rs2.sdw<31:0>);
                                if(val2 == 0) throw division_by_zero;
                                Rd.sdw = resTemp = (INT64)((xc->regs.MiscRegs.yFields.value << 32) | Rs1.sdw<31:0>) / val2;
                                INT32 overflow = (resTemp<63:31> != 0);
                                INT32 underflow = (resTemp<63:> && resTemp<62:31> != 0xFFFFFFFF);
                                if(overflow) rd.udw = resTemp = 0x7FFFFFFF;
                                else if(underflow) rd.udw = resTemp = 0xFFFFFFFF80000000;
                                else rd.udw = resTemp;}},
                                {{0}},
                                {{overflow || underflow}},
                                {{0}},
                                {{0}}
                        );//SDIVcc
                        0x20: taddcc({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1 + val2;
                                INT32 overflow = Rs1<1:0> || val2<1:0> || (Rs1<31:> == val2<31:> && val2<31:> != resTemp<31:>);}},
                                {{((Rs1 & 0xFFFFFFFF + val2 & 0xFFFFFFFF) >> 31)}},
                                {{overflow}},
                                {{((Rs1 >> 1) + (val2 >> 1) + (Rs1 & val2 & 0x1))<63:>}},
                                {{Rs1<63:> == val2<63:> && val2<63:> != resTemp<63:>}}
                        );//TADDcc
                        0x21: tsubcc({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1 + val2;
                                INT32 overflow = Rs1<1:0> || val2<1:0> || (Rs1<31:> == val2<31:> && val2<31:> != resTemp<31:>);}},
                                {{(Rs1 & 0xFFFFFFFF + val2 & 0xFFFFFFFF) >> 31)}},
                                {{overflow}},
                                {{((Rs1 >> 1) + (val2 >> 1) + (Rs1 & val2 & 0x1))<63:>}},
                                {{Rs1<63:> == val2<63:> && val2<63:> != resTemp<63:>}}
                        );//TSUBcc
                        0x22: taddcctv({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1 + val2;
                                INT32 overflow = Rs1<1:0> || val2<1:0> || (Rs1<31:> == val2<31:> && val2<31:> != resTemp<31:>);
                                if(overflow) throw tag_overflow;}},
                                {{((Rs1 & 0xFFFFFFFF + val2 & 0xFFFFFFFF) >> 31)}},
                                {{overflow}},
                                {{((Rs1 >> 1) + (val2 >> 1) + (Rs1 & val2 & 0x1))<63:>}},
                                {{Rs1<63:> == val2<63:> && val2<63:> != resTemp<63:>}}
                        );//TADDccTV
                        0x23: tsubcctv({{
                                INT64 resTemp, val2 = (I ? SIMM13.sdw : Rs2);
                                Rd = resTemp = Rs1 + val2;
                                INT32 overflow = Rs1<1:0> || val2<1:0> || (Rs1<31:> == val2<31:> && val2<31:> != resTemp<31:>);
                                if(overflow) throw tag_overflow;}},
                                {{((Rs1 & 0xFFFFFFFF + val2 & 0xFFFFFFFF) >> 31)}},
                                {{overflow}},
                                {{((Rs1 >> 1) + (val2 >> 1) + (Rs1 & val2 & 0x1))<63:>}},
                                {{Rs1<63:> == val2<63:> && val2<63:> != resTemp<63:>}}
                        );//TSUBccTV
                        0x24: mulscc({{
                                INT64 resTemp, multiplicand = (I ? SIMM13.sdw : Rs2);
                                INT32 multiplier = Rs1<31:0>;
                                INT32 savedLSB = Rs1<0:>;
                                multiplier = multipler<31:1> |
                                        ((xc->regs.MiscRegs.ccrFields.iccFields.n
                                        ^ xc->regs.MiscRegs.ccrFields.iccFields.v) << 32);
                                if(!xc->regs.MiscRegs.yFields.value<0:>)
                                        multiplicand = 0;
                                Rd = resTemp = multiplicand + multiplier;
                                xc->regs.MiscRegs.yFields.value = xc->regs.MiscRegs.yFields.value<31:1> | (savedLSB << 31);}},
                                {{((multiplicand & 0xFFFFFFFF + multiplier & 0xFFFFFFFF) >> 31)}},
                                {{multiplicand<31:> == multiplier<31:> && multiplier<31:> != resTemp<31:>}},
                                {{((multiplicand >> 1) + (multiplier >> 1) + (multiplicand & multiplier & 0x1))<63:>}},
                                {{multiplicand<63:> == multiplier<63:> && multiplier<63:> != resTemp<63:>}}
                        );//MULScc
                }
                format IntegerOp
                {
                        0x25: decode X {
                                0x0: sll({{Rd = Rs1 << (I ? SHCNT32 : Rs2<4:0>);}}); //SLL
                                0x1: sllx({{Rd = Rs1 << (I ? SHCNT64 : Rs2<5:0>);}}); //SLLX
                        }
                        0x26: decode X {
                                0x0: srl({{Rd = Rs1.udw<31:0> >> (I ? SHCNT32 : Rs2<4:0>);}}); //SRL
                                0x1: srlx({{Rd = Rs1.udw >> (I ? SHCNT64 : Rs2<5:0>);}});//SRLX
                        }
                        0x27: decode X {
                                0x0: sra({{Rd = Rs1.sdw<31:0> >> (I ? SHCNT32 : Rs2<4:0>);}}); //SRA
                                0x1: srax({{Rd = Rs1.sdw >> (I ? SHCNT64 : Rs2<5:0>);}});//SRAX
                        }
                        0x28: decode RS1 {
                                0x0: rdy({{Rd = xc->regs.MiscRegs.yFields.value;}}); //RDY
                                0x2: rdccr({{Rd = xc->regs.MiscRegs.ccr;}}); //RDCCR
                                0x3: rdasi({{Rd = xc->regs.MiscRegs.asi;}}); //RDASI
                                0x4: rdtick({{
                                        if(xc->regs.MiscRegs.pstateFields.priv == 0 &&
                                                xc->regs.MiscRegs.tickFields.npt == 1)
                                                throw privileged_action;
                                        Rd = xc->regs.MiscRegs.tick;
                                }});//RDTICK
                                0x5: rdpc({{Rd = xc->regs.pc;}}); //RDPC
                                0x6: rdfprs({{Rd = xc->regs.MiscRegs.fprs;}}); //RDFPRS
                                0xF: decode I {
                                        0x0: Noop::membar({{//Membar isn't needed yet}}); //MEMBAR
                                        0x1: Noop::stbar({{//Stbar isn/'t needed yet}}); //STBAR
                                }
                        }

                        0x2A: decode RS1 {
                                0x0: rdprtpc({{checkPriv Rd = xc->regs.MiscRegs.tpc[xc->regs.MiscRegs.tl];}});
                                0x1: rdprtnpc({{checkPriv Rd = xc->regs.MiscRegs.tnpc[xc->regs.MiscRegs.tl];}});
                                0x2: rdprtstate({{checkPriv Rd = xc->regs.MiscRegs.tstate[xc->regs.MiscRegs.tl];}});
                                0x3: rdprtt({{checkPriv Rd = xc->regs.MiscRegs.tt[xc->regs.MiscRegs.tl];}});
                                0x4: rdprtick({{checkPriv Rd = xc->regs.MiscRegs.tick;}});
                                0x5: rdprtba({{checkPriv Rd = xc->regs.MiscRegs.tba;}});
                                0x6: rdprpstate({{checkPriv Rd = xc->regs.MiscRegs.pstate;}});
                                0x7: rdprtl({{checkPriv Rd = xc->regs.MiscRegs.tl;}});
                                0x8: rdprpil({{checkPriv Rd = xc->regs.MiscRegs.pil;}});
                                0x9: rdprcwp({{checkPriv Rd = xc->regs.MiscRegs.cwp;}});
                                0xA: rdprcansave({{checkPriv Rd = xc->regs.MiscRegs.cansave;}});
                                0xB: rdprcanrestore({{checkPriv Rd = xc->regs.MiscRegs.canrestore;}});
                                0xC: rdprcleanwin({{checkPriv Rd = xc->regs.MiscRegs.cleanwin;}});
                                0xD: rdprotherwin({{checkPriv Rd = xc->regs.MiscRegs.otherwin;}});
                                0xE: rdprwstate({{checkPriv Rd = xc->regs.MiscRegs.wstate;}});
                                0xF: rdprfq({{throw illegal_instruction;}}); //The floating point queue isn't implemented right now.
                        }
                        0x2B: BasicOperate::flushw({{\\window toilet}}); //FLUSHW
                        0x2C: movcc({{
                                ccBank = (CC24 << 2) | (CC14 << 1) | (CC04 << 0);
                                switch(ccBank)
                                {
                                        case 0: case 1: case 2: case 3:
                                                throw fp_disabled;
                                        break;
                                        case 5: case 7:
                                                throw illegal_instruction;
                                        break;
                                        case 4:
                                                if(passesCondition(xc->regs.MiscRegs.ccrFields.icc, COND4))
                                                        Rd = (I ? SIMM11.sdw : RS2);
                                        break;
                                        case 6:
                                                if(passesCondition(xc->regs.MiscRegs.ccrFields.xcc, COND4))
                                                        Rd = (I ? SIMM11.sdw : RS2);
                                        break;
                                }
                        }});//MOVcc
                        0x2D: sdivx({{
                                INT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                if(val2 == 0) throw division_by_zero;
                                Rd.sdw = Rs1.sdw / val2;
                        }});//SDIVX
                        0x2E: decode RS1 {
                                0x0: IntegerOp::popc({{
                                INT64 count = 0, val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                UINT8 oneBits[] = {0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4}
                                for(unsigned int x = 0; x < 16; x++)
                                {
                                        count += oneBits[val2 & 0xF];
                                        val2 >> 4;
                                }
                                }});//POPC
                        }
                        0x2F: movr({{
                                UINT64 val2 = (I ? SIMM10.sdw : Rs2.sdw);
                                switch(RCOND)
                                {
                                        case 0: case 4:
                                                throw illegal_instruction;
                                        break;
                                        case 1:
                                                if(Rs1 == 0) Rd = val2;
                                        break;
                                        case 2:
                                                if(Rs1 <= 0) Rd = val2;
                                        break;
                                        case 3:
                                                if(Rs1 = 0) Rd = val2;
                                        break;
                                        case 5:
                                                if(Rs1 != 0) Rd = val2;
                                        break;
                                        case 6:
                                                if(Rs1 > 0) Rd = val2;
                                        break;
                                        case 7:
                                                if(Rs1 >= 0) Rd = val2;
                                        break;
                                }
                        }});//MOVR
                        0x30: decode RD {
                                0x0: wry({{
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.y = Rs1 ^ val2;
                                }});//WRY
                                0x2: wrccr({{
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.ccr = Rs1 ^ val2;
                                }});//WRCCR
                                0x3: wrasi({{
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.asi = Rs1 ^ val2;
                                }});//WRASI
                                0x6: wrfprs({{
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.asi = Rs1 ^ val2;
                                }});//WRFPRS
                                0xF: Trap::sir({{software_initiated_reset}}); //SIR
                        }
                        0x31: decode FCN {
                                0x0: BasicOperate::saved({{\\Boogy Boogy}}); //SAVED
                                0x1: BasicOperate::restored({{\\Boogy Boogy}}); //RESTORED
                        }
                        0x32: decode RD {
                                0x0: wrprtpc({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.tpc[xc->regs.MiscRegs.tl] = Rs1 ^ val2;
                                }});
                                0x1: wrprtnpc({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.tnpc[xc->regs.MiscRegs.tl] = Rs1 ^ val2;
                                }});
                                0x2: wrprtstate({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.tstate[xc->regs.MiscRegs.tl] = Rs1 ^ val2;
                                }});
                                0x3: wrprtt({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.tt[xc->regs.MiscRegs.tl] = Rs1 ^ val2;
                                }});
                                0x4: wrprtick({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.tick = Rs1 ^ val2;
                                }});
                                0x5: wrprtba({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.tba = Rs1 ^ val2;
                                }});
                                0x6: wrprpstate({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.pstate = Rs1 ^ val2;
                                }});
                                0x7: wrprtl({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.tl = Rs1 ^ val2;
                                }});
                                0x8: wrprpil({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.pil = Rs1 ^ val2;
                                }});
                                0x9: wrprcwp({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.cwp = Rs1 ^ val2;
                                }});
                                0xA: wrprcansave({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.cansave = Rs1 ^ val2;
                                }});
                                0xB: wrprcanrestore({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.canrestore = Rs1 ^ val2;
                                }});
                                0xC: wrprcleanwin({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.cleanwin = Rs1 ^ val2;
                                }});
                                0xD: wrprotherwin({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.otherwin = Rs1 ^ val2;
                                }});
                                0xE: wrprwstate({{checkPriv
                                        UINT64 val2 = (I ? SIMM13.sdw : Rs2.sdw);
                                        xc->regs.MiscRegs.wstate = Rs1 ^ val2;
                                }});
                        }

                        0x34: Trap::fpop1({{Throw fp_disabled;}}); //FPOP1
                        0x35: Trap::fpop2({{Throw fp_disabled;}}); //FPOP2


                        0x38: Branch::jmpl({{//Stuff}}); //JMPL
                        0x39: Branch::return({{//Other Stuff}}); //RETURN
                        0x3A: Trap::tcc({{
                                switch((CC14 << 1) | (CC04 << 0))
                                {
                                        case 1: case 3:
                                                throw illegal_instruction;
                                        case 0:
                                                if(passesCondition(xc->regs.MiscRegs.ccrFields.icc, machInst<25:28>))
                                                        throw trap_instruction;
                                        break;
                                        case 2:
                                                if(passesCondition(xc->regs.MiscRegs.ccrFields.xcc, machInst<25:28>))
                                                        throw trap_instruction;
                                        break;
                                }
                        }}); //Tcc
                        0x3B: BasicOperate::flush({{//Lala}}); //FLUSH
                        0x3C: BasicOperate::save({{//leprechauns); //SAVE
                        0x3D: BasicOperate::restore({{//Eat my short int}}); //RESTORE
                        0x3E: decode FCN {
                                0x1: BasicOperate::done({{//Done thing}}); //DONE
                                0x2: BasicOperate::retry({{//Retry thing}}); //RETRY
                        }
                }
        }
        0x3: decode OP3 {
                format Mem {
                        0x00: lduw({{Rd.uw = Mem.uw;}}); //LDUW
                        0x01: ldub({{Rd.ub = Mem.ub;}}); //LDUB
                        0x02: lduh({{Rd.uhw = Mem.uhw;}}); //LDUH
                        0x03: ldd({{
                                UINT64 val = Mem.udw;
                                setIntReg(RD & (~1), val<31:0>);
                                setIntReg(RD | 1, val<63:32>);
                        }});//LDD
                        0x04: stw({{Mem.sw = Rd.sw;}}); //STW
                        0x05: stb({{Mem.sb = Rd.sb;}}); //STB
                        0x06: sth({{Mem.shw = Rd.shw;}}); //STH
                        0x07: std({{
                                Mem.udw = readIntReg(RD & (~1))<31:0> | (readIntReg(RD | 1)<31:0> << 32);
                        }});//STD
                        0x08: ldsw({{Rd.sw = Mem.sw;}}); //LDSW
                        0x09: ldsb({{Rd.sb = Mem.sb;}}); //LDSB
                        0x0A: ldsh({{Rd.shw = Mem.shw;}}); //LDSH
                        0x0B: ldx({{Rd.udw = Mem.udw;}}); //LDX

                        0x0D: ldstub({{
                                Rd.ub = Mem.ub;
                                Mem.ub = 0xFF;
                        }}); //LDSTUB
                        0x0E: stx({{Rd.udw = Mem.udw;}}); //STX
                        0x0F: swap({{
                                UINT32 temp = Rd.uw;
                                Rd.uw = Mem.uw;
                                Mem.uw = temp;
                        }}); //SWAP
                        0x10: lduwa({{Rd.uw = Mem.uw;}}); //LDUWA
                        0x11: lduba({{Rd.ub = Mem.ub;}}); //LDUBA
                        0x12: lduha({{Rd.uhw = Mem.uhw;}}); //LDUHA
                        0x13: ldda({{
                                UINT64 val = Mem.udw;
                                setIntReg(RD & (~1), val<31:0>);
                                setIntReg(RD | 1, val<63:32>);
                        }}); //LDDA
                        0x14: stwa({{Mem.uw = Rd.uw;}}); //STWA
                        0x15: stba({{Mem.ub = Rd.ub;}}); //STBA
                        0x16: stha({{Mem.uhw = Rd.uhw;}}); //STHA
                        0x17: stda({{
                                Mem.udw = readIntReg(RD & (~1))<31:0> | (readIntReg(RD | 1)<31:0> << 32);
                        }}); //STDA
                        0x18: ldswa({{Rd.sw = Mem.sw;}}); //LDSWA
                        0x19: ldsba({{Rd.sb = Mem.sb;}}); //LDSBA
                        0x1A: ldsha({{Rd.shw = Mem.shw;}}); //LDSHA
                        0x1B: ldxa({{Rd.sdw = Mem.sdw;}}); //LDXA

                        0x1D: ldstuba({{
                                Rd.ub = Mem.ub;
                                Mem.ub = 0xFF;
                        }}); //LDSTUBA
                        0x1E: stxa({{Mem.sdw = Rd.sdw}}); //STXA
                        0x1F: swapa({{
                                UINT32 temp = Rd.uw;
                                Rd.uw = Mem.uw;
                                Mem.uw = temp;
                        }}); //SWAPA
                        0x20: Trap::ldf({{throw fp_disabled;}}); //LDF
                        0x21: decode X {
                                0x0: Trap::ldfsr({{throw fp_disabled;}}); //LDFSR
                                0x1: Trap::ldxfsr({{throw fp_disabled;}}); //LDXFSR
                        }
                        0x22: Trap::ldqf({{throw fp_disabled;}}); //LDQF
                        0x23: Trap::lddf({{throw fp_disabled;}}); //LDDF
                        0x24: Trap::stf({{throw fp_disabled;}}); //STF
                        0x25: decode X {
                                0x0: Trap::stfsr({{throw fp_disabled;}}); //STFSR
                                0x1: Trap::stxfsr({{throw fp_disabled;}}); //STXFSR
                        }
                        0x26: Trap::stqf({{throw fp_disabled;}}); //STQF
                        0x27: Trap::stdf({{throw fp_disabled;}}); //STDF





                        0x2D: Noop::prefetch({{ }}); //PREFETCH


                        0x30: Trap::ldfa({{throw fp_disabled;}}); //LDFA

                        0x32: Trap::ldqfa({{throw fp_disabled;}}); //LDQFA
                        0x33: Trap::lddfa({{throw fp_disabled;}}); //LDDFA
                        0x34: Trap::stfa({{throw fp_disabled;}}); //STFA
                        0x35: Trap::stqfa({{throw fp_disabled;}}); //STQFA
                        0x36: Trap::stdfa({{throw fp_disabled;}}); //STDFA





                        0x3C: Cas::casa(
                                {{UINT64 val = Mem.uw;
                                if(Rs2.uw == val)
                                        Mem.uw = Rd.uw;
                                Rd.uw = val;
                        }}); //CASA
                        0x3D: Noop::prefetcha({{ }}); //PREFETCHA
                        0x3E: Cas::casxa(
                                {{UINT64 val = Mem.udw;
                                if(Rs2 == val)
                                        Mem.udw = Rd;
                                Rd = val;
                        }}); //CASXA
                }
        }
}
