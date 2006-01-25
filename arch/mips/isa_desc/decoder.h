////////////////////////////////////////////////////////////////////
//
// The actual MIPS32 ISA decoder
// -----------------------------
// The following instructions are specified in the MIPS32 ISA
// Specification. Decoding closely follows the style specified
// in the MIPS32 ISAthe specification document starting with Table
// A-2 (document available @ www.mips.com)
//
//@todo: Distinguish "unknown/future" use insts from "reserved"
// ones
decode OPCODE_HI default FailUnimpl::unknown() {

    // Derived From ... Table A-2 MIPS32 ISA Manual
    0x0: decode OPCODE_LO default FailUnimpl::reserved(){

        0x0: decode SPECIAL_HI {
            0x0: decode SPECIAL_LO {
              0x1: decode MOVCI {
                format Move {
                  0: movc({{ }});
                  1: movt({{ }});
                }
              }

              format ShiftRotate {
                //Table A-3 Note: "1. Specific encodings of the rt, rd, and sa fields
                //are used to distinguish among the SLL, NOP, SSNOP and EHB functions."
                0x0: sll({{ }});

                0x2: decode SRL {
                   0: srl({{ }});
                   1: rotr({{ }});
                 }

                 0x3: sar({{ }});

                 0x4: sllv({{ }});

                 0x6: decode SRLV {
                   0: srlv({{ }});
                   1: rotrv({{ }});
                 }

                 0x7: srav({{ }});
              }
            }

            0x1: decode SPECIAL_LO {

              //Table A-3 Note: "Specific encodings of the hint field are used
              //to distinguish JR from JR.HB and JALR from JALR.HB"
              format Jump {
                0x0: jr({{ }});
                0x1: jalr({{ }});
              }

              format Move {
                0x2: movz({{ }});
                0x3: movn({{ }});
              }

              0x4: Syscall::syscall({{ }});
              0x5: Break::break({{ }});
              0x7: Synchronize::synch({{ }});
            }

            0x2: decode SPECIAL_LO {
              format MultDiv {
                0x0: mfhi({{ }});
                0x1: mthi({{ }});
                0x2: mflo({{ }});
                0x3: mtlo({{ }});
              }
            };

            0x3: decode SPECIAL_LO {
              format MultDiv {
                0x0: mult({{ }});
                0x1: multu({{ }});
                0x2: div({{ }});
                0x3: divu({{ }});
              }
            };

            0x4: decode SPECIAL_LO {
              format Arithmetic {
                0x0: add({{ }});
                0x1: addu({{ }});
                0x2: sub({{ }});
                0x3: subu({{ }});
              }

              format Logical {
                0x0: and({{ }});
                0x1: or({{ }});
                0x2: xor({{ }});
                0x3: nor({{ }});
              }
            }

            0x5: decode SPECIAL_LO {
              format SetInstructions{
                0x2: slt({{ }});
                0x3: sltu({{ }});
              }
            };

            0x6: decode SPECIAL_LO {
              format Trap {
                 0x0: tge({{ }});
                 0x1: tgeu({{ }});
                 0x2: tlt({{ }});
                 0x3: tltu({{ }});
                 0x4: teq({{ }});
                 0x6: tne({{ }});
              }
            }
        }

        0x1: decode REGIMM_HI {
            0x0: decode REGIMM_LO {
              format Branch {
                0x0: bltz({{ }});
                0x1: bgez({{ }});

                //MIPS obsolete instructions
                0x2: bltzl({{ }});
                0x3: bgezl({{ }});
              }
            }

            0x1: decode REGIMM_LO {
              format Trap {
                 0x0: tgei({{ }});
                 0x1: tgeiu({{ }});
                 0x2: tlti({{ }});
                 0x3: tltiu({{ }});
                 0x4: teqi({{ }});
                 0x6: tnei({{ }});
              }
            }

            0x2: decode REGIMM_LO {
              format Branch {
                0x0: bltzal({{ }});
                0x1: bgezal({{ }});

                //MIPS obsolete instructions
                0x2: bltzall({{ }});
                0x3: bgezall({{ }});
              }
            }

            0x3: decode REGIMM_LO {
              0x7: synci({{ }});
            }
        }

        format Jump {
            0x2: j({{ }});
            0x3: jal({{ }});
        }

        format Branch {
            0x4: beq({{ }});
            0x5: bne({{ }});
            0x6: blez({{ }});
            0x7: bgtz({{ }});
        }
    };

    0x1: decode OPCODE_LO default FailUnimpl::reserved(){
        format IntImmediate {
            0x0: addi({{ }});
            0x1: addiu({{ }});
            0x2: slti({{ }});
            0x3: sltiu({{ }});
            0x4: andi({{ }});
            0x5: ori({{ }});
            0x6: xori({{ }});
            0x7: lui({{ }});
        };
    };

    0x2: decode OPCODE_LO default FailUnimpl::reserved(){

      0x0: decode RS {
        //Table A-11 MIPS32 COP0 Encoding of rs Field
      }

      0x1: decode RS {
        //Table A-13 MIPS32 COP1 Encoding of rs Field
      }

      0x2: decode RS {
        //Table A-19 MIPS32 COP2 Encoding of rs Field
      }

      0x3: decode FUNCTION_HI {
        //Table A-20 MIPS64 COP1X Encoding of Function Field 1
      }

      //MIPS obsolete instructions
      0x4: beql({{ }});
      0x5: bnel({{ }});
      0x6: blezl({{ }});
      0x7: bgtzl({{ }});
    };

    0x3: decode OPCODE_LO default FailUnimpl::reserved(){
        format FailUnimpl{
            0x0: reserved_inst_exception({{ }})
            0x1: reserved_inst_exception({{ }})
            0x2: reserved_inst_exception({{ }})
            0x3: reserved_inst_exception({{ }})
            0x5: reserved_inst_exception({{ }})
            0x6: reserved_inst_exception({{ }})
        };

        4: decode SPECIAL2 {
            0x0:;
            0x1:;
            0x2:;
            0x3:;
            0x4:;
            0x5:;
            0x6:;
        }

        7: decode SPECIAL3 {
            0x0:;
            0x1:;
            0x2:;
            0x3:;
            0x4:;
            0x5:;
            0x6:;
        }
    };

    0x4: decode OPCODE_LO default FailUnimpl::reserved(){
        format LoadMemory{
            0x0: lb({{ }});
            0x1: lh({{ }});
            0x2: lwl({{ }});
            0x3: lw({{ }});
            0x4: lbu({{ }});
            0x5: lhu({{ }});
            0x6: lhu({{ }});
        };

        0x7: FailUnimpl::reserved_inst_exception({{ }});
    };

    0x5: decode OPCODE_LO default FailUnimpl::reserved(){
        format StoreMemory{
            0x0: sb({{ }});
            0x1: sh({{ }});
            0x2: swl({{ }});
            0x3: sw({{ }});
            0x6: swr({{ }});
        };

        format FailUnimpl{
            0x4: reserved_inst_exception({{ }});
            0x5: reserved_inst_exception({{ }});
            0x2: cache({{ }});
        };

    };

    0x6: decode OPCODE_LO default FailUnimpl::reserved(){
        format LoadMemory{
            0x0: ll({{ }});
            0x1: lwc1({{ }});
            0x5: ldc1({{ }});
        };

        format FailUnimpl{
            0x2: lwc2({{ }});
            0x3: pref({{ }});
            0x4: reserved_inst_exception({{ }});
            0x6: ldc2({{ }});
            0x7: reserved_inst_exception({{ }});
        };

    };

    0x7: decode OPCODE_LO default FailUnimpl::reserved(){
        format StoreMemory{
            0x0: sc({{ }});
            0x1: swc1({{ }});
            0x5: sdc1({{ }});
        };

        format FailUnimpl{
            0x2: swc2({{ }});
            0x3: reserved_inst_exception({{ }});
            0x4: reserved_inst_exception({{ }});
            0x6: sdc2({{ }});
            0x7: reserved_inst_exception({{ }});
        };

    };

}

