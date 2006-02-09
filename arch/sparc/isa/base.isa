////////////////////////////////////////////////////////////////////
//
// Base class for sparc instructions, and some support functions
//

output header {{
        /**
         * Base class for all SPARC static instructions.
         */
        class SparcStaticInst : public StaticInst<SPARCISA>
        {
        protected:

                // Constructor.
                SparcStaticInst(const char *mnem, MachInst _machInst, OpClass __opClass)
                    : StaticInst<SPARCISA>(mnem, _machInst, __opClass)
                {
                }

                std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
        };

        bool passesCondition(struct {uint8_t c:1; uint8_t v:1; uint8_t z:1; uint8_t n:1} codes, uint8_t condition);
}};

output decoder {{

        std::string SparcStaticInst::generateDisassembly(Addr pc, const SymbolTable *symtab) const
        {
                std::stringstream ss;

                ccprintf(ss, "%-10s ", mnemonic);

                // just print the first two source regs... if there's
                // a third one, it's a read-modify-write dest (Rc),
                // e.g. for CMOVxx
                if(_numSrcRegs > 0)
                {
                        printReg(ss, _srcRegIdx[0]);
                }
                if(_numSrcRegs > 1)
                {
                        ss << ",";
                        printReg(ss, _srcRegIdx[1]);
                }

                // just print the first dest... if there's a second one,
                // it's generally implicit
                if(_numDestRegs > 0)
                {
                        if(_numSrcRegs > 0)
                                ss << ",";
                        printReg(ss, _destRegIdx[0]);
                }

                return ss.str();
        }

        bool passesCondition(struct {uint8_t c:1; uint8_t v:1; uint8_t z:1; uint8_t n:1} codes, uint8_t condition)
        {
                switch(condition)
                {
                        case 0b1000: return true;
                        case 0b0000: return false;
                        case 0b1001: return !codes.z;
                        case 0b0001: return codes.z;
                        case 0b1010: return !(codes.z | (codes.n ^ codes.v));
                        case 0b0010: return codes.z | (codes.n ^ codes.v);
                        case 0b1011: return !(codes.n ^ codes.v);
                        case 0b0011: return (codes.n ^ codes.v);
                        case 0b1100: return !(codes.c | codes.z);
                        case 0b0100: return (codes.c | codes.z);
                        case 0b1101: return !codes.c;
                        case 0b0101: return codes.c;
                        case 0b1110: return !codes.n;
                        case 0b0110: return codes.n;
                        case 0b1111: return !codes.v;
                        case 0b0111: return codes.v;
                }
        }
}};

