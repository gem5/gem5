# Add all ARMv8 state
def upgrader(cpt):
    if cpt.get('root','isa') != 'arm':
        return
    import re
    print "Warning: The size of the FP register file has changed. "\
          "To get similar results you need to adjust the number of "\
          "physical registers in the CPU you're restoring into by "\
          "NNNN."
    # Find the CPU context's and upgrade their registers
    for sec in cpt.sections():
        re_xc_match = re.match('^.*?sys.*?\.cpu(\d+)*\.xc\.*', sec)
        if not re_xc_match:
            continue

        # Update floating point regs
        fpr = cpt.get(sec, 'floatRegs.i').split()
        # v8 has 128 normal fp and 32 special fp regs compared
        # to v7's 64 normal fp and 8 special fp regs.
        # Insert the extra normal fp registers at end of v7 normal fp regs
        for x in xrange(64):
            fpr.insert(64, "0")
        # Append the extra special registers
        for x in xrange(24):
            fpr.append("0")
        cpt.set(sec, 'floatRegs.i', ' '.join(str(x) for x in fpr))

        ir = cpt.get(sec, 'intRegs').split()
        # Add in v8 int reg state
        # Splice in R13_HYP
        ir.insert(20, "0")
        # Splice in INTREG_DUMMY and SP0 - SP3
        ir.extend(["0", "0", "0", "0", "0"])
        cpt.set(sec, 'intRegs', ' '.join(str(x) for x in ir))

    # Update the cpu interrupt field
    for sec in cpt.sections():
        re_int_match = re.match("^.*?sys.*?\.cpu(\d+)*$", sec)
        if not re_int_match:
            continue

        irqs = cpt.get(sec, "interrupts").split()
        irqs.append("false")
        irqs.append("false")
        cpt.set(sec, "interrupts", ' '.join(str(x) for x in irqs))

    # Update the per cpu interrupt structure
    for sec in cpt.sections():
        re_int_match = re.match("^.*?sys.*?\.cpu(\d+)*\.interrupts$", sec)
        if not re_int_match:
            continue

        irqs = cpt.get(sec, "interrupts").split()
        irqs.append("false")
        irqs.append("false")
        cpt.set(sec, "interrupts", ' '.join(str(x) for x in irqs))

    # Update the misc regs and add in new isa specific fields
    for sec in cpt.sections():
        re_isa_match = re.match("^.*?sys.*?\.cpu(\d+)*\.isa$", sec)
        if not re_isa_match:
            continue

        cpt.set(sec, 'haveSecurity', 'false')
        cpt.set(sec, 'haveLPAE', 'false')
        cpt.set(sec, 'haveVirtualization', 'false')
        cpt.set(sec, 'haveLargeAsid64', 'false')
        cpt.set(sec, 'physAddrRange64', '40')

        # splice in the new misc registers, ~200 -> 605 registers,
        # ordering does not remain consistent
        mr_old = cpt.get(sec, 'miscRegs').split()
        mr_new = [ '0' for x in xrange(605) ]

        # map old v7 miscRegs to new v8 miscRegs
        mr_new[0] = mr_old[0] # CPSR
        mr_new[16] = mr_old[1] # CPSR_Q
        mr_new[1] = mr_old[2] # SPSR
        mr_new[2] = mr_old[3] # SPSR_FIQ
        mr_new[3] = mr_old[4] # SPSR_IRQ
        mr_new[4] = mr_old[5] # SPSR_SVC
        mr_new[5] = mr_old[6] # SPSR_MON
        mr_new[8] = mr_old[7] # SPSR_UND
        mr_new[6] = mr_old[8] # SPSR_ABT
        mr_new[432] = mr_old[9] # FPSR
        mr_new[10] = mr_old[10] # FPSID
        mr_new[11] = mr_old[11] # FPSCR
        mr_new[18] = mr_old[12] # FPSCR_QC
        mr_new[17] = mr_old[13] # FPSCR_EXC
        mr_new[14] = mr_old[14] # FPEXC
        mr_new[13] = mr_old[15] # MVFR0
        mr_new[12] = mr_old[16] # MVFR1
        mr_new[28] = mr_old[17] # SCTLR_RST,
        mr_new[29] = mr_old[18] # SEV_MAILBOX,
        mr_new[30] = mr_old[19] # DBGDIDR
        mr_new[31] = mr_old[20] # DBGDSCR_INT,
        mr_new[33] = mr_old[21] # DBGDTRRX_INT,
        mr_new[34] = mr_old[22] # DBGTRTX_INT,
        mr_new[35] = mr_old[23] # DBGWFAR,
        mr_new[36] = mr_old[24] # DBGVCR,
        #mr_new[] = mr_old[25] # DBGECR -> UNUSED,
        #mr_new[] = mr_old[26] # DBGDSCCR -> UNUSED,
        #mr_new[] = mr_old[27] # DBGSMCR -> UNUSED,
        mr_new[37] = mr_old[28] # DBGDTRRX_EXT,
        mr_new[38] = mr_old[29] # DBGDSCR_EXT,
        mr_new[39] = mr_old[30] # DBGDTRTX_EXT,
        #mr_new[] = mr_old[31] # DBGDRCR -> UNUSED,
        mr_new[41] = mr_old[32] # DBGBVR,
        mr_new[47] = mr_old[33] # DBGBCR,
        #mr_new[] = mr_old[34] # DBGBVR_M -> UNUSED,
        #mr_new[] = mr_old[35] # DBGBCR_M -> UNUSED,
        mr_new[61] = mr_old[36] # DBGDRAR,
        #mr_new[] = mr_old[37] # DBGBXVR_M -> UNUSED,
        mr_new[64] = mr_old[38] # DBGOSLAR,
        #mr_new[] = mr_old[39] # DBGOSSRR -> UNUSED,
        mr_new[66] = mr_old[40] # DBGOSDLR,
        mr_new[67] = mr_old[41] # DBGPRCR,
        #mr_new[] = mr_old[42] # DBGPRSR -> UNUSED,
        mr_new[68] = mr_old[43] # DBGDSAR,
        #mr_new[] = mr_old[44] # DBGITCTRL -> UNUSED,
        mr_new[69] = mr_old[45] # DBGCLAIMSET,
        mr_new[70] = mr_old[46] # DBGCLAIMCLR,
        mr_new[71] = mr_old[47] # DBGAUTHSTATUS,
        mr_new[72] = mr_old[48] # DBGDEVID2,
        mr_new[73] = mr_old[49] # DBGDEVID1,
        mr_new[74] = mr_old[50] # DBGDEVID,
        mr_new[77] = mr_old[51] # TEEHBR,
        mr_new[109] = mr_old[52] # v7 SCTLR -> aarc32 SCTLR_NS
        mr_new[189] = mr_old[53] # DCCISW,
        mr_new[188] = mr_old[54] # DCCIMVAC,
        mr_new[183] = mr_old[55] # DCCMVAC,
        mr_new[271] = mr_old[56] # v7 CONTEXTIDR -> aarch32 CONTEXTIDR_NS,
        mr_new[274] = mr_old[57] # v7 TPIDRURW -> aarch32 TPIDRURW_NS,
        mr_new[277] = mr_old[58] # v7 TPIDRURO -> aarch32 TPIDRURO_NS,
        mr_new[280] = mr_old[59] # v7 TPIDRPRW -> aarch32 TPIDRPRW_NS,
        mr_new[170] = mr_old[60] # CP15ISB,
        mr_new[185] = mr_old[61] # CP15DSB,
        mr_new[186] = mr_old[62] # CP15DMB,
        mr_new[114] = mr_old[63] # CPACR,
        mr_new[101] = mr_old[64] # CLIDR,
        mr_new[100] = mr_old[65] # CCSIDR,
        mr_new[104] = mr_old[66] # v7 CSSELR -> aarch32 CSSELR_NS,
        mr_new[163] = mr_old[67] # ICIALLUIS,
        mr_new[168] = mr_old[68] # ICIALLU,
        mr_new[169] = mr_old[69] # ICIMVAU,
        mr_new[172] = mr_old[70] # BPIMVA,
        mr_new[164] = mr_old[71] # BPIALLIS,
        mr_new[171] = mr_old[72] # BPIALL,
        mr_new[80] = mr_old[73] # MIDR,
        mr_new[126] = mr_old[74] # v7 TTBR0 -> aarch32 TTBR0_NS,
        mr_new[129] = mr_old[75] # v7 TTBR1 -> aarch32 TTBR1_NS,
        mr_new[83] = mr_old[76] # TLBTR,
        mr_new[137] = mr_old[77] # v7 DACR -> aarch32 DACR_NS,
        mr_new[192] = mr_old[78] # TLBIALLIS,
        mr_new[193] = mr_old[79] # TLBIMVAIS,
        mr_new[194] = mr_old[80] # TLBIASIDIS,
        mr_new[195] = mr_old[81] # TLBIMVAAIS,
        mr_new[198] = mr_old[82] # ITLBIALL,
        mr_new[199] = mr_old[83] # ITLBIMVA,
        mr_new[200] = mr_old[84] # ITLBIASID,
        mr_new[201] = mr_old[85] # DTLBIALL,
        mr_new[202] = mr_old[86] # DTLBIMVA,
        mr_new[203] = mr_old[87] # DTLBIASID,
        mr_new[204] = mr_old[88] # TLBIALL,
        mr_new[205] = mr_old[89] # TLBIMVA,
        mr_new[206] = mr_old[90] # TLBIASID,
        mr_new[207] = mr_old[91] # TLBIMVAA,
        mr_new[140] = mr_old[92] # v7 DFSR -> aarch32 DFSR_NS,
        mr_new[143] = mr_old[93] # v7 IFSR -> aarch32 IFSR_NS,
        mr_new[155] = mr_old[94] # v7 DFAR -> aarch32 DFAR_NS,
        mr_new[158] = mr_old[95] # v7 IFAR -> aarch32 IFAR_NS,
        mr_new[84] = mr_old[96] # MPIDR,
        mr_new[241] = mr_old[97] # v7 PRRR -> aarch32 PRRR_NS,
        mr_new[247] = mr_old[98] # v7 NMRR -> aarch32 NMRR_NS,
        mr_new[131] = mr_old[99] # TTBCR,
        mr_new[86] = mr_old[100] # ID_PFR0,
        mr_new[81] = mr_old[101] # CTR,
        mr_new[115] = mr_old[102] # SCR,
        # Set the non-secure bit
        scr = int(mr_new[115])
        scr = scr | 0x1
        mr_new[115] = str(scr)
        ###
        mr_new[116] = mr_old[103] # SDER,
        mr_new[165] = mr_old[104] # PAR,
        mr_new[175] = mr_old[105] # V2PCWPR -> ATS1CPR,
        mr_new[176] = mr_old[106] # V2PCWPW -> ATS1CPW,
        mr_new[177] = mr_old[107] # V2PCWUR -> ATS1CUR,
        mr_new[178] = mr_old[108] # V2PCWUW -> ATS1CUW,
        mr_new[179] = mr_old[109] # V2POWPR -> ATS12NSOPR,
        mr_new[180] = mr_old[110] # V2POWPW -> ATS12NSOPW,
        mr_new[181] = mr_old[111] # V2POWUR -> ATS12NSOUR,
        mr_new[182] = mr_old[112] # V2POWUW -> ATS12NWOUW,
        mr_new[90] = mr_old[113] # ID_MMFR0,
        mr_new[92] = mr_old[114] # ID_MMFR2,
        mr_new[93] = mr_old[115] # ID_MMFR3,
        mr_new[112] = mr_old[116] # v7 ACTLR -> aarch32 ACTLR_NS
        mr_new[222] = mr_old[117] # PMCR,
        mr_new[230] = mr_old[118] # PMCCNTR,
        mr_new[223] = mr_old[119] # PMCNTENSET,
        mr_new[224] = mr_old[120] # PMCNTENCLR,
        mr_new[225] = mr_old[121] # PMOVSR,
        mr_new[226] = mr_old[122] # PMSWINC,
        mr_new[227] = mr_old[123] # PMSELR,
        mr_new[228] = mr_old[124] # PMCEID0,
        mr_new[229] = mr_old[125] # PMCEID1,
        mr_new[231] = mr_old[126] # PMXEVTYPER,
        mr_new[233] = mr_old[127] # PMXEVCNTR,
        mr_new[234] = mr_old[128] # PMUSERENR,
        mr_new[235] = mr_old[129] # PMINTENSET,
        mr_new[236] = mr_old[130] # PMINTENCLR,
        mr_new[94] = mr_old[131] # ID_ISAR0,
        mr_new[95] = mr_old[132] # ID_ISAR1,
        mr_new[96] = mr_old[133] # ID_ISAR2,
        mr_new[97] = mr_old[134] # ID_ISAR3,
        mr_new[98] = mr_old[135] # ID_ISAR4,
        mr_new[99] = mr_old[136] # ID_ISAR5,
        mr_new[20] = mr_old[137] # LOCKFLAG,
        mr_new[19] = mr_old[138] # LOCKADDR,
        mr_new[87] = mr_old[139] # ID_PFR1,
        # Set up the processor features register
        pfr = int(mr_new[87])
        pfr = pfr | 0x1011
        mr_new[87] = str(pfr)
        ###
        mr_new[238] = mr_old[140] # L2CTLR,
        mr_new[82] = mr_old[141] # TCMTR
        mr_new[88] = mr_old[142] # ID_DFR0,
        mr_new[89] = mr_old[143] # ID_AFR0,
        mr_new[91] = mr_old[144] # ID_MMFR1,
        mr_new[102] = mr_old[145] # AIDR,
        mr_new[146] = mr_old[146] # v7 ADFSR -> aarch32 ADFSR_NS,
        mr_new[148] = mr_old[147] # AIFSR,
        mr_new[173] = mr_old[148] # DCIMVAC,
        mr_new[174] = mr_old[149] # DCISW,
        mr_new[184] = mr_old[150] # MCCSW -> DCCSW,
        mr_new[187] = mr_old[151] # DCCMVAU,
        mr_new[117] = mr_old[152] # NSACR,
        mr_new[262] = mr_old[153] # VBAR,
        mr_new[265] = mr_old[154] # MVBAR,
        mr_new[267] = mr_old[155] # ISR,
        mr_new[269] = mr_old[156] # FCEIDR -> FCSEIDR,
        #mr_new[] = mr_old[157] # L2LATENCY -> UNUSED,
        #mr_new[] = mr_old[158] # CRN15 -> UNUSED,
        mr_new[599] = mr_old[159] # NOP
        mr_new[600] = mr_old[160] # RAZ,

        # Set the new miscRegs structure
        cpt.set(sec, 'miscRegs', ' '.join(str(x) for x in mr_new))

    cpu_prefix = {}
    # Add in state for ITB/DTB
    for sec in cpt.sections():
        re_tlb_match = re.match('(^.*?sys.*?\.cpu(\d+)*)\.(dtb|itb)$', sec)
        if not re_tlb_match:
            continue

        cpu_prefix[re_tlb_match.group(1)] = True # Save off prefix to add
        # Set the non-secure bit (bit 9) to 1 for attributes
        attr = int(cpt.get(sec, '_attr'))
        attr = attr | 0x200
        cpt.set(sec, '_attr', str(attr))
        cpt.set(sec, 'haveLPAE', 'false')
        cpt.set(sec, 'directToStage2', 'false')
        cpt.set(sec, 'stage2Req', 'false')
        cpt.set(sec, 'bootUncacheability', 'true')

    # Add in extra state for the new TLB Entries
    for sec in cpt.sections():
        re_tlbentry_match = re.match('(^.*?sys.*?\.cpu(\d+)*)\.(dtb|itb).TlbEntry\d+$', sec)
        if not re_tlbentry_match:
            continue

        # Add in the new entries
        cpt.set(sec, 'longDescFormat', 'false')
        cpt.set(sec, 'vmid', '0')
        cpt.set(sec, 'isHyp', 'false')
        valid = cpt.get(sec, 'valid')
        if valid == 'true':
            cpt.set(sec, 'ns', 'true')
            cpt.set(sec, 'nstid', 'true')
            cpt.set(sec, 'pxn', 'true')
            cpt.set(sec, 'hap', '3')
            # All v7 code used 2 level page tables
            cpt.set(sec, 'lookupLevel', '2')
            attr = int(cpt.get(sec, 'attributes'))
            # set the non-secure bit (bit 9) to 1
            # as no previous v7 code used secure code
            attr = attr | 0x200
            cpt.set(sec, 'attributes', str(attr))
        else:
            cpt.set(sec, 'ns', 'false')
            cpt.set(sec, 'nstid', 'false')
            cpt.set(sec, 'pxn', 'false')
            cpt.set(sec, 'hap', '0')
            cpt.set(sec, 'lookupLevel', '0')
        cpt.set(sec, 'outerShareable', 'false')

    # Add d/istage2_mmu and d/istage2_mmu.stage2_tlb
    for key in cpu_prefix:
        for suffix in ['.istage2_mmu', '.dstage2_mmu']:
            new_sec = key + suffix
            cpt.add_section(new_sec)
            new_sec = key + suffix + ".stage2_tlb"
            cpt.add_section(new_sec)
            # Fill in tlb info with some defaults
            cpt.set(new_sec, '_attr', '0')
            cpt.set(new_sec, 'haveLPAE', 'false')
            cpt.set(new_sec, 'directToStage2', 'false')
            cpt.set(new_sec, 'stage2Req', 'false')
            cpt.set(new_sec, 'bootUncacheability', 'false')
            cpt.set(new_sec, 'num_entries', '0')

legacy_version = 9
