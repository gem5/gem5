# src/cpu/pred/CMakeLists.txt
# Converted from src/cpu/pred/SConscript

gem5_add_source(bpred_unit.cc)
gem5_add_source(2bit_local.cc)
gem5_add_source(simple_indirect.cc)
gem5_add_source(conditional.cc)
gem5_add_source(it_tage.cc)
gem5_add_source(indirect.cc)
gem5_add_source(ras.cc)
gem5_add_source(tournament.cc)
gem5_add_source(bi_mode.cc)
gem5_add_source(tage_base.cc)
gem5_add_source(tage.cc)
gem5_add_source(loop_predictor.cc)
gem5_add_source(ltage.cc)
gem5_add_source(multiperspective_perceptron.cc)
gem5_add_source(multiperspective_perceptron_8KB.cc)
gem5_add_source(multiperspective_perceptron_64KB.cc)
gem5_add_source(multiperspective_perceptron_tage.cc)
gem5_add_source(multiperspective_perceptron_tage_8KB.cc)
gem5_add_source(multiperspective_perceptron_tage_64KB.cc)
gem5_add_source(statistical_corrector.cc)
gem5_add_source(tage_sc_l.cc)
gem5_add_source(tage_sc_l_8KB.cc)
gem5_add_source(tage_sc_l_64KB.cc)
gem5_add_source(gshare.cc)
gem5_add_source(btb.cc)
gem5_add_source(simple_btb.cc)

gem5_add_debug_flag(Indirect "Indirect branch predictor")
gem5_add_debug_flag(BTB "Branch Target Buffer")
gem5_add_debug_flag(RAS "Return Address Stack")
gem5_add_debug_flag(FreeList "FreeList debugging")
gem5_add_debug_flag(Branch "Branch predictor debugging")
gem5_add_debug_flag(Tage "TAGE predictor")
gem5_add_debug_flag(LTage "LTAGE predictor")
gem5_add_debug_flag(TageSCL "TAGE-SC-L predictor")

gem5_add_simobject(BranchPredictor.py
    SIM_OBJECTS
        BranchPredictor
        ConditionalPredictor
        IndirectPredictor SimpleIndirectPredictor
        BranchTargetBuffer SimpleBTB BTBIndexingPolicy BTBSetAssociative
        ReturnAddrStack
        ITTAGE ITTAGE_TAGE
        LocalBP TournamentBP BiModeBP TAGEBase TAGE LoopPredictor
        TAGE_SC_L_TAGE TAGE_SC_L_TAGE_64KB TAGE_SC_L_TAGE_8KB
        LTAGE TAGE_SC_L_LoopPredictor StatisticalCorrector TAGE_SC_L
        TAGE_SC_L_64KB_StatisticalCorrector
        TAGE_SC_L_8KB_StatisticalCorrector
        TAGE_SC_L_64KB TAGE_SC_L_8KB MultiperspectivePerceptron
        MultiperspectivePerceptron8KB MultiperspectivePerceptron64KB
        MPP_TAGE MPP_LoopPredictor MPP_StatisticalCorrector
        MultiperspectivePerceptronTAGE MPP_StatisticalCorrector_64KB
        MultiperspectivePerceptronTAGE64KB MPP_TAGE_8KB
        MPP_LoopPredictor_8KB MPP_StatisticalCorrector_8KB
        MultiperspectivePerceptronTAGE8KB GshareBP
    ENUMS BranchType TargetProvider)
