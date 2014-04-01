/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/


#ifndef CORE_H_
#define CORE_H_

#include "XML_Parse.h"
#include "array.h"
#include "basic_components.h"
#include "interconnect.h"
#include "logic.h"
#include "parameter.h"
#include "sharedcache.h"

class BranchPredictor :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        ArrayST * globalBPT;
        ArrayST * localBPT;
        ArrayST * L1_localBPT;
        ArrayST * L2_localBPT;
        ArrayST * chooser;
        ArrayST * RAS;
        bool exist;

        BranchPredictor(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exsit=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~BranchPredictor();
};


class InstFetchU :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        enum Cache_policy cache_p;
        InstCache icache;
        ArrayST * IB;
        ArrayST * BTB;
        BranchPredictor * BPT;
        inst_decoder * ID_inst;
        inst_decoder * ID_operand;
        inst_decoder * ID_misc;
        bool exist;

        InstFetchU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exsit=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~InstFetchU();
};


class SchedulerU :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        double Iw_height, fp_Iw_height,ROB_height;
        ArrayST         * int_inst_window;
        ArrayST         * fp_inst_window;
        ArrayST         * ROB;
    selection_logic * instruction_selection;
    bool exist;

    SchedulerU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~SchedulerU();
};

class RENAMINGU :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        double clockRate,executionTime;
        CoreDynParam  coredynp;
        ArrayST * iFRAT;
        ArrayST * fFRAT;
        ArrayST * iRRAT;
        ArrayST * fRRAT;
        ArrayST * ifreeL;
        ArrayST * ffreeL;
        dep_resource_conflict_check * idcl;
        dep_resource_conflict_check * fdcl;
        ArrayST * RAHT;//register alias history table Used to store GC
        bool exist;


        RENAMINGU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~RENAMINGU();
};

class LoadStoreU :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        enum Cache_policy cache_p;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        double lsq_height;
        DataCache dcache;
        ArrayST * LSQ;//it is actually the store queue but for inorder processors it serves as both loadQ and StoreQ
        ArrayST * LoadQ;
        bool exist;

        LoadStoreU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~LoadStoreU();
};

class MemManU :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        ArrayST * itlb;
        ArrayST * dtlb;
        bool exist;

        MemManU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~MemManU();
};

class RegFU :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        double int_regfile_height, fp_regfile_height;
        ArrayST * IRF;
        ArrayST * FRF;
        ArrayST * RFWIN;
        bool exist;

        RegFU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~RegFU();
};

class EXECU :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        double lsq_height;
        CoreDynParam  coredynp;
        RegFU          * rfu;
        SchedulerU     * scheu;
    FunctionalUnit * fp_u;
    FunctionalUnit * exeu;
    FunctionalUnit * mul;
        interconnect * int_bypass;
        interconnect * intTagBypass;
        interconnect * int_mul_bypass;
        interconnect * intTag_mul_Bypass;
        interconnect * fp_bypass;
        interconnect * fpTagBypass;

        Component  bypass;
        bool exist;

        EXECU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_, double lsq_height_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
        void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~EXECU();
};


class Core :public Component {
  public:

        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        InstFetchU * ifu;
        LoadStoreU * lsu;
        MemManU    * mmu;
        EXECU      * exu;
        RENAMINGU  * rnu;
    Pipeline   * corepipe;
    UndiffCore * undiffCore;
    SharedCache * l2cache;
    CoreDynParam  coredynp;
    //full_decoder 	inst_decoder;
    //clock_network	clockNetwork;
        Core(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_);
        void set_core_param();
        void computeEnergy(bool is_tdp=true);
        void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~Core();
};

#endif /* CORE_H_ */
