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


#include <cstdio>
#include <string>

#include "XML_Parse.h"
#include "xmlParser.h"

using namespace std;

void ParseXML::parse(char* filepath)
{
        unsigned int i,j,k,m,n;
        unsigned int NumofCom_4;
        unsigned int itmp;
        //Initialize all structures
        ParseXML::initialize();

        // this open and parse the XML file:
        XMLNode xMainNode=XMLNode::openFileHelper(filepath,"component"); //the 'component' in the first layer

        XMLNode xNode2=xMainNode.getChildNode("component"); // the 'component' in the second layer
        //get all params in the second layer
        itmp=xNode2.nChildNode("param");
        for(i=0; i<itmp; i++)
        {
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_of_cores")==0) {sys.number_of_cores=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_of_L1Directories")==0) {sys.number_of_L1Directories=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_of_L2Directories")==0) {sys.number_of_L2Directories=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_of_L2s")==0) {sys.number_of_L2s=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"Private_L2")==0) {sys.Private_L2=(bool)atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_of_L3s")==0) {sys.number_of_L3s=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_of_NoCs")==0) {sys.number_of_NoCs=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_of_dir_levels")==0) {sys.number_of_dir_levels=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"domain_size")==0) {sys.domain_size=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"first_level_dir")==0) {sys.first_level_dir=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"homogeneous_cores")==0) {sys.homogeneous_cores=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"core_tech_node")==0) {sys.core_tech_node=atof(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"target_core_clockrate")==0) {sys.target_core_clockrate=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"target_chip_area")==0) {sys.target_chip_area=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"temperature")==0) {sys.temperature=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"number_cache_levels")==0) {sys.number_cache_levels=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"L1_property")==0) {sys.L1_property =atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"L2_property")==0) {sys.L2_property =atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"homogeneous_L2s")==0) {sys.homogeneous_L2s=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"homogeneous_L1Directories")==0) {sys.homogeneous_L1Directories=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"homogeneous_L2Directories")==0) {sys.homogeneous_L2Directories=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"L3_property")==0) {sys.L3_property =atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"homogeneous_L3s")==0) {sys.homogeneous_L3s=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"homogeneous_ccs")==0) {sys.homogeneous_ccs=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"homogeneous_NoCs")==0) {sys.homogeneous_NoCs=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"Max_area_deviation")==0) {sys.Max_area_deviation=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"Max_power_deviation")==0) {sys.Max_power_deviation=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"device_type")==0) {sys.device_type=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"longer_channel_device")==0) {sys.longer_channel_device=(bool)atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"opt_dynamic_power")==0) {sys.opt_dynamic_power=(bool)atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"opt_lakage_power")==0) {sys.opt_lakage_power=(bool)atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"opt_clockrate")==0) {sys.opt_clockrate=(bool)atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"opt_area")==0) {sys.opt_area=(bool)atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"Embedded")==0) {sys.Embedded=(bool)atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"interconnect_projection_type")==0) {sys.interconnect_projection_type=atoi(xNode2.getChildNode("param",i).getAttribute("value"))==0?0:1;continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"machine_bits")==0) {sys.machine_bits=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"virtual_address_width")==0) {sys.virtual_address_width=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"physical_address_width")==0) {sys.physical_address_width=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
                if (strcmp(xNode2.getChildNode("param",i).getAttribute("name"),"virtual_memory_page_size")==0) {sys.virtual_memory_page_size=atoi(xNode2.getChildNode("param",i).getAttribute("value"));continue;}
        }

//	if (sys.Private_L2 && sys.number_of_cores!=sys.number_of_L2s)
//	{
//		cout<<"Private L2: Number of L2s must equal to Number of Cores"<<endl;
//		exit(0);
//	}

        itmp=xNode2.nChildNode("stat");
        for(i=0; i<itmp; i++)
        {
                if (strcmp(xNode2.getChildNode("stat",i).getAttribute("name"),"total_cycles")==0) {sys.total_cycles=atof(xNode2.getChildNode("stat",i).getAttribute("value"));continue;}

        }

        //get the number of components within the second layer
        unsigned int NumofCom_3=xNode2.nChildNode("component");
        XMLNode xNode3,xNode4; //define the third-layer(system.core0) and fourth-layer(system.core0.predictor) xnodes

        string strtmp;
        char chtmp[60];
        char chtmp1[60];
        chtmp1[0]='\0';
        unsigned int OrderofComponents_3layer=0;
        if (NumofCom_3>OrderofComponents_3layer)
        {
                //___________________________get all system.core0-n________________________________________________
                if (sys.homogeneous_cores==1) OrderofComponents_3layer=0;
                else OrderofComponents_3layer=sys.number_of_cores-1;
                for (i=0; i<=OrderofComponents_3layer; i++)
                {
                        xNode3=xNode2.getChildNode("component",i);
                        if (xNode3.isEmpty()==1) {
                                printf("The value of homogeneous_cores or number_of_cores is not correct!");
                                exit(0);
                        }
                        else{
                                if (strstr(xNode3.getAttribute("name"),"core")!=NULL)
                                {
                                        { //For cpu0-cpui
                                                //Get all params with system.core?
                                                itmp=xNode3.nChildNode("param");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clock_rate")==0) {sys.core[i].clock_rate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"opt_local")==0) {sys.core[i].opt_local=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"x86")==0) {sys.core[i].x86=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"machine_bits")==0) {sys.core[i].machine_bits=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"virtual_address_width")==0) {sys.core[i].virtual_address_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"physical_address_width")==0) {sys.core[i].physical_address_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"instruction_length")==0) {sys.core[i].instruction_length=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"opcode_width")==0) {sys.core[i].opcode_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"micro_opcode_width")==0) {sys.core[i].micro_opcode_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"machine_type")==0) {sys.core[i].machine_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"internal_datapath_width")==0) {sys.core[i].internal_datapath_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_hardware_threads")==0) {sys.core[i].number_hardware_threads=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"fetch_width")==0) {sys.core[i].fetch_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_instruction_fetch_ports")==0) {sys.core[i].number_instruction_fetch_ports=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"decode_width")==0) {sys.core[i].decode_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"issue_width")==0) {sys.core[i].issue_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"peak_issue_width")==0) {sys.core[i].peak_issue_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"commit_width")==0) {sys.core[i].commit_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"fp_issue_width")==0) {sys.core[i].fp_issue_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"prediction_width")==0) {sys.core[i].prediction_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}

                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"pipelines_per_core")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.core[i].pipelines_per_core[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.core[i].pipelines_per_core[m]=atoi(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"pipeline_depth")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.core[i].pipeline_depth[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.core[i].pipeline_depth[m]=atoi(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }

                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"FPU")==0) {strcpy(sys.core[i].FPU,xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"divider_multiplier")==0) {strcpy(sys.core[i].divider_multiplier,xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"ALU_per_core")==0) {sys.core[i].ALU_per_core=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"FPU_per_core")==0) {sys.core[i].FPU_per_core=atof(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"MUL_per_core")==0) {sys.core[i].MUL_per_core=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"instruction_buffer_size")==0) {sys.core[i].instruction_buffer_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"decoded_stream_buffer_size")==0) {sys.core[i].decoded_stream_buffer_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"instruction_window_scheme")==0) {sys.core[i].instruction_window_scheme  =atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"instruction_window_size")==0) {sys.core[i].instruction_window_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"fp_instruction_window_size")==0) {sys.core[i].fp_instruction_window_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"ROB_size")==0) {sys.core[i].ROB_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"archi_Regs_IRF_size")==0) {sys.core[i].archi_Regs_IRF_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"archi_Regs_FRF_size")==0) {sys.core[i].archi_Regs_FRF_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"phy_Regs_IRF_size")==0) {sys.core[i].phy_Regs_IRF_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"phy_Regs_FRF_size")==0) {sys.core[i].phy_Regs_FRF_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"rename_scheme")==0) {sys.core[i].rename_scheme=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"register_windows_size")==0) {sys.core[i].register_windows_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"LSU_order")==0) {strcpy(sys.core[i].LSU_order,xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"store_buffer_size")==0) {sys.core[i].store_buffer_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"load_buffer_size")==0) {sys.core[i].load_buffer_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"memory_ports")==0) {sys.core[i].memory_ports=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"Dcache_dual_pump")==0) {strcpy(sys.core[i].Dcache_dual_pump,xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"RAS_size")==0) {sys.core[i].RAS_size=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                }
                                                //Get all stats with system.core?
                                                itmp=xNode3.nChildNode("stat");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_instructions")==0) {sys.core[i].total_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"int_instructions")==0) {sys.core[i].int_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fp_instructions")==0) {sys.core[i].fp_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"branch_instructions")==0) {sys.core[i].branch_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"branch_mispredictions")==0) {sys.core[i].branch_mispredictions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"committed_instructions")==0) {sys.core[i].committed_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"committed_int_instructions")==0) {sys.core[i].committed_int_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"committed_fp_instructions")==0) {sys.core[i].committed_fp_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"load_instructions")==0) {sys.core[i].load_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"store_instructions")==0) {sys.core[i].store_instructions=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_cycles")==0) {sys.core[i].total_cycles=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"idle_cycles")==0) {sys.core[i].idle_cycles=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"busy_cycles")==0) {sys.core[i].busy_cycles=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"instruction_buffer_reads")==0) {sys.core[i].instruction_buffer_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"instruction_buffer_write")==0) {sys.core[i].instruction_buffer_write=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"ROB_reads")==0) {sys.core[i].ROB_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"ROB_writes")==0) {sys.core[i].ROB_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"rename_reads")==0) {sys.core[i].rename_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"rename_writes")==0) {sys.core[i].rename_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fp_rename_reads")==0) {sys.core[i].fp_rename_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fp_rename_writes")==0) {sys.core[i].fp_rename_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"inst_window_reads")==0) {sys.core[i].inst_window_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"inst_window_writes")==0) {sys.core[i].inst_window_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"inst_window_wakeup_accesses")==0) {sys.core[i].inst_window_wakeup_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"inst_window_selections")==0) {sys.core[i].inst_window_selections=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fp_inst_window_reads")==0) {sys.core[i].fp_inst_window_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fp_inst_window_writes")==0) {sys.core[i].fp_inst_window_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fp_inst_window_wakeup_accesses")==0) {sys.core[i].fp_inst_window_wakeup_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"archi_int_regfile_reads")==0) {sys.core[i].archi_int_regfile_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"archi_float_regfile_reads")==0) {sys.core[i].archi_float_regfile_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"phy_int_regfile_reads")==0) {sys.core[i].phy_int_regfile_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"phy_float_regfile_reads")==0) {sys.core[i].phy_float_regfile_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"phy_int_regfile_writes")==0) {sys.core[i].archi_int_regfile_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"phy_float_regfile_writes")==0) {sys.core[i].archi_float_regfile_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"archi_int_regfile_writes")==0) {sys.core[i].phy_int_regfile_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"archi_float_regfile_writes")==0) {sys.core[i].phy_float_regfile_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"int_regfile_reads")==0) {sys.core[i].int_regfile_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"float_regfile_reads")==0) {sys.core[i].float_regfile_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"int_regfile_writes")==0) {sys.core[i].int_regfile_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"float_regfile_writes")==0) {sys.core[i].float_regfile_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"windowed_reg_accesses")==0) {sys.core[i].windowed_reg_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"windowed_reg_transports")==0) {sys.core[i].windowed_reg_transports=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"function_calls")==0) {sys.core[i].function_calls=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"context_switches")==0) {sys.core[i].context_switches=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"ialu_accesses")==0) {sys.core[i].ialu_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fpu_accesses")==0) {sys.core[i].fpu_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"mul_accesses")==0) {sys.core[i].mul_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"cdb_alu_accesses")==0) {sys.core[i].cdb_alu_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"cdb_mul_accesses")==0) {sys.core[i].cdb_mul_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"cdb_fpu_accesses")==0) {sys.core[i].cdb_fpu_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"load_buffer_reads")==0) {sys.core[i].load_buffer_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"load_buffer_writes")==0) {sys.core[i].load_buffer_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"load_buffer_cams")==0) {sys.core[i].load_buffer_cams=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"store_buffer_reads")==0) {sys.core[i].store_buffer_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"store_buffer_writes")==0) {sys.core[i].store_buffer_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"store_buffer_cams")==0) {sys.core[i].store_buffer_cams=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"store_buffer_forwards")==0) {sys.core[i].store_buffer_forwards=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"main_memory_access")==0) {sys.core[i].main_memory_access=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"main_memory_read")==0) {sys.core[i].main_memory_read=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"main_memory_write")==0) {sys.core[i].main_memory_write=atoi(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"pipeline_duty_cycle")==0) {sys.core[i].pipeline_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}

                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"IFU_duty_cycle")==0) {sys.core[i].IFU_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"BR_duty_cycle")==0) {sys.core[i].BR_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"LSU_duty_cycle")==0) {sys.core[i].LSU_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"MemManU_I_duty_cycle")==0) {sys.core[i].MemManU_I_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"MemManU_D_duty_cycle")==0) {sys.core[i].MemManU_D_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"ALU_duty_cycle")==0) {sys.core[i].ALU_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"MUL_duty_cycle")==0) {sys.core[i].MUL_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"FPU_duty_cycle")==0) {sys.core[i].FPU_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"ALU_cdb_duty_cycle")==0) {sys.core[i].ALU_cdb_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"MUL_cdb_duty_cycle")==0) {sys.core[i].MUL_cdb_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"FPU_cdb_duty_cycle")==0) {sys.core[i].FPU_cdb_duty_cycle=atoi(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                }
                                        }

                                        NumofCom_4=xNode3.nChildNode("component"); //get the number of components within the third layer
                                        for(j=0; j<NumofCom_4; j++)
                                        {
                                                xNode4=xNode3.getChildNode("component",j);
                                                if (strcmp(xNode4.getAttribute("name"),"PBT")==0)
                                                { //find PBT
                                                        itmp=xNode4.nChildNode("param");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of param in system.core0.predictor--PBT
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"prediction_width")==0) {sys.core[i].predictor.prediction_width=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"prediction_scheme")==0) {strcpy(sys.core[i].predictor.prediction_scheme,xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"predictor_size")==0) {sys.core[i].predictor.predictor_size=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"predictor_entries")==0) {sys.core[i].predictor.predictor_entries=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"local_predictor_size")==0)
                                                                {
                                                                        strtmp.assign(xNode4.getChildNode("param",k).getAttribute("value"));
                                                                        m=0;
                                                                        for(n=0; n<strtmp.length(); n++)
                                                                        {
                                                                                if (strtmp[n]!=',')
                                                                                {
                                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                                        strcat(chtmp1,chtmp);
                                                                                }
                                                                                else{
                                                                                        sys.core[i].predictor.local_predictor_size[m]=atoi(chtmp1);
                                                                                        m++;
                                                                                        chtmp1[0]='\0';
                                                                                }
                                                                        }
                                                                        sys.core[i].predictor.local_predictor_size[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                        continue;
                                                                }
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"local_predictor_entries")==0) {sys.core[i].predictor.local_predictor_entries=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"global_predictor_entries")==0) {sys.core[i].predictor.global_predictor_entries=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"global_predictor_bits")==0) {sys.core[i].predictor.global_predictor_bits=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"chooser_predictor_entries")==0) {sys.core[i].predictor.chooser_predictor_entries=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"chooser_predictor_bits")==0) {sys.core[i].predictor.chooser_predictor_bits=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                        }
                                                        itmp=xNode4.nChildNode("stat");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of stat in system.core0.predictor--PBT
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"predictor_accesses")==0) sys.core[i].predictor.predictor_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));
                                                        }
                                                }
                                                if (strcmp(xNode4.getAttribute("name"),"itlb")==0)
                                                {//find system.core0.itlb
                                                        itmp=xNode4.nChildNode("param");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of param in system.core0.itlb--itlb
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"number_entries")==0) sys.core[i].itlb.number_entries=atoi(xNode4.getChildNode("param",k).getAttribute("value"));
                                                        }
                                                        itmp=xNode4.nChildNode("stat");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of stat in itlb
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_hits")==0) {sys.core[i].itlb.total_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.core[i].itlb.total_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_misses")==0) {sys.core[i].itlb.total_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.core[i].itlb.conflicts=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        }
                                                }
                                                if (strcmp(xNode4.getAttribute("name"),"icache")==0)
                                                {//find system.core0.icache
                                                        itmp=xNode4.nChildNode("param");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of param in system.core0.icache--icache
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"icache_config")==0)
                                                                {
                                                                        strtmp.assign(xNode4.getChildNode("param",k).getAttribute("value"));
                                                                        m=0;
                                                                        for(n=0; n<strtmp.length(); n++)
                                                                        {
                                                                                if (strtmp[n]!=',')
                                                                                {
                                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                                        strcat(chtmp1,chtmp);
                                                                                }
                                                                                else{
                                                                                        sys.core[i].icache.icache_config[m]=atof(chtmp1);
                                                                                        m++;
                                                                                        chtmp1[0]='\0';
                                                                                }
                                                                        }
                                                                        sys.core[i].icache.icache_config[m]=atof(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                        continue;
                                                                }
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"buffer_sizes")==0)
                                                                {
                                                                        strtmp.assign(xNode4.getChildNode("param",k).getAttribute("value"));
                                                                        m=0;
                                                                        for(n=0; n<strtmp.length(); n++)
                                                                        {
                                                                                if (strtmp[n]!=',')
                                                                                {
                                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                                        strcat(chtmp1,chtmp);
                                                                                }
                                                                                else{
                                                                                        sys.core[i].icache.buffer_sizes[m]=atoi(chtmp1);
                                                                                        m++;
                                                                                        chtmp1[0]='\0';
                                                                                }
                                                                        }
                                                                        sys.core[i].icache.buffer_sizes[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        itmp=xNode4.nChildNode("stat");
                                                        for(k=0; k<itmp; k++)
                                                        {
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.core[i].icache.total_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.core[i].icache.read_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.core[i].icache.read_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"replacements")==0) {sys.core[i].icache.replacements=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_hits")==0) {sys.core[i].icache.read_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_hits")==0) {sys.core[i].icache.total_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_misses")==0) {sys.core[i].icache.total_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"miss_buffer_access")==0) {sys.core[i].icache.miss_buffer_access=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"fill_buffer_accesses")==0) {sys.core[i].icache.fill_buffer_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_accesses")==0) {sys.core[i].icache.prefetch_buffer_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_writes")==0) {sys.core[i].icache.prefetch_buffer_writes=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_reads")==0) {sys.core[i].icache.prefetch_buffer_reads=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_hits")==0) {sys.core[i].icache.prefetch_buffer_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.core[i].icache.conflicts=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        }
                                                }
                                                if (strcmp(xNode4.getAttribute("name"),"dtlb")==0)
                                                {//find system.core0.dtlb
                                                        itmp=xNode4.nChildNode("param");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of param in system.core0.dtlb--dtlb
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"number_entries")==0) sys.core[i].dtlb.number_entries=atoi(xNode4.getChildNode("param",k).getAttribute("value"));
                                                        }
                                                        itmp=xNode4.nChildNode("stat");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of stat in dtlb
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.core[i].dtlb.total_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.core[i].dtlb.read_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_accesses")==0) {sys.core[i].dtlb.write_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_hits")==0) {sys.core[i].dtlb.read_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_hits")==0) {sys.core[i].dtlb.write_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.core[i].dtlb.read_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_misses")==0) {sys.core[i].dtlb.write_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_hits")==0) {sys.core[i].dtlb.total_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_misses")==0) {sys.core[i].dtlb.total_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.core[i].dtlb.conflicts=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}

                                                        }
                                                }
                                                if (strcmp(xNode4.getAttribute("name"),"dcache")==0)
                                                {//find system.core0.dcache
                                                        itmp=xNode4.nChildNode("param");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of param in system.core0.dcache--dcache
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"dcache_config")==0)
                                                                {
                                                                        strtmp.assign(xNode4.getChildNode("param",k).getAttribute("value"));
                                                                        m=0;
                                                                        for(n=0; n<strtmp.length(); n++)
                                                                        {
                                                                                if (strtmp[n]!=',')
                                                                                {
                                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                                        strcat(chtmp1,chtmp);
                                                                                }
                                                                                else{
                                                                                        sys.core[i].dcache.dcache_config[m]=atof(chtmp1);
                                                                                        m++;
                                                                                        chtmp1[0]='\0';
                                                                                }
                                                                        }
                                                                        sys.core[i].dcache.dcache_config[m]=atof(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                        continue;
                                                                }
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"buffer_sizes")==0)
                                                                {
                                                                        strtmp.assign(xNode4.getChildNode("param",k).getAttribute("value"));
                                                                        m=0;
                                                                        for(n=0; n<strtmp.length(); n++)
                                                                        {
                                                                                if (strtmp[n]!=',')
                                                                                {
                                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                                        strcat(chtmp1,chtmp);
                                                                                }
                                                                                else{
                                                                                        sys.core[i].dcache.buffer_sizes[m]=atoi(chtmp1);
                                                                                        m++;
                                                                                        chtmp1[0]='\0';
                                                                                }
                                                                        }
                                                                        sys.core[i].dcache.buffer_sizes[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        itmp=xNode4.nChildNode("stat");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of stat in dcache
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.core[i].dcache.total_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.core[i].dcache.read_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_accesses")==0) {sys.core[i].dcache.write_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_hits")==0) {sys.core[i].dcache.total_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_misses")==0) {sys.core[i].dcache.total_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_hits")==0) {sys.core[i].dcache.read_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_hits")==0) {sys.core[i].dcache.write_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.core[i].dcache.read_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_misses")==0) {sys.core[i].dcache.write_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"replacements")==0) {sys.core[i].dcache.replacements=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_backs")==0) {sys.core[i].dcache.write_backs=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"miss_buffer_access")==0) {sys.core[i].dcache.miss_buffer_access=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"fill_buffer_accesses")==0) {sys.core[i].dcache.fill_buffer_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_accesses")==0) {sys.core[i].dcache.prefetch_buffer_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_writes")==0) {sys.core[i].dcache.prefetch_buffer_writes=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_reads")==0) {sys.core[i].dcache.prefetch_buffer_reads=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_hits")==0) {sys.core[i].dcache.prefetch_buffer_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"wbb_writes")==0) {sys.core[i].dcache.wbb_writes=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"wbb_reads")==0) {sys.core[i].dcache.wbb_reads=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.core[i].dcache.conflicts=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}

                                                        }
                                                }
                                                if (strcmp(xNode4.getAttribute("name"),"BTB")==0)
                                                {//find system.core0.BTB
                                                        itmp=xNode4.nChildNode("param");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of param in system.core0.BTB--BTB
                                                                if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"BTB_config")==0)
                                                                {
                                                                        strtmp.assign(xNode4.getChildNode("param",k).getAttribute("value"));
                                                                        m=0;
                                                                        for(n=0; n<strtmp.length(); n++)
                                                                        {
                                                                                if (strtmp[n]!=',')
                                                                                {
                                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                                        strcat(chtmp1,chtmp);
                                                                                }
                                                                                else{
                                                                                        sys.core[i].BTB.BTB_config[m]=atoi(chtmp1);
                                                                                        m++;
                                                                                        chtmp1[0]='\0';
                                                                                }
                                                                        }
                                                                        sys.core[i].BTB.BTB_config[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        itmp=xNode4.nChildNode("stat");
                                                        for(k=0; k<itmp; k++)
                                                        { //get all items of stat in BTB
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.core[i].BTB.total_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.core[i].BTB.read_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_accesses")==0) {sys.core[i].BTB.write_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_hits")==0) {sys.core[i].BTB.total_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"total_misses")==0) {sys.core[i].BTB.total_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_hits")==0) {sys.core[i].BTB.read_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_hits")==0) {sys.core[i].BTB.write_hits=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.core[i].BTB.read_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"write_misses")==0) {sys.core[i].BTB.write_misses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                                if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"replacements")==0) {sys.core[i].BTB.replacements=atof(xNode4.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        }
                                                }
                                        }
                                }
                                else {
                                        printf("The value of homogeneous_cores or number_of_cores is not correct!");
                                        exit(0);
                                }
                        }
                }

                //__________________________________________Get system.L1Directory0-n____________________________________________
                int w,tmpOrderofComponents_3layer;
                w=OrderofComponents_3layer+1;
                tmpOrderofComponents_3layer=OrderofComponents_3layer;
                if (sys.homogeneous_L1Directories==1) OrderofComponents_3layer=OrderofComponents_3layer+1;
                else OrderofComponents_3layer=OrderofComponents_3layer+sys.number_of_L1Directories;

                for (i=0; i<(OrderofComponents_3layer-tmpOrderofComponents_3layer); i++)
                {
                        xNode3=xNode2.getChildNode("component",w);
                        if (xNode3.isEmpty()==1) {
                                printf("The value of homogeneous_L1Directories or number_of_L1Directories is not correct!");
                                exit(0);
                        }
                        else
                        {
                                if (strstr(xNode3.getAttribute("id"),"L1Directory")!=NULL)
                                {
                                        itmp=xNode3.nChildNode("param");
                                        for(k=0; k<itmp; k++)
                                        { //get all items of param in system.L1Directory
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"Dir_config")==0)
                                                {
                                                        strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                        m=0;
                                                        for(n=0; n<strtmp.length(); n++)
                                                        {
                                                                if (strtmp[n]!=',')
                                                                {
                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                        strcat(chtmp1,chtmp);
                                                                }
                                                                else{
                                                                        sys.L1Directory[i].Dir_config[m]=atof(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        sys.L1Directory[i].Dir_config[m]=atof(chtmp1);
                                                        m++;
                                                        chtmp1[0]='\0';
                                                        continue;
                                                }
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"buffer_sizes")==0)
                                                {
                                                        strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                        m=0;
                                                        for(n=0; n<strtmp.length(); n++)
                                                        {
                                                                if (strtmp[n]!=',')
                                                                {
                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                        strcat(chtmp1,chtmp);
                                                                }
                                                                else{
                                                                        sys.L1Directory[i].buffer_sizes[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        sys.L1Directory[i].buffer_sizes[m]=atoi(chtmp1);
                                                        m++;
                                                        chtmp1[0]='\0';
                                                        continue;
                                                }

                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clockrate")==0) {sys.L1Directory[i].clockrate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"ports")==0)
                                                {
                                                        strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                        m=0;
                                                        for(n=0; n<strtmp.length(); n++)
                                                        {
                                                                if (strtmp[n]!=',')
                                                                {
                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                        strcat(chtmp1,chtmp);
                                                                }
                                                                else{
                                                                        sys.L1Directory[i].ports[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        sys.L1Directory[i].ports[m]=atoi(chtmp1);
                                                        m++;
                                                        chtmp1[0]='\0';
                                                        continue;
                                                }
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"device_type")==0) {sys.L1Directory[i].device_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"Directory_type")==0) {sys.L1Directory[i].Directory_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"3D_stack")==0) {strcpy(sys.L1Directory[i].threeD_stack,xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                        }
                                        itmp=xNode3.nChildNode("stat");
                                        for(k=0; k<itmp; k++)
                                        { //get all items of stat in system.L2directorydirectory
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.L1Directory[i].total_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.L1Directory[i].read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_accesses")==0) {sys.L1Directory[i].write_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.L1Directory[i].read_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_misses")==0) {sys.L1Directory[i].write_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.L1Directory[i].conflicts=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) {sys.L1Directory[i].duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                        }
                                        w=w+1;
                                }
                                else {
                                        printf("The value of homogeneous_L1Directories or number_of_L1Directories is not correct!");
                                        exit(0);
                                }
                        }
                }

                //__________________________________________Get system.L2Directory0-n____________________________________________
                w=OrderofComponents_3layer+1;
                tmpOrderofComponents_3layer=OrderofComponents_3layer;
                if (sys.homogeneous_L2Directories==1) OrderofComponents_3layer=OrderofComponents_3layer+1;
                else OrderofComponents_3layer=OrderofComponents_3layer+sys.number_of_L2Directories;

                for (i=0; i<(OrderofComponents_3layer-tmpOrderofComponents_3layer); i++)
                {
                        xNode3=xNode2.getChildNode("component",w);
                        if (xNode3.isEmpty()==1) {
                                printf("The value of homogeneous_L2Directories or number_of_L2Directories is not correct!");
                                exit(0);
                        }
                        else
                        {
                                if (strstr(xNode3.getAttribute("id"),"L2Directory")!=NULL)
                                {
                                        itmp=xNode3.nChildNode("param");
                                        for(k=0; k<itmp; k++)
                                        { //get all items of param in system.L2Directory
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"Dir_config")==0)
                                                {
                                                        strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                        m=0;
                                                        for(n=0; n<strtmp.length(); n++)
                                                        {
                                                                if (strtmp[n]!=',')
                                                                {
                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                        strcat(chtmp1,chtmp);
                                                                }
                                                                else{
                                                                        sys.L2Directory[i].Dir_config[m]=atof(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        sys.L2Directory[i].Dir_config[m]=atof(chtmp1);
                                                        m++;
                                                        chtmp1[0]='\0';
                                                        continue;
                                                }
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"buffer_sizes")==0)
                                                {
                                                        strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                        m=0;
                                                        for(n=0; n<strtmp.length(); n++)
                                                        {
                                                                if (strtmp[n]!=',')
                                                                {
                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                        strcat(chtmp1,chtmp);
                                                                }
                                                                else{
                                                                        sys.L2Directory[i].buffer_sizes[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        sys.L2Directory[i].buffer_sizes[m]=atoi(chtmp1);
                                                        m++;
                                                        chtmp1[0]='\0';
                                                        continue;
                                                }

                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clockrate")==0) {sys.L2Directory[i].clockrate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"Directory_type")==0) {sys.L2Directory[i].Directory_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"ports")==0)
                                                {
                                                        strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                        m=0;
                                                        for(n=0; n<strtmp.length(); n++)
                                                        {
                                                                if (strtmp[n]!=',')
                                                                {
                                                                        sprintf(chtmp,"%c",strtmp[n]);
                                                                        strcat(chtmp1,chtmp);
                                                                }
                                                                else{
                                                                        sys.L2Directory[i].ports[m]=atoi(chtmp1);
                                                                        m++;
                                                                        chtmp1[0]='\0';
                                                                }
                                                        }
                                                        sys.L2Directory[i].ports[m]=atoi(chtmp1);
                                                        m++;
                                                        chtmp1[0]='\0';
                                                        continue;
                                                }
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"device_type")==0) {sys.L2Directory[i].device_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"3D_stack")==0) {strcpy(sys.L2Directory[i].threeD_stack,xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                        }
                                        itmp=xNode3.nChildNode("stat");
                                        for(k=0; k<itmp; k++)
                                        { //get all items of stat in system.L2directorydirectory
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.L2Directory[i].total_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.L2Directory[i].read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_accesses")==0) {sys.L2Directory[i].write_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.L2Directory[i].read_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_misses")==0) {sys.L2Directory[i].write_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.L2Directory[i].conflicts=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) {sys.L2Directory[i].duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}

                                        }
                                        w=w+1;
                                }
                                else {
                                        printf("The value of homogeneous_L2Directories or number_of_L2Directories is not correct!");
                                        exit(0);
                                }
                        }
                }

                //__________________________________________Get system.L2[0..n]____________________________________________
                w=OrderofComponents_3layer+1;
                tmpOrderofComponents_3layer=OrderofComponents_3layer;
                if (sys.homogeneous_L2s==1) OrderofComponents_3layer=OrderofComponents_3layer+1;
                else OrderofComponents_3layer=OrderofComponents_3layer+sys.number_of_L2s;

                for (i=0; i<(OrderofComponents_3layer-tmpOrderofComponents_3layer); i++)
                {
                        xNode3=xNode2.getChildNode("component",w);
                        if (xNode3.isEmpty()==1) {
                                printf("The value of homogeneous_L2s or number_of_L2s is not correct!");
                                exit(0);
                        }
                        else
                        {
                                if (strstr(xNode3.getAttribute("name"),"L2")!=NULL)
                                {
                                        { //For L20-L2i
                                                //Get all params with system.L2?
                                                itmp=xNode3.nChildNode("param");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"L2_config")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.L2[i].L2_config[m]=atof(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.L2[i].L2_config[m]=atof(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clockrate")==0) {sys.L2[i].clockrate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"merged_dir")==0) {sys.L2[i].merged_dir=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"ports")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.L2[i].ports[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.L2[i].ports[m]=atoi(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"device_type")==0) {sys.L2[i].device_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"threeD_stack")==0) {strcpy(sys.L2[i].threeD_stack,(xNode3.getChildNode("param",k).getAttribute("value")));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"buffer_sizes")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.L2[i].buffer_sizes[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.L2[i].buffer_sizes[m]=atoi(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                }
                                                //Get all stats with system.L2?
                                                itmp=xNode3.nChildNode("stat");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.L2[i].total_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.L2[i].read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_accesses")==0) {sys.L2[i].write_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_hits")==0) {sys.L2[i].total_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_misses")==0) {sys.L2[i].total_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_hits")==0) {sys.L2[i].read_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_hits")==0) {sys.L2[i].write_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.L2[i].read_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_misses")==0) {sys.L2[i].write_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"replacements")==0) {sys.L2[i].replacements=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_backs")==0) {sys.L2[i].write_backs=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"miss_buffer_accesses")==0) {sys.L2[i].miss_buffer_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fill_buffer_accesses")==0) {sys.L2[i].fill_buffer_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_accesses")==0) {sys.L2[i].prefetch_buffer_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_writes")==0) {sys.L2[i].prefetch_buffer_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_reads")==0) {sys.L2[i].prefetch_buffer_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_hits")==0) {sys.L2[i].prefetch_buffer_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"wbb_writes")==0) {sys.L2[i].wbb_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"wbb_reads")==0) {sys.L2[i].wbb_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.L2[i].conflicts=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) {sys.L2[i].duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}

                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_accesses")==0) {sys.L2[i].homenode_read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_accesses")==0) {sys.L2[i].homenode_read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_hits")==0) {sys.L2[i].homenode_read_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_write_hits")==0) {sys.L2[i].homenode_write_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_misses")==0) {sys.L2[i].homenode_read_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_write_misses")==0) {sys.L2[i].homenode_write_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"dir_duty_cycle")==0) {sys.L2[i].dir_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}

                                                }
                                        }
                                        w=w+1;
                                }
                                else {
                                        printf("The value of homogeneous_L2s or number_of_L2s is not correct!");
                                        exit(0);
                                }
                        }
                }
                //__________________________________________Get system.L3[0..n]____________________________________________
                w=OrderofComponents_3layer+1;
                tmpOrderofComponents_3layer=OrderofComponents_3layer;
                if (sys.homogeneous_L3s==1) OrderofComponents_3layer=OrderofComponents_3layer+1;
                else OrderofComponents_3layer=OrderofComponents_3layer+sys.number_of_L3s;

                for (i=0; i<(OrderofComponents_3layer-tmpOrderofComponents_3layer); i++)
                {
                        xNode3=xNode2.getChildNode("component",w);
                        if (xNode3.isEmpty()==1) {
                                printf("The value of homogeneous_L3s or number_of_L3s is not correct!");
                                exit(0);
                        }
                        else
                        {
                                if (strstr(xNode3.getAttribute("name"),"L3")!=NULL)
                                {
                                        { //For L30-L3i
                                                //Get all params with system.L3?
                                                itmp=xNode3.nChildNode("param");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"L3_config")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.L3[i].L3_config[m]=atof(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.L3[i].L3_config[m]=atof(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clockrate")==0) {sys.L3[i].clockrate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"merged_dir")==0) {sys.L3[i].merged_dir=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"ports")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.L3[i].ports[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.L3[i].ports[m]=atoi(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"device_type")==0) {sys.L3[i].device_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"threeD_stack")==0) {strcpy(sys.L3[i].threeD_stack,(xNode3.getChildNode("param",k).getAttribute("value")));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"buffer_sizes")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.L3[i].buffer_sizes[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.L3[i].buffer_sizes[m]=atoi(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                }
                                                //Get all stats with system.L3?
                                                itmp=xNode3.nChildNode("stat");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) {sys.L3[i].total_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_accesses")==0) {sys.L3[i].read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_accesses")==0) {sys.L3[i].write_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_hits")==0) {sys.L3[i].total_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_misses")==0) {sys.L3[i].total_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_hits")==0) {sys.L3[i].read_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_hits")==0) {sys.L3[i].write_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"read_misses")==0) {sys.L3[i].read_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_misses")==0) {sys.L3[i].write_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"replacements")==0) {sys.L3[i].replacements=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"write_backs")==0) {sys.L3[i].write_backs=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"miss_buffer_accesses")==0) {sys.L3[i].miss_buffer_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"fill_buffer_accesses")==0) {sys.L3[i].fill_buffer_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_accesses")==0) {sys.L3[i].prefetch_buffer_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_writes")==0) {sys.L3[i].prefetch_buffer_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_reads")==0) {sys.L3[i].prefetch_buffer_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"prefetch_buffer_hits")==0) {sys.L3[i].prefetch_buffer_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"wbb_writes")==0) {sys.L3[i].wbb_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"wbb_reads")==0) {sys.L3[i].wbb_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"conflicts")==0) {sys.L3[i].conflicts=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) {sys.L3[i].duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}

                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_accesses")==0) {sys.L3[i].homenode_read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_accesses")==0) {sys.L3[i].homenode_read_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_hits")==0) {sys.L3[i].homenode_read_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_write_hits")==0) {sys.L3[i].homenode_write_hits=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_read_misses")==0) {sys.L3[i].homenode_read_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"homenode_write_misses")==0) {sys.L3[i].homenode_write_misses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"dir_duty_cycle")==0) {sys.L3[i].dir_duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}

                                                }
                                        }
                                        w=w+1;
                                }
                                else {
                                        printf("The value of homogeneous_L3s or number_of_L3s is not correct!");
                                        exit(0);
                                }
                        }
                }
                //__________________________________________Get system.NoC[0..n]____________________________________________
                w=OrderofComponents_3layer+1;
                tmpOrderofComponents_3layer=OrderofComponents_3layer;
                if (sys.homogeneous_NoCs==1) OrderofComponents_3layer=OrderofComponents_3layer+1;
                else OrderofComponents_3layer=OrderofComponents_3layer+sys.number_of_NoCs;

                for (i=0; i<(OrderofComponents_3layer-tmpOrderofComponents_3layer); i++)
                {
                        xNode3=xNode2.getChildNode("component",w);
                        if (xNode3.isEmpty()==1) {
                                printf("The value of homogeneous_NoCs or number_of_NoCs is not correct!");
                                exit(0);
                        }
                        else
                        {
                                if (strstr(xNode3.getAttribute("name"),"noc")!=NULL)
                                {
                                        { //For NoC0-NoCi
                                                //Get all params with system.NoC?
                                                itmp=xNode3.nChildNode("param");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clockrate")==0) {sys.NoC[i].clockrate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"type")==0) {sys.NoC[i].type=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"topology")==0) {strcpy(sys.NoC[i].topology,(xNode3.getChildNode("param",k).getAttribute("value")));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"horizontal_nodes")==0) {sys.NoC[i].horizontal_nodes=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"vertical_nodes")==0) {sys.NoC[i].vertical_nodes=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"has_global_link")==0) {sys.NoC[i].has_global_link=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"link_throughput")==0) {sys.NoC[i].link_throughput=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"link_latency")==0) {sys.NoC[i].link_latency=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"input_ports")==0) {sys.NoC[i].input_ports=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"output_ports")==0) {sys.NoC[i].output_ports=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"virtual_channel_per_port")==0) {sys.NoC[i].virtual_channel_per_port=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"flit_bits")==0) {sys.NoC[i].flit_bits=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"input_buffer_entries_per_vc")==0) {sys.NoC[i].input_buffer_entries_per_vc=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"dual_pump")==0) {sys.NoC[i].dual_pump=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"chip_coverage")==0) {sys.NoC[i].chip_coverage=atof(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"link_routing_over_percentage")==0) {sys.NoC[i].route_over_perc=atof(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"ports_of_input_buffer")==0)
                                                        {
                                                                strtmp.assign(xNode3.getChildNode("param",k).getAttribute("value"));
                                                                m=0;
                                                                for(n=0; n<strtmp.length(); n++)
                                                                {
                                                                        if (strtmp[n]!=',')
                                                                        {
                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                strcat(chtmp1,chtmp);
                                                                        }
                                                                        else{
                                                                                sys.NoC[i].ports_of_input_buffer[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                sys.NoC[i].ports_of_input_buffer[m]=atoi(chtmp1);
                                                                m++;
                                                                chtmp1[0]='\0';
                                                                continue;
                                                        }
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_of_crossbars")==0) {sys.NoC[i].number_of_crossbars=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"crossbar_type")==0) {strcpy(sys.NoC[i].crossbar_type,(xNode3.getChildNode("param",k).getAttribute("value")));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"crosspoint_type")==0) {strcpy(sys.NoC[i].crosspoint_type,(xNode3.getChildNode("param",k).getAttribute("value")));continue;}
                                                        if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"arbiter_type")==0) {sys.NoC[i].arbiter_type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                                }
                                                NumofCom_4=xNode3.nChildNode("component"); //get the number of components within the third layer
                                                for(j=0; j<NumofCom_4; j++)
                                                {
                                                        xNode4=xNode3.getChildNode("component",j);
                                                        if (strcmp(xNode4.getAttribute("name"),"xbar0")==0)
                                                        { //find PBT
                                                                itmp=xNode4.nChildNode("param");
                                                                for(k=0; k<itmp; k++)
                                                                { //get all items of param in system.XoC0.xbar0--xbar0
                                                                        if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"number_of_inputs_of_crossbars")==0) {sys.NoC[i].xbar0.number_of_inputs_of_crossbars=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                        if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"number_of_outputs_of_crossbars")==0) {sys.NoC[i].xbar0.number_of_outputs_of_crossbars=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                        if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"flit_bits")==0) {sys.NoC[i].xbar0.flit_bits=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                        if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"input_buffer_entries_per_port")==0) {sys.NoC[i].xbar0.input_buffer_entries_per_port=atoi(xNode4.getChildNode("param",k).getAttribute("value"));continue;}
                                                                        if (strcmp(xNode4.getChildNode("param",k).getAttribute("name"),"ports_of_input_buffer")==0)
                                                                        {
                                                                                strtmp.assign(xNode4.getChildNode("param",k).getAttribute("value"));
                                                                                m=0;
                                                                                for(n=0; n<strtmp.length(); n++)
                                                                                {
                                                                                        if (strtmp[n]!=',')
                                                                                        {
                                                                                                sprintf(chtmp,"%c",strtmp[n]);
                                                                                                strcat(chtmp1,chtmp);
                                                                                        }
                                                                                        else{
                                                                                                sys.NoC[i].xbar0.ports_of_input_buffer[m]=atoi(chtmp1);
                                                                                                m++;
                                                                                                chtmp1[0]='\0';
                                                                                        }
                                                                                }
                                                                                sys.NoC[i].xbar0.ports_of_input_buffer[m]=atoi(chtmp1);
                                                                                m++;
                                                                                chtmp1[0]='\0';
                                                                        }
                                                                }
                                                                itmp=xNode4.nChildNode("stat");
                                                                for(k=0; k<itmp; k++)
                                                                { //get all items of stat in system.core0.predictor--PBT
                                                                        if (strcmp(xNode4.getChildNode("stat",k).getAttribute("name"),"predictor_accesses")==0) sys.core[i].predictor.predictor_accesses=atof(xNode4.getChildNode("stat",k).getAttribute("value"));
                                                                }
                                                        }
                                                }
                                                //Get all stats with system.NoC?
                                                itmp=xNode3.nChildNode("stat");
                                                for(k=0; k<itmp; k++)
                                                {
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_accesses")==0) sys.NoC[i].total_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));
                                                        if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) sys.NoC[i].duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));
                                                }
                                        }
                                        w=w+1;
                                }
                        }
                }
                //__________________________________________Get system.mem____________________________________________
                if (OrderofComponents_3layer>0) OrderofComponents_3layer=OrderofComponents_3layer+1;
                xNode3=xNode2.getChildNode("component",OrderofComponents_3layer);
                if (xNode3.isEmpty()==1) {
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                if (strstr(xNode3.getAttribute("id"),"system.mem")!=NULL)
                {

                        itmp=xNode3.nChildNode("param");
                        for(k=0; k<itmp; k++)
                        { //get all items of param in system.mem
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"mem_tech_node")==0) {sys.mem.mem_tech_node=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"device_clock")==0) {sys.mem.device_clock=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"peak_transfer_rate")==0) {sys.mem.peak_transfer_rate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"capacity_per_channel")==0) {sys.mem.capacity_per_channel=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_ranks")==0) {sys.mem.number_ranks=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"num_banks_of_DRAM_chip")==0) {sys.mem.num_banks_of_DRAM_chip=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"Block_width_of_DRAM_chip")==0) {sys.mem.Block_width_of_DRAM_chip=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"output_width_of_DRAM_chip")==0) {sys.mem.output_width_of_DRAM_chip=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"page_size_of_DRAM_chip")==0) {sys.mem.page_size_of_DRAM_chip=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"burstlength_of_DRAM_chip")==0) {sys.mem.burstlength_of_DRAM_chip=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"internal_prefetch_of_DRAM_chip")==0) {sys.mem.internal_prefetch_of_DRAM_chip=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                        }
                        itmp=xNode3.nChildNode("stat");
                        for(k=0; k<itmp; k++)
                        { //get all items of stat in system.mem
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_accesses")==0) {sys.mem.memory_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_reads")==0) {sys.mem.memory_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_writes")==0) {sys.mem.memory_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                        }
                }
                else{
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                //__________________________________________Get system.mc____________________________________________
                if (OrderofComponents_3layer>0) OrderofComponents_3layer=OrderofComponents_3layer+1;
                xNode3=xNode2.getChildNode("component",OrderofComponents_3layer);
                if (xNode3.isEmpty()==1) {
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                if (strstr(xNode3.getAttribute("id"),"system.mc")!=NULL)
                {
                        itmp=xNode3.nChildNode("param");
                        for(k=0; k<itmp; k++)
                        { //get all items of param in system.mem
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"mc_clock")==0) {sys.mc.mc_clock=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"block_size")==0) {sys.mc.llc_line_length=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_mcs")==0) {sys.mc.number_mcs=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"memory_channels_per_mc")==0) {sys.mc.memory_channels_per_mc=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"req_window_size_per_channel")==0) {sys.mc.req_window_size_per_channel=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"IO_buffer_size_per_channel")==0) {sys.mc.IO_buffer_size_per_channel=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"databus_width")==0) {sys.mc.databus_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"addressbus_width")==0) {sys.mc.addressbus_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"peak_transfer_rate")==0) {sys.mc.peak_transfer_rate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_ranks")==0) {sys.mc.number_ranks=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"LVDS")==0) {sys.mc.LVDS=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"type")==0) {sys.mc.type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"withPHY")==0) {sys.mc.withPHY=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}

                        }
                        itmp=xNode3.nChildNode("stat");
                        for(k=0; k<itmp; k++)
                        { //get all items of stat in system.mendirectory
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_accesses")==0) {sys.mc.memory_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_reads")==0) {sys.mc.memory_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_writes")==0) {sys.mc.memory_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                        }
                }
                else{
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                //__________________________________________Get system.niu____________________________________________
                if (OrderofComponents_3layer>0) OrderofComponents_3layer=OrderofComponents_3layer+1;
                xNode3=xNode2.getChildNode("component",OrderofComponents_3layer);
                if (xNode3.isEmpty()==1) {
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                if (strstr(xNode3.getAttribute("id"),"system.niu")!=NULL)
                {
                        itmp=xNode3.nChildNode("param");
                        for(k=0; k<itmp; k++)
                        { //get all items of param in system.mem
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clockrate")==0) {sys.niu.clockrate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_units")==0) {sys.niu.number_units=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"type")==0) {sys.niu.type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                        }
                        itmp=xNode3.nChildNode("stat");
                        for(k=0; k<itmp; k++)
                        { //get all items of stat in system.mendirectory
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) {sys.niu.duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_load_perc")==0) {sys.niu.total_load_perc=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                        }
                }
                else{
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }

                //__________________________________________Get system.pcie____________________________________________
                if (OrderofComponents_3layer>0) OrderofComponents_3layer=OrderofComponents_3layer+1;
                xNode3=xNode2.getChildNode("component",OrderofComponents_3layer);
                if (xNode3.isEmpty()==1) {
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                if (strstr(xNode3.getAttribute("id"),"system.pcie")!=NULL)
                {
                        itmp=xNode3.nChildNode("param");
                        for(k=0; k<itmp; k++)
                        { //get all items of param in system.mem
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"clockrate")==0) {sys.pcie.clockrate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_units")==0) {sys.pcie.number_units=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"num_channels")==0) {sys.pcie.num_channels=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"type")==0) {sys.pcie.type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"withPHY")==0) {sys.pcie.withPHY=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}

                        }
                        itmp=xNode3.nChildNode("stat");
                        for(k=0; k<itmp; k++)
                        { //get all items of stat in system.mendirectory
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) {sys.pcie.duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_load_perc")==0) {sys.pcie.total_load_perc=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                        }
                }
                else{
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                //__________________________________________Get system.flashcontroller____________________________________________
                if (OrderofComponents_3layer>0) OrderofComponents_3layer=OrderofComponents_3layer+1;
                xNode3=xNode2.getChildNode("component",OrderofComponents_3layer);
                if (xNode3.isEmpty()==1) {
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }
                if (strstr(xNode3.getAttribute("id"),"system.flashc")!=NULL)
                {
                        itmp=xNode3.nChildNode("param");
                        for(k=0; k<itmp; k++)
                        { //get all items of param in system.mem
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"flashc_clock")==0) {sys.flashc.mc_clock=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"block_size")==0) {sys.flashc.llc_line_length=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_flashcs")==0) {sys.flashc.number_mcs=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"memory_channels_per_flashc")==0) {sys.flashc.memory_channels_per_mc=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"req_window_size_per_channel")==0) {sys.flashc.req_window_size_per_channel=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"IO_buffer_size_per_channel")==0) {sys.flashc.IO_buffer_size_per_channel=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"databus_width")==0) {sys.flashc.databus_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"addressbus_width")==0) {sys.flashc.addressbus_width=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"peak_transfer_rate")==0) {sys.flashc.peak_transfer_rate=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"number_ranks")==0) {sys.flashc.number_ranks=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"LVDS")==0) {sys.flashc.LVDS=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"type")==0) {sys.flashc.type=atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("param",k).getAttribute("name"),"withPHY")==0) {sys.flashc.withPHY=(bool)atoi(xNode3.getChildNode("param",k).getAttribute("value"));continue;}

                        }
                        itmp=xNode3.nChildNode("stat");
                        for(k=0; k<itmp; k++)
                        { //get all items of stat in system.mendirectory
//				if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_accesses")==0) {sys.flashc.memory_accesses=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_reads")==0) {sys.flashc.memory_reads=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
//				if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"memory_writes")==0) {sys.flashc.memory_writes=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"duty_cycle")==0) {sys.flashc.duty_cycle=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}
                                if (strcmp(xNode3.getChildNode("stat",k).getAttribute("name"),"total_load_perc")==0) {sys.flashc.total_load_perc=atof(xNode3.getChildNode("stat",k).getAttribute("value"));continue;}

                        }
                }
                else{
                        printf("some value(s) of number_of_cores/number_of_L2s/number_of_L3s/number_of_NoCs is/are not correct!");
                        exit(0);
                }

        }
}
void ParseXML::initialize() //Initialize all
{
        //All number_of_* at the level of 'system' 03/21/2009
        sys.number_of_cores=1;
        sys.number_of_L1Directories=1;
        sys.number_of_L2Directories=1;
        sys.number_of_L2s=1;
        sys.Private_L2 = false;
        sys.number_of_L3s=1;
        sys.number_of_NoCs=1;
        // All params at the level of 'system'
        //strcpy(sys.homogeneous_cores,"default");
        sys.core_tech_node=1;
        sys.target_core_clockrate=1;
        sys.target_chip_area=1;
        sys.temperature=1;
        sys.number_cache_levels=1;
        sys.homogeneous_cores=1;
        sys.homogeneous_L1Directories=1;
        sys.homogeneous_L2Directories=1;
        sys.homogeneous_L2s=1;
        sys.homogeneous_L3s=1;
        sys.homogeneous_NoCs=1;
        sys.homogeneous_ccs=1;

        sys.Max_area_deviation=1;
        sys.Max_power_deviation=1;
        sys.device_type=1;
        sys.longer_channel_device =true;
        sys.Embedded =false;
        sys.opt_dynamic_power=false;
        sys.opt_lakage_power=false;
        sys.opt_clockrate=true;
        sys.opt_area=false;
        sys.interconnect_projection_type=1;
        int i,j;
        for (i=0; i<=63; i++)
        {
                sys.core[i].clock_rate=1;
                sys.core[i].opt_local = true;
                sys.core[i].x86 = false;
                sys.core[i].machine_bits=1;
                sys.core[i].virtual_address_width=1;
                sys.core[i].physical_address_width=1;
                sys.core[i].opcode_width=1;
                sys.core[i].micro_opcode_width=1;
                //strcpy(sys.core[i].machine_type,"default");
                sys.core[i].internal_datapath_width=1;
                sys.core[i].number_hardware_threads=1;
                sys.core[i].fetch_width=1;
                sys.core[i].number_instruction_fetch_ports=1;
                sys.core[i].decode_width=1;
                sys.core[i].issue_width=1;
                sys.core[i].peak_issue_width=1;
                sys.core[i].commit_width=1;
                for (j=0; j<20; j++) sys.core[i].pipelines_per_core[j]=1;
                for (j=0; j<20; j++) sys.core[i].pipeline_depth[j]=1;
                strcpy(sys.core[i].FPU,"default");
                strcpy(sys.core[i]. divider_multiplier,"default");
                sys.core[i].ALU_per_core=1;
                sys.core[i].FPU_per_core=1.0;
                sys.core[i].MUL_per_core=1;
                sys.core[i].instruction_buffer_size=1;
                sys.core[i].decoded_stream_buffer_size=1;
                //strcpy(sys.core[i].instruction_window_scheme,"default");
                sys.core[i].instruction_window_size=1;
                sys.core[i].ROB_size=1;
                sys.core[i].archi_Regs_IRF_size=1;
                sys.core[i].archi_Regs_FRF_size=1;
                sys.core[i].phy_Regs_IRF_size=1;
                sys.core[i].phy_Regs_FRF_size=1;
                //strcpy(sys.core[i].rename_scheme,"default");
                sys.core[i].register_windows_size=1;
                strcpy(sys.core[i].LSU_order,"default");
                sys.core[i].store_buffer_size=1;
                sys.core[i].load_buffer_size=1;
                sys.core[i].memory_ports=1;
                strcpy(sys.core[i].Dcache_dual_pump,"default");
                sys.core[i].RAS_size=1;
                //all stats at the level of system.core(0-n)
                sys.core[i].total_instructions=1;
                sys.core[i].int_instructions=1;
                sys.core[i].fp_instructions=1;
                sys.core[i].branch_instructions=1;
                sys.core[i].branch_mispredictions=1;
                sys.core[i].committed_instructions=1;
                sys.core[i].load_instructions=1;
                sys.core[i].store_instructions=1;
                sys.core[i].total_cycles=1;
                sys.core[i].idle_cycles=1;
                sys.core[i].busy_cycles=1;
                sys.core[i].instruction_buffer_reads=1;
                sys.core[i].instruction_buffer_write=1;
                sys.core[i].ROB_reads=1;
                sys.core[i].ROB_writes=1;
                sys.core[i].rename_accesses=1;
                sys.core[i].inst_window_reads=1;
                sys.core[i].inst_window_writes=1;
                sys.core[i].inst_window_wakeup_accesses=1;
                sys.core[i].inst_window_selections=1;
                sys.core[i].archi_int_regfile_reads=1;
                sys.core[i].archi_float_regfile_reads=1;
                sys.core[i].phy_int_regfile_reads=1;
                sys.core[i].phy_float_regfile_reads=1;
                sys.core[i].windowed_reg_accesses=1;
                sys.core[i].windowed_reg_transports=1;
                sys.core[i].function_calls=1;
                sys.core[i].ialu_accesses=1;
                sys.core[i].fpu_accesses=1;
                sys.core[i].mul_accesses=1;
                sys.core[i].cdb_alu_accesses=1;
                sys.core[i].cdb_mul_accesses=1;
                sys.core[i].cdb_fpu_accesses=1;
                sys.core[i].load_buffer_reads=1;
                sys.core[i].load_buffer_writes=1;
                sys.core[i].load_buffer_cams=1;
                sys.core[i].store_buffer_reads=1;
                sys.core[i].store_buffer_writes=1;
                sys.core[i].store_buffer_cams=1;
                sys.core[i].store_buffer_forwards=1;
                sys.core[i].main_memory_access=1;
                sys.core[i].main_memory_read=1;
                sys.core[i].main_memory_write=1;
                sys.core[i].IFU_duty_cycle = 1;
                sys.core[i].BR_duty_cycle = 1;
                sys.core[i].LSU_duty_cycle = 1;
                sys.core[i].MemManU_I_duty_cycle =1;
                sys.core[i].MemManU_D_duty_cycle =1;
                sys.core[i].ALU_duty_cycle =1;
                sys.core[i].MUL_duty_cycle =1;
                sys.core[i].FPU_duty_cycle =1;
                sys.core[i].ALU_cdb_duty_cycle =1;
                sys.core[i].MUL_cdb_duty_cycle =1;
                sys.core[i].FPU_cdb_duty_cycle =1;
                //system.core?.predictor
                sys.core[i].predictor.prediction_width=1;
                strcpy(sys.core[i].predictor.prediction_scheme,"default");
                sys.core[i].predictor.predictor_size=1;
                sys.core[i].predictor.predictor_entries=1;
                sys.core[i].predictor.local_predictor_entries=1;
                for (j=0; j<20; j++) sys.core[i].predictor.local_predictor_size[j]=1;
                sys.core[i].predictor.global_predictor_entries=1;
                sys.core[i].predictor.global_predictor_bits=1;
                sys.core[i].predictor.chooser_predictor_entries=1;
                sys.core[i].predictor.chooser_predictor_bits=1;
                sys.core[i].predictor.predictor_accesses=1;
                //system.core?.itlb
                sys.core[i].itlb.number_entries=1;
                sys.core[i].itlb.total_hits=1;
                sys.core[i].itlb.total_accesses=1;
                sys.core[i].itlb.total_misses=1;
                //system.core?.icache
                for (j=0; j<20; j++) sys.core[i].icache.icache_config[j]=1;
                //strcpy(sys.core[i].icache.buffer_sizes,"default");
                sys.core[i].icache.total_accesses=1;
                sys.core[i].icache.read_accesses=1;
                sys.core[i].icache.read_misses=1;
                sys.core[i].icache.replacements=1;
                sys.core[i].icache.read_hits=1;
                sys.core[i].icache.total_hits=1;
                sys.core[i].icache.total_misses=1;
                sys.core[i].icache.miss_buffer_access=1;
                sys.core[i].icache.fill_buffer_accesses=1;
                sys.core[i].icache.prefetch_buffer_accesses=1;
                sys.core[i].icache.prefetch_buffer_writes=1;
                sys.core[i].icache.prefetch_buffer_reads=1;
                sys.core[i].icache.prefetch_buffer_hits=1;
                //system.core?.dtlb
                sys.core[i].dtlb.number_entries=1;
                sys.core[i].dtlb.total_accesses=1;
                sys.core[i].dtlb.read_accesses=1;
                sys.core[i].dtlb.write_accesses=1;
                sys.core[i].dtlb.write_hits=1;
                sys.core[i].dtlb.read_hits=1;
                sys.core[i].dtlb.read_misses=1;
                sys.core[i].dtlb.write_misses=1;
                sys.core[i].dtlb.total_hits=1;
                sys.core[i].dtlb.total_misses=1;
                //system.core?.dcache
                for (j=0; j<20; j++) sys.core[i].dcache.dcache_config[j]=1;
                //strcpy(sys.core[i].dcache.buffer_sizes,"default");
                sys.core[i].dcache.total_accesses=1;
                sys.core[i].dcache.read_accesses=1;
                sys.core[i].dcache.write_accesses=1;
                sys.core[i].dcache.total_hits=1;
                sys.core[i].dcache.total_misses=1;
                sys.core[i].dcache.read_hits=1;
                sys.core[i].dcache.write_hits=1;
                sys.core[i].dcache.read_misses=1;
                sys.core[i].dcache.write_misses=1;
                sys.core[i].dcache.replacements=1;
                sys.core[i].dcache.write_backs=1;
                sys.core[i].dcache.miss_buffer_access=1;
                sys.core[i].dcache.fill_buffer_accesses=1;
                sys.core[i].dcache.prefetch_buffer_accesses=1;
                sys.core[i].dcache.prefetch_buffer_writes=1;
                sys.core[i].dcache.prefetch_buffer_reads=1;
                sys.core[i].dcache.prefetch_buffer_hits=1;
                sys.core[i].dcache.wbb_writes=1;
                sys.core[i].dcache.wbb_reads=1;
                //system.core?.BTB
                for (j=0; j<20; j++) sys.core[i].BTB.BTB_config[j]=1;
                sys.core[i].BTB.total_accesses=1;
                sys.core[i].BTB.read_accesses=1;
                sys.core[i].BTB.write_accesses=1;
                sys.core[i].BTB.total_hits=1;
                sys.core[i].BTB.total_misses=1;
                sys.core[i].BTB.read_hits=1;
                sys.core[i].BTB.write_hits=1;
                sys.core[i].BTB.read_misses=1;
                sys.core[i].BTB.write_misses=1;
                sys.core[i].BTB.replacements=1;
        }

        //system_L1directory
        for (i=0; i<=63; i++)
        {
                for (j=0; j<20; j++) sys.L1Directory[i].Dir_config[j]=1;
                for (j=0; j<20; j++) sys.L1Directory[i].buffer_sizes[j]=1;
                sys.L1Directory[i].clockrate=1;
                sys.L1Directory[i].ports[20]=1;
                sys.L1Directory[i].device_type=1;
                strcpy(sys.L1Directory[i].threeD_stack,"default");
                sys.L1Directory[i].total_accesses=1;
                sys.L1Directory[i].read_accesses=1;
                sys.L1Directory[i].write_accesses=1;
                sys.L1Directory[i].duty_cycle =1;
        }
        //system_L2directory
        for (i=0; i<=63; i++)
        {
                for (j=0; j<20; j++) sys.L2Directory[i].Dir_config[j]=1;
                for (j=0; j<20; j++) sys.L2Directory[i].buffer_sizes[j]=1;
                sys.L2Directory[i].clockrate=1;
                sys.L2Directory[i].ports[20]=1;
                sys.L2Directory[i].device_type=1;
                strcpy(sys.L2Directory[i].threeD_stack,"default");
                sys.L2Directory[i].total_accesses=1;
                sys.L2Directory[i].read_accesses=1;
                sys.L2Directory[i].write_accesses=1;
                sys.L2Directory[i].duty_cycle =1;
        }
        for (i=0; i<=63; i++)
        {
                //system_L2
                for (j=0; j<20; j++) sys.L2[i].L2_config[j]=1;
                sys.L2[i].clockrate=1;
                for (j=0; j<20; j++) sys.L2[i].ports[j]=1;
                sys.L2[i].device_type=1;
                strcpy(sys.L2[i].threeD_stack,"default");
                for (j=0; j<20; j++) sys.L2[i].buffer_sizes[j]=1;
                sys.L2[i].total_accesses=1;
                sys.L2[i].read_accesses=1;
                sys.L2[i].write_accesses=1;
                sys.L2[i].total_hits=1;
                sys.L2[i].total_misses=1;
                sys.L2[i].read_hits=1;
                sys.L2[i].write_hits=1;
                sys.L2[i].read_misses=1;
                sys.L2[i].write_misses=1;
                sys.L2[i].replacements=1;
                sys.L2[i].write_backs=1;
                sys.L2[i].miss_buffer_accesses=1;
                sys.L2[i].fill_buffer_accesses=1;
                sys.L2[i].prefetch_buffer_accesses=1;
                sys.L2[i].prefetch_buffer_writes=1;
                sys.L2[i].prefetch_buffer_reads=1;
                sys.L2[i].prefetch_buffer_hits=1;
                sys.L2[i].wbb_writes=1;
                sys.L2[i].wbb_reads=1;
                sys.L2[i].duty_cycle =1;
                sys.L2[i].merged_dir=false;
                sys.L2[i].homenode_read_accesses =1;
                sys.L2[i].homenode_write_accesses=1;
                sys.L2[i].homenode_read_hits=1;
                sys.L2[i].homenode_write_hits=1;
                sys.L2[i].homenode_read_misses=1;
                sys.L2[i].homenode_write_misses=1;
                sys.L2[i].dir_duty_cycle=1;
        }
        for (i=0; i<=63; i++)
        {
                //system_L3
                for (j=0; j<20; j++) sys.L3[i].L3_config[j]=1;
                sys.L3[i].clockrate=1;
                for (j=0; j<20; j++) sys.L3[i].ports[j]=1;
                sys.L3[i].device_type=1;
                strcpy(sys.L3[i].threeD_stack,"default");
                for (j=0; j<20; j++) sys.L3[i].buffer_sizes[j]=1;
                sys.L3[i].total_accesses=1;
                sys.L3[i].read_accesses=1;
                sys.L3[i].write_accesses=1;
                sys.L3[i].total_hits=1;
                sys.L3[i].total_misses=1;
                sys.L3[i].read_hits=1;
                sys.L3[i].write_hits=1;
                sys.L3[i].read_misses=1;
                sys.L3[i].write_misses=1;
                sys.L3[i].replacements=1;
                sys.L3[i].write_backs=1;
                sys.L3[i].miss_buffer_accesses=1;
                sys.L3[i].fill_buffer_accesses=1;
                sys.L3[i].prefetch_buffer_accesses=1;
                sys.L3[i].prefetch_buffer_writes=1;
                sys.L3[i].prefetch_buffer_reads=1;
                sys.L3[i].prefetch_buffer_hits=1;
                sys.L3[i].wbb_writes=1;
                sys.L3[i].wbb_reads=1;
                sys.L3[i].duty_cycle =1;
                sys.L3[i].merged_dir=false;
                sys.L3[i].homenode_read_accesses =1;
                sys.L3[i].homenode_write_accesses=1;
                sys.L3[i].homenode_read_hits=1;
                sys.L3[i].homenode_write_hits=1;
                sys.L3[i].homenode_read_misses=1;
                sys.L3[i].homenode_write_misses=1;
                sys.L3[i].dir_duty_cycle=1;
        }
        //system_NoC
        for (i=0; i<=63; i++)
        {
                sys.NoC[i].clockrate=1;
                sys.NoC[i].type=true;
                sys.NoC[i].chip_coverage=1;
                sys.NoC[i].has_global_link = true;
                strcpy(sys.NoC[i].topology,"default");
                sys.NoC[i].horizontal_nodes=1;
                sys.NoC[i].vertical_nodes=1;
                sys.NoC[i].input_ports=1;
                sys.NoC[i].output_ports=1;
                sys.NoC[i].virtual_channel_per_port=1;
                sys.NoC[i].flit_bits=1;
                sys.NoC[i].input_buffer_entries_per_vc=1;
                sys.NoC[i].total_accesses=1;
                sys.NoC[i].duty_cycle=1;
                sys.NoC[i].route_over_perc = 0.5;
                for (j=0; j<20; j++) sys.NoC[i].ports_of_input_buffer[j]=1;
                sys.NoC[i].number_of_crossbars=1;
                strcpy(sys.NoC[i].crossbar_type,"default");
                strcpy(sys.NoC[i].crosspoint_type,"default");
                //system.NoC?.xbar0;
                sys.NoC[i].xbar0.number_of_inputs_of_crossbars=1;
                sys.NoC[i].xbar0.number_of_outputs_of_crossbars=1;
                sys.NoC[i].xbar0.flit_bits=1;
                sys.NoC[i].xbar0.input_buffer_entries_per_port=1;
                sys.NoC[i].xbar0.ports_of_input_buffer[20]=1;
                sys.NoC[i].xbar0.crossbar_accesses=1;
        }
        //system_mem
        sys.mem.mem_tech_node=1;
        sys.mem.device_clock=1;
        sys.mem.capacity_per_channel=1;
        sys.mem.number_ranks=1;
        sys.mem.peak_transfer_rate =1;
        sys.mem.num_banks_of_DRAM_chip=1;
        sys.mem.Block_width_of_DRAM_chip=1;
        sys.mem.output_width_of_DRAM_chip=1;
        sys.mem.page_size_of_DRAM_chip=1;
        sys.mem.burstlength_of_DRAM_chip=1;
        sys.mem.internal_prefetch_of_DRAM_chip=1;
        sys.mem.memory_accesses=1;
        sys.mem.memory_reads=1;
        sys.mem.memory_writes=1;
        //system_mc
        sys.mc.mc_clock =1;
        sys.mc.number_mcs=1;
        sys.mc.peak_transfer_rate =1;
        sys.mc.memory_channels_per_mc=1;
        sys.mc.number_ranks=1;
        sys.mc.req_window_size_per_channel=1;
        sys.mc.IO_buffer_size_per_channel=1;
        sys.mc.databus_width=1;
        sys.mc.addressbus_width=1;
        sys.mc.memory_accesses=1;
        sys.mc.memory_reads=1;
        sys.mc.memory_writes=1;
        sys.mc.LVDS=true;
        sys.mc.type=1;
        //system_niu
        sys.niu.clockrate =1;
        sys.niu.number_units=1;
        sys.niu.type = 1;
        sys.niu.duty_cycle =1;
        sys.niu.total_load_perc=1;
        //system_pcie
        sys.pcie.clockrate =1;
        sys.pcie.number_units=1;
        sys.pcie.num_channels=1;
        sys.pcie.type = 1;
        sys.pcie.withPHY = false;
        sys.pcie.duty_cycle =1;
        sys.pcie.total_load_perc=1;
        //system_flash_controller
        sys.flashc.mc_clock =1;
        sys.flashc.number_mcs=1;
        sys.flashc.peak_transfer_rate =1;
        sys.flashc.memory_channels_per_mc=1;
        sys.flashc.number_ranks=1;
        sys.flashc.req_window_size_per_channel=1;
        sys.flashc.IO_buffer_size_per_channel=1;
        sys.flashc.databus_width=1;
        sys.flashc.addressbus_width=1;
        sys.flashc.memory_accesses=1;
        sys.flashc.memory_reads=1;
        sys.flashc.memory_writes=1;
        sys.flashc.LVDS=true;
        sys.flashc.withPHY = false;
        sys.flashc.type =1;
        sys.flashc.duty_cycle =1;
        sys.flashc.total_load_perc=1;
}
