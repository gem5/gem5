/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "basic_circuit.h"
#include "basic_components.h"
#include "common.h"
#include "const.h"
#include "core.h"
#include "io.h"
#include "parameter.h"

int RegFU::RFWIN_ACCESS_MULTIPLIER = 16;

// The five bits are: busy, Issued, Finished, speculative, valid
int SchedulerU::ROB_STATUS_BITS = 5;

InstFetchU::InstFetchU(XMLNode* _xml_data, InputParameter* interface_ip_,
                       const CoreParameters & _core_params,
                       const CoreStatistics & _core_stats, bool exist_)
    : McPATComponent(_xml_data), icache(NULL), IB(NULL), BTB(NULL),
      BPT(NULL), ID_inst(NULL), ID_operand(NULL), ID_misc(NULL),
      interface_ip(*interface_ip_),
      core_params(_core_params), core_stats(_core_stats), exist(exist_) {
    if (!exist) return;
    int idx, tag, data, size, line, assoc, banks;
    bool is_default = true;

    clockRate = core_params.clockRate;
    name = "Instruction Fetch Unit";
    // Check if there is an icache child:
    int i;
    icache = NULL;
    for( i = 0; i < xml_data->nChildNode("component"); i++ ) {
        XMLNode* childXML = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = childXML->getAttribute("type");

        if (!type)
            warnMissingComponentType(childXML->getAttribute("id"));

        STRCMP(type, "CacheUnit") {
            XMLCSTR name = childXML->getAttribute("name");
            if (strcmp(name, "Instruction Cache") == 0 ||
                strcmp(name, "icache") == 0) {
                icache = new CacheUnit(childXML, &interface_ip);
                children.push_back(icache);
            }
        }
    }

    set_params_stats();

    //Instruction buffer
    data = core_params.instruction_length * core_params.peak_issueW;
    line = int(ceil(data / BITS_PER_BYTE));
    size = core_params.num_hthreads * core_params.instruction_buffer_size *
        line;
    if (size < MIN_BUFFER_SIZE) {
        size = MIN_BUFFER_SIZE;
    }

    interface_ip.cache_sz = size;
    interface_ip.line_sz = line;
    interface_ip.assoc = core_params.instruction_buffer_assoc;
    interface_ip.nbanks = core_params.instruction_buffer_nbanks;
    interface_ip.out_w = line * BITS_PER_BYTE;
    interface_ip.specific_tag = core_params.instruction_buffer_tag_width > 0;
    interface_ip.tag_w = core_params.instruction_buffer_tag_width;
    interface_ip.access_mode = Normal;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports =
        core_params.number_instruction_fetch_ports;
    interface_ip.num_rd_ports = 0;
    interface_ip.num_wr_ports = 0;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.is_cache = false;
    interface_ip.pure_ram = true;
    interface_ip.pure_cam = false;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;

    IB = new ArrayST(xml_data, &interface_ip, "Instruction Buffer",
                     Core_device, clockRate, core_params.opt_local,
                     core_params.core_ty);
    IB->area.set_area(IB->area.get_area() + IB->local_result.area);
    area.set_area(area.get_area() + IB->local_result.area);

    if (core_params.predictionW > 0) {
        /*
         * BTB branch target buffer, accessed during IF stage. Virtually indexed and virtually tagged
         * It is only a cache without all the buffers in the cache controller since it is more like a
         * look up table than a cache with cache controller. When access miss, no load from other places
         * such as main memory (not actively fill the misses), it is passively updated under two circumstances:
         * 1)  when BPT@ID stage finds out current is a taken branch while BTB missed
         * 2)  When BPT@ID stage predicts differently than BTB
         * 3)  When ID stage finds out current instruction is not a branch while BTB had a hit.(mark as invalid)
         * 4)  when EXEU find out wrong target has been provided from BTB.
         *
         */
        size = inst_fetch_params.btb_size;
        line = inst_fetch_params.btb_block_size;
        assoc = inst_fetch_params.btb_assoc;
        banks = inst_fetch_params.btb_num_banks;
        idx = int(ceil(log2(size / line / assoc)));
        tag = virtual_address_width + int(ceil(log2(core_params.num_hthreads)))
            + EXTRA_TAG_BITS;

        interface_ip.cache_sz = size;
        interface_ip.line_sz = line;
        interface_ip.assoc = assoc;
        interface_ip.nbanks = banks;
        interface_ip.out_w = line * BITS_PER_BYTE;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = Normal;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1;
        interface_ip.num_rd_ports = core_params.predictionW;
        interface_ip.num_wr_ports = core_params.predictionW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = 0;
        interface_ip.is_cache = true;
        interface_ip.pure_ram = false;
        interface_ip.pure_cam = false;
        interface_ip.throughput = inst_fetch_params.btb_throughput / clockRate;
        interface_ip.latency = inst_fetch_params.btb_latency / clockRate;

        BTB = new ArrayST(xml_data, &interface_ip, "Branch Target Buffer",
                          Core_device, clockRate, core_params.opt_local,
                          core_params.core_ty);
        area.set_area(area.get_area() + BTB->local_result.area);

        BPT = new BranchPredictor(xml_data, &interface_ip,
                                  core_params, core_stats);
        area.set_area(area.get_area() + BPT->area.get_area());
    }

    ID_inst = new InstructionDecoder(xml_data, "Instruction Opcode Decoder",
                                     is_default, &interface_ip,
                                     core_params.opcode_width,
                                     core_params.decodeW,
                                     core_params.x86, clockRate,
                                     Core_device, core_params.core_ty);

    ID_operand = new InstructionDecoder(xml_data,
                                        "Instruction Operand Decoder",
                                        is_default, &interface_ip,
                                        core_params.arch_ireg_width,
                                        core_params.decodeW,
                                        core_params.x86, clockRate,
                                        Core_device, core_params.core_ty);

    ID_misc = new InstructionDecoder(xml_data, "Instruction Microcode Decoder",
                                     is_default, &interface_ip,
                                     core_params.micro_opcode_length,
                                     core_params.decodeW,
                                     core_params.x86, clockRate,
                                     Core_device, core_params.core_ty);
    area.set_area(area.get_area()+ (ID_inst->area.get_area()
                                    + ID_operand->area.get_area()
                                    + ID_misc->area.get_area())
                  * core_params.decodeW);
}

void
InstFetchU::set_params_stats() {
    int num_children = xml_data->nChildNode("component");
    int i;
    memset(&inst_fetch_params,0,sizeof(InstFetchParameters));
    for (i = 0; i < num_children; i++) {
        XMLNode* child = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = child->getAttribute("type");

        if (!type)
            warnMissingComponentType(child->getAttribute("id"));

        STRCMP(type, "BranchTargetBuffer") {
            int sub_num_children = child->nChildNode("param");
            int j;
            for (j = 0; j < sub_num_children; j++) {
                XMLNode* paramNode = child->getChildNodePtr("param", &j);
                XMLCSTR node_name = paramNode->getAttribute("name");
                XMLCSTR value = paramNode->getAttribute("value");

                if (!node_name)
                    warnMissingParamName(paramNode->getAttribute("id"));

                ASSIGN_INT_IF("size", inst_fetch_params.btb_size);
                ASSIGN_INT_IF("block_size", inst_fetch_params.btb_block_size);
                ASSIGN_INT_IF("assoc", inst_fetch_params.btb_assoc);
                ASSIGN_INT_IF("num_banks", inst_fetch_params.btb_num_banks);
                ASSIGN_INT_IF("latency", inst_fetch_params.btb_latency);
                ASSIGN_INT_IF("throughput", inst_fetch_params.btb_throughput);
                ASSIGN_INT_IF("rw_ports", inst_fetch_params.btb_rw_ports);

                else {
                    warnUnrecognizedParam(node_name);
                }
            }

            sub_num_children = child->nChildNode("stat");
            for (j = 0; j < sub_num_children; j++) {
                XMLNode* statNode = child->getChildNodePtr("stat", &j);
                XMLCSTR node_name = statNode->getAttribute("name");
                XMLCSTR value = statNode->getAttribute("value");

                if (!node_name)
                    warnMissingStatName(statNode->getAttribute("id"));

                ASSIGN_FP_IF("read_accesses",
                             inst_fetch_stats.btb_read_accesses);
                ASSIGN_FP_IF("write_accesses",
                             inst_fetch_stats.btb_write_accesses);
                else {
                    warnUnrecognizedStat(node_name);
                }
            }
        }
    }

    // Parameter sanity check
    if (inst_fetch_params.btb_size <= 0) {
        errorNonPositiveParam("size");
    }

    if (inst_fetch_params.btb_block_size <= 0) {
        errorNonPositiveParam("block_size");
    }

    if (inst_fetch_params.btb_assoc <= 0) {
        errorNonPositiveParam("assoc");
    }

    if (inst_fetch_params.btb_num_banks <= 0) {
        errorNonPositiveParam("num_banks");
    }
}

BranchPredictor::BranchPredictor(XMLNode* _xml_data,
                                 InputParameter* interface_ip_,
                                 const CoreParameters & _core_params,
                                 const CoreStatistics & _core_stats,
                                 bool exist_)
    : McPATComponent(_xml_data), globalBPT(NULL), localBPT(NULL),
      L1_localBPT(NULL), L2_localBPT(NULL), chooser(NULL), RAS(NULL),
      interface_ip(*interface_ip_),
      core_params(_core_params), core_stats(_core_stats), exist(exist_) {
    if (!exist) return;
    int tag;
    int data;
    int size;

    clockRate = core_params.clockRate;
    name = "Branch Predictor";

    // Common interface parameters for the branch predictor structures
    interface_ip.pure_cam = false;

    if (core_params.multithreaded) {
        tag = int(log2(core_params.num_hthreads) + EXTRA_TAG_BITS);
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;
        interface_ip.is_cache = true;
        interface_ip.pure_ram = false;
    } else {
        interface_ip.specific_tag = 0;
        interface_ip.tag_w = 0;
        interface_ip.is_cache = false;
        interface_ip.pure_ram = true;
    }

    // Parse params and stats from XML
    set_params_stats();

    // Common interface parameters for the branch predictor structures
    interface_ip.assoc = branch_pred_params.assoc;
    interface_ip.nbanks = branch_pred_params.nbanks;

    //Global predictor
    data = int(ceil(branch_pred_params.global_predictor_bits / BITS_PER_BYTE));
    size = data * branch_pred_params.global_predictor_entries;

    interface_ip.cache_sz = size;
    interface_ip.line_sz = data;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.access_mode = Fast;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.predictionW;
    interface_ip.num_wr_ports = core_params.predictionW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    globalBPT = new ArrayST(xml_data, &interface_ip, "Global Predictor",
                            Core_device, clockRate, core_params.opt_local,
                            core_params.core_ty);
    area.set_area(area.get_area() + globalBPT->local_result.area);

    //Local BPT (Level 1)
    data = int(ceil(branch_pred_params.local_l1_predictor_size /
                    BITS_PER_BYTE));
    size = data * branch_pred_params.local_predictor_entries;

    interface_ip.cache_sz = size;
    interface_ip.line_sz = data;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.access_mode = Fast;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.predictionW;
    interface_ip.num_wr_ports = core_params.predictionW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    L1_localBPT = new ArrayST(xml_data, &interface_ip,
                              "Local Predictor, Level 1",
                              Core_device, clockRate, core_params.opt_local,
                              core_params.core_ty);
    L1_localBPT->area.set_area(L1_localBPT->area.get_area() +
                               L1_localBPT->local_result.area);
    area.set_area(area.get_area()+ L1_localBPT->local_result.area);

    //Local BPT (Level 2)
    data = int(ceil(branch_pred_params.local_l2_predictor_size /
                    BITS_PER_BYTE));
    size = data * branch_pred_params.local_predictor_entries;

    interface_ip.cache_sz = size;
    interface_ip.line_sz = data;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.access_mode = Fast;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.predictionW;
    interface_ip.num_wr_ports = core_params.predictionW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    L2_localBPT = new ArrayST(xml_data, &interface_ip,
                              "Local Predictor, Level 2",
                              Core_device, clockRate, core_params.opt_local,
                              core_params.core_ty);
    area.set_area(area.get_area() + L2_localBPT->local_result.area);

    //Chooser
    data = int(ceil(branch_pred_params.chooser_predictor_bits /
                    BITS_PER_BYTE));
    size = data * branch_pred_params.chooser_predictor_entries;

    interface_ip.cache_sz = size;
    interface_ip.line_sz = data;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.access_mode = Fast;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.predictionW;
    interface_ip.num_wr_ports = core_params.predictionW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    chooser = new ArrayST(xml_data, &interface_ip, "Predictor Chooser",
                          Core_device, clockRate, core_params.opt_local,
                          core_params.core_ty);
    area.set_area(area.get_area() + chooser->local_result.area);

    //RAS return address stacks are Duplicated for each thread.
    data = int(ceil(core_params.pc_width / BITS_PER_BYTE));
    size = data * core_params.RAS_size;

    interface_ip.cache_sz = size;
    interface_ip.line_sz = data;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.access_mode = Fast;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.predictionW;
    interface_ip.num_wr_ports = core_params.predictionW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.is_cache = false;
    interface_ip.pure_ram = true;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    RAS = new ArrayST(xml_data, &interface_ip, "RAS", Core_device, clockRate,
                      core_params.opt_local, core_params.core_ty);
    RAS->output_data.area *= core_params.num_hthreads;
    area.set_area(area.get_area() + RAS->local_result.area *
                  core_params.num_hthreads);

}

void
BranchPredictor::set_params_stats() {
    int num_children = xml_data->nChildNode("component");
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* child = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = child->getAttribute("type");

        if (!type)
            warnMissingComponentType(child->getAttribute("id"));

        STRCMP(type, "BranchPredictor") {
            int sub_num_children = child->nChildNode("param");
            int j;
            for (j = 0; j < sub_num_children; j++) {
                XMLNode* paramNode = child->getChildNodePtr("param", &j);
                XMLCSTR node_name = paramNode->getAttribute("name");
                XMLCSTR value = paramNode->getAttribute("value");

                if (!node_name)
                    warnMissingParamName(paramNode->getAttribute("id"));

                ASSIGN_INT_IF("assoc", branch_pred_params.assoc);
                ASSIGN_INT_IF("nbanks", branch_pred_params.nbanks);
                ASSIGN_INT_IF("local_l1_predictor_size",
                              branch_pred_params.local_l1_predictor_size);
                ASSIGN_INT_IF("local_l2_predictor_size",
                              branch_pred_params.local_l2_predictor_size);
                ASSIGN_INT_IF("local_predictor_entries",
                              branch_pred_params.local_predictor_entries);
                ASSIGN_INT_IF("global_predictor_entries",
                              branch_pred_params.global_predictor_entries);
                ASSIGN_INT_IF("global_predictor_bits",
                              branch_pred_params.global_predictor_bits);
                ASSIGN_INT_IF("chooser_predictor_entries",
                              branch_pred_params.chooser_predictor_entries);
                ASSIGN_INT_IF("chooser_predictor_bits",
                              branch_pred_params.chooser_predictor_bits);

                else {
                    warnUnrecognizedParam(node_name);
                }
            }
            // The core reads in the number of branches and the number of
            // function calls and these values are passed through the
            // core_stats variable, so we don't need to read them in here
        }
    }
}

SchedulerU::SchedulerU(XMLNode* _xml_data, InputParameter* interface_ip_,
                       const CoreParameters & _core_params,
                       const CoreStatistics & _core_stats, bool exist_)
    : McPATComponent(_xml_data), int_inst_window(NULL),
      fp_inst_window(NULL), ROB(NULL), int_instruction_selection(NULL),
      fp_instruction_selection(NULL),
      interface_ip(*interface_ip_),
      core_params(_core_params), core_stats(_core_stats), exist(exist_) {
    if (!exist) return;
    int tag;
    int data;
    int size;
    int line;
    bool is_default = true;
    string tmp_name;

    clockRate = core_params.clockRate;
    name = "Instruction Scheduler";
    if ((core_params.core_ty == Inorder && core_params.multithreaded)) {
        //Instruction issue queue, in-order multi-issue or multithreaded
        //processor also has this structure. Unified window for Inorder
        //processors
        //This tag width is the normal thread state bits based on
        //Niagara Design
        tag = int(log2(core_params.num_hthreads) * core_params.perThreadState);
        data = core_params.instruction_length;
        line = int(ceil(data / BITS_PER_BYTE));
        size = core_params.instruction_window_size * line;
        if (size < MIN_BUFFER_SIZE) {
            size = MIN_BUFFER_SIZE;
        }

        //NOTE: x86 inst can be very lengthy, up to 15B.
        //Source: Intel® 64 and IA-32 Architectures
        //Software Developer’s Manual
        interface_ip.cache_sz = size;
        interface_ip.line_sz = line;
        interface_ip.assoc = core_params.scheduler_assoc;
        interface_ip.nbanks = core_params.scheduler_nbanks;
        interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = Sequential;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 0;
        interface_ip.num_rd_ports = core_params.peak_issueW;
        interface_ip.num_wr_ports = core_params.peak_issueW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = core_params.peak_issueW;
        interface_ip.is_cache = true;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = false;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        int_inst_window = new ArrayST(xml_data, &interface_ip,
                                      "InstFetchQueue", Core_device, clockRate,
                                      core_params.opt_local,
                                      core_params.core_ty);
        int_inst_window->output_data.area *= core_params.num_pipelines;
        area.set_area(area.get_area() + int_inst_window->local_result.area *
                      core_params.num_pipelines);
        Iw_height = int_inst_window->local_result.cache_ht;

        /*
         * selection logic
         * In a single-issue Inorder multithreaded processor like Niagara, issue width=1*number_of_threads since the processor does need to pick up
         * instructions from multiple ready ones(although these ready ones are from different threads).While SMT processors do not distinguish which thread belongs to who
         * at the issue stage.
         */

        int_instruction_selection =
            new selection_logic(xml_data, is_default,
                                core_params.instruction_window_size,
                                core_params.peak_issueW *
                                core_params.num_hthreads,
                                &interface_ip,
                                "Int Instruction Selection Logic",
                                core_stats.inst_window_wakeup_accesses,
                                clockRate, Core_device, core_params.core_ty);

        if (core_params.fp_instruction_window_size > 0) {
            fp_instruction_selection =
                new selection_logic(xml_data, is_default,
                                    core_params.fp_instruction_window_size,
                                    core_params.fp_issueW *
                                    core_params.num_hthreads,
                                    &interface_ip,
                                    "FP Instruction Selection Logic",
                                    core_stats.fp_inst_window_wakeup_accesses,
                                    clockRate, Core_device,
                                    core_params.core_ty);
        }
    }

    if (core_params.core_ty == OOO) {
        /*
         * CAM based instruction window
         * For physicalRegFilebased OOO it is the instruction issue queue, where only tags of phy regs are stored
         * For RS based OOO it is the Reservation station, where both tags and values of phy regs are stored
         * It is written once and read twice(two operands) before an instruction can be issued.
         * X86 instruction can be very long up to 15B. add instruction length in XML
         */
        if (core_params.scheu_ty == PhysicalRegFile) {
            tag = core_params.phy_ireg_width;
            data = int((ceil((core_params.instruction_length +
                              NUM_SOURCE_OPERANDS *
                              (core_params.phy_ireg_width -
                               core_params.arch_ireg_width)) /
                             (double)NUM_SOURCE_OPERANDS) /
                        BITS_PER_BYTE));
            tmp_name = "Integer Instruction Window";
        } else {
            tag = core_params.phy_ireg_width;
            data = int(ceil(((core_params.instruction_length +
                              NUM_SOURCE_OPERANDS *
                              (core_params.phy_ireg_width -
                               core_params.arch_ireg_width) +
                               2 * core_params.int_data_width) /
                                (double)NUM_SOURCE_OPERANDS) /
                            BITS_PER_BYTE));
            tmp_name = "Integer Reservation Station";
        }

        size = data * core_params.instruction_window_size;

        interface_ip.cache_sz = size;
        interface_ip.line_sz = data;
        interface_ip.assoc = core_params.scheduler_assoc;
        interface_ip.nbanks = core_params.scheduler_nbanks;
        interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = Normal;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 0;
        interface_ip.num_rd_ports = core_params.peak_issueW;
        interface_ip.num_wr_ports = core_params.peak_issueW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = core_params.peak_issueW;
        interface_ip.is_cache = true;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = false;
        interface_ip.throughput = NUM_SOURCE_OPERANDS * 1.0 / clockRate;
        interface_ip.latency = NUM_SOURCE_OPERANDS * 1.0 / clockRate;
        int_inst_window = new ArrayST(xml_data, &interface_ip, tmp_name,
                                      Core_device, clockRate,
                                      core_params.opt_local,
                                      core_params.core_ty);
        int_inst_window->output_data.area *= core_params.num_pipelines;
        area.set_area(area.get_area() + int_inst_window->local_result.area *
                      core_params.num_pipelines);
        Iw_height = int_inst_window->local_result.cache_ht;

        //FU inst window
        if (core_params.scheu_ty == PhysicalRegFile) {
            tag = NUM_SOURCE_OPERANDS * core_params.phy_freg_width;
            data = int(ceil((core_params.instruction_length +
                             NUM_SOURCE_OPERANDS *
                             (core_params.phy_freg_width -
                              core_params.arch_freg_width)) / BITS_PER_BYTE));
            tmp_name = "FP Instruction Window";
        } else {
            tag = NUM_SOURCE_OPERANDS * core_params.phy_ireg_width;
            data = int(ceil((core_params.instruction_length +
                             NUM_SOURCE_OPERANDS *
                             (core_params.phy_freg_width -
                              core_params.arch_freg_width) +
                             NUM_SOURCE_OPERANDS * core_params.fp_data_width) /
                            BITS_PER_BYTE));
            tmp_name = "FP Reservation Station";
        }

        size = data * core_params.fp_instruction_window_size;

        interface_ip.cache_sz = size;
        interface_ip.line_sz = data;
        interface_ip.assoc = core_params.scheduler_assoc;
        interface_ip.nbanks = core_params.scheduler_nbanks;
        interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = Normal;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 0;
        interface_ip.num_rd_ports = core_params.fp_issueW;
        interface_ip.num_wr_ports = core_params.fp_issueW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = core_params.fp_issueW;
        interface_ip.is_cache = true;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = false;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        fp_inst_window =
            new ArrayST(xml_data, &interface_ip, tmp_name, Core_device,
                        clockRate, core_params.opt_local, core_params.core_ty);
        fp_inst_window->output_data.area *= core_params.num_fp_pipelines;
        area.set_area(area.get_area() + fp_inst_window->local_result.area
                      *core_params.num_fp_pipelines);
        fp_Iw_height = fp_inst_window->local_result.cache_ht;

        if (core_params.ROB_size > 0) {
            /*
             *  if ROB_size = 0, then the target processor does not support hardware-based
             *  speculation, i.e. , the processor allow OOO issue as well as OOO completion, which
             *  means branch must be resolved before instruction issued into instruction window, since
             *  there is no change to flush miss-predict branch path after instructions are issued in this situation.
             *
             *  ROB.ROB size = inflight inst. ROB is unified for int and fp inst.
             *  One old approach is to combine the RAT and ROB as a huge CAM structure as in AMD K7.
             *  However, this approach is abandoned due to its high power and poor scalablility.
                         *      McPAT uses current implementation of ROB as circular buffer.
                         *      ROB is written once when instruction is issued and read once when the instruction is committed.         *
             */
            int robExtra = int(ceil(ROB_STATUS_BITS +
                                    log2(core_params.num_hthreads)));

            if (core_params.scheu_ty == PhysicalRegFile) {
                //PC is to id the instruction for recover exception.
                //inst is used to map the renamed dest. registers. so that
                //commit stage can know which reg/RRAT to update
                data = int(ceil((robExtra + core_params.pc_width +
                                 core_params.phy_ireg_width) / BITS_PER_BYTE));
            } else {
                //in RS based OOO, ROB also contains value of destination reg
                data  = int(ceil((robExtra + core_params.pc_width +
                                  core_params.phy_ireg_width +
                                  core_params.fp_data_width) / BITS_PER_BYTE));
            }

            interface_ip.cache_sz = data * core_params.ROB_size;
            interface_ip.line_sz = data;
            interface_ip.assoc = core_params.ROB_assoc;
            interface_ip.nbanks = core_params.ROB_nbanks;
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
            interface_ip.specific_tag = core_params.ROB_tag_width > 0;
            interface_ip.tag_w = core_params.ROB_tag_width;
            interface_ip.access_mode = Sequential;
            interface_ip.obj_func_dyn_energy = 0;
            interface_ip.obj_func_dyn_power = 0;
            interface_ip.obj_func_leak_power = 0;
            interface_ip.obj_func_cycle_t = 1;
            interface_ip.num_rw_ports = 0;
            interface_ip.num_rd_ports = core_params.peak_commitW;
            interface_ip.num_wr_ports = core_params.peak_issueW;
            interface_ip.num_se_rd_ports = 0;
            interface_ip.num_search_ports    = 0;
            interface_ip.is_cache = false;
            interface_ip.pure_cam = false;
            interface_ip.pure_ram = true;
            interface_ip.throughput = 1.0 / clockRate;
            interface_ip.latency = 1.0 / clockRate;
            ROB = new ArrayST(xml_data, &interface_ip, "Reorder Buffer",
                              Core_device, clockRate, core_params.opt_local,
                              core_params.core_ty);
            ROB->output_data.area *= core_params.num_pipelines;
            area.set_area(area.get_area() + ROB->local_result.area *
                          core_params.num_pipelines);
            ROB_height = ROB->local_result.cache_ht;
        }

        int_instruction_selection =
            new selection_logic(xml_data, is_default,
                                core_params.instruction_window_size,
                                core_params.peak_issueW, &interface_ip,
                                "Int Instruction Selection Logic",
                                core_stats.inst_window_wakeup_accesses,
                                clockRate, Core_device, core_params.core_ty);

        if (core_params.fp_instruction_window_size > 0) {
            fp_instruction_selection =
                new selection_logic(xml_data, is_default,
                                    core_params.fp_instruction_window_size,
                                    core_params.fp_issueW, &interface_ip,
                                    "FP Instruction Selection Logic",
                                    core_stats.fp_inst_window_wakeup_accesses,
                                    clockRate, Core_device,
                                    core_params.core_ty);
        }

    }
}

LoadStoreU::LoadStoreU(XMLNode* _xml_data, InputParameter* interface_ip_,
                       const CoreParameters & _core_params,
                       const CoreStatistics & _core_stats, bool exist_)
    : McPATComponent(_xml_data), dcache(NULL), LSQ(NULL), LoadQ(NULL),
      interface_ip(*interface_ip_),
      core_params(_core_params), core_stats(_core_stats), exist(exist_) {
    if (!exist) return;
    int  tag;
    int line;
    int size;
    int ldst_opcode = core_params.opcode_width;

    clockRate = core_params.clockRate;
    name = "Load/Store Unit";

    // Check if there is a dcache child:
    int i;
    dcache = NULL;
    for( i = 0; i < xml_data->nChildNode("component"); i++ ) {
        XMLNode* childXML = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = childXML->getAttribute("type");

        if (!type)
            warnMissingComponentType(childXML->getAttribute("id"));

        STRCMP(type, "CacheUnit") {
            XMLCSTR name = childXML->getAttribute("name");
            if (strcmp(name, "Data Cache") == 0 ||
                strcmp(name, "dcache") == 0) {
                dcache = new CacheUnit(childXML, &interface_ip);
                children.push_back(dcache);
            }
        }
    }

    /*
     * LSU--in-order processors do not have separate load queue: unified lsq
     * partitioned among threads
     * it is actually the store queue but for inorder processors it serves as both loadQ and StoreQ
     */
    tag = ldst_opcode + virtual_address_width +
        int(ceil(log2(core_params.num_hthreads))) + EXTRA_TAG_BITS;
    line = int(ceil(data_path_width / BITS_PER_BYTE));
    size = core_params.store_buffer_size * line * core_params.num_hthreads;

    interface_ip.cache_sz = size;
    interface_ip.line_sz = line;
    interface_ip.assoc = core_params.store_buffer_assoc;
    interface_ip.nbanks = core_params.store_buffer_nbanks;
    interface_ip.out_w = line * BITS_PER_BYTE;
    interface_ip.specific_tag = tag > 0;
    interface_ip.tag_w = tag;
    interface_ip.access_mode = Sequential;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.memory_ports;
    interface_ip.num_wr_ports = core_params.memory_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = core_params.memory_ports;
    interface_ip.is_cache = true;
    interface_ip.pure_ram = false;
    interface_ip.pure_cam = false;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    LSQ = new ArrayST(xml_data, &interface_ip, "Store Queue", Core_device,
                      clockRate, core_params.opt_local, core_params.core_ty);
    area.set_area(area.get_area() + LSQ->local_result.area);
    area.set_area(area.get_area()*cdb_overhead);
    lsq_height = LSQ->local_result.cache_ht * sqrt(cdb_overhead);

    if ((core_params.core_ty == OOO) && (core_params.load_buffer_size > 0)) {
        tag = ldst_opcode + virtual_address_width +
            int(ceil(log2(core_params.num_hthreads))) + EXTRA_TAG_BITS;
        line = int(ceil(data_path_width / BITS_PER_BYTE));
        size = core_params.load_buffer_size * line * core_params.num_hthreads;

        interface_ip.cache_sz = size;
        interface_ip.line_sz = line;
        interface_ip.assoc = core_params.load_buffer_assoc;
        interface_ip.nbanks = core_params.load_buffer_nbanks;
        interface_ip.out_w = line * BITS_PER_BYTE;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = Sequential;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 0;
        interface_ip.num_rd_ports = core_params.memory_ports;
        interface_ip.num_wr_ports = core_params.memory_ports;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = core_params.memory_ports;
        interface_ip.is_cache = true;
        interface_ip.pure_ram = false;
        interface_ip.pure_cam = false;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        LoadQ = new ArrayST(xml_data, &interface_ip, "Load Queue", Core_device,
                            clockRate, core_params.opt_local,
                            core_params.core_ty);
        LoadQ->area.set_area(LoadQ->area.get_area() +
                             LoadQ->local_result.area);
        area.set_area(area.get_area()*cdb_overhead);
        lsq_height = (LSQ->local_result.cache_ht +
                      LoadQ->local_result.cache_ht) * sqrt(cdb_overhead);
    }

}

MemManU::MemManU(XMLNode* _xml_data, InputParameter* interface_ip_,
                 const CoreParameters & _core_params,
                 const CoreStatistics & _core_stats, bool exist_)
    : McPATComponent(_xml_data), itlb(NULL), dtlb(NULL),
      interface_ip(*interface_ip_),
      core_params(_core_params), core_stats(_core_stats), exist(exist_) {
    if (!exist) return;
    int tag;
    int data;
    int line;

    clockRate = core_params.clockRate;
    name = "Memory Management Unit";

    set_params_stats();

    // These are shared between ITLB and DTLB
    interface_ip.is_cache            = true;
    interface_ip.pure_cam            = false;
    interface_ip.pure_ram            = false;
    //Itlb TLBs are partioned among threads according to Nigara and Nehalem
    tag = virtual_address_width - int(floor(log2(virtual_memory_page_size))) +
        int(ceil(log2(core_params.num_hthreads))) + EXTRA_TAG_BITS;
    data = physical_address_width - int(floor(log2(virtual_memory_page_size)));
    line = int(ceil(data / BITS_PER_BYTE));

    interface_ip.cache_sz = mem_man_params.itlb_number_entries * line;
    interface_ip.line_sz = line;
    interface_ip.assoc = mem_man_params.itlb_assoc;
    interface_ip.nbanks = mem_man_params.itlb_nbanks;
    interface_ip.out_w = line * BITS_PER_BYTE;
    interface_ip.specific_tag = tag > 0;
    interface_ip.tag_w = tag;
    interface_ip.access_mode = Normal;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = core_params.number_instruction_fetch_ports;
    interface_ip.num_rd_ports = 0;
    interface_ip.num_wr_ports = 0;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = core_params.number_instruction_fetch_ports;
    interface_ip.throughput = mem_man_params.itlb_throughput / clockRate;
    interface_ip.latency = mem_man_params.itlb_latency / clockRate;
    itlb = new ArrayST(xml_data, &interface_ip, "Instruction TLB", Core_device,
                       clockRate, core_params.opt_local, core_params.core_ty);
    area.set_area(area.get_area() + itlb->local_result.area);

    //dtlb
    tag = virtual_address_width - int(floor(log2(virtual_memory_page_size))) +
        int(ceil(log2(core_params.num_hthreads))) + EXTRA_TAG_BITS;
    data = physical_address_width - int(floor(log2(virtual_memory_page_size)));
    line = int(ceil(data / BITS_PER_BYTE));

    interface_ip.cache_sz = mem_man_params.dtlb_number_entries * line;
    interface_ip.line_sz = line;
    interface_ip.assoc = mem_man_params.dtlb_assoc;
    interface_ip.nbanks = mem_man_params.dtlb_nbanks;
    interface_ip.out_w = line * BITS_PER_BYTE;
    interface_ip.specific_tag = tag > 0;
    interface_ip.tag_w = tag;
    interface_ip.access_mode = Normal;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.memory_ports;
    interface_ip.num_wr_ports = core_params.memory_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = core_params.memory_ports;
    interface_ip.throughput = mem_man_params.dtlb_throughput / clockRate;
    interface_ip.latency = mem_man_params.dtlb_latency / clockRate;
    dtlb = new ArrayST(xml_data, &interface_ip, "Data TLB", Core_device,
                       clockRate, core_params.opt_local, core_params.core_ty);
    area.set_area(area.get_area() + dtlb->local_result.area);

}

void
MemManU::set_params_stats() {
    memset(&mem_man_params, 0, sizeof(MemoryManagementParams));
    memset(&mem_man_stats, 0, sizeof(MemoryManagementStats));
    int num_children = xml_data->nChildNode("component");
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* child = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = child->getAttribute("type");

        if (!type)
            warnMissingComponentType(child->getAttribute("id"));

        STRCMP(type, "InstructionTLB") {
            int sub_num_children = child->nChildNode("param");
            int j;
            for (j = 0; j < sub_num_children; j++) {
                XMLNode* paramNode = child->getChildNodePtr("param", &j);
                XMLCSTR node_name = paramNode->getAttribute("name");
                XMLCSTR value = paramNode->getAttribute("value");

                if (!node_name)
                    warnMissingParamName(paramNode->getAttribute("id"));

                ASSIGN_INT_IF("number_entries",
                              mem_man_params.itlb_number_entries);
                ASSIGN_FP_IF("latency", mem_man_params.itlb_latency);
                ASSIGN_FP_IF("throughput", mem_man_params.itlb_throughput);
                ASSIGN_FP_IF("assoc", mem_man_params.itlb_assoc);
                ASSIGN_FP_IF("nbanks", mem_man_params.itlb_nbanks);

                else {
                    warnUnrecognizedParam(node_name);
                }
            }
            sub_num_children = child->nChildNode("stat");
            for (j = 0; j < sub_num_children; j++) {
                XMLNode* statNode = child->getChildNodePtr("stat", &j);
                XMLCSTR node_name = statNode->getAttribute("name");
                XMLCSTR value = statNode->getAttribute("value");

                if (!node_name)
                    warnMissingStatName(statNode->getAttribute("id"));

                ASSIGN_FP_IF("total_accesses",
                             mem_man_stats.itlb_total_accesses);
                ASSIGN_FP_IF("total_misses", mem_man_stats.itlb_total_misses);
                ASSIGN_FP_IF("conflicts", mem_man_stats.itlb_conflicts);

                else {
                    warnUnrecognizedStat(node_name);
                }
            }
        } STRCMP(type, "DataTLB") {
            int sub_num_children = child->nChildNode("param");
            int j;
            for (j = 0; j < sub_num_children; j++) {
                XMLNode* paramNode = child->getChildNodePtr("param", &j);
                XMLCSTR node_name = paramNode->getAttribute("name");
                XMLCSTR value = paramNode->getAttribute("value");

                if (!node_name)
                    warnMissingParamName(paramNode->getAttribute("id"));

                ASSIGN_INT_IF("number_entries",
                              mem_man_params.dtlb_number_entries);
                ASSIGN_FP_IF("latency", mem_man_params.dtlb_latency);
                ASSIGN_FP_IF("throughput", mem_man_params.dtlb_throughput);
                ASSIGN_FP_IF("assoc", mem_man_params.dtlb_assoc);
                ASSIGN_FP_IF("nbanks", mem_man_params.dtlb_nbanks);

                else {
                    warnUnrecognizedParam(node_name);
                }
            }
            sub_num_children = child->nChildNode("stat");
            for (j = 0; j < sub_num_children; j++) {
                XMLNode* statNode = child->getChildNodePtr("stat", &j);
                XMLCSTR node_name = statNode->getAttribute("name");
                XMLCSTR value = statNode->getAttribute("value");

                if (!node_name)
                    warnMissingStatName(statNode->getAttribute("id"));

                ASSIGN_FP_IF("read_accesses",
                             mem_man_stats.dtlb_read_accesses);
                ASSIGN_FP_IF("read_misses", mem_man_stats.dtlb_read_misses);
                ASSIGN_FP_IF("write_accesses",
                             mem_man_stats.dtlb_write_accesses);
                ASSIGN_FP_IF("write_misses", mem_man_stats.dtlb_write_misses);
                ASSIGN_FP_IF("conflicts", mem_man_stats.dtlb_conflicts);

                else {
                    warnUnrecognizedStat(node_name);
                }
            }
        }
    }
}

RegFU::RegFU(XMLNode* _xml_data, InputParameter* interface_ip_,
             const CoreParameters & _core_params,
             const CoreStatistics & _core_stats, bool exist_)
        : McPATComponent(_xml_data), IRF(NULL), FRF(NULL), RFWIN(NULL),
          interface_ip(*interface_ip_),
          core_params(_core_params), core_stats(_core_stats), exist(exist_) {
    /*
     * processors have separate architectural register files for each thread.
     * therefore, the bypass buses need to travel across all the register files.
     */
    if (!exist) return;
    int data;
    int line;

    clockRate = core_params.clockRate;
    name = "Register File Unit";

    //**********************************IRF************************************
    data = core_params.int_data_width;
    line = int(ceil(data / BITS_PER_BYTE));

    interface_ip.cache_sz = core_params.num_IRF_entry * line;
    interface_ip.line_sz = line;
    interface_ip.assoc = core_params.phy_Regs_IRF_assoc;
    interface_ip.nbanks = core_params.phy_Regs_IRF_nbanks;
    interface_ip.out_w = line * BITS_PER_BYTE;
    interface_ip.specific_tag = core_params.phy_Regs_IRF_tag_width > 0;
    interface_ip.tag_w = core_params.phy_Regs_IRF_tag_width;
    interface_ip.access_mode = Sequential;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.phy_Regs_IRF_rd_ports;
    interface_ip.num_wr_ports = core_params.phy_Regs_IRF_wr_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.is_cache = false;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = true;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    IRF = new ArrayST(xml_data, &interface_ip, "Integer Register File",
                      Core_device, clockRate, core_params.opt_local,
                      core_params.core_ty);
    IRF->output_data.area *= core_params.num_hthreads *
        core_params.num_pipelines * cdb_overhead;
    area.set_area(area.get_area() + IRF->local_result.area *
                  core_params.num_hthreads * core_params.num_pipelines *
                  cdb_overhead);

    //**********************************FRF************************************
    data = core_params.fp_data_width;
    line = int(ceil(data / BITS_PER_BYTE));

    interface_ip.cache_sz = core_params.num_FRF_entry * line;
    interface_ip.line_sz = line;
    interface_ip.assoc = core_params.phy_Regs_FRF_assoc;
    interface_ip.nbanks = core_params.phy_Regs_FRF_nbanks;
    interface_ip.out_w = line * BITS_PER_BYTE;
    interface_ip.specific_tag = core_params.phy_Regs_FRF_tag_width > 0;
    interface_ip.tag_w = core_params.phy_Regs_FRF_tag_width;
    interface_ip.access_mode = Sequential;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = core_params.phy_Regs_FRF_rd_ports;
    interface_ip.num_wr_ports = core_params.phy_Regs_FRF_wr_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.is_cache = false;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = true;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    FRF = new ArrayST(xml_data, &interface_ip, "FP Register File", Core_device,
                      clockRate, core_params.opt_local, core_params.core_ty);
    FRF->output_data.area *= core_params.num_hthreads *
        core_params.num_fp_pipelines * cdb_overhead;
    area.set_area(area.get_area() + FRF->local_result.area *
                  core_params.num_hthreads * core_params.num_fp_pipelines *
                  cdb_overhead);
    int_regfile_height = IRF->local_result.cache_ht *
        core_params.num_hthreads * sqrt(cdb_overhead);
    fp_regfile_height = FRF->local_result.cache_ht * core_params.num_hthreads *
        sqrt(cdb_overhead);
    //since a EXU is associated with each pipeline, the cdb should not have
    //longer length.

    if (core_params.regWindowing) {
        //*********************************REG_WIN*****************************
        //ECC, and usually 2 regs are transfered together during window
        //shifting.Niagara Mega cell
        data = core_params.int_data_width;
        line = int(ceil(data / BITS_PER_BYTE));

        interface_ip.cache_sz = core_params.register_window_size *
            IRF->l_ip.cache_sz * core_params.num_hthreads;
        interface_ip.line_sz = line;
        interface_ip.assoc = core_params.register_window_assoc;
        interface_ip.nbanks = core_params.register_window_nbanks;
        interface_ip.out_w = line * BITS_PER_BYTE;
        interface_ip.specific_tag = core_params.register_window_tag_width > 0;
        interface_ip.tag_w = core_params.register_window_tag_width;
        interface_ip.access_mode = Sequential;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = core_params.register_window_rw_ports;
        interface_ip.num_rd_ports = 0;
        interface_ip.num_wr_ports = 0;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = 0;
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.throughput =
            core_params.register_window_throughput / clockRate;
        interface_ip.latency =
            core_params.register_window_latency / clockRate;
        RFWIN = new ArrayST(xml_data, &interface_ip, "RegWindow", Core_device,
                            clockRate, core_params.opt_local,
                            core_params.core_ty);
        RFWIN->output_data.area *= core_params.num_pipelines;
        area.set_area(area.get_area() + RFWIN->local_result.area *
                      core_params.num_pipelines);
    }
}

EXECU::EXECU(XMLNode* _xml_data,
             InputParameter* interface_ip_, double lsq_height_,
             const CoreParameters & _core_params,
             const CoreStatistics & _core_stats, bool exist_)
    : McPATComponent(_xml_data), rfu(NULL), scheu(NULL), fp_u(NULL),
      exeu(NULL), mul(NULL), int_bypass(NULL), intTagBypass(NULL),
      int_mul_bypass(NULL), intTag_mul_Bypass(NULL), fp_bypass(NULL),
      fpTagBypass(NULL), interface_ip(*interface_ip_),
      lsq_height(lsq_height_), core_params(_core_params),
      core_stats(_core_stats), exist(exist_) {
    if (!exist) return;
    double fu_height = 0.0;
    clockRate = core_params.clockRate;
    name = "Execution Unit";
    rfu = new RegFU(xml_data, &interface_ip, core_params, core_stats);
    if (core_params.core_ty == OOO ||
        (core_params.core_ty == Inorder && core_params.multithreaded)) {
        scheu = new SchedulerU(xml_data, &interface_ip, core_params,
                               core_stats);
        area.set_area(area.get_area() + scheu->area.get_area() );
    }
    exeu  = new FunctionalUnit(xml_data, &interface_ip, core_params,
                               core_stats, ALU);
    area.set_area(area.get_area() + exeu->area.get_area() +
                  rfu->area.get_area());
    fu_height = exeu->FU_height;
    if (core_params.num_fpus > 0) {
        fp_u  = new FunctionalUnit(xml_data, &interface_ip,
                                   core_params, core_stats, FPU);
        area.set_area(area.get_area() + fp_u->area.get_area());
    }
    if (core_params.num_muls > 0) {
        mul   = new FunctionalUnit(xml_data, &interface_ip,
                                   core_params, core_stats, MUL);
        area.set_area(area.get_area() + mul->area.get_area());
        fu_height +=  mul->FU_height;
    }
    /*
     * broadcast logic, including int-broadcast; int_tag-broadcast;
     * fp-broadcast; fp_tag-broadcast
     * integer by pass has two paths and fp has 3 paths.
     * on the same bus there are multiple tri-state drivers and muxes that go
     * to different components on the same bus
     */
    interface_ip.wt = core_params.execu_broadcast_wt;
    interface_ip.wire_is_mat_type = core_params.execu_wire_mat_type;
    interface_ip.wire_os_mat_type = core_params.execu_wire_mat_type;
    interface_ip.throughput = core_params.broadcast_numerator / clockRate;
    interface_ip.latency = core_params.broadcast_numerator / clockRate;
    double scheu_Iw_height = 0.0;
    double scheu_ROB_height = 0.0;
    double scheu_fp_Iw_height = 0.0;
    if (scheu) {
        scheu_Iw_height = scheu->Iw_height;
        scheu_ROB_height = scheu->ROB_height;
        scheu_fp_Iw_height = scheu->fp_Iw_height;
    }

    // Common bypass logic parameters
    double base_w = core_params.execu_bypass_base_width;
    double base_h = core_params.execu_bypass_base_height;
    int level = core_params.execu_bypass_start_wiring_level;
    double route_over_perc = core_params.execu_bypass_route_over_perc;
    Wire_type wire_type = core_params.execu_bypass_wire_type;
    int data_w;
    double len;

    if (core_params.core_ty == Inorder) {
        data_w = int(ceil(data_path_width / 32.0)*32);
        len = rfu->int_regfile_height + exeu->FU_height + lsq_height;
        int_bypass = new Interconnect(xml_data, "Int Bypass Data", Core_device,
                                      base_w, base_h, data_w, len,
                                      &interface_ip, level, clockRate, false,
                                      route_over_perc, core_params.opt_local,
                                      core_params.core_ty, wire_type);

        data_w = core_params.perThreadState;
        len = rfu->int_regfile_height + exeu->FU_height + lsq_height +
            scheu_Iw_height;
        intTagBypass = new Interconnect(xml_data, "Int Bypass Tag",
                                        Core_device,
                                        base_w, base_h, data_w, len,
                                        &interface_ip, level, clockRate, false,
                                        route_over_perc, core_params.opt_local,
                                        core_params.core_ty, wire_type);

        if (core_params.num_muls > 0) {
            data_w = int(ceil(data_path_width / 32.0)*32*1.5);
            len = rfu->fp_regfile_height + exeu->FU_height + mul->FU_height +
                lsq_height;
            int_mul_bypass = new Interconnect(xml_data, "Mul Bypass Data",
                                              Core_device, base_w, base_h,
                                              data_w, len, &interface_ip,
                                              level, clockRate, false,
                                              route_over_perc,
                                              core_params.opt_local,
                                              core_params.core_ty, wire_type);

            data_w = core_params.perThreadState;
            len = rfu->fp_regfile_height + exeu->FU_height + mul->FU_height +
                lsq_height + scheu_Iw_height;
            intTag_mul_Bypass = new Interconnect(xml_data, "Mul Bypass Tag",
                                                 Core_device, base_w, base_h,
                                                 data_w, len, &interface_ip,
                                                 level, clockRate, false,
                                                 route_over_perc,
                                                 core_params.opt_local,
                                                 core_params.core_ty,
                                                 wire_type);
        }

        if (core_params.num_fpus > 0) {
            data_w = int(ceil(data_path_width / 32.0)*32*1.5);
            len = rfu->fp_regfile_height + fp_u->FU_height;
            fp_bypass = new Interconnect(xml_data, "FP Bypass Data",
                                         Core_device,
                                         base_w, base_h, data_w, len,
                                         &interface_ip, level, clockRate,
                                         false, route_over_perc,
                                         core_params.opt_local,
                                         core_params.core_ty, wire_type);

            data_w = core_params.perThreadState;
            len = rfu->fp_regfile_height + fp_u->FU_height + lsq_height +
                scheu_Iw_height;
            fpTagBypass = new Interconnect(xml_data, "FP Bypass Tag",
                                           Core_device, base_w, base_h, data_w,
                                           len, &interface_ip, level,
                                           clockRate, false, route_over_perc,
                                           core_params.opt_local,
                                           core_params.core_ty, wire_type);
        }
    } else {//OOO
        if (core_params.scheu_ty == PhysicalRegFile) {
            /* For physical register based OOO,
             * data broadcast interconnects cover across functional units, lsq,
             * inst windows and register files,
             * while tag broadcast interconnects also cover across ROB
             */
            data_w = int(ceil(core_params.int_data_width));
            len = rfu->int_regfile_height + exeu->FU_height + lsq_height;
            int_bypass = new Interconnect(xml_data, "Int Bypass Data",
                                          Core_device, base_w, base_h, data_w,
                                          len, &interface_ip, level, clockRate,
                                          false, route_over_perc,
                                          core_params.opt_local,
                                          core_params.core_ty, wire_type);

            data_w = core_params.phy_ireg_width;
            len = rfu->int_regfile_height + exeu->FU_height + lsq_height +
                scheu_Iw_height + scheu_ROB_height;
            intTagBypass = new Interconnect(xml_data, "Int Bypass Tag",
                                            Core_device, base_w, base_h,
                                            data_w, len, &interface_ip, level,
                                            clockRate, false, route_over_perc,
                                            core_params.opt_local,
                                            core_params.core_ty, wire_type);

            if (core_params.num_muls > 0) {
                data_w = int(ceil(core_params.int_data_width));
                len = rfu->int_regfile_height + exeu->FU_height +
                    mul->FU_height + lsq_height;
                int_mul_bypass = new Interconnect(xml_data, "Mul Bypass Data",
                                                  Core_device, base_w, base_h,
                                                  data_w, len, &interface_ip,
                                                  level, clockRate, false,
                                                  route_over_perc,
                                                  core_params.opt_local,
                                                  core_params.core_ty,
                                                  wire_type);

                data_w = core_params.phy_ireg_width;
                len = rfu->int_regfile_height + exeu->FU_height +
                    mul->FU_height + lsq_height + scheu_Iw_height +
                    scheu_ROB_height;
                intTag_mul_Bypass = new Interconnect(xml_data,
                                                     "Mul Bypass Tag",
                                                     Core_device, base_w,
                                                     base_h, data_w, len,
                                                     &interface_ip, level,
                                                     clockRate, false,
                                                     route_over_perc,
                                                     core_params.opt_local,
                                                     core_params.core_ty,
                                                     wire_type);
            }

            if (core_params.num_fpus > 0) {
                data_w = int(ceil(core_params.fp_data_width));
                len = rfu->fp_regfile_height + fp_u->FU_height;
                fp_bypass = new Interconnect(xml_data, "FP Bypass Data",
                                             Core_device, base_w, base_h,
                                             data_w, len, &interface_ip, level,
                                             clockRate, false, route_over_perc,
                                             core_params.opt_local,
                                             core_params.core_ty, wire_type);

                data_w = core_params.phy_freg_width;
                len = rfu->fp_regfile_height + fp_u->FU_height + lsq_height +
                    scheu_fp_Iw_height + scheu_ROB_height;
                fpTagBypass = new Interconnect(xml_data, "FP Bypass Tag",
                                               Core_device, base_w, base_h,
                                               data_w, len, &interface_ip,
                                               level, clockRate, false,
                                               route_over_perc,
                                               core_params.opt_local,
                                               core_params.core_ty, wire_type);
            }
        } else {
            /*
             * In RS based processor both data and tag are broadcast together,
             * covering functional units, lsq, nst windows, register files, and ROBs
             */
            data_w = int(ceil(core_params.int_data_width));
            len = rfu->int_regfile_height + exeu->FU_height + lsq_height +
                scheu_Iw_height + scheu_ROB_height;
            int_bypass = new Interconnect(xml_data, "Int Bypass Data",
                                          Core_device, base_w, base_h, data_w,
                                          len, &interface_ip, level, clockRate,
                                          false, route_over_perc,
                                          core_params.opt_local,
                                          core_params.core_ty, wire_type);

            data_w = core_params.phy_ireg_width;
            len = rfu->int_regfile_height + exeu->FU_height + lsq_height +
                scheu_Iw_height + scheu_ROB_height;
            intTagBypass = new Interconnect(xml_data, "Int Bypass Tag",
                                            Core_device, base_w, base_h,
                                            data_w, len, &interface_ip, level,
                                            clockRate, false, route_over_perc,
                                            core_params.opt_local,
                                            core_params.core_ty, wire_type);
            if (core_params.num_muls > 0) {
                data_w = int(ceil(core_params.int_data_width));
                len = rfu->int_regfile_height + exeu->FU_height +
                    mul->FU_height + lsq_height + scheu_Iw_height +
                    scheu_ROB_height;
                int_mul_bypass = new Interconnect(xml_data, "Mul Bypass Data",
                                                  Core_device, base_w, base_h,
                                                  data_w, len, &interface_ip,
                                                  level, clockRate, false,
                                                  route_over_perc,
                                                  core_params.opt_local,
                                                  core_params.core_ty,
                                                  wire_type);

                data_w = core_params.phy_ireg_width;
                len = rfu->int_regfile_height + exeu->FU_height +
                    mul->FU_height + lsq_height + scheu_Iw_height +
                    scheu_ROB_height;
                intTag_mul_Bypass = new Interconnect(xml_data,
                                                     "Mul Bypass Tag",
                                                     Core_device, base_w,
                                                     base_h, data_w, len,
                                                     &interface_ip, level,
                                                     clockRate, false,
                                                     route_over_perc,
                                                     core_params.opt_local,
                                                     core_params.core_ty,
                                                     wire_type);
            }

            if (core_params.num_fpus > 0) {
                data_w = int(ceil(core_params.fp_data_width));
                len = rfu->fp_regfile_height + fp_u->FU_height + lsq_height +
                    scheu_fp_Iw_height + scheu_ROB_height;
                fp_bypass = new Interconnect(xml_data, "FP Bypass Data",
                                             Core_device, base_w, base_h,
                                             data_w, len, &interface_ip, level,
                                             clockRate, false, route_over_perc,
                                             core_params.opt_local,
                                             core_params.core_ty, wire_type);

                data_w = core_params.phy_freg_width;
                len = rfu->fp_regfile_height + fp_u->FU_height + lsq_height +
                    scheu_fp_Iw_height + scheu_ROB_height;
                fpTagBypass = new Interconnect(xml_data, "FP Bypass Tag",
                                               Core_device, base_w, base_h,
                                               data_w, len, &interface_ip,
                                               level, clockRate, false,
                                               route_over_perc,
                                               core_params.opt_local,
                                               core_params.core_ty, wire_type);
            }
        }
    }
    if (int_bypass) {
        children.push_back(int_bypass);
    }
    if (intTagBypass) {
        children.push_back(intTagBypass);
    }
    if (int_mul_bypass) {
        children.push_back(int_mul_bypass);
    }
    if (intTag_mul_Bypass) {
        children.push_back(intTag_mul_Bypass);
    }
    if (fp_bypass) {
        children.push_back(fp_bypass);
    }
    if (fpTagBypass) {
        children.push_back(fpTagBypass);
    }

    area.set_area(area.get_area() + int_bypass->area.get_area() +
                  intTagBypass->area.get_area());
    if (core_params.num_muls > 0) {
        area.set_area(area.get_area() + int_mul_bypass->area.get_area() +
                      intTag_mul_Bypass->area.get_area());
    }
    if (core_params.num_fpus > 0) {
        area.set_area(area.get_area() + fp_bypass->area.get_area() +
                      fpTagBypass->area.get_area());
    }
}

RENAMINGU::RENAMINGU(XMLNode* _xml_data, InputParameter* interface_ip_,
                     const CoreParameters & _core_params,
                     const CoreStatistics & _core_stats, bool exist_)
    : McPATComponent(_xml_data), iFRAT(NULL), fFRAT(NULL), iRRAT(NULL),
      fRRAT(NULL), ifreeL(NULL), ffreeL(NULL), idcl(NULL), fdcl(NULL),
      RAHT(NULL), interface_ip(*interface_ip_),
      core_params(_core_params), core_stats(_core_stats), exist(exist_) {
    if (!exist) return;
    int tag;
    int data;
    int out_w;
    int size;

    // Assumption:
    //   We make an implicit design assumption based on the specific structure
    //   that is being modeled.
    //   1. RAM-based RATs are direct mapped. However, if the associated
    //      scheduler is a reservation station style, the RATs are fully
    //      associative.
    //   2. Non-CAM based RATs and free lists do not have tags.
    //   3. Free lists are direct mapped.

    const int RAM_BASED_RAT_ASSOC = 1;
    const int RS_RAT_ASSOC = 0;
    const int NON_CAM_BASED_TAG_WIDTH = 0;
    const int FREELIST_ASSOC = 1;

    clockRate = core_params.clockRate;
    name = "Rename Unit";
    if (core_params.core_ty == OOO) {
        //integer pipeline
        if (core_params.scheu_ty == PhysicalRegFile) {
            if (core_params.rm_ty == RAMbased) {
                //FRAT with global checkpointing (GCs) please see paper tech
                //report for detailed explaintions

                data = int(ceil(core_params.phy_ireg_width *
                                (1 + core_params.globalCheckpoint) /
                                BITS_PER_BYTE));
                out_w = int(ceil(core_params.phy_ireg_width / BITS_PER_BYTE));

                size = data * core_params.archi_Regs_IRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = RAM_BASED_RAT_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
                interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports =
                    NUM_SOURCE_OPERANDS * core_params.decodeW;
                interface_ip.num_wr_ports = core_params.decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports = 0;
                interface_ip.is_cache = false;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = true;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                iFRAT = new ArrayST(xml_data, &interface_ip, "Int Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                iFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + iFRAT->area.get_area());

                //FRAT floating point
                data = int(ceil(core_params.phy_freg_width *
                                (1 + core_params.globalCheckpoint) /
                                BITS_PER_BYTE));
                out_w = int(ceil(core_params.phy_freg_width / BITS_PER_BYTE));
                size = data * core_params.archi_Regs_FRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = RAM_BASED_RAT_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
                interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports =
                    NUM_SOURCE_OPERANDS * core_params.fp_decodeW;
                interface_ip.num_wr_ports = core_params.fp_decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports = 0;
                interface_ip.is_cache = false;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = true;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                fFRAT = new ArrayST(xml_data, &interface_ip, "FP Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                fFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + fFRAT->area.get_area());

            } else if ((core_params.rm_ty == CAMbased)) {
                //IRAT
                tag = core_params.arch_ireg_width;
                //the address of CAM needed to be sent out
                data = int(ceil((core_params.arch_ireg_width + 1 *
                                 core_params.globalCheckpoint) /
                                BITS_PER_BYTE));
                out_w = int(ceil(core_params.arch_ireg_width / BITS_PER_BYTE));
                size = data * core_params.phy_Regs_IRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = CAM_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w  = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = tag > 0;
                interface_ip.tag_w = tag;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports = core_params.decodeW;
                interface_ip.num_wr_ports = core_params.decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports =
                    NUM_SOURCE_OPERANDS * core_params.decodeW;
                interface_ip.is_cache = true;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = false;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                iFRAT = new ArrayST(xml_data, &interface_ip, "Int Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                iFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + iFRAT->area.get_area());

                //FRAT for FP
                tag = core_params.arch_freg_width;
                //the address of CAM needed to be sent out
                data = int(ceil((core_params.arch_freg_width + 1 *
                                 core_params.globalCheckpoint) /
                                BITS_PER_BYTE));
                out_w = int(ceil(core_params.arch_freg_width / BITS_PER_BYTE));
                size = data * core_params.phy_Regs_FRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = CAM_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = tag > 0;
                interface_ip.tag_w = tag;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports = core_params.fp_decodeW;
                interface_ip.num_wr_ports = core_params.fp_decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports =
                    NUM_SOURCE_OPERANDS * core_params.fp_decodeW;
                interface_ip.is_cache = true;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = false;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                fFRAT = new ArrayST(xml_data, &interface_ip, "FP Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                fFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + fFRAT->area.get_area());
            }

            //RRAT is always RAM based, does not have GCs, and is used only for
            //record latest non-speculative mapping
            data = int(ceil(core_params.phy_ireg_width / BITS_PER_BYTE));
            size = data * core_params.archi_Regs_IRF_size *
                NUM_SOURCE_OPERANDS;

            interface_ip.cache_sz = size;
            interface_ip.line_sz = data;
            interface_ip.assoc = RAM_BASED_RAT_ASSOC;
            interface_ip.nbanks = core_params.retire_rat_nbanks;
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
            interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
            interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
            interface_ip.access_mode = Sequential;
            interface_ip.obj_func_dyn_energy = 0;
            interface_ip.obj_func_dyn_power = 0;
            interface_ip.obj_func_leak_power = 0;
            interface_ip.obj_func_cycle_t = 1;
            interface_ip.num_rw_ports = core_params.retire_rat_rw_ports;
            interface_ip.num_rd_ports = core_params.commitW;
            interface_ip.num_wr_ports = core_params.commitW;
            interface_ip.num_se_rd_ports = 0;
            interface_ip.num_search_ports = 0;
            interface_ip.is_cache = false;
            interface_ip.pure_cam = false;
            interface_ip.pure_ram = true;
            interface_ip.throughput = 1.0 / clockRate;
            interface_ip.latency = 1.0 / clockRate;
            iRRAT = new ArrayST(xml_data, &interface_ip, "Int Retire RAT",
                                Core_device, clockRate, core_params.opt_local,
                                core_params.core_ty);
            iRRAT->output_data.area *= core_params.num_hthreads;
            area.set_area(area.get_area() + iRRAT->area.get_area());

            //RRAT for FP
            data = int(ceil(core_params.phy_freg_width / BITS_PER_BYTE));
            size = data * core_params.archi_Regs_FRF_size *
                NUM_SOURCE_OPERANDS;

            interface_ip.cache_sz = size;
            interface_ip.line_sz = data;
            interface_ip.assoc = RAM_BASED_RAT_ASSOC;
            interface_ip.nbanks = core_params.retire_rat_nbanks;
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
            interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
            interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
            interface_ip.access_mode = Sequential;
            interface_ip.obj_func_dyn_energy = 0;
            interface_ip.obj_func_dyn_power = 0;
            interface_ip.obj_func_leak_power = 0;
            interface_ip.obj_func_cycle_t = 1;
            interface_ip.num_rw_ports = core_params.retire_rat_rw_ports;
            interface_ip.num_rd_ports = core_params.fp_decodeW;
            interface_ip.num_wr_ports = core_params.fp_decodeW;
            interface_ip.num_se_rd_ports = 0;
            interface_ip.num_search_ports = 0;
            interface_ip.is_cache = false;
            interface_ip.pure_cam = false;
            interface_ip.pure_ram = true;
            interface_ip.throughput = 1.0 / clockRate;
            interface_ip.latency = 1.0 / clockRate;
            fRRAT = new ArrayST(xml_data, &interface_ip, "FP Retire RAT",
                                Core_device, clockRate, core_params.opt_local,
                                core_params.core_ty);
            fRRAT->output_data.area *= core_params.num_hthreads;
            area.set_area(area.get_area() + fRRAT->area.get_area());

            //Freelist of renaming unit always RAM based
            //Recycle happens at two places: 1)when DCL check there are WAW, the Phyregisters/ROB directly recycles into freelist
            // 2)When instruction commits the Phyregisters/ROB needed to be recycled.
            //therefore num_wr port = decode-1(-1 means at least one phy reg will be used for the current renaming group) + commit width
            data = int(ceil(core_params.phy_ireg_width / BITS_PER_BYTE));
            size = data * core_params.num_ifreelist_entries;

            interface_ip.cache_sz = size;
            interface_ip.line_sz = data;
            interface_ip.assoc = FREELIST_ASSOC;
            interface_ip.nbanks = core_params.freelist_nbanks;
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
            interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
            interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
            interface_ip.access_mode = Sequential;
            interface_ip.obj_func_dyn_energy = 0;
            interface_ip.obj_func_dyn_power = 0;
            interface_ip.obj_func_leak_power = 0;
            interface_ip.obj_func_cycle_t = 1;
            interface_ip.num_rw_ports = core_params.freelist_rw_ports;
            interface_ip.num_rd_ports = core_params.decodeW;
            interface_ip.num_wr_ports =
                core_params.decodeW - 1 + core_params.commitW;
            interface_ip.num_se_rd_ports = 0;
            interface_ip.num_search_ports = 0;
            interface_ip.is_cache = false;
            interface_ip.pure_cam = false;
            interface_ip.pure_ram = true;
            interface_ip.throughput = 1.0 / clockRate;
            interface_ip.latency = 1.0 / clockRate;
            ifreeL = new ArrayST(xml_data, &interface_ip, "Integer Free List",
                                 Core_device, clockRate, core_params.opt_local,
                                 core_params.core_ty);
            ifreeL->output_data.area *= core_params.num_hthreads;
            area.set_area(area.get_area() + ifreeL->area.get_area());

            //freelist for FP
            data = int(ceil(core_params.phy_freg_width / BITS_PER_BYTE));
            size = data * core_params.num_ffreelist_entries;

            interface_ip.cache_sz = size;
            interface_ip.line_sz = data;
            interface_ip.assoc = FREELIST_ASSOC;
            interface_ip.nbanks = core_params.freelist_nbanks;
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
            interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
            interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
            interface_ip.access_mode = Sequential;
            interface_ip.obj_func_dyn_energy = 0;
            interface_ip.obj_func_dyn_power = 0;
            interface_ip.obj_func_leak_power = 0;
            interface_ip.obj_func_cycle_t = 1;
            interface_ip.num_rw_ports = core_params.freelist_rw_ports;
            interface_ip.num_rd_ports = core_params.fp_decodeW;
            interface_ip.num_wr_ports =
                core_params.fp_decodeW - 1 + core_params.commitW;
            interface_ip.num_se_rd_ports = 0;
            interface_ip.num_search_ports = 0;
            interface_ip.is_cache = false;
            interface_ip.pure_cam = false;
            interface_ip.pure_ram = true;
            interface_ip.throughput = 1.0 / clockRate;
            interface_ip.latency = 1.0 / clockRate;
            ffreeL = new ArrayST(xml_data, &interface_ip, "FP Free List",
                                 Core_device, clockRate, core_params.opt_local,
                                 core_params.core_ty);
            ffreeL->output_data.area *= core_params.num_hthreads;
            area.set_area(area.get_area() + ffreeL->area.get_area());

        } else if (core_params.scheu_ty == ReservationStation) {
            if (core_params.rm_ty == RAMbased) {
                tag = core_params.phy_ireg_width;
                data = int(ceil(core_params.phy_ireg_width *
                                (1 + core_params.globalCheckpoint) /
                                BITS_PER_BYTE));
                out_w = int(ceil(core_params.phy_ireg_width / BITS_PER_BYTE));
                size = data * core_params.archi_Regs_IRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = RS_RAT_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
                interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports =
                    NUM_SOURCE_OPERANDS * core_params.decodeW;
                interface_ip.num_wr_ports = core_params.decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports = core_params.commitW;
                interface_ip.is_cache = true;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = false;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                iFRAT = new ArrayST(xml_data, &interface_ip, "Int Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                iFRAT->local_result.adjust_area();
                iFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + iFRAT->area.get_area());

                //FP
                tag = core_params.phy_freg_width;
                data = int(ceil(core_params.phy_freg_width *
                                (1 + core_params.globalCheckpoint) /
                                BITS_PER_BYTE));
                out_w = int(ceil(core_params.phy_freg_width / BITS_PER_BYTE));
                size = data * core_params.archi_Regs_FRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = RS_RAT_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
                interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports =
                    NUM_SOURCE_OPERANDS * core_params.fp_decodeW;
                interface_ip.num_wr_ports = core_params.fp_decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports = core_params.fp_issueW;
                interface_ip.is_cache = true;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = false;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                fFRAT = new ArrayST(xml_data, &interface_ip, "FP Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                fFRAT->local_result.adjust_area();
                fFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + fFRAT->area.get_area());

            } else if ((core_params.rm_ty == CAMbased)) {
                //FRAT
                //the address of CAM needed to be sent out
                tag = core_params.arch_ireg_width;
                data = int(ceil (core_params.arch_ireg_width +
                                 1 * core_params.globalCheckpoint /
                                 BITS_PER_BYTE));
                out_w = int(ceil (core_params.arch_ireg_width /
                                  BITS_PER_BYTE));
                size = data * core_params.phy_Regs_IRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = CAM_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = tag > 0;
                interface_ip.tag_w = tag;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports = core_params.decodeW;
                interface_ip.num_wr_ports = core_params.decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports =
                    NUM_SOURCE_OPERANDS * core_params.decodeW;
                interface_ip.is_cache = true;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = false;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                iFRAT = new ArrayST(xml_data, &interface_ip, "Int Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                iFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + iFRAT->area.get_area());

                //FRAT
                tag = core_params.arch_freg_width;
                //the address of CAM needed to be sent out
                data = int(ceil(core_params.arch_freg_width +
                                1 * core_params.globalCheckpoint /
                               BITS_PER_BYTE));
                out_w = int(ceil(core_params.arch_freg_width / BITS_PER_BYTE));
                size = data * core_params.phy_Regs_FRF_size;

                interface_ip.cache_sz = size;
                interface_ip.line_sz = data;
                interface_ip.assoc = CAM_ASSOC;
                interface_ip.nbanks = core_params.front_rat_nbanks;
                interface_ip.out_w = out_w * BITS_PER_BYTE;
                interface_ip.specific_tag = tag > 0;
                interface_ip.tag_w = tag;
                interface_ip.access_mode = Fast;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t = 1;
                interface_ip.num_rw_ports = core_params.front_rat_rw_ports;
                interface_ip.num_rd_ports = core_params.decodeW;
                interface_ip.num_wr_ports = core_params.fp_decodeW;
                interface_ip.num_se_rd_ports = 0;
                interface_ip.num_search_ports =
                    NUM_SOURCE_OPERANDS * core_params.fp_decodeW;
                interface_ip.is_cache = true;
                interface_ip.pure_cam = false;
                interface_ip.pure_ram = false;
                interface_ip.throughput = 1.0 / clockRate;
                interface_ip.latency = 1.0 / clockRate;
                fFRAT = new ArrayST(xml_data, &interface_ip, "FP Front RAT",
                                    Core_device, clockRate,
                                    core_params.opt_local,
                                    core_params.core_ty);
                fFRAT->output_data.area *= core_params.num_hthreads;
                area.set_area(area.get_area() + fFRAT->area.get_area());

            }
            //No RRAT for RS based OOO
            //Freelist of renaming unit of RS based OOO is unifed for both int and fp renaming unit since the ROB is unified
            data = int(ceil(core_params.phy_ireg_width / BITS_PER_BYTE));
            size = data * core_params.num_ifreelist_entries;

            interface_ip.cache_sz = size;
            interface_ip.line_sz = data;
            interface_ip.assoc = FREELIST_ASSOC;
            interface_ip.nbanks = core_params.freelist_nbanks;
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
            interface_ip.specific_tag = NON_CAM_BASED_TAG_WIDTH > 0;
            interface_ip.tag_w = NON_CAM_BASED_TAG_WIDTH;
            interface_ip.access_mode = Fast;
            interface_ip.obj_func_dyn_energy = 0;
            interface_ip.obj_func_dyn_power = 0;
            interface_ip.obj_func_leak_power = 0;
            interface_ip.obj_func_cycle_t = 1;
            interface_ip.num_rw_ports = core_params.freelist_rw_ports;
            interface_ip.num_rd_ports = core_params.decodeW;
            interface_ip.num_wr_ports =
                core_params.decodeW - 1 + core_params.commitW;
            interface_ip.num_se_rd_ports = 0;
            interface_ip.num_search_ports = 0;
            interface_ip.is_cache = false;
            interface_ip.pure_cam = false;
            interface_ip.pure_ram = true;
            interface_ip.throughput = 1.0 / clockRate;
            interface_ip.latency = 1.0 / clockRate;
            ifreeL = new ArrayST(xml_data, &interface_ip, "Unified Free List",
                                 Core_device, clockRate, core_params.opt_local,
                                 core_params.core_ty);
            ifreeL->output_data.area *= core_params.num_hthreads;
            area.set_area(area.get_area() + ifreeL->area.get_area());
        }

    }
    idcl =
        new dep_resource_conflict_check(xml_data,
                                        "Instruction Dependency Check?",
                                        &interface_ip, core_params,
                                        core_params.phy_ireg_width,
                                        clockRate);
    fdcl =
        new dep_resource_conflict_check(xml_data,
                                        "FP Dependency Check?", &interface_ip,
                                        core_params,
                                        core_params.phy_freg_width, clockRate);
}

Core::Core(XMLNode* _xml_data, int _ithCore, InputParameter* interface_ip_)
    : McPATComponent(_xml_data), ifu(NULL), lsu(NULL), mmu(NULL),
      exu(NULL), rnu(NULL), corepipe (NULL), undiffCore(NULL), l2cache (NULL),
      ithCore(_ithCore), interface_ip(*interface_ip_) {

    ostringstream os;
    os << ithCore;
    name = "Core " + os.str();

    int i = 0;
    XMLNode* childXML;
    for (i = 0; i < xml_data->nChildNode("component"); i++) {
        childXML = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = childXML->getAttribute("type");
        if (!type)
            warnMissingComponentType(childXML->getAttribute("id"));

        STRCMP(type, "CacheUnit") {
            XMLCSTR comp_name = childXML->getAttribute("id");
            if (!comp_name)
                continue;

            STRCMP(comp_name, "system.L20") {
                l2cache = new CacheUnit(childXML, &interface_ip);
                children.push_back(l2cache);
            }
        }
    }

    set_core_param();
    clockRate = core_params.clockRate;

    ifu = new InstFetchU(xml_data, &interface_ip, core_params,
                         core_stats);
    children.push_back(ifu);
    lsu = new LoadStoreU(xml_data, &interface_ip, core_params,
                         core_stats);
    children.push_back(lsu);
    mmu = new MemManU(xml_data, &interface_ip, core_params,
                      core_stats);
    children.push_back(mmu);
    exu = new EXECU(xml_data, &interface_ip, lsu->lsq_height,
                    core_params, core_stats);
    children.push_back(exu);
    undiffCore = new UndiffCore(xml_data, &interface_ip, core_params);
    children.push_back(undiffCore);
    if (core_params.core_ty == OOO) {
        rnu = new RENAMINGU(xml_data, &interface_ip, core_params,
                            core_stats);
        children.push_back(rnu);
    }
    corepipe = new Pipeline(xml_data, &interface_ip, core_params);
    children.push_back(corepipe);

    double pipeline_area_per_unit;
    if (core_params.core_ty == OOO) {
        pipeline_area_per_unit = (corepipe->area.get_area() *
                                  core_params.num_pipelines) / 5.0;
        if (rnu->exist) {
            rnu->area.set_area(rnu->area.get_area() + pipeline_area_per_unit);
        }
    } else {
        pipeline_area_per_unit = (corepipe->area.get_area() *
                                  core_params.num_pipelines) / 4.0;
    }

    // Move all of this to computeArea
    //area.set_area(area.get_area()+ corepipe->area.get_area());
    if (ifu->exist) {
        ifu->area.set_area(ifu->area.get_area() + pipeline_area_per_unit);
        area.set_area(area.get_area() + ifu->area.get_area());
    }
    if (lsu->exist) {
        lsu->area.set_area(lsu->area.get_area() + pipeline_area_per_unit);
        area.set_area(area.get_area() + lsu->area.get_area());
    }
    if (exu->exist) {
        exu->area.set_area(exu->area.get_area() + pipeline_area_per_unit);
        area.set_area(area.get_area() + exu->area.get_area());
    }
    if (mmu->exist) {
        mmu->area.set_area(mmu->area.get_area() + pipeline_area_per_unit);
        area.set_area(area.get_area() + mmu->area.get_area());
    }

    if (core_params.core_ty == OOO) {
        if (rnu->exist) {

            area.set_area(area.get_area() + rnu->area.get_area());
        }
    }

    if (undiffCore->exist) {
        area.set_area(area.get_area() + undiffCore->area.get_area());
    }

    if (l2cache) {
        area.set_area(area.get_area() + l2cache->area.get_area());
    }
}


void BranchPredictor::computeEnergy() {
    if (!exist) return;

    // ASSUMPTION: All instructions access the branch predictors at Fetch and
    //             only branch instrucions update the predictors regardless
    //             of the correctness of the prediction.
    double tdp_read_accesses =
        core_params.predictionW * core_stats.BR_duty_cycle;
    globalBPT->tdp_stats.reset();
    globalBPT->tdp_stats.readAc.access  = tdp_read_accesses;
    globalBPT->tdp_stats.writeAc.access = 0;
    globalBPT->rtp_stats.reset();
    globalBPT->rtp_stats.readAc.access  = core_stats.total_instructions;
    globalBPT->rtp_stats.writeAc.access = core_stats.branch_instructions;
    globalBPT->power_t.reset();
    globalBPT->power_t.readOp.dynamic +=
        globalBPT->local_result.power.readOp.dynamic *
        globalBPT->tdp_stats.readAc.access +
        globalBPT->local_result.power.writeOp.dynamic *
        globalBPT->tdp_stats.writeAc.access;
    globalBPT->power_t = globalBPT->power_t +
        globalBPT->local_result.power * pppm_lkg;
    globalBPT->rt_power.reset();
    globalBPT->rt_power.readOp.dynamic +=
        globalBPT->local_result.power.readOp.dynamic *
        globalBPT->rtp_stats.readAc.access +
        globalBPT->local_result.power.writeOp.dynamic *
        globalBPT->rtp_stats.writeAc.access;

    L1_localBPT->tdp_stats.reset();
    L1_localBPT->tdp_stats.readAc.access  = tdp_read_accesses;
    L1_localBPT->tdp_stats.writeAc.access = 0;
    L1_localBPT->rtp_stats.reset();
    L1_localBPT->rtp_stats.readAc.access  = core_stats.total_instructions;
    L1_localBPT->rtp_stats.writeAc.access = core_stats.branch_instructions;
    L1_localBPT->power_t.reset();
    L1_localBPT->power_t.readOp.dynamic +=
        L1_localBPT->local_result.power.readOp.dynamic *
        L1_localBPT->tdp_stats.readAc.access +
        L1_localBPT->local_result.power.writeOp.dynamic *
        L1_localBPT->tdp_stats.writeAc.access;
    L1_localBPT->power_t = L1_localBPT->power_t +
        L1_localBPT->local_result.power * pppm_lkg;
    L1_localBPT->rt_power.reset();
    L1_localBPT->rt_power.readOp.dynamic +=
        L1_localBPT->local_result.power.readOp.dynamic *
        L1_localBPT->rtp_stats.readAc.access +
        L1_localBPT->local_result.power.writeOp.dynamic *
        L1_localBPT->rtp_stats.writeAc.access;

    L2_localBPT->tdp_stats.reset();
    L2_localBPT->tdp_stats.readAc.access  = tdp_read_accesses;
    L2_localBPT->tdp_stats.writeAc.access = 0;
    L2_localBPT->rtp_stats.reset();
    L2_localBPT->rtp_stats.readAc.access  = core_stats.branch_instructions;
    L2_localBPT->rtp_stats.writeAc.access = core_stats.branch_instructions;
    L2_localBPT->power_t.reset();
    L2_localBPT->power_t.readOp.dynamic +=
        L2_localBPT->local_result.power.readOp.dynamic *
        L2_localBPT->tdp_stats.readAc.access +
        L2_localBPT->local_result.power.writeOp.dynamic *
        L2_localBPT->tdp_stats.writeAc.access;
    L2_localBPT->power_t = L2_localBPT->power_t +
        L2_localBPT->local_result.power * pppm_lkg;
    L2_localBPT->rt_power.reset();
    L2_localBPT->rt_power.readOp.dynamic +=
        L2_localBPT->local_result.power.readOp.dynamic *
        L2_localBPT->rtp_stats.readAc.access +
        L2_localBPT->local_result.power.writeOp.dynamic *
        L2_localBPT->rtp_stats.writeAc.access;

    chooser->tdp_stats.reset();
    chooser->tdp_stats.readAc.access  = tdp_read_accesses;
    chooser->tdp_stats.writeAc.access = 0;
    chooser->rtp_stats.reset();
    chooser->rtp_stats.readAc.access  = core_stats.total_instructions;
    chooser->rtp_stats.writeAc.access = core_stats.branch_instructions;
    chooser->power_t.reset();
    chooser->power_t.readOp.dynamic +=
        chooser->local_result.power.readOp.dynamic *
        chooser->tdp_stats.readAc.access +
        chooser->local_result.power.writeOp.dynamic *
        chooser->tdp_stats.writeAc.access;
    chooser->power_t =
        chooser->power_t + chooser->local_result.power * pppm_lkg;
    chooser->rt_power.reset();
    chooser->rt_power.readOp.dynamic +=
        chooser->local_result.power.readOp.dynamic *
        chooser->rtp_stats.readAc.access +
        chooser->local_result.power.writeOp.dynamic *
        chooser->rtp_stats.writeAc.access;

    RAS->tdp_stats.reset();
    RAS->tdp_stats.readAc.access  = tdp_read_accesses;
    RAS->tdp_stats.writeAc.access = 0;
    RAS->rtp_stats.reset();
    RAS->rtp_stats.readAc.access  = core_stats.function_calls;
    RAS->rtp_stats.writeAc.access = core_stats.function_calls;
    RAS->power_t.reset();
    RAS->power_t.readOp.dynamic +=
        RAS->local_result.power.readOp.dynamic * RAS->tdp_stats.readAc.access +
        RAS->local_result.power.writeOp.dynamic *
        RAS->tdp_stats.writeAc.access;
    RAS->power_t = RAS->power_t + RAS->local_result.power *
        core_params.pppm_lkg_multhread;
    RAS->rt_power.reset();
    RAS->rt_power.readOp.dynamic += RAS->local_result.power.readOp.dynamic *
        RAS->rtp_stats.readAc.access +
        RAS->local_result.power.writeOp.dynamic *
        RAS->rtp_stats.writeAc.access;

    output_data.reset();
    if (globalBPT) {
        globalBPT->output_data.peak_dynamic_power =
            globalBPT->power_t.readOp.dynamic * clockRate;
        globalBPT->output_data.runtime_dynamic_energy =
            globalBPT->rt_power.readOp.dynamic;
        output_data += globalBPT->output_data;
    }
    if (L1_localBPT) {
        L1_localBPT->output_data.peak_dynamic_power =
            L1_localBPT->power_t.readOp.dynamic * clockRate;
        L1_localBPT->output_data.runtime_dynamic_energy =
            L1_localBPT->rt_power.readOp.dynamic;
        output_data += L1_localBPT->output_data;
    }
    if (L2_localBPT) {
        L2_localBPT->output_data.peak_dynamic_power =
            L2_localBPT->power_t.readOp.dynamic * clockRate;
        L2_localBPT->output_data.runtime_dynamic_energy =
            L2_localBPT->rt_power.readOp.dynamic;
        output_data += L2_localBPT->output_data;
    }
    if (chooser) {
        chooser->output_data.peak_dynamic_power =
            chooser->power_t.readOp.dynamic * clockRate;
        chooser->output_data.runtime_dynamic_energy =
            chooser->rt_power.readOp.dynamic;
        output_data += chooser->output_data;
    }
    if (RAS) {
        RAS->output_data.peak_dynamic_power =
            RAS->power_t.readOp.dynamic * clockRate;
        RAS->output_data.subthreshold_leakage_power =
            RAS->power_t.readOp.leakage * core_params.num_hthreads;
        RAS->output_data.gate_leakage_power =
            RAS->power_t.readOp.gate_leakage * core_params.num_hthreads;
        RAS->output_data.runtime_dynamic_energy = RAS->rt_power.readOp.dynamic;
        output_data += RAS->output_data;
    }
}

void BranchPredictor::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    globalBPT->displayData(indent + 4, plevel);
    L1_localBPT->displayData(indent + 4, plevel);
    L2_localBPT->displayData(indent + 4, plevel);
    chooser->displayData(indent + 4, plevel);
    RAS->displayData(indent + 4, plevel);
}

void InstFetchU::computeEnergy() {
    if (!exist) return;

    if (BPT) {
        BPT->computeEnergy();
    }

    IB->tdp_stats.reset();
    IB->tdp_stats.readAc.access = core_params.peak_issueW;
    IB->tdp_stats.writeAc.access = core_params.peak_issueW;
    IB->rtp_stats.reset();
    IB->rtp_stats.readAc.access = core_stats.total_instructions;
    IB->rtp_stats.writeAc.access = core_stats.total_instructions;
    IB->power_t.reset();
    IB->power_t.readOp.dynamic += IB->local_result.power.readOp.dynamic *
        IB->tdp_stats.readAc.access +
        IB->local_result.power.writeOp.dynamic * IB->tdp_stats.writeAc.access;
    IB->power_t = IB->power_t + IB->local_result.power * pppm_lkg;
    IB->rt_power.reset();
    IB->rt_power.readOp.dynamic += IB->local_result.power.readOp.dynamic *
        IB->rtp_stats.readAc.access +
        IB->local_result.power.writeOp.dynamic * IB->rtp_stats.writeAc.access;

    if (core_params.predictionW > 0) {
        BTB->tdp_stats.reset();
        BTB->tdp_stats.readAc.access = core_params.predictionW;
        BTB->tdp_stats.writeAc.access = 0;
        BTB->rtp_stats.reset();
        BTB->rtp_stats.readAc.access = inst_fetch_stats.btb_read_accesses;
        BTB->rtp_stats.writeAc.access = inst_fetch_stats.btb_write_accesses;
        BTB->power_t.reset();
        BTB->power_t.readOp.dynamic += BTB->local_result.power.readOp.dynamic *
            BTB->tdp_stats.readAc.access +
            BTB->local_result.power.writeOp.dynamic *
            BTB->tdp_stats.writeAc.access;
        BTB->rt_power.reset();
        BTB->rt_power.readOp.dynamic +=
            BTB->local_result.power.readOp.dynamic *
            BTB->rtp_stats.readAc.access +
            BTB->local_result.power.writeOp.dynamic *
            BTB->rtp_stats.writeAc.access;
    }

    ID_inst->tdp_stats.reset();
    ID_inst->tdp_stats.readAc.access = core_params.decodeW;
    ID_inst->power_t.reset();
    ID_inst->power_t = ID_misc->power;
    ID_inst->power_t.readOp.dynamic = ID_inst->power.readOp.dynamic *
        ID_inst->tdp_stats.readAc.access;
    ID_inst->rtp_stats.reset();
    ID_inst->rtp_stats.readAc.access = core_stats.total_instructions;
    ID_inst->rt_power.reset();
    ID_inst->rt_power.readOp.dynamic = ID_inst->power.readOp.dynamic *
        ID_inst->rtp_stats.readAc.access;

    ID_operand->tdp_stats.reset();
    ID_operand->tdp_stats.readAc.access = core_params.decodeW;
    ID_operand->power_t.reset();
    ID_operand->power_t = ID_misc->power;
    ID_operand->power_t.readOp.dynamic = ID_operand->power.readOp.dynamic *
        ID_operand->tdp_stats.readAc.access;
    ID_operand->rtp_stats.reset();
    ID_operand->rtp_stats.readAc.access = core_stats.total_instructions;
    ID_operand->rt_power.reset();
    ID_operand->rt_power.readOp.dynamic = ID_operand->power.readOp.dynamic *
        ID_operand->rtp_stats.readAc.access;

    ID_misc->tdp_stats.reset();
    ID_misc->tdp_stats.readAc.access = core_params.decodeW;
    ID_misc->power_t.reset();
    ID_misc->power_t = ID_misc->power;
    ID_misc->power_t.readOp.dynamic = ID_misc->power.readOp.dynamic *
        ID_misc->tdp_stats.readAc.access;
    ID_misc->rtp_stats.reset();
    ID_misc->rtp_stats.readAc.access = core_stats.total_instructions;
    ID_misc->rt_power.reset();
    ID_misc->rt_power.readOp.dynamic = ID_misc->power.readOp.dynamic *
        ID_misc->rtp_stats.readAc.access;

    power.reset();
    rt_power.reset();
    McPATComponent::computeEnergy();

    output_data.reset();
    if (icache) {
        output_data += icache->output_data;
    }
    if (IB) {
        IB->output_data.peak_dynamic_power =
            IB->power_t.readOp.dynamic * clockRate;
        IB->output_data.runtime_dynamic_energy = IB->rt_power.readOp.dynamic;
        output_data += IB->output_data;
    }
    if (BTB) {
        BTB->output_data.peak_dynamic_power =
            BTB->power_t.readOp.dynamic * clockRate;
        BTB->output_data.runtime_dynamic_energy = BTB->rt_power.readOp.dynamic;
        output_data += BTB->output_data;
    }
    if (BPT) {
        output_data += BPT->output_data;
    }
    if (ID_inst) {
        ID_inst->output_data.peak_dynamic_power =
            ID_inst->power_t.readOp.dynamic * clockRate;
        ID_inst->output_data.runtime_dynamic_energy =
            ID_inst->rt_power.readOp.dynamic;
        output_data += ID_inst->output_data;
    }
    if (ID_operand) {
        ID_operand->output_data.peak_dynamic_power =
            ID_operand->power_t.readOp.dynamic * clockRate;
        ID_operand->output_data.runtime_dynamic_energy =
            ID_operand->rt_power.readOp.dynamic;
        output_data += ID_operand->output_data;
    }
    if (ID_misc) {
        ID_misc->output_data.peak_dynamic_power =
            ID_misc->power_t.readOp.dynamic * clockRate;
        ID_misc->output_data.runtime_dynamic_energy =
            ID_misc->rt_power.readOp.dynamic;
        output_data += ID_misc->output_data;
    }
}

void InstFetchU::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    if (core_params.predictionW > 0) {
        BTB->displayData(indent + 4, plevel);
        if (BPT->exist) {
            BPT->displayData(indent + 4, plevel);
        }
    }
    IB->displayData(indent + 4, plevel);
    ID_inst->displayData(indent + 4, plevel);
    ID_operand->displayData(indent + 4, plevel);
    ID_misc->displayData(indent + 4, plevel);
}

void RENAMINGU::computeEnergy() {
    if (!exist) return;

    idcl->tdp_stats.reset();
    idcl->rtp_stats.reset();
    idcl->power_t.reset();
    idcl->rt_power.reset();
    if (core_params.core_ty == OOO) {
        idcl->tdp_stats.readAc.access = core_params.decodeW;
        idcl->rtp_stats.readAc.access = 3 * core_params.decodeW *
            core_params.decodeW * core_stats.rename_reads;
    } else if (core_params.issueW > 1) {
        idcl->tdp_stats.readAc.access = core_params.decodeW;
        idcl->rtp_stats.readAc.access = 2 * core_stats.int_instructions;
    }
    idcl->power_t.readOp.dynamic = idcl->tdp_stats.readAc.access *
        idcl->power.readOp.dynamic;
    idcl->power_t.readOp.leakage = idcl->power.readOp.leakage *
        core_params.num_hthreads;
    idcl->power_t.readOp.gate_leakage = idcl->power.readOp.gate_leakage *
        core_params.num_hthreads;
    idcl->rt_power.readOp.dynamic = idcl->rtp_stats.readAc.access *
        idcl->power.readOp.dynamic;

    fdcl->tdp_stats.reset();
    fdcl->rtp_stats.reset();
    fdcl->power_t.reset();
    fdcl->rt_power.reset();
    if (core_params.core_ty == OOO) {
        fdcl->tdp_stats.readAc.access = core_params.decodeW;
        fdcl->rtp_stats.readAc.access = 3 * core_params.fp_issueW *
            core_params.fp_issueW * core_stats.fp_rename_writes;
    } else if (core_params.issueW > 1) {
        fdcl->tdp_stats.readAc.access = core_params.decodeW;
        fdcl->rtp_stats.readAc.access = core_stats.fp_instructions;
    }
    fdcl->power_t.readOp.dynamic = fdcl->tdp_stats.readAc.access *
        fdcl->power.readOp.dynamic;
    fdcl->power_t.readOp.leakage = fdcl->power.readOp.leakage *
        core_params.num_hthreads;
    fdcl->power_t.readOp.gate_leakage = fdcl->power.readOp.gate_leakage *
        core_params.num_hthreads;
    fdcl->rt_power.readOp.dynamic = fdcl->rtp_stats.readAc.access *
        fdcl->power.readOp.dynamic;

    if (iRRAT) {
        iRRAT->tdp_stats.reset();
        iRRAT->tdp_stats.readAc.access = iRRAT->l_ip.num_rd_ports;
        iRRAT->tdp_stats.writeAc.access = iRRAT->l_ip.num_wr_ports;
        iRRAT->rtp_stats.reset();
        iRRAT->rtp_stats.readAc.access = core_stats.rename_writes;
        iRRAT->rtp_stats.writeAc.access = core_stats.rename_writes;
        iRRAT->power_t.reset();
        iRRAT->power_t.readOp.dynamic +=
            iRRAT->tdp_stats.readAc.access * iRRAT->power.readOp.dynamic +
            iRRAT->tdp_stats.writeAc.access * iRRAT->power.writeOp.dynamic;
        iRRAT->rt_power.reset();
        iRRAT->rt_power.readOp.dynamic +=
            iRRAT->rtp_stats.readAc.access * iRRAT->power.readOp.dynamic +
            iRRAT->rtp_stats.writeAc.access * iRRAT->power.writeOp.dynamic;
        iRRAT->power_t.readOp.leakage =
            iRRAT->power.readOp.leakage * core_params.num_hthreads;
        iRRAT->power_t.readOp.gate_leakage =
            iRRAT->power.readOp.gate_leakage * core_params.num_hthreads;
    }

    if (ifreeL) {
        ifreeL->tdp_stats.reset();
        ifreeL->tdp_stats.readAc.access = core_params.decodeW;
        ifreeL->tdp_stats.writeAc.access = core_params.decodeW;
        ifreeL->rtp_stats.reset();
        if (core_params.scheu_ty == PhysicalRegFile) {
            ifreeL->rtp_stats.readAc.access = core_stats.rename_reads;
            ifreeL->rtp_stats.writeAc.access = 2 * core_stats.rename_writes;
        } else if (core_params.scheu_ty == ReservationStation) {
            ifreeL->rtp_stats.readAc.access =
                core_stats.rename_reads + core_stats.fp_rename_reads;
            ifreeL->rtp_stats.writeAc.access =
                2 * (core_stats.rename_writes + core_stats.fp_rename_writes);
        }
        ifreeL->power_t.reset();
        ifreeL->power_t.readOp.dynamic +=
            ifreeL->tdp_stats.readAc.access * ifreeL->power.readOp.dynamic +
            ifreeL->tdp_stats.writeAc.access * ifreeL->power.writeOp.dynamic;
        ifreeL->rt_power.reset();
        ifreeL->rt_power.readOp.dynamic +=
            ifreeL->rtp_stats.readAc.access * ifreeL->power.readOp.dynamic +
            ifreeL->rtp_stats.writeAc.access * ifreeL->power.writeOp.dynamic;
        ifreeL->power_t.readOp.leakage =
            ifreeL->power.readOp.leakage * core_params.num_hthreads;
        ifreeL->power_t.readOp.gate_leakage =
            ifreeL->power.readOp.gate_leakage * core_params.num_hthreads;
    }

    if (fRRAT) {
        fRRAT->tdp_stats.reset();
        fRRAT->tdp_stats.readAc.access = fRRAT->l_ip.num_rd_ports;
        fRRAT->tdp_stats.writeAc.access = fRRAT->l_ip.num_wr_ports;
        fRRAT->rtp_stats.reset();
        fRRAT->rtp_stats.readAc.access = core_stats.fp_rename_writes;
        fRRAT->rtp_stats.writeAc.access = core_stats.fp_rename_writes;
        fRRAT->power_t.reset();
        fRRAT->power_t.readOp.dynamic +=
            fRRAT->tdp_stats.readAc.access * fRRAT->power.readOp.dynamic +
            fRRAT->tdp_stats.writeAc.access * fRRAT->power.writeOp.dynamic;
        fRRAT->rt_power.reset();
        fRRAT->rt_power.readOp.dynamic +=
            fRRAT->rtp_stats.readAc.access * fRRAT->power.readOp.dynamic +
            fRRAT->rtp_stats.writeAc.access * fRRAT->power.writeOp.dynamic;
        fRRAT->power_t.readOp.leakage =
            fRRAT->power.readOp.leakage * core_params.num_hthreads;
        fRRAT->power_t.readOp.gate_leakage =
            fRRAT->power.readOp.gate_leakage * core_params.num_hthreads;
    }

    if (ffreeL) {
        ffreeL->tdp_stats.reset();
        ffreeL->tdp_stats.readAc.access = core_params.decodeW;
        ffreeL->tdp_stats.writeAc.access = core_params.decodeW;
        ffreeL->rtp_stats.reset();
        ffreeL->rtp_stats.readAc.access = core_stats.fp_rename_reads;
        ffreeL->rtp_stats.writeAc.access = 2 * core_stats.fp_rename_writes;
        ffreeL->power_t.reset();
        ffreeL->power_t.readOp.dynamic +=
            ffreeL->tdp_stats.readAc.access * ffreeL->power.readOp.dynamic +
            ffreeL->tdp_stats.writeAc.access * ffreeL->power.writeOp.dynamic;
        ffreeL->rt_power.reset();
        ffreeL->rt_power.readOp.dynamic +=
            ffreeL->rtp_stats.readAc.access * ffreeL->power.readOp.dynamic +
            ffreeL->rtp_stats.writeAc.access * ffreeL->power.writeOp.dynamic;
        ffreeL->power_t.readOp.leakage =
            ffreeL->power.readOp.leakage * core_params.num_hthreads;
        ffreeL->power_t.readOp.gate_leakage =
            ffreeL->power.readOp.gate_leakage * core_params.num_hthreads;
    }

    if (iFRAT) {
        tdp_stats.reset();
        if (core_params.rm_ty == RAMbased) {
            iFRAT->tdp_stats.readAc.access = iFRAT->l_ip.num_rd_ports;
            iFRAT->tdp_stats.writeAc.access = iFRAT->l_ip.num_wr_ports;
            iFRAT->tdp_stats.searchAc.access = iFRAT->l_ip.num_search_ports;
        } else if ((core_params.rm_ty == CAMbased)) {
            iFRAT->tdp_stats.readAc.access = iFRAT->l_ip.num_search_ports;
            iFRAT->tdp_stats.writeAc.access = iFRAT->l_ip.num_wr_ports;
        }
        rtp_stats.reset();
        iFRAT->rtp_stats.readAc.access = core_stats.rename_reads;
        iFRAT->rtp_stats.writeAc.access = core_stats.rename_writes;
        if (core_params.scheu_ty == ReservationStation &&
            core_params.rm_ty == RAMbased) {
            iFRAT->rtp_stats.searchAc.access =
                core_stats.committed_int_instructions;
        }
        iFRAT->power_t.reset();
        iFRAT->power_t.readOp.dynamic += iFRAT->tdp_stats.readAc.access
            * (iFRAT->local_result.power.readOp.dynamic
               + idcl->power.readOp.dynamic)
            + iFRAT->tdp_stats.writeAc.access
            * iFRAT->local_result.power.writeOp.dynamic
            + iFRAT->tdp_stats.searchAc.access
            * iFRAT->local_result.power.searchOp.dynamic;
        iFRAT->power_t.readOp.leakage =
            iFRAT->power.readOp.leakage * core_params.num_hthreads;
        iFRAT->power_t.readOp.gate_leakage =
            iFRAT->power.readOp.gate_leakage * core_params.num_hthreads;
        iFRAT->rt_power.reset();
        iFRAT->rt_power.readOp.dynamic += iFRAT->rtp_stats.readAc.access
            * (iFRAT->local_result.power.readOp.dynamic
               + idcl->power.readOp.dynamic)
            + iFRAT->rtp_stats.writeAc.access
            * iFRAT->local_result.power.writeOp.dynamic
            + iFRAT->rtp_stats.searchAc.access
            * iFRAT->local_result.power.searchOp.dynamic;
    }

    if (fFRAT) {
        tdp_stats.reset();
        fFRAT->tdp_stats.writeAc.access = fFRAT->l_ip.num_wr_ports;
        if ((core_params.rm_ty == CAMbased)) {
            fFRAT->tdp_stats.readAc.access = fFRAT->l_ip.num_search_ports;
        } else if (core_params.rm_ty == RAMbased) {
            fFRAT->tdp_stats.readAc.access = fFRAT->l_ip.num_rd_ports;
            if (core_params.scheu_ty == ReservationStation) {
                fFRAT->tdp_stats.searchAc.access = fFRAT->l_ip.num_search_ports;
            }
        }
        rtp_stats.reset();
        fFRAT->rtp_stats.readAc.access = core_stats.fp_rename_reads;
        fFRAT->rtp_stats.writeAc.access = core_stats.fp_rename_writes;
        if (core_params.scheu_ty == ReservationStation &&
            core_params.rm_ty == RAMbased) {
            fFRAT->rtp_stats.searchAc.access =
                core_stats.committed_fp_instructions;
        }
        fFRAT->power_t.reset();
        fFRAT->power_t.readOp.dynamic += fFRAT->tdp_stats.readAc.access
            * (fFRAT->local_result.power.readOp.dynamic
               + fdcl->power.readOp.dynamic)
            + fFRAT->tdp_stats.writeAc.access
            * fFRAT->local_result.power.writeOp.dynamic
            + fFRAT->tdp_stats.searchAc.access
            * fFRAT->local_result.power.searchOp.dynamic;
        fFRAT->power_t.readOp.leakage =
            fFRAT->power.readOp.leakage * core_params.num_hthreads;
        fFRAT->power_t.readOp.gate_leakage =
            fFRAT->power.readOp.gate_leakage * core_params.num_hthreads;
        fFRAT->rt_power.reset();
        fFRAT->rt_power.readOp.dynamic += fFRAT->rtp_stats.readAc.access
            * (fFRAT->local_result.power.readOp.dynamic
               + fdcl->power.readOp.dynamic)
            + fFRAT->rtp_stats.writeAc.access
            * fFRAT->local_result.power.writeOp.dynamic
            + fFRAT->rtp_stats.searchAc.access
            * fFRAT->local_result.power.searchOp.dynamic;
    }

    output_data.reset();
    if (iFRAT) {
        iFRAT->output_data.peak_dynamic_power =
            iFRAT->power_t.readOp.dynamic * clockRate;
        iFRAT->output_data.subthreshold_leakage_power =
            iFRAT->power_t.readOp.leakage;
        iFRAT->output_data.gate_leakage_power =
            iFRAT->power_t.readOp.gate_leakage;
        iFRAT->output_data.runtime_dynamic_energy =
            iFRAT->rt_power.readOp.dynamic;
        output_data += iFRAT->output_data;
    }
    if (fFRAT) {
        fFRAT->output_data.peak_dynamic_power =
            fFRAT->power_t.readOp.dynamic * clockRate;
        fFRAT->output_data.subthreshold_leakage_power =
            fFRAT->power_t.readOp.leakage;
        fFRAT->output_data.gate_leakage_power =
            fFRAT->power_t.readOp.gate_leakage;
        fFRAT->output_data.runtime_dynamic_energy =
            fFRAT->rt_power.readOp.dynamic;
        output_data += fFRAT->output_data;
    }
    if (iRRAT) {
        iRRAT->output_data.peak_dynamic_power =
            iRRAT->power_t.readOp.dynamic * clockRate;
        iRRAT->output_data.subthreshold_leakage_power =
            iRRAT->power_t.readOp.leakage;
        iRRAT->output_data.gate_leakage_power =
            iRRAT->power_t.readOp.gate_leakage;
        iRRAT->output_data.runtime_dynamic_energy =
            iRRAT->rt_power.readOp.dynamic;
        output_data += iRRAT->output_data;
    }
    if (fRRAT) {
        fRRAT->output_data.peak_dynamic_power =
            fRRAT->power_t.readOp.dynamic * clockRate;
        fRRAT->output_data.subthreshold_leakage_power =
            fRRAT->power_t.readOp.leakage;
        fRRAT->output_data.gate_leakage_power =
            fRRAT->power_t.readOp.gate_leakage;
        fRRAT->output_data.runtime_dynamic_energy =
            fRRAT->rt_power.readOp.dynamic;
        output_data += fRRAT->output_data;
    }
    if (ifreeL) {
        ifreeL->output_data.peak_dynamic_power =
            ifreeL->power_t.readOp.dynamic * clockRate;
        ifreeL->output_data.subthreshold_leakage_power =
            ifreeL->power_t.readOp.leakage;
        ifreeL->output_data.gate_leakage_power =
            ifreeL->power_t.readOp.gate_leakage;
        ifreeL->output_data.runtime_dynamic_energy =
            ifreeL->rt_power.readOp.dynamic;
        output_data += ifreeL->output_data;
    }
    if (ffreeL) {
        ffreeL->output_data.peak_dynamic_power =
            ffreeL->power_t.readOp.dynamic * clockRate;
        ffreeL->output_data.subthreshold_leakage_power =
            ffreeL->power_t.readOp.leakage;
        ffreeL->output_data.gate_leakage_power =
            ffreeL->power_t.readOp.gate_leakage;
        ffreeL->output_data.runtime_dynamic_energy =
            ffreeL->rt_power.readOp.dynamic;
        output_data += ffreeL->output_data;
    }
    if (idcl) {
        idcl->output_data.peak_dynamic_power =
            idcl->power_t.readOp.dynamic * clockRate;
        idcl->output_data.subthreshold_leakage_power =
            idcl->power_t.readOp.leakage;
        idcl->output_data.gate_leakage_power =
            idcl->power_t.readOp.gate_leakage;
        idcl->output_data.runtime_dynamic_energy =
            idcl->rt_power.readOp.dynamic;
        output_data += idcl->output_data;
    }
    if (fdcl) {
        fdcl->output_data.peak_dynamic_power =
            fdcl->power_t.readOp.dynamic * clockRate;
        fdcl->output_data.subthreshold_leakage_power =
            fdcl->power_t.readOp.leakage;
        fdcl->output_data.gate_leakage_power =
            fdcl->power_t.readOp.gate_leakage;
        fdcl->output_data.runtime_dynamic_energy =
            fdcl->rt_power.readOp.dynamic;
        output_data += fdcl->output_data;
    }
    if (RAHT) {
        output_data += RAHT->output_data;
    }
}

void RENAMINGU::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    if (core_params.core_ty == OOO) {
        iFRAT->displayData(indent + 4, plevel);
        fFRAT->displayData(indent + 4, plevel);
        ifreeL->displayData(indent + 4, plevel);

        if (core_params.scheu_ty == PhysicalRegFile) {
            iRRAT->displayData(indent + 4, plevel);
            fRRAT->displayData(indent + 4, plevel);
            ffreeL->displayData(indent + 4, plevel);
        }
    }
    idcl->displayData(indent + 4, plevel);
    fdcl->displayData(indent + 4, plevel);
}

void SchedulerU::computeEnergy() {
    if (!exist) return;

    double ROB_duty_cycle;
    ROB_duty_cycle = 1;

    if (int_instruction_selection) {
        int_instruction_selection->computeEnergy();
    }

    if (fp_instruction_selection) {
        fp_instruction_selection->computeEnergy();
    }

    if (int_inst_window) {
        int_inst_window->tdp_stats.reset();
        int_inst_window->rtp_stats.reset();
        int_inst_window->power_t.reset();
        int_inst_window->rt_power.reset();
        if (core_params.core_ty == OOO) {
            int_inst_window->tdp_stats.readAc.access =
                core_params.issueW * core_params.num_pipelines;
            int_inst_window->tdp_stats.writeAc.access =
                core_params.issueW * core_params.num_pipelines;
            int_inst_window->tdp_stats.searchAc.access =
                core_params.issueW * core_params.num_pipelines;

            int_inst_window->power_t.readOp.dynamic +=
                int_inst_window->local_result.power.readOp.dynamic *
                int_inst_window->tdp_stats.readAc.access +
                int_inst_window->local_result.power.searchOp.dynamic *
                int_inst_window->tdp_stats.searchAc.access +
                int_inst_window->local_result.power.writeOp.dynamic *
                int_inst_window->tdp_stats.writeAc.access;

            int_inst_window->rtp_stats.readAc.access =
                core_stats.inst_window_reads;
            int_inst_window->rtp_stats.writeAc.access =
                core_stats.inst_window_writes;
            int_inst_window->rtp_stats.searchAc.access =
                core_stats.inst_window_wakeup_accesses;

            int_inst_window->rt_power.readOp.dynamic +=
                int_inst_window->local_result.power.readOp.dynamic *
                int_inst_window->rtp_stats.readAc.access +
                int_inst_window->local_result.power.searchOp.dynamic *
                int_inst_window->rtp_stats.searchAc.access +
                int_inst_window->local_result.power.writeOp.dynamic *
                int_inst_window->rtp_stats.writeAc.access;
        } else if (core_params.multithreaded) {
            int_inst_window->tdp_stats.readAc.access =
                core_params.issueW * core_params.num_pipelines;
            int_inst_window->tdp_stats.writeAc.access =
                core_params.issueW * core_params.num_pipelines;
            int_inst_window->tdp_stats.searchAc.access =
                core_params.issueW * core_params.num_pipelines;

            int_inst_window->power_t.readOp.dynamic +=
                int_inst_window->local_result.power.readOp.dynamic *
                int_inst_window->tdp_stats.readAc.access +
                int_inst_window->local_result.power.searchOp.dynamic *
                int_inst_window->tdp_stats.searchAc.access +
                int_inst_window->local_result.power.writeOp.dynamic *
                int_inst_window->tdp_stats.writeAc.access;

            int_inst_window->rtp_stats.readAc.access =
                core_stats.int_instructions + core_stats.fp_instructions;
            int_inst_window->rtp_stats.writeAc.access =
                core_stats.int_instructions + core_stats.fp_instructions;
            int_inst_window->rtp_stats.searchAc.access =
                2 * (core_stats.int_instructions + core_stats.fp_instructions);

            int_inst_window->rt_power.readOp.dynamic  +=
                int_inst_window->local_result.power.readOp.dynamic *
                int_inst_window->rtp_stats.readAc.access +
                int_inst_window->local_result.power.searchOp.dynamic *
                int_inst_window->rtp_stats.searchAc.access +
                int_inst_window->local_result.power.writeOp.dynamic *
                int_inst_window->rtp_stats.writeAc.access;
        }
    }

    if (fp_inst_window) {
        fp_inst_window->tdp_stats.reset();
        fp_inst_window->tdp_stats.readAc.access =
            fp_inst_window->l_ip.num_rd_ports * core_params.num_fp_pipelines;
        fp_inst_window->tdp_stats.writeAc.access =
            fp_inst_window->l_ip.num_wr_ports * core_params.num_fp_pipelines;
        fp_inst_window->tdp_stats.searchAc.access =
            fp_inst_window->l_ip.num_search_ports *
            core_params.num_fp_pipelines;

        fp_inst_window->rtp_stats.reset();
        fp_inst_window->rtp_stats.readAc.access =
            core_stats.fp_inst_window_reads;
        fp_inst_window->rtp_stats.writeAc.access =
            core_stats.fp_inst_window_writes;
        fp_inst_window->rtp_stats.searchAc.access =
            core_stats.fp_inst_window_wakeup_accesses;

        fp_inst_window->power_t.reset();
        fp_inst_window->power_t.readOp.dynamic +=
            fp_inst_window->power.readOp.dynamic *
            fp_inst_window->tdp_stats.readAc.access +
            fp_inst_window->power.searchOp.dynamic *
            fp_inst_window->tdp_stats.searchAc.access +
            fp_inst_window->power.writeOp.dynamic *
            fp_inst_window->tdp_stats.writeAc.access;

        fp_inst_window->rt_power.reset();
        fp_inst_window->rt_power.readOp.dynamic +=
            fp_inst_window->power.readOp.dynamic *
            fp_inst_window->rtp_stats.readAc.access +
            fp_inst_window->power.searchOp.dynamic *
            fp_inst_window->rtp_stats.searchAc.access +
            fp_inst_window->power.writeOp.dynamic *
            fp_inst_window->rtp_stats.writeAc.access;
    }

    if (ROB) {
        ROB->tdp_stats.reset();
        ROB->tdp_stats.readAc.access = core_params.commitW *
            core_params.num_pipelines * ROB_duty_cycle;
        ROB->tdp_stats.writeAc.access = core_params.issueW *
            core_params.num_pipelines * ROB_duty_cycle;
        ROB->rtp_stats.reset();
        ROB->rtp_stats.readAc.access = core_stats.ROB_reads;
        ROB->rtp_stats.writeAc.access = core_stats.ROB_writes;
        ROB->power_t.reset();
        ROB->power_t.readOp.dynamic +=
            ROB->local_result.power.readOp.dynamic *
            ROB->tdp_stats.readAc.access +
            ROB->local_result.power.writeOp.dynamic *
            ROB->tdp_stats.writeAc.access;
        ROB->rt_power.reset();
        ROB->rt_power.readOp.dynamic +=
            ROB->local_result.power.readOp.dynamic *
            ROB->rtp_stats.readAc.access +
            ROB->local_result.power.writeOp.dynamic *
            ROB->rtp_stats.writeAc.access;
    }

    output_data.reset();
    if (int_inst_window) {
        int_inst_window->output_data.subthreshold_leakage_power =
            int_inst_window->power_t.readOp.leakage;
        int_inst_window->output_data.gate_leakage_power =
            int_inst_window->power_t.readOp.gate_leakage;
        int_inst_window->output_data.peak_dynamic_power =
            int_inst_window->power_t.readOp.dynamic * clockRate;
        int_inst_window->output_data.runtime_dynamic_energy =
            int_inst_window->rt_power.readOp.dynamic;
        output_data += int_inst_window->output_data;
    }
    if (fp_inst_window) {
        fp_inst_window->output_data.subthreshold_leakage_power =
            fp_inst_window->power_t.readOp.leakage;
        fp_inst_window->output_data.gate_leakage_power =
            fp_inst_window->power_t.readOp.gate_leakage;
        fp_inst_window->output_data.peak_dynamic_power =
            fp_inst_window->power_t.readOp.dynamic * clockRate;
        fp_inst_window->output_data.runtime_dynamic_energy =
            fp_inst_window->rt_power.readOp.dynamic;
        output_data += fp_inst_window->output_data;
    }
    if (ROB) {
        ROB->output_data.peak_dynamic_power =
            ROB->power_t.readOp.dynamic * clockRate;
        ROB->output_data.runtime_dynamic_energy =
            ROB->rt_power.readOp.dynamic;
        output_data += ROB->output_data;
    }

    // Integer and FP instruction selection logic is not included in the
    // roll-up due to the uninitialized area
    /*
    if (int_instruction_selection) {
        output_data += int_instruction_selection->output_data;
    }
    if (fp_instruction_selection) {
        output_data += fp_instruction_selection->output_data;
    }
    */
}

void SchedulerU::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    if (core_params.core_ty == OOO) {
        int_inst_window->displayData(indent + 4, plevel);
        fp_inst_window->displayData(indent + 4, plevel);
        if (core_params.ROB_size > 0) {
            ROB->displayData(indent + 4, plevel);
        }
    } else if (core_params.multithreaded) {
        int_inst_window->displayData(indent + 4, plevel);
    }

    // Integer and FP instruction selection logic is not included in the
    // roll-up due to the uninitialized area
    /*
    if (int_instruction_selection) {
        int_instruction_selection->displayData(indent + 4, plevel);
    }
    if (fp_instruction_selection) {
        fp_instruction_selection->displayData(indent + 4, plevel);
    }
    */
}

void LoadStoreU::computeEnergy() {
    if (!exist) return;

    LSQ->tdp_stats.reset();
    LSQ->tdp_stats.readAc.access = LSQ->l_ip.num_search_ports *
        core_stats.LSU_duty_cycle;
    LSQ->tdp_stats.writeAc.access = LSQ->l_ip.num_search_ports *
        core_stats.LSU_duty_cycle;
    LSQ->rtp_stats.reset();
    // Flush overhead conidered
    LSQ->rtp_stats.readAc.access  = (core_stats.load_instructions +
                                     core_stats.store_instructions) * 2;
    LSQ->rtp_stats.writeAc.access = (core_stats.load_instructions +
                                     core_stats.store_instructions) * 2;
    LSQ->power_t.reset();
    //every memory access invloves at least two operations on LSQ
    LSQ->power_t.readOp.dynamic += LSQ->tdp_stats.readAc.access *
        (LSQ->local_result.power.searchOp.dynamic +
         LSQ->local_result.power.readOp.dynamic) +
        LSQ->tdp_stats.writeAc.access * LSQ->local_result.power.writeOp.dynamic;
    LSQ->rt_power.reset();
    //every memory access invloves at least two operations on LSQ
    LSQ->rt_power.readOp.dynamic += LSQ->rtp_stats.readAc.access *
        (LSQ->local_result.power.searchOp.dynamic +
         LSQ->local_result.power.readOp.dynamic) +
        LSQ->rtp_stats.writeAc.access * LSQ->local_result.power.writeOp.dynamic;

    if (LoadQ) {
        LoadQ->tdp_stats.reset();
        LoadQ->tdp_stats.readAc.access = LoadQ->l_ip.num_search_ports *
            core_stats.LSU_duty_cycle;
        LoadQ->tdp_stats.writeAc.access = LoadQ->l_ip.num_search_ports *
            core_stats.LSU_duty_cycle;
        LoadQ->rtp_stats.reset();
        LoadQ->rtp_stats.readAc.access = core_stats.load_instructions +
            core_stats.store_instructions;
        LoadQ->rtp_stats.writeAc.access = core_stats.load_instructions +
            core_stats.store_instructions;
        LoadQ->power_t.reset();
        //every memory access invloves at least two operations on LoadQ
        LoadQ->power_t.readOp.dynamic +=
            LoadQ->tdp_stats.readAc.access *
            (LoadQ->local_result.power.searchOp.dynamic +
             LoadQ->local_result.power.readOp.dynamic) +
            LoadQ->tdp_stats.writeAc.access *
            LoadQ->local_result.power.writeOp.dynamic;
        LoadQ->rt_power.reset();
        //every memory access invloves at least two operations on LoadQ
        LoadQ->rt_power.readOp.dynamic += LoadQ->rtp_stats.readAc.access *
            (LoadQ->local_result.power.searchOp.dynamic +
             LoadQ->local_result.power.readOp.dynamic) +
            LoadQ->rtp_stats.writeAc.access *
            LoadQ->local_result.power.writeOp.dynamic;
    }

    McPATComponent::computeEnergy();

    output_data.reset();
    if (dcache) {
        output_data += dcache->output_data;
    }
    if (LSQ) {
        LSQ->output_data.peak_dynamic_power =
            LSQ->power_t.readOp.dynamic * clockRate;
        LSQ->output_data.runtime_dynamic_energy = LSQ->rt_power.readOp.dynamic;
        output_data += LSQ->output_data;
    }
    if (LoadQ) {
        LoadQ->output_data.peak_dynamic_power =
            LoadQ->power_t.readOp.dynamic * clockRate;
        LoadQ->output_data.runtime_dynamic_energy =
            LoadQ->rt_power.readOp.dynamic;
        output_data += LoadQ->output_data;
    }
}

void LoadStoreU::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    if (LoadQ) {
        LoadQ->displayData(indent + 4, plevel);
    }
    LSQ->displayData(indent + 4, plevel);

}

void MemManU::computeEnergy() {
    if (!exist) return;

    itlb->tdp_stats.reset();
    itlb->tdp_stats.readAc.access = itlb->l_ip.num_search_ports;
    itlb->tdp_stats.readAc.miss = 0;
    itlb->tdp_stats.readAc.hit = itlb->tdp_stats.readAc.access -
        itlb->tdp_stats.readAc.miss;
    itlb->rtp_stats.reset();
    itlb->rtp_stats.readAc.access = mem_man_stats.itlb_total_accesses;
    itlb->rtp_stats.writeAc.access = mem_man_stats.itlb_total_misses;

    itlb->power_t.reset();
    //FA spent most power in tag, so use total access not hits
    itlb->power_t.readOp.dynamic += itlb->tdp_stats.readAc.access *
        itlb->local_result.power.searchOp.dynamic +
        itlb->tdp_stats.readAc.miss *
        itlb->local_result.power.writeOp.dynamic;
    itlb->rt_power.reset();
    //FA spent most power in tag, so use total access not hits
    itlb->rt_power.readOp.dynamic += itlb->rtp_stats.readAc.access *
        itlb->local_result.power.searchOp.dynamic +
        itlb->rtp_stats.writeAc.access *
        itlb->local_result.power.writeOp.dynamic;

    dtlb->tdp_stats.reset();
    dtlb->tdp_stats.readAc.access = dtlb->l_ip.num_search_ports *
        core_stats.LSU_duty_cycle;
    dtlb->tdp_stats.readAc.miss = 0;
    dtlb->tdp_stats.readAc.hit = dtlb->tdp_stats.readAc.access -
        dtlb->tdp_stats.readAc.miss;
    dtlb->rtp_stats.reset();
    dtlb->rtp_stats.readAc.access = mem_man_stats.dtlb_read_accesses +
        mem_man_stats.dtlb_write_misses;
    dtlb->rtp_stats.writeAc.access = mem_man_stats.dtlb_write_accesses +
        mem_man_stats.dtlb_read_misses;

    dtlb->power_t.reset();
    //FA spent most power in tag, so use total access not hits
    dtlb->power_t.readOp.dynamic += dtlb->tdp_stats.readAc.access *
        dtlb->local_result.power.searchOp.dynamic +
        dtlb->tdp_stats.readAc.miss *
        dtlb->local_result.power.writeOp.dynamic;
    dtlb->rt_power.reset();
    //FA spent most power in tag, so use total access not hits
    dtlb->rt_power.readOp.dynamic += dtlb->rtp_stats.readAc.access *
        dtlb->local_result.power.searchOp.dynamic +
        dtlb->rtp_stats.writeAc.access *
        dtlb->local_result.power.writeOp.dynamic;

    output_data.reset();
    if (itlb) {
        itlb->output_data.peak_dynamic_power = itlb->power_t.readOp.dynamic *
            clockRate;
        itlb->output_data.runtime_dynamic_energy =
            itlb->rt_power.readOp.dynamic;
        output_data += itlb->output_data;
    }
    if (dtlb) {
        dtlb->output_data.peak_dynamic_power =
            dtlb->power_t.readOp.dynamic * clockRate;
        dtlb->output_data.runtime_dynamic_energy =
            dtlb->rt_power.readOp.dynamic;
        output_data += dtlb->output_data;
    }
}

void MemManU::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    itlb->displayData(indent + 4, plevel);
    dtlb->displayData(indent + 4, plevel);
}

void RegFU::computeEnergy() {
    /*
     * Architecture RF and physical RF cannot be present at the same time.
     * Therefore, the RF stats can only refer to either ARF or PRF;
     * And the same stats can be used for both.
     */
    if (!exist) return;

    IRF->tdp_stats.reset();
    IRF->tdp_stats.readAc.access =
        core_params.issueW * NUM_INT_INST_SOURCE_OPERANDS *
        (core_stats.ALU_duty_cycle * 1.1 +
         (core_params.num_muls > 0 ? core_stats.MUL_duty_cycle : 0)) *
        core_params.num_pipelines;
    IRF->tdp_stats.writeAc.access =
        core_params.issueW *
        (core_stats.ALU_duty_cycle * 1.1 +
         (core_params.num_muls > 0 ? core_stats.MUL_duty_cycle : 0)) *
        core_params.num_pipelines;
    IRF->rtp_stats.reset();
    IRF->rtp_stats.readAc.access  = core_stats.int_regfile_reads;
    IRF->rtp_stats.writeAc.access  = core_stats.int_regfile_writes;
    if (core_params.regWindowing) {
        IRF->rtp_stats.readAc.access += core_stats.function_calls *
            RFWIN_ACCESS_MULTIPLIER;
        IRF->rtp_stats.writeAc.access += core_stats.function_calls *
            RFWIN_ACCESS_MULTIPLIER;
    }
    IRF->power_t.reset();
    IRF->power_t.readOp.dynamic += IRF->tdp_stats.readAc.access *
        IRF->local_result.power.readOp.dynamic +
        IRF->tdp_stats.writeAc.access *
        IRF->local_result.power.writeOp.dynamic;
    IRF->rt_power.reset();
    IRF->rt_power.readOp.dynamic +=
        IRF->rtp_stats.readAc.access * IRF->local_result.power.readOp.dynamic +
        IRF->rtp_stats.writeAc.access * IRF->local_result.power.writeOp.dynamic;

    FRF->tdp_stats.reset();
    FRF->tdp_stats.readAc.access  =
        FRF->l_ip.num_rd_ports * core_stats.FPU_duty_cycle * 1.05 *
        core_params.num_fp_pipelines;
    FRF->tdp_stats.writeAc.access  =
        FRF->l_ip.num_wr_ports * core_stats.FPU_duty_cycle * 1.05 *
        core_params.num_fp_pipelines;
    FRF->rtp_stats.reset();
    FRF->rtp_stats.readAc.access = core_stats.float_regfile_reads;
    FRF->rtp_stats.writeAc.access = core_stats.float_regfile_writes;
    if (core_params.regWindowing) {
        FRF->rtp_stats.readAc.access += core_stats.function_calls *
            RFWIN_ACCESS_MULTIPLIER;
        FRF->rtp_stats.writeAc.access += core_stats.function_calls *
            RFWIN_ACCESS_MULTIPLIER;
    }
    FRF->power_t.reset();
    FRF->power_t.readOp.dynamic +=
        FRF->tdp_stats.readAc.access * FRF->local_result.power.readOp.dynamic +
        FRF->tdp_stats.writeAc.access * FRF->local_result.power.writeOp.dynamic;
    FRF->rt_power.reset();
    FRF->rt_power.readOp.dynamic +=
        FRF->rtp_stats.readAc.access * FRF->local_result.power.readOp.dynamic +
        FRF->rtp_stats.writeAc.access * FRF->local_result.power.writeOp.dynamic;

    if (core_params.regWindowing) {
        RFWIN->tdp_stats.reset();
        RFWIN->tdp_stats.readAc.access = 0;
        RFWIN->tdp_stats.writeAc.access = 0;
        RFWIN->rtp_stats.reset();
        RFWIN->rtp_stats.readAc.access =
            core_stats.function_calls * RFWIN_ACCESS_MULTIPLIER;
        RFWIN->rtp_stats.writeAc.access =
            core_stats.function_calls * RFWIN_ACCESS_MULTIPLIER;
        RFWIN->power_t.reset();
        RFWIN->power_t.readOp.dynamic +=
            RFWIN->tdp_stats.readAc.access *
            RFWIN->local_result.power.readOp.dynamic +
            RFWIN->tdp_stats.writeAc.access *
            RFWIN->local_result.power.writeOp.dynamic;
        RFWIN->rt_power.reset();
        RFWIN->rt_power.readOp.dynamic +=
            RFWIN->rtp_stats.readAc.access *
            RFWIN->local_result.power.readOp.dynamic +
            RFWIN->rtp_stats.writeAc.access *
            RFWIN->local_result.power.writeOp.dynamic;
    }

    output_data.reset();
    if (IRF) {
        IRF->output_data.peak_dynamic_power =
            IRF->power_t.readOp.dynamic * clockRate;
        IRF->output_data.subthreshold_leakage_power *=
            core_params.num_hthreads;
        IRF->output_data.gate_leakage_power *= core_params.num_hthreads;
        IRF->output_data.runtime_dynamic_energy = IRF->rt_power.readOp.dynamic;
        output_data += IRF->output_data;
    }
    if (FRF) {
        FRF->output_data.peak_dynamic_power =
            FRF->power_t.readOp.dynamic * clockRate;
        FRF->output_data.subthreshold_leakage_power *=
            core_params.num_hthreads;
        FRF->output_data.gate_leakage_power *= core_params.num_hthreads;
        FRF->output_data.runtime_dynamic_energy = FRF->rt_power.readOp.dynamic;
        output_data += FRF->output_data;
    }
    if (RFWIN) {
        RFWIN->output_data.peak_dynamic_power =
            RFWIN->power_t.readOp.dynamic * clockRate;
        RFWIN->output_data.runtime_dynamic_energy =
            RFWIN->rt_power.readOp.dynamic;
        output_data += RFWIN->output_data;
    }
}

void RegFU::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    IRF->displayData(indent + 4, plevel);
    FRF->displayData(indent + 4, plevel);
    if (core_params.regWindowing) {
        RFWIN->displayData(indent + 4, plevel);
    }
}

void EXECU::computeEnergy() {
    if (!exist) return;

    int_bypass->set_params_stats(core_params.execu_int_bypass_ports,
                                 core_stats.ALU_cdb_duty_cycle,
                                 core_stats.cdb_alu_accesses);

    intTagBypass->set_params_stats(core_params.execu_int_bypass_ports,
                                   core_stats.ALU_cdb_duty_cycle,
                                   core_stats.cdb_alu_accesses);

    if (core_params.num_muls > 0) {
        int_mul_bypass->set_params_stats(core_params.execu_mul_bypass_ports,
                                         core_stats.MUL_cdb_duty_cycle,
                                         core_stats.cdb_mul_accesses);

        intTag_mul_Bypass->set_params_stats(core_params.execu_mul_bypass_ports,
                                            core_stats.MUL_cdb_duty_cycle,
                                            core_stats.cdb_mul_accesses);
    }

    if (core_params.num_fpus > 0) {
        fp_bypass->set_params_stats(core_params.execu_fp_bypass_ports,
                                    core_stats.FPU_cdb_duty_cycle,
                                    core_stats.cdb_fpu_accesses);

        fpTagBypass->set_params_stats(core_params.execu_fp_bypass_ports,
                                      core_stats.FPU_cdb_duty_cycle,
                                      core_stats.cdb_fpu_accesses);
    }

    McPATComponent::computeEnergy();

    if (rfu) {
        rfu->computeEnergy();
        output_data += rfu->output_data;
    }
    if (scheu) {
        scheu->computeEnergy();
        output_data += scheu->output_data;
    }
    if (fp_u) {
        fp_u->computeEnergy();
        output_data += fp_u->output_data;
    }
    if (exeu) {
        exeu->computeEnergy();
        output_data += exeu->output_data;
    }
    if (mul) {
        mul->computeEnergy();
        output_data += mul->output_data;
    }
}

void EXECU::displayData(uint32_t indent, int plevel) {
    if (!exist) return;

    McPATComponent::displayData(indent, plevel);

    rfu->displayData(indent + 4, plevel);
    if (scheu) {
        scheu->displayData(indent + 4, plevel);
    }
    exeu->displayData(indent + 4, plevel);
    if (core_params.num_fpus > 0) {
        fp_u->displayData(indent + 4, plevel);
    }
    if (core_params.num_muls > 0) {
        mul->displayData(indent + 4, plevel);
    }
}

void Core::computeEnergy() {
    ifu->computeEnergy();
    lsu->computeEnergy();
    mmu->computeEnergy();
    exu->computeEnergy();
    if (core_params.core_ty == OOO) {
        rnu->computeEnergy();
    }

    output_data.reset();
    if (ifu) {
        output_data += ifu->output_data;
    }
    if (lsu) {
        output_data += lsu->output_data;
    }
    if (mmu) {
        output_data += mmu->output_data;
    }
    if (exu) {
        output_data += exu->output_data;
    }
    if (rnu) {
        output_data += rnu->output_data;
    }
    if (corepipe) {
        output_data += corepipe->output_data;
    }
    if (undiffCore) {
        output_data += undiffCore->output_data;
    }
    if (l2cache) {
        output_data += l2cache->output_data;
    }
}

InstFetchU ::~InstFetchU() {

    if (!exist) return;
    if (IB) {
        delete IB;
        IB = NULL;
    }
    if (ID_inst) {
        delete ID_inst;
        ID_inst = NULL;
    }
    if (ID_operand) {
        delete ID_operand;
        ID_operand = NULL;
    }
    if (ID_misc) {
        delete ID_misc;
        ID_misc = NULL;
    }
    if (core_params.predictionW > 0) {
        if (BTB) {
            delete BTB;
            BTB = NULL;
        }
        if (BPT) {
            delete BPT;
            BPT = NULL;
        }
    }
    if (icache) {
        delete icache;
    }
}

BranchPredictor ::~BranchPredictor() {

    if (!exist) return;
    if (globalBPT) {
        delete globalBPT;
        globalBPT = NULL;
    }
    if (localBPT) {
        delete localBPT;
        localBPT = NULL;
    }
    if (L1_localBPT) {
        delete L1_localBPT;
        L1_localBPT = NULL;
    }
    if (L2_localBPT) {
        delete L2_localBPT;
        L2_localBPT = NULL;
    }
    if (chooser) {
        delete chooser;
        chooser = NULL;
    }
    if (RAS) {
        delete RAS;
        RAS = NULL;
    }
}

RENAMINGU ::~RENAMINGU() {

    if (!exist) return;
    if (iFRAT) {
        delete iFRAT;
        iFRAT = NULL;
    }
    if (fFRAT) {
        delete fFRAT;
        fFRAT = NULL;
    }
    if (iRRAT) {
        delete iRRAT;
        iRRAT = NULL;
    }
    if (iFRAT) {
        delete iFRAT;
        iFRAT = NULL;
    }
    if (ifreeL) {
        delete ifreeL;
        ifreeL = NULL;
    }
    if (ffreeL) {
        delete ffreeL;
        ffreeL = NULL;
    }
    if (idcl) {
        delete idcl;
        idcl = NULL;
    }
    if (fdcl) {
        delete fdcl;
        fdcl = NULL;
    }
    if (RAHT) {
        delete RAHT;
        RAHT = NULL;
    }
}

LoadStoreU ::~LoadStoreU() {

    if (!exist) return;
    if (LSQ) {
        delete LSQ;
        LSQ = NULL;
    }
    if (dcache) {
        delete dcache;
        dcache = NULL;
    }
}

MemManU ::~MemManU() {

    if (!exist) return;
    if (itlb) {
        delete itlb;
        itlb = NULL;
    }
    if (dtlb) {
        delete dtlb;
        dtlb = NULL;
    }
}

RegFU ::~RegFU() {

    if (!exist) return;
    if (IRF) {
        delete IRF;
        IRF = NULL;
    }
    if (FRF) {
        delete FRF;
        FRF = NULL;
    }
    if (RFWIN) {
        delete RFWIN;
        RFWIN = NULL;
    }
}

SchedulerU ::~SchedulerU() {

    if (!exist) return;
    if (int_inst_window) {
        delete int_inst_window;
        int_inst_window = NULL;
    }
    if (fp_inst_window) {
        delete int_inst_window;
        int_inst_window = NULL;
    }
    if (ROB) {
        delete ROB;
        ROB = NULL;
    }
    if (int_instruction_selection) {
        delete int_instruction_selection;
        int_instruction_selection = NULL;
    }
    if (fp_instruction_selection) {
        delete fp_instruction_selection;
        fp_instruction_selection = NULL;
    }
}

EXECU ::~EXECU() {

    if (!exist) return;
    if (int_bypass) {
        delete int_bypass;
        int_bypass = NULL;
    }
    if (intTagBypass) {
        delete intTagBypass;
        intTagBypass = NULL;
    }
    if (int_mul_bypass) {
        delete int_mul_bypass;
        int_mul_bypass = NULL;
    }
    if (intTag_mul_Bypass) {
        delete intTag_mul_Bypass;
        intTag_mul_Bypass = NULL;
    }
    if (fp_bypass) {
        delete fp_bypass;
        fp_bypass = NULL;
    }
    if (fpTagBypass) {
        delete fpTagBypass;
        fpTagBypass = NULL;
    }
    if (fp_u) {
        delete fp_u;
        fp_u = NULL;
    }
    if (exeu) {
        delete exeu;
        exeu = NULL;
    }
    if (mul) {
        delete mul;
        mul = NULL;
    }
    if (rfu) {
        delete rfu;
        rfu = NULL;
    }
    if (scheu) {
        delete scheu;
        scheu = NULL;
    }
}

Core::~Core() {

    if (ifu) {
        delete ifu;
        ifu = NULL;
    }
    if (lsu) {
        delete lsu;
        lsu = NULL;
    }
    if (rnu) {
        delete rnu;
        rnu = NULL;
    }
    if (mmu) {
        delete mmu;
        mmu = NULL;
    }
    if (exu) {
        delete exu;
        exu = NULL;
    }
    if (corepipe) {
        delete corepipe;
        corepipe = NULL;
    }
    if (undiffCore) {
        delete undiffCore;
        undiffCore = NULL;
    }
    if (l2cache) {
        delete l2cache;
        l2cache = NULL;
    }
}

void Core::initialize_params() {
    memset(&core_params, 0, sizeof(CoreParameters));
    core_params.peak_issueW = -1;
    core_params.peak_commitW = -1;
}

void Core::initialize_stats() {
    memset(&core_stats, 0, sizeof(CoreStatistics));
    core_stats.IFU_duty_cycle = 1.0;
    core_stats.ALU_duty_cycle = 1.0;
    core_stats.FPU_duty_cycle = 1.0;
    core_stats.MUL_duty_cycle = 1.0;
    core_stats.ALU_cdb_duty_cycle = 1.0;
    core_stats.FPU_cdb_duty_cycle = 1.0;
    core_stats.MUL_cdb_duty_cycle = 1.0;
    core_stats.pipeline_duty_cycle = 1.0;
    core_stats.IFU_duty_cycle = 1.0;
    core_stats.LSU_duty_cycle = 1.0;
    core_stats.MemManU_D_duty_cycle = 1.0;
    core_stats.MemManU_I_duty_cycle = 1.0;
}

void Core::set_core_param() {
    initialize_params();
    initialize_stats();

    int num_children = xml_data->nChildNode("param");
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_STR_IF("name", name);
        ASSIGN_INT_IF("opt_local", core_params.opt_local);
        ASSIGN_FP_IF("clock_rate", core_params.clockRate);
        ASSIGN_INT_IF("instruction_length", core_params.instruction_length);
        ASSIGN_INT_IF("opcode_width", core_params.opcode_width);
        ASSIGN_INT_IF("x86", core_params.x86);
        ASSIGN_INT_IF("Embedded", core_params.Embedded);
        ASSIGN_ENUM_IF("machine_type", core_params.core_ty, Core_type);
        ASSIGN_INT_IF("micro_opcode_width", core_params.micro_opcode_length);
        ASSIGN_INT_IF("number_hardware_threads", core_params.num_hthreads);
        ASSIGN_INT_IF("fetch_width", core_params.fetchW);
        ASSIGN_INT_IF("decode_width", core_params.decodeW);
        ASSIGN_INT_IF("issue_width", core_params.issueW);
        ASSIGN_INT_IF("peak_issue_width", core_params.peak_issueW);
        ASSIGN_INT_IF("commit_width", core_params.commitW);
        ASSIGN_INT_IF("prediction_width", core_params.predictionW);
        ASSIGN_INT_IF("ALU_per_core", core_params.num_alus);
        ASSIGN_INT_IF("FPU_per_core", core_params.num_fpus);
        ASSIGN_INT_IF("MUL_per_core", core_params.num_muls);
        ASSIGN_INT_IF("fp_issue_width", core_params.fp_issueW);
        ASSIGN_ENUM_IF("instruction_window_scheme", core_params.scheu_ty,
                       Scheduler_type);
        ASSIGN_ENUM_IF("rename_scheme", core_params.rm_ty, Renaming_type);
        ASSIGN_INT_IF("archi_Regs_IRF_size", core_params.archi_Regs_IRF_size);
        ASSIGN_INT_IF("archi_Regs_FRF_size", core_params.archi_Regs_FRF_size);
        ASSIGN_INT_IF("ROB_size", core_params.ROB_size);
        ASSIGN_INT_IF("ROB_assoc", core_params.ROB_assoc);
        ASSIGN_INT_IF("ROB_nbanks", core_params.ROB_nbanks);
        ASSIGN_INT_IF("ROB_tag_width", core_params.ROB_tag_width);
        ASSIGN_INT_IF("scheduler_assoc", core_params.scheduler_assoc);
        ASSIGN_INT_IF("scheduler_nbanks", core_params.scheduler_nbanks);
        ASSIGN_INT_IF("register_window_size",
                      core_params.register_window_size);
        ASSIGN_INT_IF("register_window_throughput",
                      core_params.register_window_throughput);
        ASSIGN_INT_IF("register_window_latency",
                      core_params.register_window_latency);
        ASSIGN_INT_IF("register_window_assoc",
                      core_params.register_window_assoc);
        ASSIGN_INT_IF("register_window_nbanks",
                      core_params.register_window_nbanks);
        ASSIGN_INT_IF("register_window_tag_width",
                      core_params.register_window_tag_width);
        ASSIGN_INT_IF("register_window_rw_ports",
                      core_params.register_window_rw_ports);
        ASSIGN_INT_IF("phy_Regs_IRF_size", core_params.phy_Regs_IRF_size);
        ASSIGN_INT_IF("phy_Regs_IRF_assoc", core_params.phy_Regs_IRF_assoc);
        ASSIGN_INT_IF("phy_Regs_IRF_nbanks", core_params.phy_Regs_IRF_nbanks);
        ASSIGN_INT_IF("phy_Regs_IRF_tag_width",
                      core_params.phy_Regs_IRF_tag_width);
        ASSIGN_INT_IF("phy_Regs_IRF_rd_ports",
                      core_params.phy_Regs_IRF_rd_ports);
        ASSIGN_INT_IF("phy_Regs_IRF_wr_ports",
                      core_params.phy_Regs_IRF_wr_ports);
        ASSIGN_INT_IF("phy_Regs_FRF_size", core_params.phy_Regs_FRF_size);
        ASSIGN_INT_IF("phy_Regs_FRF_assoc", core_params.phy_Regs_FRF_assoc);
        ASSIGN_INT_IF("phy_Regs_FRF_nbanks", core_params.phy_Regs_FRF_nbanks);
        ASSIGN_INT_IF("phy_Regs_FRF_tag_width",
                      core_params.phy_Regs_FRF_tag_width);
        ASSIGN_INT_IF("phy_Regs_FRF_rd_ports",
                      core_params.phy_Regs_FRF_rd_ports);
        ASSIGN_INT_IF("phy_Regs_FRF_wr_ports",
                      core_params.phy_Regs_FRF_wr_ports);
        ASSIGN_INT_IF("front_rat_nbanks", core_params.front_rat_nbanks);
        ASSIGN_INT_IF("front_rat_rw_ports", core_params.front_rat_rw_ports);
        ASSIGN_INT_IF("retire_rat_nbanks", core_params.retire_rat_nbanks);
        ASSIGN_INT_IF("retire_rat_rw_ports", core_params.retire_rat_rw_ports);
        ASSIGN_INT_IF("freelist_nbanks", core_params.freelist_nbanks);
        ASSIGN_INT_IF("freelist_rw_ports", core_params.freelist_rw_ports);
        ASSIGN_INT_IF("memory_ports", core_params.memory_ports);
        ASSIGN_INT_IF("load_buffer_size", core_params.load_buffer_size);
        ASSIGN_INT_IF("load_buffer_assoc", core_params.load_buffer_assoc);
        ASSIGN_INT_IF("load_buffer_nbanks", core_params.load_buffer_nbanks);
        ASSIGN_INT_IF("store_buffer_size", core_params.store_buffer_size);
        ASSIGN_INT_IF("store_buffer_assoc", core_params.store_buffer_assoc);
        ASSIGN_INT_IF("store_buffer_nbanks", core_params.store_buffer_nbanks);
        ASSIGN_INT_IF("instruction_window_size",
                      core_params.instruction_window_size);
        ASSIGN_INT_IF("fp_instruction_window_size",
                      core_params.fp_instruction_window_size);
        ASSIGN_INT_IF("instruction_buffer_size",
                      core_params.instruction_buffer_size);
        ASSIGN_INT_IF("instruction_buffer_assoc",
                      core_params.instruction_buffer_assoc);
        ASSIGN_INT_IF("instruction_buffer_nbanks",
                      core_params.instruction_buffer_nbanks);
        ASSIGN_INT_IF("instruction_buffer_tag_width",
                      core_params.instruction_buffer_tag_width);
        ASSIGN_INT_IF("number_instruction_fetch_ports",
                      core_params.number_instruction_fetch_ports);
        ASSIGN_INT_IF("RAS_size", core_params.RAS_size);
        ASSIGN_ENUM_IF("execu_broadcast_wt", core_params.execu_broadcast_wt,
                       Wire_type);
        ASSIGN_INT_IF("execu_wire_mat_type", core_params.execu_wire_mat_type);
        ASSIGN_INT_IF("execu_int_bypass_ports",
                      core_params.execu_int_bypass_ports);
        ASSIGN_INT_IF("execu_mul_bypass_ports",
                      core_params.execu_mul_bypass_ports);
        ASSIGN_INT_IF("execu_fp_bypass_ports",
                      core_params.execu_fp_bypass_ports);
        ASSIGN_ENUM_IF("execu_bypass_wire_type",
                       core_params.execu_bypass_wire_type, Wire_type);
        ASSIGN_FP_IF("execu_bypass_base_width",
                     core_params.execu_bypass_base_width);
        ASSIGN_FP_IF("execu_bypass_base_height",
                     core_params.execu_bypass_base_height);
        ASSIGN_INT_IF("execu_bypass_start_wiring_level",
                      core_params.execu_bypass_start_wiring_level);
        ASSIGN_FP_IF("execu_bypass_route_over_perc",
                     core_params.execu_bypass_route_over_perc);
        ASSIGN_FP_IF("broadcast_numerator", core_params.broadcast_numerator);
        ASSIGN_INT_IF("int_pipeline_depth", core_params.pipeline_stages);
        ASSIGN_INT_IF("fp_pipeline_depth", core_params.fp_pipeline_stages);
        ASSIGN_INT_IF("int_pipelines", core_params.num_pipelines);
        ASSIGN_INT_IF("fp_pipelines", core_params.num_fp_pipelines);
        ASSIGN_INT_IF("globalCheckpoint", core_params.globalCheckpoint);
        ASSIGN_INT_IF("perThreadState", core_params.perThreadState);
        ASSIGN_INT_IF("instruction_length", core_params.instruction_length);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    // Change from MHz to Hz
    core_params.clockRate *= 1e6;
    clockRate = core_params.clockRate;

    core_params.peak_commitW = core_params.peak_issueW;
    core_params.fp_decodeW = core_params.fp_issueW;


    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("ALU_duty_cycle", core_stats.ALU_duty_cycle);
        ASSIGN_FP_IF("FPU_duty_cycle", core_stats.FPU_duty_cycle);
        ASSIGN_FP_IF("MUL_duty_cycle", core_stats.MUL_duty_cycle);
        ASSIGN_FP_IF("ALU_cdb_duty_cycle", core_stats.ALU_cdb_duty_cycle);
        ASSIGN_FP_IF("FPU_cdb_duty_cycle", core_stats.FPU_cdb_duty_cycle);
        ASSIGN_FP_IF("MUL_cdb_duty_cycle", core_stats.MUL_cdb_duty_cycle);
        ASSIGN_FP_IF("pipeline_duty_cycle", core_stats.pipeline_duty_cycle);
        ASSIGN_FP_IF("total_cycles", core_stats.total_cycles);
        ASSIGN_FP_IF("busy_cycles", core_stats.busy_cycles);
        ASSIGN_FP_IF("idle_cycles", core_stats.idle_cycles);
        ASSIGN_FP_IF("IFU_duty_cycle", core_stats.IFU_duty_cycle);
        ASSIGN_FP_IF("BR_duty_cycle", core_stats.BR_duty_cycle);
        ASSIGN_FP_IF("LSU_duty_cycle", core_stats.LSU_duty_cycle);
        ASSIGN_FP_IF("MemManU_D_duty_cycle", core_stats.MemManU_D_duty_cycle);
        ASSIGN_FP_IF("MemManU_I_duty_cycle", core_stats.MemManU_I_duty_cycle);
        ASSIGN_FP_IF("cdb_fpu_accesses", core_stats.cdb_fpu_accesses);
        ASSIGN_FP_IF("cdb_alu_accesses", core_stats.cdb_alu_accesses);
        ASSIGN_FP_IF("cdb_mul_accesses", core_stats.cdb_mul_accesses);
        ASSIGN_FP_IF("function_calls", core_stats.function_calls);
        ASSIGN_FP_IF("total_instructions", core_stats.total_instructions);
        ASSIGN_FP_IF("int_instructions", core_stats.int_instructions);
        ASSIGN_FP_IF("fp_instructions", core_stats.fp_instructions);
        ASSIGN_FP_IF("branch_instructions", core_stats.branch_instructions);
        ASSIGN_FP_IF("branch_mispredictions",
                     core_stats.branch_mispredictions);
        ASSIGN_FP_IF("load_instructions", core_stats.load_instructions);
        ASSIGN_FP_IF("store_instructions", core_stats.store_instructions);
        ASSIGN_FP_IF("committed_instructions",
                     core_stats.committed_instructions);
        ASSIGN_FP_IF("committed_int_instructions",
                     core_stats.committed_int_instructions);
        ASSIGN_FP_IF("committed_fp_instructions",
                     core_stats.committed_fp_instructions);
        ASSIGN_FP_IF("ROB_reads", core_stats.ROB_reads);
        ASSIGN_FP_IF("ROB_writes", core_stats.ROB_writes);
        ASSIGN_FP_IF("rename_reads", core_stats.rename_reads);
        ASSIGN_FP_IF("rename_writes", core_stats.rename_writes);
        ASSIGN_FP_IF("fp_rename_reads", core_stats.fp_rename_reads);
        ASSIGN_FP_IF("fp_rename_writes", core_stats.fp_rename_writes);
        ASSIGN_FP_IF("inst_window_reads", core_stats.inst_window_reads);
        ASSIGN_FP_IF("inst_window_writes", core_stats.inst_window_writes);
        ASSIGN_FP_IF("inst_window_wakeup_accesses",
                     core_stats.inst_window_wakeup_accesses);
        ASSIGN_FP_IF("fp_inst_window_reads", core_stats.fp_inst_window_reads);
        ASSIGN_FP_IF("fp_inst_window_writes",
                     core_stats.fp_inst_window_writes);
        ASSIGN_FP_IF("fp_inst_window_wakeup_accesses",
                     core_stats.fp_inst_window_wakeup_accesses);
        ASSIGN_FP_IF("int_regfile_reads", core_stats.int_regfile_reads);
        ASSIGN_FP_IF("float_regfile_reads", core_stats.float_regfile_reads);
        ASSIGN_FP_IF("int_regfile_writes", core_stats.int_regfile_writes);
        ASSIGN_FP_IF("float_regfile_writes", core_stats.float_regfile_writes);
        ASSIGN_FP_IF("context_switches", core_stats.context_switches);
        ASSIGN_FP_IF("ialu_accesses", core_stats.ialu_accesses);
        ASSIGN_FP_IF("fpu_accesses", core_stats.fpu_accesses);
        ASSIGN_FP_IF("mul_accesses", core_stats.mul_accesses);

        else {
            warnUnrecognizedStat(node_name);
        }
    }

    // Initialize a few variables
    core_params.multithreaded = core_params.num_hthreads > 1 ? true : false;
    core_params.pc_width = virtual_address_width;
    core_params.v_address_width = virtual_address_width;
    core_params.p_address_width = physical_address_width;
    core_params.int_data_width = int(ceil(data_path_width / 32.0)) * 32;
    core_params.fp_data_width = core_params.int_data_width;
    core_params.arch_ireg_width =
        int(ceil(log2(core_params.archi_Regs_IRF_size)));
    core_params.arch_freg_width
        = int(ceil(log2(core_params.archi_Regs_FRF_size)));
    core_params.num_IRF_entry = core_params.archi_Regs_IRF_size;
    core_params.num_FRF_entry = core_params.archi_Regs_FRF_size;

    if (core_params.instruction_length <= 0) {
        errorNonPositiveParam("instruction_length");
    }

    if (core_params.num_hthreads <= 0) {
        errorNonPositiveParam("number_hardware_threads");
    }

    if (core_params.opcode_width <= 0) {
        errorNonPositiveParam("opcode_width");
    }

    if (core_params.instruction_buffer_size <= 0) {
        errorNonPositiveParam("instruction_buffer_size");
    }

    if (core_params.number_instruction_fetch_ports <= 0) {
        errorNonPositiveParam("number_instruction_fetch_ports");
    }

    if (core_params.peak_issueW <= 0) {
        errorNonPositiveParam("peak_issue_width");
    } else {
        assert(core_params.peak_commitW > 0);
    }

    if (core_params.core_ty == OOO) {
        if (core_params.scheu_ty == PhysicalRegFile) {
            core_params.phy_ireg_width =
                int(ceil(log2(core_params.phy_Regs_IRF_size)));
            core_params.phy_freg_width =
                int(ceil(log2(core_params.phy_Regs_FRF_size)));
            core_params.num_ifreelist_entries =
                core_params.num_IRF_entry = core_params.phy_Regs_IRF_size;
            core_params.num_ffreelist_entries =
                core_params.num_FRF_entry = core_params.phy_Regs_FRF_size;
        } else if (core_params.scheu_ty == ReservationStation) {
            core_params.phy_ireg_width = int(ceil(log2(core_params.ROB_size)));
            core_params.phy_freg_width = int(ceil(log2(core_params.ROB_size)));
            core_params.num_ifreelist_entries = core_params.ROB_size;
            core_params.num_ffreelist_entries = core_params.ROB_size;
        }
    }

    core_params.regWindowing =
        (core_params.register_window_size > 0 &&
         core_params.core_ty == Inorder) ? true : false;

    if (core_params.regWindowing) {
        if (core_params.register_window_throughput <= 0) {
            errorNonPositiveParam("register_window_throughput");
        } else if (core_params.register_window_latency <= 0) {
            errorNonPositiveParam("register_window_latency");
        }
    }

    set_pppm(core_params.pppm_lkg_multhread, 0, core_params.num_hthreads,
             core_params.num_hthreads, 0);

    if (!((core_params.core_ty == OOO) || (core_params.core_ty == Inorder))) {
        cout << "Invalid Core Type" << endl;
        exit(0);
    }

    if (!((core_params.scheu_ty == PhysicalRegFile) ||
          (core_params.scheu_ty == ReservationStation))) {
        cout << "Invalid OOO Scheduler Type" << endl;
        exit(0);
    }

    if (!((core_params.rm_ty == RAMbased) ||
          (core_params.rm_ty == CAMbased))) {
        cout << "Invalid OOO Renaming Type" << endl;
        exit(0);
    }

}
