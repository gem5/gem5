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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "XML_Parse.h"
#include "basic_circuit.h"
#include "const.h"
#include "core.h"
#include "io.h"
#include "parameter.h"
//#include "globalvar.h"

InstFetchU::InstFetchU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 IB  (0),
 BTB (0),
 ID_inst  (0),
 ID_operand  (0),
 ID_misc  (0),
 exist(exist_)
{
          if (!exist) return;
          int  idx, tag, data, size, line, assoc, banks;
          bool debug= false, is_default = true;

          clockRate = coredynp.clockRate;
          executionTime = coredynp.executionTime;
          cache_p = (Cache_policy)XML->sys.core[ithCore].icache.icache_config[7];
          //Assuming all L1 caches are virtually idxed physically tagged.
          //cache

          size                             = (int)XML->sys.core[ithCore].icache.icache_config[0];
          line                             = (int)XML->sys.core[ithCore].icache.icache_config[1];
          assoc                            = (int)XML->sys.core[ithCore].icache.icache_config[2];
          banks                            = (int)XML->sys.core[ithCore].icache.icache_config[3];
          idx    					 	   = debug?9:int(ceil(log2(size/line/assoc)));
          tag							   = debug?51:(int)XML->sys.physical_address_width-idx-int(ceil(log2(line))) + EXTRA_TAG_BITS;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.cache_sz            = debug?32768:(int)XML->sys.core[ithCore].icache.icache_config[0];
          interface_ip.line_sz             = debug?64:(int)XML->sys.core[ithCore].icache.icache_config[1];
          interface_ip.assoc               = debug?8:(int)XML->sys.core[ithCore].icache.icache_config[2];
          interface_ip.nbanks              = debug?1:(int)XML->sys.core[ithCore].icache.icache_config[3];
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;//debug?0:XML->sys.core[ithCore].icache.icache_config[5];
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[4]/clockRate;
          interface_ip.latency             = debug?3.0/clockRate:XML->sys.core[ithCore].icache.icache_config[5]/clockRate;
          interface_ip.is_cache			 = true;
          interface_ip.pure_cam			 = false;
          interface_ip.pure_ram			 = false;
        //  interface_ip.obj_func_dyn_energy = 0;
        //  interface_ip.obj_func_dyn_power  = 0;
        //  interface_ip.obj_func_leak_power = 0;
        //  interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          icache.caches = new ArrayST(&interface_ip, "icache", Core_device, coredynp.opt_local, coredynp.core_ty);
          scktRatio = g_tp.sckt_co_eff;
          chip_PR_overhead = g_tp.chip_layout_overhead;
          macro_PR_overhead = g_tp.macro_layout_overhead;
          icache.area.set_area(icache.area.get_area()+ icache.caches->local_result.area);
          area.set_area(area.get_area()+ icache.caches->local_result.area);
          //output_data_csv(icache.caches.local_result);


          /*
           *iCache controllers
           *miss buffer Each MSHR contains enough state
           *to handle one or more accesses of any type to a single memory line.
           *Due to the generality of the MSHR mechanism,
           *the amount of state involved is non-trivial:
           *including the address, pointers to the cache entry and destination register,
           *written data, and various other pieces of state.
           */
          interface_ip.num_search_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = (XML->sys.physical_address_width) + int(ceil(log2(size/line))) + icache.caches->l_ip.line_sz*8;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
          interface_ip.cache_sz            = XML->sys.core[ithCore].icache.buffer_sizes[0]*interface_ip.line_sz;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[4]/clockRate;//means cycle time
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[5]/clockRate;//means access time
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          interface_ip.num_search_ports = XML->sys.core[ithCore].number_instruction_fetch_ports;
          icache.missb = new ArrayST(&interface_ip, "icacheMissBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          icache.area.set_area(icache.area.get_area()+ icache.missb->local_result.area);
          area.set_area(area.get_area()+ icache.missb->local_result.area);
          //output_data_csv(icache.missb.local_result);

          //fill buffer
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = icache.caches->l_ip.line_sz;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
          interface_ip.cache_sz            = data*XML->sys.core[ithCore].icache.buffer_sizes[1];
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[4]/clockRate;
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          interface_ip.num_search_ports = XML->sys.core[ithCore].number_instruction_fetch_ports;
          icache.ifb = new ArrayST(&interface_ip, "icacheFillBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          icache.area.set_area(icache.area.get_area()+ icache.ifb->local_result.area);
          area.set_area(area.get_area()+ icache.ifb->local_result.area);
          //output_data_csv(icache.ifb.local_result);

          //prefetch buffer
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;//check with previous entries to decide wthether to merge.
          data							   = icache.caches->l_ip.line_sz;//separate queue to prevent from cache polution.
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
          interface_ip.cache_sz            = XML->sys.core[ithCore].icache.buffer_sizes[2]*interface_ip.line_sz;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[4]/clockRate;
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          interface_ip.num_search_ports = XML->sys.core[ithCore].number_instruction_fetch_ports;
          icache.prefetchb = new ArrayST(&interface_ip, "icacheprefetchBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          icache.area.set_area(icache.area.get_area()+ icache.prefetchb->local_result.area);
          area.set_area(area.get_area()+ icache.prefetchb->local_result.area);
          //output_data_csv(icache.prefetchb.local_result);

          //Instruction buffer
          data							   = XML->sys.core[ithCore].instruction_length*XML->sys.core[ithCore].peak_issue_width;//icache.caches.l_ip.line_sz; //multiple threads timing sharing the instruction buffer.
          interface_ip.is_cache			   = false;
          interface_ip.pure_ram            = true;
          interface_ip.pure_cam            = false;
          interface_ip.line_sz             = int(ceil(data/8.0));
          interface_ip.cache_sz            = XML->sys.core[ithCore].number_hardware_threads*XML->sys.core[ithCore].instruction_buffer_size*interface_ip.line_sz>64?
                                                     XML->sys.core[ithCore].number_hardware_threads*XML->sys.core[ithCore].instruction_buffer_size*interface_ip.line_sz:64;
          interface_ip.assoc               = 1;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = 1.0/clockRate;
          interface_ip.latency             = 1.0/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          //NOTE: Assuming IB is time slice shared among threads, every fetch op will at least fetch "fetch width" instructions.
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;//XML->sys.core[ithCore].fetch_width;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          IB = new ArrayST(&interface_ip, "InstBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          IB->area.set_area(IB->area.get_area()+ IB->local_result.area);
          area.set_area(area.get_area()+ IB->local_result.area);
          //output_data_csv(IB.IB.local_result);

          //	  inst_decoder.opcode_length = XML->sys.core[ithCore].opcode_width;
          //	  inst_decoder.init_decoder(is_default, &interface_ip);
          //	  inst_decoder.full_decoder_power();

      if (coredynp.predictionW>0)
      {
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
          size                             = XML->sys.core[ithCore].BTB.BTB_config[0];
          line                             = XML->sys.core[ithCore].BTB.BTB_config[1];
          assoc                            = XML->sys.core[ithCore].BTB.BTB_config[2];
          banks                            = XML->sys.core[ithCore].BTB.BTB_config[3];
          idx    					 	   = debug?9:int(ceil(log2(size/line/assoc)));
//    	  tag							   = debug?51:XML->sys.virtual_address_width-idx-int(ceil(log2(line))) + int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads))) +EXTRA_TAG_BITS;
          tag							   = debug?51:XML->sys.virtual_address_width + int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads))) +EXTRA_TAG_BITS;
          interface_ip.is_cache			   = true;
          interface_ip.pure_ram            = false;
          interface_ip.pure_cam            = false;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.cache_sz            = debug?32768:size;
          interface_ip.line_sz             = debug?64:line;
          interface_ip.assoc               = debug?8:assoc;
          interface_ip.nbanks              = debug?1:banks;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;//debug?0:XML->sys.core[ithCore].dcache.dcache_config[5];
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].BTB.BTB_config[4]/clockRate;
          interface_ip.latency             = debug?3.0/clockRate:XML->sys.core[ithCore].BTB.BTB_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 1;
          interface_ip.num_rd_ports    = coredynp.predictionW;
          interface_ip.num_wr_ports    = coredynp.predictionW;
          interface_ip.num_se_rd_ports = 0;
          BTB = new ArrayST(&interface_ip, "Branch Target Buffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          BTB->area.set_area(BTB->area.get_area()+ BTB->local_result.area);
          area.set_area(area.get_area()+ BTB->local_result.area);
          ///cout<<"area="<<area<<endl;

          BPT = new BranchPredictor(XML, ithCore, &interface_ip,coredynp);
          area.set_area(area.get_area()+ BPT->area.get_area());
      }

      ID_inst = new inst_decoder(is_default, &interface_ip,
                  coredynp.opcode_length, 1/*Decoder should not know how many by itself*/,
                  coredynp.x86,
                  Core_device, coredynp.core_ty);

      ID_operand = new inst_decoder(is_default, &interface_ip,
                  coredynp.arch_ireg_width, 1,
                  coredynp.x86,
                  Core_device, coredynp.core_ty);

      ID_misc = new inst_decoder(is_default, &interface_ip,
                  8/* Prefix field etc upto 14B*/, 1,
                  coredynp.x86,
                  Core_device, coredynp.core_ty);
      //TODO: X86 decoder should decode the inst in cyclic mode under the control of squencer.
      //So the dynamic power should be multiplied by a few times.
      area.set_area(area.get_area()+ (ID_inst->area.get_area()
                  +ID_operand->area.get_area()
                  +ID_misc->area.get_area())*coredynp.decodeW);

}


BranchPredictor::BranchPredictor(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 globalBPT(0),
 localBPT(0),
 L1_localBPT(0),
 L2_localBPT(0),
 chooser(0),
 RAS(0),
 exist(exist_)
{
        /*
         * Branch Predictor, accessed during ID stage.
         * McPAT's branch predictor model is the tournament branch predictor used in Alpha 21264,
         * including global predictor, local two level predictor, and Chooser.
         * The Branch predictor also includes a RAS (return address stack) for function calls
         * Branch predictors are tagged by thread ID and modeled as 1-way associative $
         * However RAS return address stacks are duplicated for each thread.
         * TODO:Data Width need to be computed more precisely	 *
         */
        if (!exist) return;
        int  tag, data;

        clockRate = coredynp.clockRate;
        executionTime = coredynp.executionTime;
        interface_ip.assoc               = 1;
        interface_ip.pure_cam            = false;
        if (coredynp.multithreaded)
        {

                tag							     = int(log2(coredynp.num_hthreads)+ EXTRA_TAG_BITS);
                interface_ip.specific_tag        = 1;
                interface_ip.tag_w               = tag;

                interface_ip.is_cache			 = true;
                interface_ip.pure_ram            = false;
                }
        else
        {
                interface_ip.is_cache			 = false;
                interface_ip.pure_ram            = true;

        }
        //Global predictor
        data							 = int(ceil(XML->sys.core[ithCore].predictor.global_predictor_bits/8.0));
        interface_ip.line_sz             = data;
        interface_ip.cache_sz            = data*XML->sys.core[ithCore].predictor.global_predictor_entries;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.access_mode         = 2;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports    = 0;
        interface_ip.num_rd_ports    = coredynp.predictionW;
        interface_ip.num_wr_ports    = coredynp.predictionW;
        interface_ip.num_se_rd_ports = 0;
        globalBPT = new ArrayST(&interface_ip, "Global Predictor", Core_device, coredynp.opt_local, coredynp.core_ty);
        globalBPT->area.set_area(globalBPT->area.get_area()+ globalBPT->local_result.area);
        area.set_area(area.get_area()+ globalBPT->local_result.area);

        //Local BPT (Level 1)
        data							 = int(ceil(XML->sys.core[ithCore].predictor.local_predictor_size[0]/8.0));
        interface_ip.line_sz             = data;
        interface_ip.cache_sz            = data*XML->sys.core[ithCore].predictor.local_predictor_entries;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.access_mode         = 2;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports    = 0;
        interface_ip.num_rd_ports    = coredynp.predictionW;
        interface_ip.num_wr_ports    = coredynp.predictionW;
        interface_ip.num_se_rd_ports = 0;
        L1_localBPT = new ArrayST(&interface_ip, "L1 local Predictor", Core_device, coredynp.opt_local, coredynp.core_ty);
        L1_localBPT->area.set_area(L1_localBPT->area.get_area()+ L1_localBPT->local_result.area);
        area.set_area(area.get_area()+ L1_localBPT->local_result.area);

        //Local BPT (Level 2)
        data							 = int(ceil(XML->sys.core[ithCore].predictor.local_predictor_size[1]/8.0));
        interface_ip.line_sz             = data;
        interface_ip.cache_sz            = data*XML->sys.core[ithCore].predictor.local_predictor_entries;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.access_mode         = 2;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports    = 0;
        interface_ip.num_rd_ports    = coredynp.predictionW;
        interface_ip.num_wr_ports    = coredynp.predictionW;
        interface_ip.num_se_rd_ports = 0;
        L2_localBPT = new ArrayST(&interface_ip, "L2 local Predictor", Core_device, coredynp.opt_local, coredynp.core_ty);
        L2_localBPT->area.set_area(L2_localBPT->area.get_area()+ L2_localBPT->local_result.area);
        area.set_area(area.get_area()+ L2_localBPT->local_result.area);

        //Chooser
        data							 = int(ceil(XML->sys.core[ithCore].predictor.chooser_predictor_bits/8.0));
        interface_ip.line_sz             = data;
        interface_ip.cache_sz            = data*XML->sys.core[ithCore].predictor.chooser_predictor_entries;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.access_mode         = 2;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports    = 0;
        interface_ip.num_rd_ports    = coredynp.predictionW;
        interface_ip.num_wr_ports    = coredynp.predictionW;
        interface_ip.num_se_rd_ports = 0;
        chooser = new ArrayST(&interface_ip, "Predictor Chooser", Core_device, coredynp.opt_local, coredynp.core_ty);
        chooser->area.set_area(chooser->area.get_area()+ chooser->local_result.area);
        area.set_area(area.get_area()+ chooser->local_result.area);

        //RAS return address stacks are Duplicated for each thread.
        interface_ip.is_cache			 = false;
        interface_ip.pure_ram            = true;
        data							 = int(ceil(coredynp.pc_width/8.0));
        interface_ip.line_sz             = data;
        interface_ip.cache_sz            = data*XML->sys.core[ithCore].RAS_size;
        interface_ip.assoc               = 1;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.access_mode         = 2;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports    = 0;
        interface_ip.num_rd_ports    = coredynp.predictionW;
        interface_ip.num_wr_ports    = coredynp.predictionW;
        interface_ip.num_se_rd_ports = 0;
        RAS = new ArrayST(&interface_ip, "RAS", Core_device, coredynp.opt_local, coredynp.core_ty);
        RAS->area.set_area(RAS->area.get_area()+ RAS->local_result.area*coredynp.num_hthreads);
        area.set_area(area.get_area()+ RAS->local_result.area*coredynp.num_hthreads);

}

SchedulerU::SchedulerU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 int_inst_window(0),
 fp_inst_window(0),
 ROB(0),
 instruction_selection(0),
 exist(exist_)
 {
        if (!exist) return;
        int   tag, data;
        bool  is_default=true;
        string tmp_name;

        clockRate = coredynp.clockRate;
        executionTime = coredynp.executionTime;
        if ((coredynp.core_ty==Inorder && coredynp.multithreaded))
        {
                //Instruction issue queue, in-order multi-issue or multithreaded processor also has this structure. Unified window for Inorder processors
                tag							     = int(log2(XML->sys.core[ithCore].number_hardware_threads)*coredynp.perThreadState);//This is the normal thread state bits based on Niagara Design
                data							 = XML->sys.core[ithCore].instruction_length;
                //NOTE: x86 inst can be very lengthy, up to 15B. Source: Intel® 64 and IA-32 Architectures
                //Software Developer’s Manual
                interface_ip.is_cache			 = true;
                interface_ip.pure_cam            = false;
                interface_ip.pure_ram            = false;
                interface_ip.line_sz             = int(ceil(data/8.0));
                interface_ip.specific_tag        = 1;
                interface_ip.tag_w               = tag;
                interface_ip.cache_sz            = XML->sys.core[ithCore].instruction_window_size*interface_ip.line_sz>64?XML->sys.core[ithCore].instruction_window_size*interface_ip.line_sz:64;
                interface_ip.assoc               = 0;
                interface_ip.nbanks              = 1;
                interface_ip.out_w               = interface_ip.line_sz*8;
                interface_ip.access_mode         = 1;
                interface_ip.throughput          = 1.0/clockRate;
                interface_ip.latency             = 1.0/clockRate;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power  = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t    = 1;
                interface_ip.num_rw_ports        = 0;
                interface_ip.num_rd_ports        = coredynp.peak_issueW;
                interface_ip.num_wr_ports        = coredynp.peak_issueW;
                interface_ip.num_se_rd_ports     = 0;
                interface_ip.num_search_ports    = coredynp.peak_issueW;
                int_inst_window = new ArrayST(&interface_ip, "InstFetchQueue", Core_device, coredynp.opt_local, coredynp.core_ty);
                int_inst_window->area.set_area(int_inst_window->area.get_area()+ int_inst_window->local_result.area*coredynp.num_pipelines);
                area.set_area(area.get_area()+ int_inst_window->local_result.area*coredynp.num_pipelines);
                //output_data_csv(iRS.RS.local_result);
                Iw_height      =int_inst_window->local_result.cache_ht;

                /*
                 * selection logic
                 * In a single-issue Inorder multithreaded processor like Niagara, issue width=1*number_of_threads since the processor does need to pick up
                 * instructions from multiple ready ones(although these ready ones are from different threads).While SMT processors do not distinguish which thread belongs to who
                 * at the issue stage.
                 */

                instruction_selection = new selection_logic(is_default, XML->sys.core[ithCore].instruction_window_size,
                                coredynp.peak_issueW*XML->sys.core[ithCore].number_hardware_threads,
                                &interface_ip, Core_device, coredynp.core_ty);
        }

    if (coredynp.core_ty==OOO)
    {
        /*
         * CAM based instruction window
         * For physicalRegFilebased OOO it is the instruction issue queue, where only tags of phy regs are stored
         * For RS based OOO it is the Reservation station, where both tags and values of phy regs are stored
         * It is written once and read twice(two operands) before an instruction can be issued.
         * X86 instruction can be very long up to 15B. add instruction length in XML
         */
        if(coredynp.scheu_ty==PhysicalRegFile)
        {
                tag	 = coredynp.phy_ireg_width;
                // Each time only half of the tag is compared, but two tag should be stored.
                // This underestimate the search power
                data = int((ceil((coredynp.instruction_length+2*(coredynp.phy_ireg_width - coredynp.arch_ireg_width))/2.0)/8.0));
                //Data width being divided by 2 means only after both operands available the whole data will be read out.
                //This is modeled using two equivalent readouts with half of the data width
                tmp_name = "InstIssueQueue";
        }
        else
        {
                tag	  = coredynp.phy_ireg_width;
                // Each time only half of the tag is compared, but two tag should be stored.
                // This underestimate the search power
                data  = int(ceil(((coredynp.instruction_length+2*(coredynp.phy_ireg_width - coredynp.arch_ireg_width)+
                                2*coredynp.int_data_width)/2.0)/8.0));
                //Data width being divided by 2 means only after both operands available the whole data will be read out.
                //This is modeled using two equivalent readouts with half of the data width

                tmp_name = "IntReservationStation";
        }
        interface_ip.is_cache			 = true;
        interface_ip.pure_cam            = false;
        interface_ip.pure_ram            = false;
        interface_ip.line_sz             = data;
        interface_ip.cache_sz            = data*XML->sys.core[ithCore].instruction_window_size;
        interface_ip.assoc               = 0;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.specific_tag        = 1;
        interface_ip.tag_w               = tag;
        interface_ip.access_mode         = 0;
        interface_ip.throughput          = 2*1.0/clockRate;
        interface_ip.latency             = 2*1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports       = 0;
        interface_ip.num_rd_ports       = coredynp.peak_issueW;
        interface_ip.num_wr_ports       = coredynp.peak_issueW;
        interface_ip.num_se_rd_ports    = 0;
                interface_ip.num_search_ports   = coredynp.peak_issueW;
                int_inst_window = new ArrayST(&interface_ip, tmp_name, Core_device, coredynp.opt_local, coredynp.core_ty);
                int_inst_window->area.set_area(int_inst_window->area.get_area()+ int_inst_window->local_result.area*coredynp.num_pipelines);
                area.set_area(area.get_area()+ int_inst_window->local_result.area*coredynp.num_pipelines);
                Iw_height      =int_inst_window->local_result.cache_ht;
                //FU inst window
        if(coredynp.scheu_ty==PhysicalRegFile)
        {
                tag	 = 2*coredynp.phy_freg_width;// TODO: each time only half of the tag is compared
                data = int(ceil((coredynp.instruction_length+2*(coredynp.phy_freg_width - coredynp.arch_freg_width))/8.0));
                tmp_name = "FPIssueQueue";
        }
        else
        {
                tag	  = 2*coredynp.phy_ireg_width;
                data  = int(ceil((coredynp.instruction_length+2*(coredynp.phy_freg_width - coredynp.arch_freg_width)+
                                2*coredynp.fp_data_width)/8.0));
                tmp_name = "FPReservationStation";
        }
        interface_ip.is_cache			 = true;
        interface_ip.pure_cam            = false;
        interface_ip.pure_ram            = false;
        interface_ip.line_sz             = data;
        interface_ip.cache_sz            = data*XML->sys.core[ithCore].fp_instruction_window_size;
        interface_ip.assoc               = 0;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.specific_tag        = 1;
        interface_ip.tag_w               = tag;
        interface_ip.access_mode         = 0;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports       = 0;
        interface_ip.num_rd_ports       = coredynp.fp_issueW;
        interface_ip.num_wr_ports       = coredynp.fp_issueW;
        interface_ip.num_se_rd_ports    = 0;
                interface_ip.num_search_ports   = coredynp.fp_issueW;
                fp_inst_window = new ArrayST(&interface_ip, tmp_name, Core_device, coredynp.opt_local, coredynp.core_ty);
                fp_inst_window->area.set_area(fp_inst_window->area.get_area()+ fp_inst_window->local_result.area*coredynp.num_fp_pipelines);
                area.set_area(area.get_area()+ fp_inst_window->local_result.area*coredynp.num_fp_pipelines);
                fp_Iw_height      =fp_inst_window->local_result.cache_ht;

                if (XML->sys.core[ithCore].ROB_size >0)
                {
                        /*
                         *  if ROB_size = 0, then the target processor does not support hardware-based
                         *  speculation, i.e. , the processor allow OOO issue as well as OOO completion, which
                         *  means branch must be resolved before instruction issued into instruction window, since
                         *  there is no change to flush miss-predict branch path after instructions are issued in this situation.
                         *
                         *  ROB.ROB size = inflight inst. ROB is unified for int and fp inst.
                         *  One old approach is to combine the RAT and ROB as a huge CAM structure as in AMD K7.
                         *  However, this approach is abandoned due to its high power and poor scalablility.
                         *	McPAT uses current implementation of ROB as circular buffer.
                         *	ROB is written once when instruction is issued and read once when the instruction is committed.         *
                         */
                        int robExtra = int(ceil(5 + log2(coredynp.num_hthreads)));
                        //5 bits are: busy, Issued, Finished, speculative, valid
                        if(coredynp.scheu_ty==PhysicalRegFile)
                        {
                                //PC is to id the instruction for recover exception.
                                //inst is used to map the renamed dest. registers.so that commit stage can know which reg/RRAT to update
//				data = int(ceil((robExtra+coredynp.pc_width +
//						coredynp.instruction_length + 2*coredynp.phy_ireg_width)/8.0));
                                data = int(ceil((robExtra+coredynp.pc_width +
                                                        coredynp.phy_ireg_width)/8.0));
                        }
                        else
                        {
                                //in RS based OOO, ROB also contains value of destination reg
//				data  = int(ceil((robExtra+coredynp.pc_width +
//						coredynp.instruction_length + 2*coredynp.phy_ireg_width + coredynp.fp_data_width)/8.0));
                                data  = int(ceil((robExtra + coredynp.pc_width +
                                                coredynp.phy_ireg_width + coredynp.fp_data_width)/8.0));
                        }
                        interface_ip.is_cache			 = false;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = true;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].ROB_size;//The XML ROB size is for all threads
                        interface_ip.assoc               = 1;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = interface_ip.line_sz*8;
                        interface_ip.access_mode         = 1;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports       = 0;
                        interface_ip.num_rd_ports       = coredynp.peak_commitW;
                        interface_ip.num_wr_ports       = coredynp.peak_issueW;
                        interface_ip.num_se_rd_ports    = 0;
                        interface_ip.num_search_ports   = 0;
                        ROB = new ArrayST(&interface_ip, "ReorderBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
                        ROB->area.set_area(ROB->area.get_area()+ ROB->local_result.area*coredynp.num_pipelines);
                        area.set_area(area.get_area()+ ROB->local_result.area*coredynp.num_pipelines);
                        ROB_height      =ROB->local_result.cache_ht;
                }

                instruction_selection = new selection_logic(is_default, XML->sys.core[ithCore].instruction_window_size,
                                coredynp.peak_issueW, &interface_ip, Core_device, coredynp.core_ty);
    }
}

LoadStoreU::LoadStoreU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 LSQ(0),
 exist(exist_)
{
          if (!exist) return;
          int  idx, tag, data, size, line, assoc, banks;
          bool debug= false;
          int ldst_opcode = XML->sys.core[ithCore].opcode_width;//16;

          clockRate = coredynp.clockRate;
          executionTime = coredynp.executionTime;
          cache_p = (Cache_policy)XML->sys.core[ithCore].dcache.dcache_config[7];

          interface_ip.num_search_ports    = XML->sys.core[ithCore].memory_ports;
          interface_ip.is_cache			   = true;
          interface_ip.pure_cam            = false;
          interface_ip.pure_ram            = false;
          //Dcache
          size                             = (int)XML->sys.core[ithCore].dcache.dcache_config[0];
          line                             = (int)XML->sys.core[ithCore].dcache.dcache_config[1];
          assoc                            = (int)XML->sys.core[ithCore].dcache.dcache_config[2];
          banks                            = (int)XML->sys.core[ithCore].dcache.dcache_config[3];
          idx    					 	   = debug?9:int(ceil(log2(size/line/assoc)));
          tag							   = debug?51:XML->sys.physical_address_width-idx-int(ceil(log2(line))) + EXTRA_TAG_BITS;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.cache_sz            = debug?32768:(int)XML->sys.core[ithCore].dcache.dcache_config[0];
          interface_ip.line_sz             = debug?64:(int)XML->sys.core[ithCore].dcache.dcache_config[1];
          interface_ip.assoc               = debug?8:(int)XML->sys.core[ithCore].dcache.dcache_config[2];
          interface_ip.nbanks              = debug?1:(int)XML->sys.core[ithCore].dcache.dcache_config[3];
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;//debug?0:XML->sys.core[ithCore].dcache.dcache_config[5];
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[4]/clockRate;
          interface_ip.latency             = debug?3.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[5]/clockRate;
          interface_ip.is_cache			 = true;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;//usually In-order has 1 and OOO has 2 at least.
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          dcache.caches = new ArrayST(&interface_ip, "dcache", Core_device, coredynp.opt_local, coredynp.core_ty);
          dcache.area.set_area(dcache.area.get_area()+ dcache.caches->local_result.area);
          area.set_area(area.get_area()+ dcache.caches->local_result.area);
          //output_data_csv(dcache.caches.local_result);

          //dCache controllers
          //miss buffer
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = (XML->sys.physical_address_width) + int(ceil(log2(size/line))) + dcache.caches->l_ip.line_sz*8;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
          interface_ip.cache_sz            = XML->sys.core[ithCore].dcache.buffer_sizes[0]*interface_ip.line_sz;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 2;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[4]/clockRate;
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          dcache.missb = new ArrayST(&interface_ip, "dcacheMissBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          dcache.area.set_area(dcache.area.get_area()+ dcache.missb->local_result.area);
          area.set_area(area.get_area()+ dcache.missb->local_result.area);
          //output_data_csv(dcache.missb.local_result);

          //fill buffer
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = dcache.caches->l_ip.line_sz;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
          interface_ip.cache_sz            = data*XML->sys.core[ithCore].dcache.buffer_sizes[1];
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 2;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[4]/clockRate;
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          dcache.ifb = new ArrayST(&interface_ip, "dcacheFillBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          dcache.area.set_area(dcache.area.get_area()+ dcache.ifb->local_result.area);
          area.set_area(area.get_area()+ dcache.ifb->local_result.area);
          //output_data_csv(dcache.ifb.local_result);

          //prefetch buffer
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;//check with previous entries to decide wthether to merge.
          data							   = dcache.caches->l_ip.line_sz;//separate queue to prevent from cache polution.
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
          interface_ip.cache_sz            = XML->sys.core[ithCore].dcache.buffer_sizes[2]*interface_ip.line_sz;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 2;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[4]/clockRate;
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          dcache.prefetchb = new ArrayST(&interface_ip, "dcacheprefetchBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
          dcache.area.set_area(dcache.area.get_area()+ dcache.prefetchb->local_result.area);
          area.set_area(area.get_area()+ dcache.prefetchb->local_result.area);
          //output_data_csv(dcache.prefetchb.local_result);

          //WBB

          if (cache_p==Write_back)
          {
                  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
                  data							   = dcache.caches->l_ip.line_sz;
                  interface_ip.specific_tag        = 1;
                  interface_ip.tag_w               = tag;
                  interface_ip.line_sz             = data;
                  interface_ip.cache_sz            = XML->sys.core[ithCore].dcache.buffer_sizes[3]*interface_ip.line_sz;
                  interface_ip.assoc               = 0;
                  interface_ip.nbanks              = 1;
                  interface_ip.out_w               = interface_ip.line_sz*8;
                  interface_ip.access_mode         = 2;
                  interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[4]/clockRate;
                  interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[5]/clockRate;
                  interface_ip.obj_func_dyn_energy = 0;
                  interface_ip.obj_func_dyn_power  = 0;
                  interface_ip.obj_func_leak_power = 0;
                  interface_ip.obj_func_cycle_t    = 1;
                  interface_ip.num_rw_ports    = XML->sys.core[ithCore].memory_ports;
                  interface_ip.num_rd_ports    = 0;
                  interface_ip.num_wr_ports    = 0;
                  interface_ip.num_se_rd_ports = 0;
                  dcache.wbb = new ArrayST(&interface_ip, "dcacheWBB", Core_device, coredynp.opt_local, coredynp.core_ty);
                  dcache.area.set_area(dcache.area.get_area()+ dcache.wbb->local_result.area);
                  area.set_area(area.get_area()+ dcache.wbb->local_result.area);
                  //output_data_csv(dcache.wbb.local_result);
          }

          /*
           * LSU--in-order processors do not have separate load queue: unified lsq
           * partitioned among threads
           * it is actually the store queue but for inorder processors it serves as both loadQ and StoreQ
           */
          tag							   = ldst_opcode+XML->sys.virtual_address_width +int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads))) + EXTRA_TAG_BITS;
          data							   = XML->sys.machine_bits;
          interface_ip.is_cache			   = true;
          interface_ip.line_sz             = int(ceil(data/32.0))*4;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.cache_sz            = XML->sys.core[ithCore].store_buffer_size*interface_ip.line_sz*XML->sys.core[ithCore].number_hardware_threads;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 1;
          interface_ip.throughput          = 1.0/clockRate;
          interface_ip.latency             = 1.0/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports        = 0;
          interface_ip.num_rd_ports        = XML->sys.core[ithCore].memory_ports;
          interface_ip.num_wr_ports        = XML->sys.core[ithCore].memory_ports;
          interface_ip.num_se_rd_ports     = 0;
          interface_ip.num_search_ports    =XML->sys.core[ithCore].memory_ports;
          LSQ = new ArrayST(&interface_ip, "Load(Store)Queue", Core_device, coredynp.opt_local, coredynp.core_ty);
          LSQ->area.set_area(LSQ->area.get_area()+ LSQ->local_result.area);
          area.set_area(area.get_area()+ LSQ->local_result.area);
          area.set_area(area.get_area()*cdb_overhead);
          //output_data_csv(LSQ.LSQ.local_result);
          lsq_height=LSQ->local_result.cache_ht*sqrt(cdb_overhead);/*XML->sys.core[ithCore].number_hardware_threads*/

          if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].load_buffer_size >0))
          {
                  interface_ip.line_sz             = int(ceil(data/32.0))*4;
                  interface_ip.specific_tag        = 1;
                  interface_ip.tag_w               = tag;
                  interface_ip.cache_sz            = XML->sys.core[ithCore].load_buffer_size*interface_ip.line_sz*XML->sys.core[ithCore].number_hardware_threads;
                  interface_ip.assoc               = 0;
                  interface_ip.nbanks              = 1;
                  interface_ip.out_w               = interface_ip.line_sz*8;
                  interface_ip.access_mode         = 1;
                  interface_ip.throughput          = 1.0/clockRate;
                  interface_ip.latency             = 1.0/clockRate;
                  interface_ip.obj_func_dyn_energy = 0;
                  interface_ip.obj_func_dyn_power  = 0;
                  interface_ip.obj_func_leak_power = 0;
                  interface_ip.obj_func_cycle_t    = 1;
                  interface_ip.num_rw_ports        = 0;
                  interface_ip.num_rd_ports        = XML->sys.core[ithCore].memory_ports;
                  interface_ip.num_wr_ports        = XML->sys.core[ithCore].memory_ports;
                  interface_ip.num_se_rd_ports     = 0;
                  interface_ip.num_search_ports    =XML->sys.core[ithCore].memory_ports;
                  LoadQ = new ArrayST(&interface_ip, "LoadQueue", Core_device, coredynp.opt_local, coredynp.core_ty);
                  LoadQ->area.set_area(LoadQ->area.get_area()+ LoadQ->local_result.area);
                  area.set_area(area.get_area()+ LoadQ->local_result.area);
                  area.set_area(area.get_area()*cdb_overhead);
                  //output_data_csv(LoadQ.LoadQ.local_result);
                  lsq_height=(LSQ->local_result.cache_ht + LoadQ->local_result.cache_ht)*sqrt(cdb_overhead);/*XML->sys.core[ithCore].number_hardware_threads*/
          }

}

MemManU::MemManU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 itlb(0),
 dtlb(0),
 exist(exist_)
{
          if (!exist) return;
          int  tag, data;
          bool debug= false;

          clockRate = coredynp.clockRate;
          executionTime = coredynp.executionTime;
          interface_ip.is_cache			   = true;
          interface_ip.pure_cam            = false;
          interface_ip.pure_ram            = false;
          interface_ip.specific_tag        = 1;
          //Itlb TLBs are partioned among threads according to Nigara and Nehalem
          tag							   = XML->sys.virtual_address_width- int(floor(log2(XML->sys.virtual_memory_page_size))) + int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads)))+ EXTRA_TAG_BITS;
          data							   = XML->sys.physical_address_width- int(floor(log2(XML->sys.virtual_memory_page_size)));
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
          interface_ip.cache_sz            = XML->sys.core[ithCore].itlb.number_entries*interface_ip.line_sz;//*XML->sys.core[ithCore].number_hardware_threads;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[4]/clockRate;
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].icache.icache_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 0;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;
          interface_ip.num_se_rd_ports = 0;
          interface_ip.num_search_ports    = debug?1:XML->sys.core[ithCore].number_instruction_fetch_ports;
          itlb = new ArrayST(&interface_ip, "ITLB", Core_device, coredynp.opt_local, coredynp.core_ty);
          itlb->area.set_area(itlb->area.get_area()+ itlb->local_result.area);
          area.set_area(area.get_area()+ itlb->local_result.area);
          //output_data_csv(itlb.tlb.local_result);

          //dtlb
          tag							   = XML->sys.virtual_address_width- int(floor(log2(XML->sys.virtual_memory_page_size))) +int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads)))+ EXTRA_TAG_BITS;
          data							   = XML->sys.physical_address_width- int(floor(log2(XML->sys.virtual_memory_page_size)));
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
          interface_ip.cache_sz            = XML->sys.core[ithCore].dtlb.number_entries*interface_ip.line_sz;//*XML->sys.core[ithCore].number_hardware_threads;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[4]/clockRate;
          interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].dcache.dcache_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 0;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = XML->sys.core[ithCore].memory_ports;
          interface_ip.num_se_rd_ports = 0;
          interface_ip.num_search_ports = XML->sys.core[ithCore].memory_ports;
          dtlb = new ArrayST(&interface_ip, "DTLB", Core_device, coredynp.opt_local, coredynp.core_ty);
          dtlb->area.set_area(dtlb->area.get_area()+ dtlb->local_result.area);
          area.set_area(area.get_area()+ dtlb->local_result.area);
          //output_data_csv(dtlb.tlb.local_result);

}

RegFU::RegFU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 IRF (0),
 FRF (0),
 RFWIN (0),
 exist(exist_)
 {
        /*
         * processors have separate architectural register files for each thread.
         * therefore, the bypass buses need to travel across all the register files.
         */
        if (!exist) return;
        int  data;

        clockRate = coredynp.clockRate;
        executionTime = coredynp.executionTime;
        //**********************************IRF***************************************
        data							 = coredynp.int_data_width;
        interface_ip.is_cache			 = false;
        interface_ip.pure_cam            = false;
        interface_ip.pure_ram            = true;
        interface_ip.line_sz             = int(ceil(data/32.0))*4;
        interface_ip.cache_sz            = coredynp.num_IRF_entry*interface_ip.line_sz;
        interface_ip.assoc               = 1;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.access_mode         = 1;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
        interface_ip.num_rd_ports    = 2*coredynp.peak_issueW;
        interface_ip.num_wr_ports    = coredynp.peak_issueW;
        interface_ip.num_se_rd_ports = 0;
        IRF = new ArrayST(&interface_ip, "Integer Register File", Core_device, coredynp.opt_local, coredynp.core_ty);
        IRF->area.set_area(IRF->area.get_area()+ IRF->local_result.area*XML->sys.core[ithCore].number_hardware_threads*coredynp.num_pipelines*cdb_overhead);
        area.set_area(area.get_area()+ IRF->local_result.area*XML->sys.core[ithCore].number_hardware_threads*coredynp.num_pipelines*cdb_overhead);
        //area.set_area(area.get_area()*cdb_overhead);
        //output_data_csv(IRF.RF.local_result);

        //**********************************FRF***************************************
        data							 = coredynp.fp_data_width;
        interface_ip.is_cache			 = false;
        interface_ip.pure_cam            = false;
        interface_ip.pure_ram            = true;
        interface_ip.line_sz             = int(ceil(data/32.0))*4;
        interface_ip.cache_sz            = coredynp.num_FRF_entry*interface_ip.line_sz;
        interface_ip.assoc               = 1;
        interface_ip.nbanks              = 1;
        interface_ip.out_w               = interface_ip.line_sz*8;
        interface_ip.access_mode         = 1;
        interface_ip.throughput          = 1.0/clockRate;
        interface_ip.latency             = 1.0/clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power  = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t    = 1;
        interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
        interface_ip.num_rd_ports    = 2*XML->sys.core[ithCore].issue_width;
        interface_ip.num_wr_ports    = XML->sys.core[ithCore].issue_width;
        interface_ip.num_se_rd_ports = 0;
        FRF = new ArrayST(&interface_ip, "Floating point Register File", Core_device, coredynp.opt_local, coredynp.core_ty);
        FRF->area.set_area(FRF->area.get_area()+ FRF->local_result.area*XML->sys.core[ithCore].number_hardware_threads*coredynp.num_fp_pipelines*cdb_overhead);
        area.set_area(area.get_area()+ FRF->local_result.area*XML->sys.core[ithCore].number_hardware_threads*coredynp.num_fp_pipelines*cdb_overhead);
        //area.set_area(area.get_area()*cdb_overhead);
        //output_data_csv(FRF.RF.local_result);
        int_regfile_height= IRF->local_result.cache_ht*XML->sys.core[ithCore].number_hardware_threads*sqrt(cdb_overhead);
        fp_regfile_height = FRF->local_result.cache_ht*XML->sys.core[ithCore].number_hardware_threads*sqrt(cdb_overhead);
    //since a EXU is associated with each pipeline, the cdb should not have longer length.
        if (coredynp.regWindowing)
        {
                //*********************************REG_WIN************************************
                data							 = coredynp.int_data_width; //ECC, and usually 2 regs are transfered together during window shifting.Niagara Mega cell
                interface_ip.is_cache			 = false;
                interface_ip.pure_cam            = false;
                interface_ip.pure_ram            = true;
                interface_ip.line_sz             = int(ceil(data/8.0));
                interface_ip.cache_sz            = XML->sys.core[ithCore].register_windows_size*IRF->l_ip.cache_sz*XML->sys.core[ithCore].number_hardware_threads;
                interface_ip.assoc               = 1;
                interface_ip.nbanks              = 1;
                interface_ip.out_w               = interface_ip.line_sz*8;
                interface_ip.access_mode         = 1;
                interface_ip.throughput          = 4.0/clockRate;
                interface_ip.latency             = 4.0/clockRate;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power  = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t    = 1;
                interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
                interface_ip.num_rd_ports    = 0;
                interface_ip.num_wr_ports    = 0;
                interface_ip.num_se_rd_ports = 0;
                RFWIN = new ArrayST(&interface_ip, "RegWindow", Core_device, coredynp.opt_local, coredynp.core_ty);
                RFWIN->area.set_area(RFWIN->area.get_area()+ RFWIN->local_result.area*coredynp.num_pipelines);
                area.set_area(area.get_area()+ RFWIN->local_result.area*coredynp.num_pipelines);
                //output_data_csv(RFWIN.RF.local_result);
        }


 }

EXECU::EXECU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, double lsq_height_, const CoreDynParam & dyn_p_, bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 lsq_height(lsq_height_),
 coredynp(dyn_p_),
 rfu(0),
 scheu(0),
 fp_u(0),
 exeu(0),
 mul(0),
 int_bypass(0),
 intTagBypass(0),
 int_mul_bypass(0),
 intTag_mul_Bypass(0),
 fp_bypass(0),
 fpTagBypass(0),
 exist(exist_)
{
          if (!exist) return;
          double fu_height = 0.0;
      clockRate = coredynp.clockRate;
      executionTime = coredynp.executionTime;
          rfu   = new RegFU(XML, ithCore, &interface_ip,coredynp);
          scheu = new SchedulerU(XML, ithCore, &interface_ip,coredynp);
          exeu  = new FunctionalUnit(XML, ithCore,&interface_ip, coredynp, ALU);
          area.set_area(area.get_area()+ exeu->area.get_area() + rfu->area.get_area() +scheu->area.get_area() );
          fu_height = exeu->FU_height;
          if (coredynp.num_fpus >0)
          {
                  fp_u  = new FunctionalUnit(XML, ithCore,&interface_ip, coredynp, FPU);
                  area.set_area(area.get_area()+ fp_u->area.get_area());
          }
          if (coredynp.num_muls >0)
          {
                  mul   = new FunctionalUnit(XML, ithCore,&interface_ip, coredynp, MUL);
                  area.set_area(area.get_area()+ mul->area.get_area());
                  fu_height +=  mul->FU_height;
          }
          /*
           * broadcast logic, including int-broadcast; int_tag-broadcast; fp-broadcast; fp_tag-broadcast
           * integer by pass has two paths and fp has 3 paths.
           * on the same bus there are multiple tri-state drivers and muxes that go to different components on the same bus
           */
          if (XML->sys.Embedded)
                        {
                        interface_ip.wt                  =Global_30;
                        interface_ip.wire_is_mat_type = 0;
                        interface_ip.wire_os_mat_type = 0;
                    interface_ip.throughput       = 1.0/clockRate;
                    interface_ip.latency          = 1.0/clockRate;
                        }
                else
                        {
                        interface_ip.wt                  =Global;
                        interface_ip.wire_is_mat_type = 2;//start from semi-global since local wires are already used
                        interface_ip.wire_os_mat_type = 2;
                    interface_ip.throughput       = 10.0/clockRate; //Do not care
                    interface_ip.latency          = 10.0/clockRate;
                        }

          if (coredynp.core_ty==Inorder)
          {
                  int_bypass   = new interconnect("Int Bypass Data", Core_device, 1, 1, int(ceil(XML->sys.machine_bits/32.0)*32),
                                  rfu->int_regfile_height + exeu->FU_height + lsq_height, &interface_ip, 3,
                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                  bypass.area.set_area(bypass.area.get_area() + int_bypass->area.get_area());
                  intTagBypass = new interconnect("Int Bypass tag" , Core_device, 1, 1, coredynp.perThreadState,
                                  rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height, &interface_ip, 3,
                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                  bypass.area.set_area(bypass.area.get_area()  +intTagBypass->area.get_area());

                  if (coredynp.num_muls>0)
                  {
                          int_mul_bypass     = new interconnect("Mul Bypass Data" , Core_device, 1, 1, int(ceil(XML->sys.machine_bits/32.0)*32*1.5),
                                          rfu->fp_regfile_height + exeu->FU_height + mul->FU_height + lsq_height, &interface_ip, 3,
                                          false, 1.0, coredynp.opt_local, coredynp.core_ty);
                          bypass.area.set_area(bypass.area.get_area()  +int_mul_bypass->area.get_area());
                          intTag_mul_Bypass  = new interconnect("Mul Bypass tag"  , Core_device, 1, 1, coredynp.perThreadState,
                                          rfu->fp_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height, &interface_ip, 3,
                                          false, 1.0, coredynp.opt_local, coredynp.core_ty);
                          bypass.area.set_area(bypass.area.get_area()  +intTag_mul_Bypass->area.get_area());
                  }

                  if (coredynp.num_fpus>0)
                  {
                          fp_bypass    = new interconnect("FP Bypass Data" , Core_device, 1, 1, int(ceil(XML->sys.machine_bits/32.0)*32*1.5),
                                          rfu->fp_regfile_height + fp_u->FU_height, &interface_ip, 3,
                                          false, 1.0, coredynp.opt_local, coredynp.core_ty);
                          bypass.area.set_area(bypass.area.get_area()  +fp_bypass->area.get_area());
                          fpTagBypass  = new interconnect("FP Bypass tag"  , Core_device, 1, 1, coredynp.perThreadState,
                                          rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->Iw_height, &interface_ip, 3,
                                          false, 1.0, coredynp.opt_local, coredynp.core_ty);
                          bypass.area.set_area(bypass.area.get_area()  +fpTagBypass->area.get_area());
                  }
          }
          else
          {//OOO
                  if (coredynp.scheu_ty==PhysicalRegFile)
                  {
                          /* For physical register based OOO,
                           * data broadcast interconnects cover across functional units, lsq, inst windows and register files,
                           * while tag broadcast interconnects also cover across ROB
                           */
                          int_bypass   = new interconnect("Int Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
                                                    rfu->int_regfile_height + exeu->FU_height + lsq_height, &interface_ip, 3,
                                                                false, 1.0, coredynp.opt_local, coredynp.core_ty);
                          bypass.area.set_area(bypass.area.get_area()  +int_bypass->area.get_area());
                          intTagBypass = new interconnect("Int Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
                                                    rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
                                                                false, 1.0, coredynp.opt_local, coredynp.core_ty);

                          if (coredynp.num_muls>0)
                          {
                                  int_mul_bypass   = new interconnect("Mul Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
                                                                                rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height, &interface_ip, 3,
                                                                                false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  intTag_mul_Bypass = new interconnect("Mul Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
                                                                                rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
                                                                                false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  bypass.area.set_area(bypass.area.get_area()  +int_mul_bypass->area.get_area());
                                  bypass.area.set_area(bypass.area.get_area()  +intTag_mul_Bypass->area.get_area());
                          }

                          if (coredynp.num_fpus>0)
                          {
                                  fp_bypass    = new interconnect("FP Bypass Data" , Core_device, 1, 1, int(ceil(coredynp.fp_data_width)),
                                                                  rfu->fp_regfile_height + fp_u->FU_height, &interface_ip, 3,
                                                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  fpTagBypass  = new interconnect("FP Bypass tag"  , Core_device, 1, 1, coredynp.phy_freg_width,
                                                                  rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->fp_Iw_height + scheu->ROB_height, &interface_ip, 3,
                                                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  bypass.area.set_area(bypass.area.get_area()  +fp_bypass->area.get_area());
                                  bypass.area.set_area(bypass.area.get_area()  +fpTagBypass->area.get_area());
                          }
                  }
                  else
                  {
             /*
              * In RS based processor both data and tag are broadcast together,
              * covering functional units, lsq, nst windows, register files, and ROBs
              */
                          int_bypass   = new interconnect("Int Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
                                                    rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height, &interface_ip, 3,
                                                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                          intTagBypass = new interconnect("Int Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
                                                    rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
                                                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                          bypass.area.set_area(bypass.area.get_area() +int_bypass->area.get_area());
                          bypass.area.set_area(bypass.area.get_area() +intTagBypass->area.get_area());
                          if (coredynp.num_muls>0)
                          {
                                  int_mul_bypass   = new interconnect("Mul Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
                                                            rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height, &interface_ip, 3,
                                                                          false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  intTag_mul_Bypass = new interconnect("Mul Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
                                                            rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
                                                                          false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  bypass.area.set_area(bypass.area.get_area() +int_mul_bypass->area.get_area());
                                  bypass.area.set_area(bypass.area.get_area() +intTag_mul_Bypass->area.get_area());
                          }

                          if (coredynp.num_fpus>0)
                          {
                                  fp_bypass    = new interconnect("FP Bypass Data" , Core_device, 1, 1, int(ceil(coredynp.fp_data_width)),
                                                  rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->fp_Iw_height + scheu->ROB_height, &interface_ip, 3,
                                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  fpTagBypass  = new interconnect("FP Bypass tag"  , Core_device, 1, 1, coredynp.phy_freg_width,
                                                  rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->fp_Iw_height + scheu->ROB_height, &interface_ip, 3,
                                                  false, 1.0, coredynp.opt_local, coredynp.core_ty);
                                  bypass.area.set_area(bypass.area.get_area() +fp_bypass->area.get_area());
                                  bypass.area.set_area(bypass.area.get_area() +fpTagBypass->area.get_area());
                          }
                  }


          }
          area.set_area(area.get_area()+ bypass.area.get_area());
}

RENAMINGU::RENAMINGU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 iFRAT(0),
 fFRAT(0),
 iRRAT(0),
 fRRAT(0),
 ifreeL(0),
 ffreeL(0),
 idcl(0),
 fdcl(0),
 RAHT(0),
 exist(exist_)
 {
        /*
         * Although renaming logic maybe be used in in-order processors,
     * McPAT assumes no renaming logic is used since the performance gain is very limited and
     * the only major inorder processor with renaming logic is Itainium
     * that is a VLIW processor and different from current McPAT's model.
         * physical register base OOO must have Dual-RAT architecture or equivalent structure.FRAT:FrontRAT, RRAT:RetireRAT;
         * i,f prefix mean int and fp
         * RAT for all Renaming logic, random accessible checkpointing is used, but only update when instruction retires.
         * FRAT will be read twice and written once per instruction;
         * RRAT will be write once per instruction when committing and reads out all when context switch
         * checkpointing is implicit
         * Renaming logic is duplicated for each different hardware threads
         *
         * No Dual-RAT is needed in RS-based OOO processors,
         * however, RAT needs to do associative search in RAT, when instruction commits and ROB release the entry,
         * to make sure all the renamings associated with the ROB to be released are updated at the same time.
         * RAM scheme has # ARchi Reg entry with each entry hold phy reg tag,
         * CAM scheme has # Phy Reg entry with each entry hold ARchi reg tag,
         *
         * Both RAM and CAM have same DCL
         */
        if (!exist) return;
        int  tag, data, out_w;
//	interface_ip.wire_is_mat_type = 0;
//	interface_ip.wire_os_mat_type = 0;
//	interface_ip.wt               = Global_30;
        clockRate = coredynp.clockRate;
        executionTime = coredynp.executionTime;
    if (coredynp.core_ty==OOO)
    {
        //integer pipeline
        if (coredynp.scheu_ty==PhysicalRegFile)
        {
                if (coredynp.rm_ty ==RAMbased)
                {	  //FRAT with global checkpointing (GCs) please see paper tech report for detailed explaintions
                        data							 = 33;//int(ceil(coredynp.phy_ireg_width*(1+coredynp.globalCheckpoint)/8.0));
//			data							 = int(ceil(coredynp.phy_ireg_width/8.0));
                        out_w                            = 1;//int(ceil(coredynp.phy_ireg_width/8.0));
                        interface_ip.is_cache			 = false;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = true;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].archi_Regs_IRF_size;
                        interface_ip.assoc               = 1;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//the extra one port is for GCs
                        interface_ip.num_rd_ports    = 2*coredynp.decodeW;
                        interface_ip.num_wr_ports    = coredynp.decodeW;
                        interface_ip.num_se_rd_ports = 0;
                        iFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        iFRAT->area.set_area(iFRAT->area.get_area()+ iFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ iFRAT->area.get_area());

//			//RAHT According to Intel, combine GC with FRAT is very costly.
//			data							 = int(ceil(coredynp.phy_ireg_width/8.0)*coredynp.num_IRF_entry);
//			out_w                            = data;
//			interface_ip.is_cache			 = false;
//			interface_ip.pure_cam            = false;
//			interface_ip.pure_ram            = true;
//			interface_ip.line_sz             = data;
//			interface_ip.cache_sz            = data*coredynp.globalCheckpoint;
//			interface_ip.assoc               = 1;
//			interface_ip.nbanks              = 1;
//			interface_ip.out_w               = out_w*8;
//			interface_ip.access_mode         = 0;
//			interface_ip.throughput          = 1.0/clockRate;
//			interface_ip.latency             = 1.0/clockRate;
//			interface_ip.obj_func_dyn_energy = 0;
//			interface_ip.obj_func_dyn_power  = 0;
//			interface_ip.obj_func_leak_power = 0;
//			interface_ip.obj_func_cycle_t    = 1;
//			interface_ip.num_rw_ports    = 1;//the extra one port is for GCs
//			interface_ip.num_rd_ports    = 2*coredynp.decodeW;
//			interface_ip.num_wr_ports    = coredynp.decodeW;
//			interface_ip.num_se_rd_ports = 0;
//			iFRAT = new ArrayST(&interface_ip, "Int FrontRAT");
//			iFRAT->area.set_area(iFRAT->area.get_area()+ iFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
//			area.set_area(area.get_area()+ iFRAT->area.get_area());

                        //FRAT floating point
                        data							 = int(ceil(coredynp.phy_freg_width*(1+coredynp.globalCheckpoint)/8.0));
                        out_w                            = int(ceil(coredynp.phy_freg_width/8.0));
                        interface_ip.is_cache			 = false;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = true;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].archi_Regs_FRF_size;
                        interface_ip.assoc               = 1;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//the extra one port is for GCs
                        interface_ip.num_rd_ports    = 2*coredynp.fp_decodeW;
                        interface_ip.num_wr_ports    = coredynp.fp_decodeW;
                        interface_ip.num_se_rd_ports = 0;
                        fFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        fFRAT->area.set_area(fFRAT->area.get_area()+ fFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ fFRAT->area.get_area());

                }
                else if ((coredynp.rm_ty ==CAMbased))
                {
                        //FRAT
                        tag							     = coredynp.arch_ireg_width;
                        data							 = int(ceil ((coredynp.arch_ireg_width+1*coredynp.globalCheckpoint)/8.0));//the address of CAM needed to be sent out
                        out_w                            = int(ceil (coredynp.arch_ireg_width/8.0));
                        interface_ip.is_cache			 = true;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = false;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].phy_Regs_IRF_size;
                        interface_ip.assoc               = 0;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.specific_tag        = 1;
                        interface_ip.tag_w               = tag;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//for GCs
                        interface_ip.num_rd_ports    = coredynp.decodeW;
                        interface_ip.num_wr_ports    = coredynp.decodeW;
                        interface_ip.num_se_rd_ports = 0;
                        interface_ip.num_search_ports= 2*coredynp.decodeW;
                        iFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        iFRAT->area.set_area(iFRAT->area.get_area()+ iFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ iFRAT->area.get_area());

                        //FRAT for FP
                        tag							     = coredynp.arch_freg_width;
                        data							 = int(ceil ((coredynp.arch_freg_width+1*coredynp.globalCheckpoint)/8.0));//the address of CAM needed to be sent out
                        out_w                            = int(ceil (coredynp.arch_freg_width/8.0));
                        interface_ip.is_cache			 = true;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = false;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].phy_Regs_FRF_size;
                        interface_ip.assoc               = 0;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.specific_tag        = 1;
                        interface_ip.tag_w               = tag;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//for GCs
                        interface_ip.num_rd_ports    = coredynp.fp_decodeW;
                        interface_ip.num_wr_ports    = coredynp.fp_decodeW;
                        interface_ip.num_se_rd_ports = 0;
                        interface_ip.num_search_ports= 2*coredynp.fp_decodeW;
                        fFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        fFRAT->area.set_area(fFRAT->area.get_area()+ fFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ fFRAT->area.get_area());

                }

                //RRAT is always RAM based, does not have GCs, and is used only for record latest non-speculative mapping
                data							 = int(ceil(coredynp.phy_ireg_width/8.0));
                interface_ip.is_cache			 = false;
                interface_ip.pure_cam            = false;
                interface_ip.pure_ram            = true;
                interface_ip.line_sz             = data;
                interface_ip.cache_sz            = data*XML->sys.core[ithCore].archi_Regs_IRF_size*2;//HACK to make it as least 64B
                interface_ip.assoc               = 1;
                interface_ip.nbanks              = 1;
                interface_ip.out_w               = interface_ip.line_sz*8;
                interface_ip.access_mode         = 1;
                interface_ip.throughput          = 1.0/clockRate;
                interface_ip.latency             = 1.0/clockRate;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power  = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t    = 1;
                interface_ip.num_rw_ports    = 0;
                interface_ip.num_rd_ports    = XML->sys.core[ithCore].commit_width;
                interface_ip.num_wr_ports    = XML->sys.core[ithCore].commit_width;
                interface_ip.num_se_rd_ports = 0;
                iRRAT = new ArrayST(&interface_ip, "Int RetireRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                iRRAT->area.set_area(iRRAT->area.get_area()+ iRRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                area.set_area(area.get_area()+ iRRAT->area.get_area());

                //RRAT for FP
                data							 = int(ceil(coredynp.phy_freg_width/8.0));
                interface_ip.is_cache			 = false;
                interface_ip.pure_cam            = false;
                interface_ip.pure_ram            = true;
                interface_ip.line_sz             = data;
                interface_ip.cache_sz            = data*XML->sys.core[ithCore].archi_Regs_FRF_size*2;//HACK to make it as least 64B
                interface_ip.assoc               = 1;
                interface_ip.nbanks              = 1;
                interface_ip.out_w               = interface_ip.line_sz*8;
                interface_ip.access_mode         = 1;
                interface_ip.throughput          = 1.0/clockRate;
                interface_ip.latency             = 1.0/clockRate;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power  = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t    = 1;
                interface_ip.num_rw_ports    = 0;
                interface_ip.num_rd_ports    = coredynp.fp_decodeW;
                interface_ip.num_wr_ports    = coredynp.fp_decodeW;
                interface_ip.num_se_rd_ports = 0;
                fRRAT = new ArrayST(&interface_ip, "Int RetireRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                fRRAT->area.set_area(fRRAT->area.get_area()+ fRRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                area.set_area(area.get_area()+ fRRAT->area.get_area());

                //Freelist of renaming unit always RAM based
                //Recycle happens at two places: 1)when DCL check there are WAW, the Phyregisters/ROB directly recycles into freelist
                // 2)When instruction commits the Phyregisters/ROB needed to be recycled.
                //therefore num_wr port = decode-1(-1 means at least one phy reg will be used for the current renaming group) + commit width
                data							 = int(ceil(coredynp.phy_ireg_width/8.0));
                interface_ip.is_cache			 = false;
                interface_ip.pure_cam            = false;
                interface_ip.pure_ram            = true;
                interface_ip.line_sz             = data;
                interface_ip.cache_sz            = data*coredynp.num_ifreelist_entries;
                interface_ip.assoc               = 1;
                interface_ip.nbanks              = 1;
                interface_ip.out_w               = interface_ip.line_sz*8;
                interface_ip.access_mode         = 1;
                interface_ip.throughput          = 1.0/clockRate;
                interface_ip.latency             = 1.0/clockRate;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power  = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t    = 1;
                interface_ip.num_rw_ports    = 1;//TODO
                interface_ip.num_rd_ports    = coredynp.decodeW;
                interface_ip.num_wr_ports    = coredynp.decodeW -1 + XML->sys.core[ithCore].commit_width;
                //every cycle, (coredynp.decodeW -1) inst may need to send back it dest tags, committW insts needs to update freelist buffers
                interface_ip.num_se_rd_ports = 0;
                ifreeL = new ArrayST(&interface_ip, "Int Free List", Core_device, coredynp.opt_local, coredynp.core_ty);
                ifreeL->area.set_area(ifreeL->area.get_area()+ ifreeL->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                area.set_area(area.get_area()+ ifreeL->area.get_area());

                //freelist for FP
                data							 = int(ceil(coredynp.phy_freg_width/8.0));
                interface_ip.is_cache			 = false;
                interface_ip.pure_cam            = false;
                interface_ip.pure_ram            = true;
                interface_ip.line_sz             = data;
                interface_ip.cache_sz            = data*coredynp.num_ffreelist_entries;
                interface_ip.assoc               = 1;
                interface_ip.nbanks              = 1;
                interface_ip.out_w               = interface_ip.line_sz*8;
                interface_ip.access_mode         = 1;
                interface_ip.throughput          = 1.0/clockRate;
                interface_ip.latency             = 1.0/clockRate;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power  = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t    = 1;
                interface_ip.num_rw_ports    = 1;
                interface_ip.num_rd_ports    = coredynp.fp_decodeW;
                interface_ip.num_wr_ports    = coredynp.fp_decodeW -1 + XML->sys.core[ithCore].commit_width;
                interface_ip.num_se_rd_ports = 0;
                ffreeL = new ArrayST(&interface_ip, "Int Free List", Core_device, coredynp.opt_local, coredynp.core_ty);
                ffreeL->area.set_area(ffreeL->area.get_area()+ ffreeL->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                area.set_area(area.get_area()+ ffreeL->area.get_area());

                idcl  = new dep_resource_conflict_check(&interface_ip,coredynp,coredynp.phy_ireg_width);//TODO:Separate 2 sections See TR
                fdcl  = new dep_resource_conflict_check(&interface_ip,coredynp,coredynp.phy_freg_width);

        }
        else if (coredynp.scheu_ty==ReservationStation){
                if (coredynp.rm_ty ==RAMbased){
                        /*
                         * however, RAT needs to do associative search in RAT, when instruction commits and ROB release the entry,
                         * to make sure all the renamings associated with the ROB to be released are updated to ARF at the same time.
                         * RAM based RAT for RS base OOO does not save the search operations. Its advantage is to have less entries than
                         * CAM based RAT so that it is more scalable as number of ROB/physical regs increases.
                         */
                        tag							     = coredynp.phy_ireg_width;
                        data							 = int(ceil(coredynp.phy_ireg_width*(1+coredynp.globalCheckpoint)/8.0));
                        out_w                            = int(ceil(coredynp.phy_ireg_width/8.0));
                        interface_ip.is_cache			 = true;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = false;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].archi_Regs_IRF_size;
                        interface_ip.assoc               = 0;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//the extra one port is for GCs
                        interface_ip.num_rd_ports    = 2*coredynp.decodeW;
                        interface_ip.num_wr_ports    = coredynp.decodeW;
                        interface_ip.num_se_rd_ports = 0;
                        interface_ip.num_search_ports= coredynp.commitW;//TODO
                        iFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        iFRAT->local_result.adjust_area();
                        iFRAT->area.set_area(iFRAT->area.get_area()+ iFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ iFRAT->area.get_area());

                        //FP
                        tag							     = coredynp.phy_freg_width;
                        data							 = int(ceil(coredynp.phy_freg_width*(1+coredynp.globalCheckpoint)/8.0));
                        out_w                            = int(ceil(coredynp.phy_freg_width/8.0));
                        interface_ip.is_cache			 = true;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = false;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].archi_Regs_FRF_size;
                        interface_ip.assoc               = 0;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//the extra one port is for GCs
                        interface_ip.num_rd_ports    = 2*coredynp.fp_decodeW;
                        interface_ip.num_wr_ports    = coredynp.fp_decodeW;
                        interface_ip.num_se_rd_ports = 0;
                        interface_ip.num_search_ports= coredynp.fp_decodeW;//actually is fp commit width
                        fFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        fFRAT->local_result.adjust_area();
                        fFRAT->area.set_area(fFRAT->area.get_area()+ fFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ fFRAT->area.get_area());

                }
                else if ((coredynp.rm_ty ==CAMbased))
                {
                        //FRAT
                        tag							     = coredynp.arch_ireg_width;
                        data							 = int(ceil (coredynp.arch_ireg_width+1*coredynp.globalCheckpoint/8.0));//the address of CAM needed to be sent out
                        out_w                            = int(ceil (coredynp.arch_ireg_width/8.0));
                        interface_ip.is_cache			 = true;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = false;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].phy_Regs_IRF_size;
                        interface_ip.assoc               = 0;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.specific_tag        = 1;
                        interface_ip.tag_w               = tag;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//for GCs
                        interface_ip.num_rd_ports    = XML->sys.core[ithCore].decode_width;//0;TODO
                        interface_ip.num_wr_ports    = XML->sys.core[ithCore].decode_width;
                        interface_ip.num_se_rd_ports = 0;
                        interface_ip.num_search_ports= 2*XML->sys.core[ithCore].decode_width;
                        iFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        iFRAT->area.set_area(iFRAT->area.get_area()+ iFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ iFRAT->area.get_area());

                        //FRAT
                        tag							     = coredynp.arch_freg_width;
                        data							 = int(ceil (coredynp.arch_freg_width+1*coredynp.globalCheckpoint/8.0));//the address of CAM needed to be sent out
                        out_w                            = int(ceil (coredynp.arch_freg_width/8.0));
                        interface_ip.is_cache			 = true;
                        interface_ip.pure_cam            = false;
                        interface_ip.pure_ram            = false;
                        interface_ip.line_sz             = data;
                        interface_ip.cache_sz            = data*XML->sys.core[ithCore].phy_Regs_FRF_size;
                        interface_ip.assoc               = 0;
                        interface_ip.nbanks              = 1;
                        interface_ip.out_w               = out_w*8;
                        interface_ip.specific_tag        = 1;
                        interface_ip.tag_w               = tag;
                        interface_ip.access_mode         = 2;
                        interface_ip.throughput          = 1.0/clockRate;
                        interface_ip.latency             = 1.0/clockRate;
                        interface_ip.obj_func_dyn_energy = 0;
                        interface_ip.obj_func_dyn_power  = 0;
                        interface_ip.obj_func_leak_power = 0;
                        interface_ip.obj_func_cycle_t    = 1;
                        interface_ip.num_rw_ports    = 1;//for GCs
                        interface_ip.num_rd_ports    = XML->sys.core[ithCore].decode_width;//0;TODO;
                        interface_ip.num_wr_ports    = coredynp.fp_decodeW;
                        interface_ip.num_se_rd_ports = 0;
                        interface_ip.num_search_ports= 2*coredynp.fp_decodeW;
                        fFRAT = new ArrayST(&interface_ip, "Int FrontRAT", Core_device, coredynp.opt_local, coredynp.core_ty);
                        fFRAT->area.set_area(fFRAT->area.get_area()+ fFRAT->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                        area.set_area(area.get_area()+ fFRAT->area.get_area());

                }
                //No RRAT for RS based OOO
                //Freelist of renaming unit of RS based OOO is unifed for both int and fp renaming unit since the ROB is unified
                data							 = int(ceil(coredynp.phy_ireg_width/8.0));
                interface_ip.is_cache			 = false;
                interface_ip.pure_cam            = false;
                interface_ip.pure_ram            = true;
                interface_ip.line_sz             = data;
                interface_ip.cache_sz            = data*coredynp.num_ifreelist_entries;
                interface_ip.assoc               = 1;
                interface_ip.nbanks              = 1;
                interface_ip.out_w               = interface_ip.line_sz*8;
                interface_ip.access_mode         = 1;
                interface_ip.throughput          = 1.0/clockRate;
                interface_ip.latency             = 1.0/clockRate;
                interface_ip.obj_func_dyn_energy = 0;
                interface_ip.obj_func_dyn_power  = 0;
                interface_ip.obj_func_leak_power = 0;
                interface_ip.obj_func_cycle_t    = 1;
                interface_ip.num_rw_ports    = 1;//TODO
                interface_ip.num_rd_ports    = XML->sys.core[ithCore].decode_width;
                interface_ip.num_wr_ports    = XML->sys.core[ithCore].decode_width -1 + XML->sys.core[ithCore].commit_width;
                interface_ip.num_se_rd_ports = 0;
                ifreeL = new ArrayST(&interface_ip, "Unified Free List", Core_device, coredynp.opt_local, coredynp.core_ty);
                ifreeL->area.set_area(ifreeL->area.get_area()+ ifreeL->local_result.area*XML->sys.core[ithCore].number_hardware_threads);
                area.set_area(area.get_area()+ ifreeL->area.get_area());

                idcl  = new dep_resource_conflict_check(&interface_ip,coredynp,coredynp.phy_ireg_width);//TODO:Separate 2 sections See TR
                fdcl  = new dep_resource_conflict_check(&interface_ip,coredynp,coredynp.phy_freg_width);
        }

}
    if (coredynp.core_ty==Inorder&& coredynp.issueW>1)
    {
          /* Dependency check logic will only present when decode(issue) width>1.
          *  Multiple issue in order processor can do without renaming, but dcl is a must.
          */
        idcl  = new dep_resource_conflict_check(&interface_ip,coredynp,coredynp.phy_ireg_width);//TODO:Separate 2 sections See TR
        fdcl  = new dep_resource_conflict_check(&interface_ip,coredynp,coredynp.phy_freg_width);
    }
}

Core::Core(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 ifu  (0),
 lsu  (0),
 mmu  (0),
 exu  (0),
 rnu  (0),
 corepipe (0),
 undiffCore (0),
 l2cache (0)
{
 /*
  * initialize, compute and optimize individual components.
  */

  double pipeline_area_per_unit;
  if (XML->sys.Private_L2)
  {
          l2cache = new SharedCache(XML,ithCore, &interface_ip);

  }
//  interface_ip.wire_is_mat_type = 2;
//  interface_ip.wire_os_mat_type = 2;
//  interface_ip.wt               =Global_30;
  set_core_param();
  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  ifu          = new InstFetchU(XML, ithCore, &interface_ip,coredynp);
  lsu          = new LoadStoreU(XML, ithCore, &interface_ip,coredynp);
  mmu          = new MemManU   (XML, ithCore, &interface_ip,coredynp);
  exu          = new EXECU     (XML, ithCore, &interface_ip,lsu->lsq_height, coredynp);
  undiffCore   = new UndiffCore(XML, ithCore, &interface_ip,coredynp);
  if (coredynp.core_ty==OOO)
  {
          rnu = new RENAMINGU(XML, ithCore, &interface_ip,coredynp);
  }
  corepipe = new Pipeline(&interface_ip,coredynp);

  if (coredynp.core_ty==OOO)
  {
          pipeline_area_per_unit    = (corepipe->area.get_area()*coredynp.num_pipelines)/5.0;
          if (rnu->exist)
          {
                  rnu->area.set_area(rnu->area.get_area() + pipeline_area_per_unit);
          }
  }
  else {
          pipeline_area_per_unit    = (corepipe->area.get_area()*coredynp.num_pipelines)/4.0;
  }

  //area.set_area(area.get_area()+ corepipe->area.get_area());
  if (ifu->exist)
  {
          ifu->area.set_area(ifu->area.get_area() + pipeline_area_per_unit);
          area.set_area(area.get_area() + ifu->area.get_area());
  }
  if (lsu->exist)
  {
          lsu->area.set_area(lsu->area.get_area() + pipeline_area_per_unit);
      area.set_area(area.get_area() + lsu->area.get_area());
  }
  if (exu->exist)
  {
          exu->area.set_area(exu->area.get_area() + pipeline_area_per_unit);
          area.set_area(area.get_area()+exu->area.get_area());
  }
  if (mmu->exist)
  {
          mmu->area.set_area(mmu->area.get_area() + pipeline_area_per_unit);
      area.set_area(area.get_area()+mmu->area.get_area());
  }

  if (coredynp.core_ty==OOO)
  {
          if (rnu->exist)
          {

                  area.set_area(area.get_area() + rnu->area.get_area());
          }
  }

  if (undiffCore->exist)
  {
          area.set_area(area.get_area() + undiffCore->area.get_area());
  }

  if (XML->sys.Private_L2)
  {
          area.set_area(area.get_area() + l2cache->area.get_area());

  }
//  //clock power
//  clockNetwork.init_wire_external(is_default, &interface_ip);
//  clockNetwork.clk_area           =area*1.1;//10% of placement overhead. rule of thumb
//  clockNetwork.end_wiring_level   =5;//toplevel metal
//  clockNetwork.start_wiring_level =5;//toplevel metal
//  clockNetwork.num_regs           = corepipe.tot_stage_vector;
//  clockNetwork.optimize_wire();
}


void BranchPredictor::computeEnergy(bool is_tdp)
{
        if (!exist) return;
        double r_access;
        double w_access;
        if (is_tdp)
    {
        r_access = coredynp.predictionW*coredynp.BR_duty_cycle;
        w_access = 0*coredynp.BR_duty_cycle;
        globalBPT->stats_t.readAc.access  = r_access;
        globalBPT->stats_t.writeAc.access = w_access;
        globalBPT->tdp_stats = globalBPT->stats_t;

        L1_localBPT->stats_t.readAc.access  = r_access;
        L1_localBPT->stats_t.writeAc.access = w_access;
        L1_localBPT->tdp_stats = L1_localBPT->stats_t;

        L2_localBPT->stats_t.readAc.access  = r_access;
        L2_localBPT->stats_t.writeAc.access = w_access;
        L2_localBPT->tdp_stats = L2_localBPT->stats_t;

        chooser->stats_t.readAc.access  = r_access;
        chooser->stats_t.writeAc.access = w_access;
        chooser->tdp_stats = chooser->stats_t;

        RAS->stats_t.readAc.access  = r_access;
        RAS->stats_t.writeAc.access = w_access;
        RAS->tdp_stats = RAS->stats_t;
    }
    else
    {
        //The resolution of BPT accesses is coarse, but this is
        //because most simulators cannot track finer grained details
        r_access = XML->sys.core[ithCore].branch_instructions;
        w_access = XML->sys.core[ithCore].branch_mispredictions + 0.1*XML->sys.core[ithCore].branch_instructions;//10% of BR will flip internal bits//0
        globalBPT->stats_t.readAc.access  = r_access;
        globalBPT->stats_t.writeAc.access = w_access;
        globalBPT->rtp_stats = globalBPT->stats_t;

        L1_localBPT->stats_t.readAc.access  = r_access;
        L1_localBPT->stats_t.writeAc.access = w_access;
        L1_localBPT->rtp_stats = L1_localBPT->stats_t;

        L2_localBPT->stats_t.readAc.access  = r_access;
        L2_localBPT->stats_t.writeAc.access = w_access;
        L2_localBPT->rtp_stats = L2_localBPT->stats_t;

        chooser->stats_t.readAc.access  = r_access;
        chooser->stats_t.writeAc.access = w_access;
        chooser->rtp_stats = chooser->stats_t;

        RAS->stats_t.readAc.access  = XML->sys.core[ithCore].function_calls;
        RAS->stats_t.writeAc.access = XML->sys.core[ithCore].function_calls;
        RAS->rtp_stats = RAS->stats_t;
   }

        globalBPT->power_t.reset();
        L1_localBPT->power_t.reset();
        L2_localBPT->power_t.reset();
        chooser->power_t.reset();
        RAS->power_t.reset();

    globalBPT->power_t.readOp.dynamic   +=  globalBPT->local_result.power.readOp.dynamic*globalBPT->stats_t.readAc.access +
                globalBPT->stats_t.writeAc.access*globalBPT->local_result.power.writeOp.dynamic;
    L1_localBPT->power_t.readOp.dynamic   +=  L1_localBPT->local_result.power.readOp.dynamic*L1_localBPT->stats_t.readAc.access +
                L1_localBPT->stats_t.writeAc.access*L1_localBPT->local_result.power.writeOp.dynamic;

    L2_localBPT->power_t.readOp.dynamic   +=  L2_localBPT->local_result.power.readOp.dynamic*L2_localBPT->stats_t.readAc.access +
                L2_localBPT->stats_t.writeAc.access*L2_localBPT->local_result.power.writeOp.dynamic;

    chooser->power_t.readOp.dynamic   +=  chooser->local_result.power.readOp.dynamic*chooser->stats_t.readAc.access +
                chooser->stats_t.writeAc.access*chooser->local_result.power.writeOp.dynamic;
    RAS->power_t.readOp.dynamic   +=  RAS->local_result.power.readOp.dynamic*RAS->stats_t.readAc.access +
                RAS->stats_t.writeAc.access*RAS->local_result.power.writeOp.dynamic;

    if (is_tdp)
    {
        globalBPT->power = globalBPT->power_t + globalBPT->local_result.power*pppm_lkg;
        L1_localBPT->power = L1_localBPT->power_t + L1_localBPT->local_result.power*pppm_lkg;
        L2_localBPT->power = L2_localBPT->power_t + L2_localBPT->local_result.power*pppm_lkg;
        chooser->power = chooser->power_t + chooser->local_result.power*pppm_lkg;
        RAS->power = RAS->power_t + RAS->local_result.power*coredynp.pppm_lkg_multhread;

        power = power + globalBPT->power + L1_localBPT->power + chooser->power + RAS->power;
    }
    else
    {
        globalBPT->rt_power = globalBPT->power_t + globalBPT->local_result.power*pppm_lkg;
        L1_localBPT->rt_power = L1_localBPT->power_t + L1_localBPT->local_result.power*pppm_lkg;
        L2_localBPT->rt_power = L2_localBPT->power_t + L2_localBPT->local_result.power*pppm_lkg;
        chooser->rt_power = chooser->power_t + chooser->local_result.power*pppm_lkg;
        RAS->rt_power = RAS->power_t + RAS->local_result.power*coredynp.pppm_lkg_multhread;
        rt_power = rt_power + globalBPT->rt_power + L1_localBPT->rt_power + chooser->rt_power + RAS->rt_power;
    }
}

void BranchPredictor::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;
        if (is_tdp)
        {
                cout << indent_str<< "Global Predictor:" << endl;
                cout << indent_str_next << "Area = " << globalBPT->area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << globalBPT->power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? globalBPT->power.readOp.longer_channel_leakage:globalBPT->power.readOp.leakage) <<" W" << endl;
                cout << indent_str_next << "Gate Leakage = " << globalBPT->power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << globalBPT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                cout << indent_str << "Local Predictor:" << endl;
                cout << indent_str << "L1_Local Predictor:" << endl;
                cout << indent_str_next << "Area = " << L1_localBPT->area.get_area() *1e-6 << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << L1_localBPT->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? L1_localBPT->power.readOp.longer_channel_leakage:L1_localBPT->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << L1_localBPT->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << L1_localBPT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                cout << indent_str << "L2_Local Predictor:" << endl;
                cout << indent_str_next << "Area = " << L2_localBPT->area.get_area() *1e-6 << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << L2_localBPT->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? L2_localBPT->power.readOp.longer_channel_leakage:L2_localBPT->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << L2_localBPT->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << L2_localBPT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;

                cout << indent_str << "Chooser:" << endl;
                cout << indent_str_next << "Area = " << chooser->area.get_area()  *1e-6 << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << chooser->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? chooser->power.readOp.longer_channel_leakage:chooser->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << chooser->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << chooser->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                cout << indent_str << "RAS:" << endl;
                cout << indent_str_next << "Area = " << RAS->area.get_area() *1e-6 << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << RAS->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? RAS->power.readOp.longer_channel_leakage:RAS->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << RAS->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << RAS->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
        }
        else
        {
//		cout << indent_str_next << "Global Predictor    Peak Dynamic = " << globalBPT->rt_power.readOp.dynamic*clockRate << " W" << endl;
//		cout << indent_str_next << "Global Predictor    Subthreshold Leakage = " << globalBPT->rt_power.readOp.leakage <<" W" << endl;
//		cout << indent_str_next << "Global Predictor    Gate Leakage = " << globalBPT->rt_power.readOp.gate_leakage << " W" << endl;
//		cout << indent_str_next << "Local Predictor   Peak Dynamic = " << L1_localBPT->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Local Predictor   Subthreshold Leakage = " << L1_localBPT->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Local Predictor   Gate Leakage = " << L1_localBPT->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "Chooser   Peak Dynamic = " << chooser->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Chooser   Subthreshold Leakage = " << chooser->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Chooser   Gate Leakage = " << chooser->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "RAS   Peak Dynamic = " << RAS->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "RAS   Subthreshold Leakage = " << RAS->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "RAS   Gate Leakage = " << RAS->rt_power.readOp.gate_leakage  << " W" << endl;
        }

}

void InstFetchU::computeEnergy(bool is_tdp)
{
        if (!exist) return;
        if (is_tdp)
    {
                //init stats for Peak
        icache.caches->stats_t.readAc.access  = icache.caches->l_ip.num_rw_ports*coredynp.IFU_duty_cycle;
        icache.caches->stats_t.readAc.miss    = 0;
        icache.caches->stats_t.readAc.hit     = icache.caches->stats_t.readAc.access - icache.caches->stats_t.readAc.miss;
        icache.caches->tdp_stats = icache.caches->stats_t;

        icache.missb->stats_t.readAc.access  = icache.missb->stats_t.readAc.hit=  icache.missb->l_ip.num_search_ports;
        icache.missb->stats_t.writeAc.access = icache.missb->stats_t.writeAc.hit= icache.missb->l_ip.num_search_ports;
        icache.missb->tdp_stats = icache.missb->stats_t;

        icache.ifb->stats_t.readAc.access  = icache.ifb->stats_t.readAc.hit=  icache.ifb->l_ip.num_search_ports;
        icache.ifb->stats_t.writeAc.access = icache.ifb->stats_t.writeAc.hit= icache.ifb->l_ip.num_search_ports;
        icache.ifb->tdp_stats = icache.ifb->stats_t;

        icache.prefetchb->stats_t.readAc.access  = icache.prefetchb->stats_t.readAc.hit= icache.prefetchb->l_ip.num_search_ports;
        icache.prefetchb->stats_t.writeAc.access = icache.ifb->stats_t.writeAc.hit= icache.ifb->l_ip.num_search_ports;
        icache.prefetchb->tdp_stats = icache.prefetchb->stats_t;

        IB->stats_t.readAc.access = IB->stats_t.writeAc.access = XML->sys.core[ithCore].peak_issue_width;
        IB->tdp_stats = IB->stats_t;

        if (coredynp.predictionW>0)
        {
                BTB->stats_t.readAc.access  = coredynp.predictionW;//XML->sys.core[ithCore].BTB.read_accesses;
                BTB->stats_t.writeAc.access = 0;//XML->sys.core[ithCore].BTB.write_accesses;
        }

        ID_inst->stats_t.readAc.access     = coredynp.decodeW;
        ID_operand->stats_t.readAc.access  = coredynp.decodeW;
        ID_misc->stats_t.readAc.access     = coredynp.decodeW;
        ID_inst->tdp_stats = ID_inst->stats_t;
        ID_operand->tdp_stats = ID_operand->stats_t;
        ID_misc->tdp_stats = ID_misc->stats_t;


    }
    else
    {
        //init stats for Runtime Dynamic (RTP)
        icache.caches->stats_t.readAc.access  = XML->sys.core[ithCore].icache.read_accesses;
        icache.caches->stats_t.readAc.miss    = XML->sys.core[ithCore].icache.read_misses;
        icache.caches->stats_t.readAc.hit     = icache.caches->stats_t.readAc.access - icache.caches->stats_t.readAc.miss;
        icache.caches->rtp_stats = icache.caches->stats_t;

        icache.missb->stats_t.readAc.access  = icache.caches->stats_t.readAc.miss;
        icache.missb->stats_t.writeAc.access = icache.caches->stats_t.readAc.miss;
        icache.missb->rtp_stats = icache.missb->stats_t;

        icache.ifb->stats_t.readAc.access  = icache.caches->stats_t.readAc.miss;
        icache.ifb->stats_t.writeAc.access = icache.caches->stats_t.readAc.miss;
        icache.ifb->rtp_stats = icache.ifb->stats_t;

        icache.prefetchb->stats_t.readAc.access  = icache.caches->stats_t.readAc.miss;
        icache.prefetchb->stats_t.writeAc.access = icache.caches->stats_t.readAc.miss;
        icache.prefetchb->rtp_stats = icache.prefetchb->stats_t;

        IB->stats_t.readAc.access = IB->stats_t.writeAc.access = XML->sys.core[ithCore].total_instructions;
        IB->rtp_stats = IB->stats_t;

        if (coredynp.predictionW>0)
        {
                BTB->stats_t.readAc.access  = XML->sys.core[ithCore].BTB.read_accesses;//XML->sys.core[ithCore].branch_instructions;
                BTB->stats_t.writeAc.access = XML->sys.core[ithCore].BTB.write_accesses;//XML->sys.core[ithCore].branch_mispredictions;
                BTB->rtp_stats = BTB->stats_t;
        }

        ID_inst->stats_t.readAc.access     = XML->sys.core[ithCore].total_instructions;
        ID_operand->stats_t.readAc.access  = XML->sys.core[ithCore].total_instructions;
        ID_misc->stats_t.readAc.access     = XML->sys.core[ithCore].total_instructions;
        ID_inst->rtp_stats = ID_inst->stats_t;
        ID_operand->rtp_stats = ID_operand->stats_t;
        ID_misc->rtp_stats = ID_misc->stats_t;

    }

    icache.power_t.reset();
    IB->power_t.reset();
//	ID_inst->power_t.reset();
//	ID_operand->power_t.reset();
//	ID_misc->power_t.reset();
    if (coredynp.predictionW>0)
    {
        BTB->power_t.reset();
    }

    icache.power_t.readOp.dynamic	+= (icache.caches->stats_t.readAc.hit*icache.caches->local_result.power.readOp.dynamic+
                //icache.caches->stats_t.readAc.miss*icache.caches->local_result.tag_array2->power.readOp.dynamic+
                icache.caches->stats_t.readAc.miss*icache.caches->local_result.power.readOp.dynamic+ //assume tag data accessed in parallel
                icache.caches->stats_t.readAc.miss*icache.caches->local_result.power.writeOp.dynamic); //read miss in Icache cause a write to Icache
    icache.power_t.readOp.dynamic	+=  icache.missb->stats_t.readAc.access*icache.missb->local_result.power.searchOp.dynamic +
            icache.missb->stats_t.writeAc.access*icache.missb->local_result.power.writeOp.dynamic;//each access to missb involves a CAM and a write
    icache.power_t.readOp.dynamic	+=  icache.ifb->stats_t.readAc.access*icache.ifb->local_result.power.searchOp.dynamic +
            icache.ifb->stats_t.writeAc.access*icache.ifb->local_result.power.writeOp.dynamic;
    icache.power_t.readOp.dynamic	+=  icache.prefetchb->stats_t.readAc.access*icache.prefetchb->local_result.power.searchOp.dynamic +
            icache.prefetchb->stats_t.writeAc.access*icache.prefetchb->local_result.power.writeOp.dynamic;

        IB->power_t.readOp.dynamic   +=  IB->local_result.power.readOp.dynamic*IB->stats_t.readAc.access +
                        IB->stats_t.writeAc.access*IB->local_result.power.writeOp.dynamic;

        if (coredynp.predictionW>0)
        {
                BTB->power_t.readOp.dynamic   +=  BTB->local_result.power.readOp.dynamic*BTB->stats_t.readAc.access +
                BTB->stats_t.writeAc.access*BTB->local_result.power.writeOp.dynamic;

                BPT->computeEnergy(is_tdp);
        }

    if (is_tdp)
    {
//    	icache.power = icache.power_t +
//    	        (icache.caches->local_result.power)*pppm_lkg +
//    			(icache.missb->local_result.power +
//    			icache.ifb->local_result.power +
//    			icache.prefetchb->local_result.power)*pppm_Isub;
        icache.power = icache.power_t +
                (icache.caches->local_result.power +
                        icache.missb->local_result.power +
                        icache.ifb->local_result.power +
                        icache.prefetchb->local_result.power)*pppm_lkg;

        IB->power = IB->power_t + IB->local_result.power*pppm_lkg;
        power     = power + icache.power + IB->power;
        if (coredynp.predictionW>0)
        {
                BTB->power = BTB->power_t + BTB->local_result.power*pppm_lkg;
                power     = power  + BTB->power + BPT->power;
        }

        ID_inst->power_t.readOp.dynamic    = ID_inst->power.readOp.dynamic;
        ID_operand->power_t.readOp.dynamic = ID_operand->power.readOp.dynamic;
        ID_misc->power_t.readOp.dynamic    = ID_misc->power.readOp.dynamic;

        ID_inst->power.readOp.dynamic    *= ID_inst->tdp_stats.readAc.access;
        ID_operand->power.readOp.dynamic *= ID_operand->tdp_stats.readAc.access;
        ID_misc->power.readOp.dynamic    *= ID_misc->tdp_stats.readAc.access;

        power = power + (ID_inst->power +
                                                        ID_operand->power +
                                                        ID_misc->power);
    }
    else
    {
//    	icache.rt_power = icache.power_t +
//    	        (icache.caches->local_result.power)*pppm_lkg +
//    			(icache.missb->local_result.power +
//    			icache.ifb->local_result.power +
//    			icache.prefetchb->local_result.power)*pppm_Isub;

        icache.rt_power = icache.power_t +
                (icache.caches->local_result.power +
                        icache.missb->local_result.power +
                        icache.ifb->local_result.power +
                        icache.prefetchb->local_result.power)*pppm_lkg;

        IB->rt_power = IB->power_t + IB->local_result.power*pppm_lkg;
        rt_power     = rt_power + icache.rt_power + IB->rt_power;
        if (coredynp.predictionW>0)
        {
                BTB->rt_power = BTB->power_t + BTB->local_result.power*pppm_lkg;
                rt_power     = rt_power + BTB->rt_power + BPT->rt_power;
        }

        ID_inst->rt_power.readOp.dynamic    = ID_inst->power_t.readOp.dynamic*ID_inst->rtp_stats.readAc.access;
        ID_operand->rt_power.readOp.dynamic = ID_operand->power_t.readOp.dynamic * ID_operand->rtp_stats.readAc.access;
        ID_misc->rt_power.readOp.dynamic    = ID_misc->power_t.readOp.dynamic * ID_misc->rtp_stats.readAc.access;

        rt_power = rt_power + (ID_inst->rt_power +
                                                        ID_operand->rt_power +
                                                        ID_misc->rt_power);
    }
}

void InstFetchU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;


        if (is_tdp)
        {

                cout << indent_str<< "Instruction Cache:" << endl;
                cout << indent_str_next << "Area = " << icache.area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << icache.power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? icache.power.readOp.longer_channel_leakage:icache.power.readOp.leakage) <<" W" << endl;
                cout << indent_str_next << "Gate Leakage = " << icache.power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << icache.rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                if (coredynp.predictionW>0)
                {
                        cout << indent_str<< "Branch Target Buffer:" << endl;
                        cout << indent_str_next << "Area = " << BTB->area.get_area() *1e-6 << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << BTB->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? BTB->power.readOp.longer_channel_leakage:BTB->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << BTB->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << BTB->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        if (BPT->exist)
                        {
                                cout << indent_str<< "Branch Predictor:" << endl;
                                cout << indent_str_next << "Area = " << BPT->area.get_area()  *1e-6<< " mm^2" << endl;
                                cout << indent_str_next << "Peak Dynamic = " << BPT->power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Subthreshold Leakage = "
                                        << (long_channel? BPT->power.readOp.longer_channel_leakage:BPT->power.readOp.leakage)  << " W" << endl;
                                cout << indent_str_next << "Gate Leakage = " << BPT->power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "Runtime Dynamic = " << BPT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                                cout <<endl;
                                if (plevel>3)
                                {
                                        BPT->displayEnergy(indent+4, plevel, is_tdp);
                                }
                        }
                }
                cout << indent_str<< "Instruction Buffer:" << endl;
                cout << indent_str_next << "Area = " << IB->area.get_area()*1e-6  << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << IB->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                << (long_channel? IB->power.readOp.longer_channel_leakage:IB->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << IB->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << IB->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                cout << indent_str<< "Instruction Decoder:" << endl;
                cout << indent_str_next << "Area = " << (ID_inst->area.get_area() +
                                ID_operand->area.get_area() +
                                ID_misc->area.get_area())*coredynp.decodeW*1e-6  << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << (ID_inst->power.readOp.dynamic +
                                ID_operand->power.readOp.dynamic +
                                ID_misc->power.readOp.dynamic)*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                << (long_channel? (ID_inst->power.readOp.longer_channel_leakage +
                                ID_operand->power.readOp.longer_channel_leakage +
                                ID_misc->power.readOp.longer_channel_leakage):
                                        (ID_inst->power.readOp.leakage +
                                                        ID_operand->power.readOp.leakage +
                                                        ID_misc->power.readOp.leakage))  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << (ID_inst->power.readOp.gate_leakage +
                                ID_operand->power.readOp.gate_leakage +
                                ID_misc->power.readOp.gate_leakage)  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << (ID_inst->rt_power.readOp.dynamic +
                                ID_operand->rt_power.readOp.dynamic +
                                ID_misc->rt_power.readOp.dynamic)/executionTime << " W" << endl;
                cout <<endl;
        }
        else
        {
//		cout << indent_str_next << "Instruction Cache    Peak Dynamic = " << icache.rt_power.readOp.dynamic*clockRate << " W" << endl;
//		cout << indent_str_next << "Instruction Cache    Subthreshold Leakage = " << icache.rt_power.readOp.leakage <<" W" << endl;
//		cout << indent_str_next << "Instruction Cache    Gate Leakage = " << icache.rt_power.readOp.gate_leakage << " W" << endl;
//		cout << indent_str_next << "Instruction Buffer   Peak Dynamic = " << IB->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Instruction Buffer   Subthreshold Leakage = " << IB->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Instruction Buffer   Gate Leakage = " << IB->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "Branch Target Buffer   Peak Dynamic = " << BTB->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Branch Target Buffer   Subthreshold Leakage = " << BTB->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Branch Target Buffer   Gate Leakage = " << BTB->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "Branch Predictor   Peak Dynamic = " << BPT->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Branch Predictor   Subthreshold Leakage = " << BPT->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Branch Predictor   Gate Leakage = " << BPT->rt_power.readOp.gate_leakage  << " W" << endl;
        }

}

void RENAMINGU::computeEnergy(bool is_tdp)
{
        if (!exist) return;
        double pppm_t[4]    = {1,1,1,1};
        if (is_tdp)
        {//init stats for Peak
                if (coredynp.core_ty==OOO){
                        if (coredynp.scheu_ty==PhysicalRegFile)
                        {
                                if (coredynp.rm_ty ==RAMbased)
                                {
                                        iFRAT->stats_t.readAc.access   = iFRAT->l_ip.num_rd_ports;
                                        iFRAT->stats_t.writeAc.access  = iFRAT->l_ip.num_wr_ports;
                                        iFRAT->tdp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access   = fFRAT->l_ip.num_rd_ports;
                                        fFRAT->stats_t.writeAc.access  = fFRAT->l_ip.num_wr_ports;
                                        fFRAT->tdp_stats = fFRAT->stats_t;

                                }
                                else if ((coredynp.rm_ty ==CAMbased))
                                {
                                        iFRAT->stats_t.readAc.access   = iFRAT->l_ip.num_search_ports;
                                        iFRAT->stats_t.writeAc.access  = iFRAT->l_ip.num_wr_ports;
                                        iFRAT->tdp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access   = fFRAT->l_ip.num_search_ports;
                                        fFRAT->stats_t.writeAc.access  = fFRAT->l_ip.num_wr_ports;
                                        fFRAT->tdp_stats = fFRAT->stats_t;
                                }

                                iRRAT->stats_t.readAc.access   = iRRAT->l_ip.num_rd_ports;
                                iRRAT->stats_t.writeAc.access  = iRRAT->l_ip.num_wr_ports;
                                iRRAT->tdp_stats = iRRAT->stats_t;

                                fRRAT->stats_t.readAc.access   = fRRAT->l_ip.num_rd_ports;
                                fRRAT->stats_t.writeAc.access  = fRRAT->l_ip.num_wr_ports;
                                fRRAT->tdp_stats = fRRAT->stats_t;

                                ifreeL->stats_t.readAc.access   = coredynp.decodeW;//ifreeL->l_ip.num_rd_ports;;
                                ifreeL->stats_t.writeAc.access  = coredynp.decodeW;//ifreeL->l_ip.num_wr_ports;
                                ifreeL->tdp_stats = ifreeL->stats_t;

                                ffreeL->stats_t.readAc.access   = coredynp.decodeW;//ffreeL->l_ip.num_rd_ports;
                                ffreeL->stats_t.writeAc.access  = coredynp.decodeW;//ffreeL->l_ip.num_wr_ports;
                                ffreeL->tdp_stats = ffreeL->stats_t;
                        }
                        else if (coredynp.scheu_ty==ReservationStation){
                                if (coredynp.rm_ty ==RAMbased)
                                {
                                        iFRAT->stats_t.readAc.access    = iFRAT->l_ip.num_rd_ports;
                                        iFRAT->stats_t.writeAc.access   = iFRAT->l_ip.num_wr_ports;
                                        iFRAT->stats_t.searchAc.access  = iFRAT->l_ip.num_search_ports;
                                        iFRAT->tdp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access    = fFRAT->l_ip.num_rd_ports;
                                        fFRAT->stats_t.writeAc.access   = fFRAT->l_ip.num_wr_ports;
                                        fFRAT->stats_t.searchAc.access  = fFRAT->l_ip.num_search_ports;
                                        fFRAT->tdp_stats = fFRAT->stats_t;

                                }
                                else if ((coredynp.rm_ty ==CAMbased))
                                {
                                        iFRAT->stats_t.readAc.access   = iFRAT->l_ip.num_search_ports;
                                        iFRAT->stats_t.writeAc.access  = iFRAT->l_ip.num_wr_ports;
                                        iFRAT->tdp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access   = fFRAT->l_ip.num_search_ports;
                                        fFRAT->stats_t.writeAc.access  = fFRAT->l_ip.num_wr_ports;
                                        fFRAT->tdp_stats = fFRAT->stats_t;
                                }
                                //Unified free list for both int and fp
                                ifreeL->stats_t.readAc.access   = coredynp.decodeW;//ifreeL->l_ip.num_rd_ports;
                                ifreeL->stats_t.writeAc.access  = coredynp.decodeW;//ifreeL->l_ip.num_wr_ports;
                                ifreeL->tdp_stats = ifreeL->stats_t;
                        }
                        idcl->stats_t.readAc.access = coredynp.decodeW;
                        fdcl->stats_t.readAc.access = coredynp.decodeW;
                        idcl->tdp_stats = idcl->stats_t;
                        fdcl->tdp_stats = fdcl->stats_t;
                }
                else
                {
                        if (coredynp.issueW>1)
                        {
                                idcl->stats_t.readAc.access = coredynp.decodeW;
                                fdcl->stats_t.readAc.access = coredynp.decodeW;
                                idcl->tdp_stats = idcl->stats_t;
                                fdcl->tdp_stats = fdcl->stats_t;
                        }
                }

        }
        else
        {//init stats for Runtime Dynamic (RTP)
                if (coredynp.core_ty==OOO){
                        if (coredynp.scheu_ty==PhysicalRegFile)
                        {
                                if (coredynp.rm_ty ==RAMbased)
                                {
                                        iFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].rename_reads;
                                        iFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].rename_writes;
                                        iFRAT->rtp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].fp_rename_reads;
                                        fFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].fp_rename_writes;
                                        fFRAT->rtp_stats = fFRAT->stats_t;
                                }
                                else if ((coredynp.rm_ty ==CAMbased))
                                {
                                        iFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].rename_reads;
                                        iFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].rename_writes;
                                        iFRAT->rtp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].fp_rename_reads;
                                        fFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].fp_rename_writes;
                                        fFRAT->rtp_stats = fFRAT->stats_t;
                                }

                                iRRAT->stats_t.readAc.access   = XML->sys.core[ithCore].rename_writes;//Hack, should be (context switch + branch mispredictions)*16
                                iRRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].rename_writes;
                                iRRAT->rtp_stats = iRRAT->stats_t;

                                fRRAT->stats_t.readAc.access   = XML->sys.core[ithCore].fp_rename_writes;//Hack, should be (context switch + branch mispredictions)*16
                                fRRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].fp_rename_writes;
                                fRRAT->rtp_stats = fRRAT->stats_t;

                                ifreeL->stats_t.readAc.access   = XML->sys.core[ithCore].rename_reads;
                                ifreeL->stats_t.writeAc.access  = 2*XML->sys.core[ithCore].rename_writes;
                                ifreeL->rtp_stats = ifreeL->stats_t;

                                ffreeL->stats_t.readAc.access   = XML->sys.core[ithCore].fp_rename_reads;
                                ffreeL->stats_t.writeAc.access  = 2*XML->sys.core[ithCore].fp_rename_writes;
                                ffreeL->rtp_stats = ffreeL->stats_t;
                        }
                        else if (coredynp.scheu_ty==ReservationStation){
                                if (coredynp.rm_ty ==RAMbased)
                                {
                                        iFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].rename_reads;
                                        iFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].rename_writes;
                                        iFRAT->stats_t.searchAc.access  = XML->sys.core[ithCore].committed_int_instructions;//hack: not all committed instructions use regs.
                                        iFRAT->rtp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].fp_rename_reads;
                                        fFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].fp_rename_writes;
                                        fFRAT->stats_t.searchAc.access  = XML->sys.core[ithCore].committed_fp_instructions;
                                        fFRAT->rtp_stats = fFRAT->stats_t;
                                }
                                else if ((coredynp.rm_ty ==CAMbased))
                                {
                                        iFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].rename_reads;
                                        iFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].rename_writes;
                                        iFRAT->rtp_stats = iFRAT->stats_t;

                                        fFRAT->stats_t.readAc.access   = XML->sys.core[ithCore].fp_rename_reads;
                                        fFRAT->stats_t.writeAc.access  = XML->sys.core[ithCore].fp_rename_writes;
                                        fFRAT->rtp_stats = fFRAT->stats_t;
                                }
                                //Unified free list for both int and fp since the ROB act as physcial registers
                                ifreeL->stats_t.readAc.access   = XML->sys.core[ithCore].rename_reads +
                                        XML->sys.core[ithCore].fp_rename_reads;
                                ifreeL->stats_t.writeAc.access  = 2*(XML->sys.core[ithCore].rename_writes +
                                        XML->sys.core[ithCore].fp_rename_writes);//HACK: 2-> since some of renaming in the same group
                                                                                                                         //are terminated early
                                ifreeL->rtp_stats = ifreeL->stats_t;
                        }
                        idcl->stats_t.readAc.access = 3*coredynp.decodeW*coredynp.decodeW*XML->sys.core[ithCore].rename_reads;
                        fdcl->stats_t.readAc.access = 3*coredynp.fp_issueW*coredynp.fp_issueW*XML->sys.core[ithCore].fp_rename_writes;
                        idcl->rtp_stats = idcl->stats_t;
                        fdcl->rtp_stats = fdcl->stats_t;
                }
                else
                {
                        if (coredynp.issueW>1)
                        {
                                idcl->stats_t.readAc.access = 2*XML->sys.core[ithCore].int_instructions;
                                fdcl->stats_t.readAc.access = XML->sys.core[ithCore].fp_instructions;
                                idcl->rtp_stats = idcl->stats_t;
                                fdcl->rtp_stats = fdcl->stats_t;
                        }
                }

        }
    /* Compute engine */
        if (coredynp.core_ty==OOO)
        {
                if (coredynp.scheu_ty==PhysicalRegFile)
                {
                        if (coredynp.rm_ty ==RAMbased)
                        {
                                iFRAT->power_t.reset();
                                fFRAT->power_t.reset();

                                iFRAT->power_t.readOp.dynamic  +=  (iFRAT->stats_t.readAc.access
                                                *(iFRAT->local_result.power.readOp.dynamic + idcl->power.readOp.dynamic)
                                                +iFRAT->stats_t.writeAc.access*iFRAT->local_result.power.writeOp.dynamic);
                                fFRAT->power_t.readOp.dynamic  +=  (fFRAT->stats_t.readAc.access
                                                *(fFRAT->local_result.power.readOp.dynamic + fdcl->power.readOp.dynamic)
                                                +fFRAT->stats_t.writeAc.access*fFRAT->local_result.power.writeOp.dynamic);
                        }
                        else if ((coredynp.rm_ty ==CAMbased))
                        {
                                iFRAT->power_t.reset();
                                fFRAT->power_t.reset();
                                iFRAT->power_t.readOp.dynamic  +=  (iFRAT->stats_t.readAc.access
                                                *(iFRAT->local_result.power.searchOp.dynamic + idcl->power.readOp.dynamic)
                                                +iFRAT->stats_t.writeAc.access*iFRAT->local_result.power.writeOp.dynamic);
                                fFRAT->power_t.readOp.dynamic  +=  (fFRAT->stats_t.readAc.access
                                                *(fFRAT->local_result.power.searchOp.dynamic + fdcl->power.readOp.dynamic)
                                                +fFRAT->stats_t.writeAc.access*fFRAT->local_result.power.writeOp.dynamic);
                        }

                        iRRAT->power_t.reset();
                        fRRAT->power_t.reset();
                        ifreeL->power_t.reset();
                        ffreeL->power_t.reset();

                        iRRAT->power_t.readOp.dynamic  +=  (iRRAT->stats_t.readAc.access*iRRAT->local_result.power.readOp.dynamic
                                        +iRRAT->stats_t.writeAc.access*iRRAT->local_result.power.writeOp.dynamic);
                        fRRAT->power_t.readOp.dynamic  +=  (fRRAT->stats_t.readAc.access*fRRAT->local_result.power.readOp.dynamic
                                        +fRRAT->stats_t.writeAc.access*fRRAT->local_result.power.writeOp.dynamic);
                        ifreeL->power_t.readOp.dynamic  +=  (ifreeL->stats_t.readAc.access*ifreeL->local_result.power.readOp.dynamic
                                        +ifreeL->stats_t.writeAc.access*ifreeL->local_result.power.writeOp.dynamic);
                        ffreeL->power_t.readOp.dynamic  +=  (ffreeL->stats_t.readAc.access*ffreeL->local_result.power.readOp.dynamic
                                        +ffreeL->stats_t.writeAc.access*ffreeL->local_result.power.writeOp.dynamic);

                }
                else if (coredynp.scheu_ty==ReservationStation)
                {
                        if (coredynp.rm_ty ==RAMbased)
                        {
                                iFRAT->power_t.reset();
                                fFRAT->power_t.reset();

                                iFRAT->power_t.readOp.dynamic  +=  (iFRAT->stats_t.readAc.access
                                                *(iFRAT->local_result.power.readOp.dynamic + idcl->power.readOp.dynamic)
                                                +iFRAT->stats_t.writeAc.access*iFRAT->local_result.power.writeOp.dynamic
                                                +iFRAT->stats_t.searchAc.access*iFRAT->local_result.power.searchOp.dynamic);
                                fFRAT->power_t.readOp.dynamic  +=  (fFRAT->stats_t.readAc.access
                                                *(fFRAT->local_result.power.readOp.dynamic + fdcl->power.readOp.dynamic)
                                                +fFRAT->stats_t.writeAc.access*fFRAT->local_result.power.writeOp.dynamic
                                                +fFRAT->stats_t.searchAc.access*fFRAT->local_result.power.searchOp.dynamic);
                        }
                        else if ((coredynp.rm_ty ==CAMbased))
                        {
                                iFRAT->power_t.reset();
                                fFRAT->power_t.reset();
                                iFRAT->power_t.readOp.dynamic  +=  (iFRAT->stats_t.readAc.access
                                                *(iFRAT->local_result.power.searchOp.dynamic + idcl->power.readOp.dynamic)
                                                +iFRAT->stats_t.writeAc.access*iFRAT->local_result.power.writeOp.dynamic);
                                fFRAT->power_t.readOp.dynamic  +=  (fFRAT->stats_t.readAc.access
                                                *(fFRAT->local_result.power.searchOp.dynamic + fdcl->power.readOp.dynamic)
                                                +fFRAT->stats_t.writeAc.access*fFRAT->local_result.power.writeOp.dynamic);
                        }
                        ifreeL->power_t.reset();
                        ifreeL->power_t.readOp.dynamic  +=  (ifreeL->stats_t.readAc.access*ifreeL->local_result.power.readOp.dynamic
                                        +ifreeL->stats_t.writeAc.access*ifreeL->local_result.power.writeOp.dynamic);
                }

        }
        else
        {
                if (coredynp.issueW>1)
                {
                        idcl->power_t.reset();
                        fdcl->power_t.reset();
                        set_pppm(pppm_t, idcl->stats_t.readAc.access, coredynp.num_hthreads, coredynp.num_hthreads, idcl->stats_t.readAc.access);
                        idcl->power_t = idcl->power * pppm_t;
                        set_pppm(pppm_t, fdcl->stats_t.readAc.access, coredynp.num_hthreads, coredynp.num_hthreads, idcl->stats_t.readAc.access);
                        fdcl->power_t = fdcl->power * pppm_t;
                }

        }

        //assign value to tpd and rtp
        if (is_tdp)
        {
                if (coredynp.core_ty==OOO)
                {
                        if (coredynp.scheu_ty==PhysicalRegFile)
                        {
                                iFRAT->power   =  iFRAT->power_t + (iFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + idcl->power_t;
                                fFRAT->power   =  fFRAT->power_t + (fFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + fdcl->power_t;
                                iRRAT->power   =  iRRAT->power_t + iRRAT->local_result.power * coredynp.pppm_lkg_multhread;
                                fRRAT->power   =  fRRAT->power_t + fRRAT->local_result.power * coredynp.pppm_lkg_multhread;
                                ifreeL->power  =  ifreeL->power_t + ifreeL->local_result.power * coredynp.pppm_lkg_multhread;
                                ffreeL->power  =  ffreeL->power_t + ffreeL->local_result.power * coredynp.pppm_lkg_multhread;
                                power	       =  power + (iFRAT->power + fFRAT->power)
                                                 + (iRRAT->power + fRRAT->power)
                                                 + (ifreeL->power + ffreeL->power);
                        }
                        else if (coredynp.scheu_ty==ReservationStation)
                        {
                                iFRAT->power   =  iFRAT->power_t + (iFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + idcl->power_t;
                                fFRAT->power   =  fFRAT->power_t + (fFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + fdcl->power_t;
                                ifreeL->power  =  ifreeL->power_t + ifreeL->local_result.power * coredynp.pppm_lkg_multhread;
                                power	       =  power + (iFRAT->power + fFRAT->power)
                                                 + ifreeL->power;
                        }
                }
                else
                {
                        power   =  power + idcl->power_t + fdcl->power_t;
                }

        }
        else
        {
                if (coredynp.core_ty==OOO)
                {
                        if (coredynp.scheu_ty==PhysicalRegFile)
                        {
                                iFRAT->rt_power   =  iFRAT->power_t + (iFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + idcl->power_t;
                                fFRAT->rt_power   =  fFRAT->power_t + (fFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + fdcl->power_t;
                                iRRAT->rt_power   =  iRRAT->power_t + iRRAT->local_result.power * coredynp.pppm_lkg_multhread;
                                fRRAT->rt_power   =  fRRAT->power_t + fRRAT->local_result.power * coredynp.pppm_lkg_multhread;
                                ifreeL->rt_power  =  ifreeL->power_t + ifreeL->local_result.power * coredynp.pppm_lkg_multhread;
                                ffreeL->rt_power  =  ffreeL->power_t + ffreeL->local_result.power * coredynp.pppm_lkg_multhread;
                                rt_power	      =  rt_power + (iFRAT->rt_power + fFRAT->rt_power)
                                                   + (iRRAT->rt_power + fRRAT->rt_power)
                                                   + (ifreeL->rt_power + ffreeL->rt_power);
                        }
                        else if (coredynp.scheu_ty==ReservationStation)
                        {
                                iFRAT->rt_power   =  iFRAT->power_t + (iFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + idcl->power_t;
                                fFRAT->rt_power   =  fFRAT->power_t + (fFRAT->local_result.power ) * coredynp.pppm_lkg_multhread + fdcl->power_t;
                                ifreeL->rt_power  =  ifreeL->power_t + ifreeL->local_result.power * coredynp.pppm_lkg_multhread;
                                rt_power	      =  rt_power + (iFRAT->rt_power + fFRAT->rt_power)
                                                   + ifreeL->rt_power;
                        }
                }
                else
                {
                        rt_power   =  rt_power + idcl->power_t + fdcl->power_t;
                }

        }
}

void RENAMINGU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;


        if (is_tdp)
        {

                if (coredynp.core_ty==OOO)
                {
                        cout << indent_str<< "Int Front End RAT:" << endl;
                        cout << indent_str_next << "Area = " << iFRAT->area.get_area()*1e-6<< " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << iFRAT->power.readOp.dynamic*clockRate << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? iFRAT->power.readOp.longer_channel_leakage:iFRAT->power.readOp.leakage) <<" W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << iFRAT->power.readOp.gate_leakage << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << iFRAT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        cout << indent_str<< "FP Front End RAT:" << endl;
                        cout << indent_str_next << "Area = " << fFRAT->area.get_area()*1e-6  << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << fFRAT->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? fFRAT->power.readOp.longer_channel_leakage:fFRAT->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << fFRAT->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << fFRAT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        cout << indent_str<<"Free List:" << endl;
                        cout << indent_str_next << "Area = " << ifreeL->area.get_area()*1e-6  << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << ifreeL->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? ifreeL->power.readOp.longer_channel_leakage:ifreeL->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << ifreeL->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << ifreeL->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;

                        if (coredynp.scheu_ty==PhysicalRegFile)
                        {
                                cout << indent_str<< "Int Retire RAT: " << endl;
                                cout << indent_str_next << "Area = " << iRRAT->area.get_area() *1e-6 << " mm^2" << endl;
                                cout << indent_str_next << "Peak Dynamic = " << iRRAT->power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Subthreshold Leakage = "
                                        << (long_channel? iRRAT->power.readOp.longer_channel_leakage:iRRAT->power.readOp.leakage)  << " W" << endl;
                                cout << indent_str_next << "Gate Leakage = " << iRRAT->power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "Runtime Dynamic = " << iRRAT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                                cout <<endl;
                                cout << indent_str<< "FP Retire RAT:" << endl;
                                cout << indent_str_next << "Area = " << fRRAT->area.get_area()  *1e-6<< " mm^2" << endl;
                                cout << indent_str_next << "Peak Dynamic = " << fRRAT->power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Subthreshold Leakage = "
                                        << (long_channel? fRRAT->power.readOp.longer_channel_leakage:fRRAT->power.readOp.leakage)  << " W" << endl;
                                cout << indent_str_next << "Gate Leakage = " << fRRAT->power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "Runtime Dynamic = " << fRRAT->rt_power.readOp.dynamic/executionTime << " W" << endl;
                                cout <<endl;
                                cout << indent_str<< "FP Free List:" << endl;
                                cout << indent_str_next << "Area = " << ffreeL->area.get_area()*1e-6  << " mm^2" << endl;
                                cout << indent_str_next << "Peak Dynamic = " << ffreeL->power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Subthreshold Leakage = "
                                        << (long_channel? ffreeL->power.readOp.longer_channel_leakage:ffreeL->power.readOp.leakage)  << " W" << endl;
                                cout << indent_str_next << "Gate Leakage = " << ffreeL->power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "Runtime Dynamic = " << ffreeL->rt_power.readOp.dynamic/executionTime << " W" << endl;
                                cout <<endl;
                        }
                }
                else
                {
                        cout << indent_str<< "Int DCL:" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << idcl->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? idcl->power.readOp.longer_channel_leakage:idcl->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << idcl->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << idcl->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout << indent_str<<"FP DCL:" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << fdcl->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? fdcl->power.readOp.longer_channel_leakage:fdcl->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << fdcl->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << fdcl->rt_power.readOp.dynamic/executionTime << " W" << endl;
                }
        }
        else
        {
                if (coredynp.core_ty==OOO)
                {
                        cout << indent_str_next << "Int Front End RAT    Peak Dynamic = " << iFRAT->rt_power.readOp.dynamic*clockRate << " W" << endl;
                        cout << indent_str_next << "Int Front End RAT    Subthreshold Leakage = " << iFRAT->rt_power.readOp.leakage <<" W" << endl;
                        cout << indent_str_next << "Int Front End RAT    Gate Leakage = " << iFRAT->rt_power.readOp.gate_leakage << " W" << endl;
                        cout << indent_str_next << "FP Front End RAT   Peak Dynamic = " << fFRAT->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "FP Front End RAT   Subthreshold Leakage = " << fFRAT->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "FP Front End RAT   Gate Leakage = " << fFRAT->rt_power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Free List   Peak Dynamic = " << ifreeL->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Free List   Subthreshold Leakage = " << ifreeL->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "Free List   Gate Leakage = " << fFRAT->rt_power.readOp.gate_leakage  << " W" << endl;
                        if (coredynp.scheu_ty==PhysicalRegFile)
                        {
                                cout << indent_str_next << "Int Retire RAT   Peak Dynamic = " << iRRAT->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Int Retire RAT   Subthreshold Leakage = " << iRRAT->rt_power.readOp.leakage  << " W" << endl;
                                cout << indent_str_next << "Int Retire RAT   Gate Leakage = " << iRRAT->rt_power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "FP Retire RAT   Peak Dynamic = " << fRRAT->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "FP Retire RAT   Subthreshold Leakage = " << fRRAT->rt_power.readOp.leakage  << " W" << endl;
                                cout << indent_str_next << "FP Retire RAT   Gate Leakage = " << fRRAT->rt_power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "FP Free List   Peak Dynamic = " << ffreeL->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "FP Free List   Subthreshold Leakage = " << ffreeL->rt_power.readOp.leakage  << " W" << endl;
                                cout << indent_str_next << "FP Free List   Gate Leakage = " << fFRAT->rt_power.readOp.gate_leakage  << " W" << endl;
                        }
                }
                else
                {
                        cout << indent_str_next << "Int DCL   Peak Dynamic = " << idcl->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Int DCL   Subthreshold Leakage = " << idcl->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "Int DCL   Gate Leakage = " << idcl->rt_power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "FP DCL   Peak Dynamic = " << fdcl->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "FP DCL   Subthreshold Leakage = " << fdcl->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "FP DCL   Gate Leakage = " << fdcl->rt_power.readOp.gate_leakage  << " W" << endl;
                }
        }

}


void SchedulerU::computeEnergy(bool is_tdp)
{
        if (!exist) return;
        double ROB_duty_cycle;
//	ROB_duty_cycle = ((coredynp.ALU_duty_cycle + coredynp.num_muls>0?coredynp.MUL_duty_cycle:0
//			+ coredynp.num_fpus>0?coredynp.FPU_duty_cycle:0))*1.1<1 ? (coredynp.ALU_duty_cycle + coredynp.num_muls>0?coredynp.MUL_duty_cycle:0
//					+ coredynp.num_fpus>0?coredynp.FPU_duty_cycle:0)*1.1:1;
        ROB_duty_cycle = 1;
        //init stats
        if (is_tdp)
        {
                if (coredynp.core_ty==OOO)
                {
                        int_inst_window->stats_t.readAc.access    = coredynp.issueW*coredynp.num_pipelines;//int_inst_window->l_ip.num_search_ports;
                        int_inst_window->stats_t.writeAc.access   = coredynp.issueW*coredynp.num_pipelines;//int_inst_window->l_ip.num_wr_ports;
                        int_inst_window->stats_t.searchAc.access  = coredynp.issueW*coredynp.num_pipelines;
                        int_inst_window->tdp_stats                = int_inst_window->stats_t;
                        fp_inst_window->stats_t.readAc.access     = fp_inst_window->l_ip.num_rd_ports*coredynp.num_fp_pipelines;
                        fp_inst_window->stats_t.writeAc.access    = fp_inst_window->l_ip.num_wr_ports*coredynp.num_fp_pipelines;
                        fp_inst_window->stats_t.searchAc.access   = fp_inst_window->l_ip.num_search_ports*coredynp.num_fp_pipelines;
                        fp_inst_window->tdp_stats                 = fp_inst_window->stats_t;

                        if (XML->sys.core[ithCore].ROB_size >0)
                        {
                                ROB->stats_t.readAc.access   = coredynp.commitW*coredynp.num_pipelines*ROB_duty_cycle;
                                ROB->stats_t.writeAc.access  = coredynp.issueW*coredynp.num_pipelines*ROB_duty_cycle;
                                ROB->tdp_stats        = ROB->stats_t;

                                /*
                                 * When inst commits, ROB must be read.
                                 * Because for Physcial register based cores, physical register tag in ROB
                                 * need to be read out and write into RRAT/CAM based RAT.
                                 * For RS based cores, register content that stored in ROB must be
                                 * read out and stored in architectural registers.
                                 *
                                 * if no-register is involved, the ROB read out operation when instruction commits can be ignored.
                                 * assuming 20% insts. belong this type.
                                 * TODO: ROB duty_cycle need to be revisited
                                 */
                        }

                }
                else if (coredynp.multithreaded)
                {
                        int_inst_window->stats_t.readAc.access   = coredynp.issueW*coredynp.num_pipelines;//int_inst_window->l_ip.num_search_ports;
                        int_inst_window->stats_t.writeAc.access  = coredynp.issueW*coredynp.num_pipelines;//int_inst_window->l_ip.num_wr_ports;
                        int_inst_window->stats_t.searchAc.access = coredynp.issueW*coredynp.num_pipelines;
                        int_inst_window->tdp_stats       = int_inst_window->stats_t;
                }

     }
    else
    {//rtp
                if (coredynp.core_ty==OOO)
                {
                        int_inst_window->stats_t.readAc.access   = XML->sys.core[ithCore].inst_window_reads;
                        int_inst_window->stats_t.writeAc.access  = XML->sys.core[ithCore].inst_window_writes;
                        int_inst_window->stats_t.searchAc.access = XML->sys.core[ithCore].inst_window_wakeup_accesses;
                        int_inst_window->rtp_stats               = int_inst_window->stats_t;
                        fp_inst_window->stats_t.readAc.access    = XML->sys.core[ithCore].fp_inst_window_reads;
                        fp_inst_window->stats_t.writeAc.access   = XML->sys.core[ithCore].fp_inst_window_writes;
                        fp_inst_window->stats_t.searchAc.access  = XML->sys.core[ithCore].fp_inst_window_wakeup_accesses;
                        fp_inst_window->rtp_stats                = fp_inst_window->stats_t;

                        if (XML->sys.core[ithCore].ROB_size >0)
                        {

                                ROB->stats_t.readAc.access   = XML->sys.core[ithCore].ROB_reads;
                                ROB->stats_t.writeAc.access  = XML->sys.core[ithCore].ROB_writes;
                                /* ROB need to be updated in RS based OOO when new values are produced,
                                 * this update may happen before the commit stage when ROB entry is released
                                 * 1. ROB write at instruction inserted in
                                 * 2. ROB write as results produced (for RS based OOO only)
                                 * 3. ROB read  as instruction committed. For RS based OOO, data values are read out and sent to ARF
                                 * For Physical reg based OOO, no data stored in ROB, but register tags need to be
                                 * read out and used to set the RRAT and to recycle the register tag to free list buffer
                                 */
                                ROB->rtp_stats        = ROB->stats_t;
                        }

                }
                else if (coredynp.multithreaded)
                {
                        int_inst_window->stats_t.readAc.access    = XML->sys.core[ithCore].int_instructions + XML->sys.core[ithCore].fp_instructions;
                        int_inst_window->stats_t.writeAc.access   = XML->sys.core[ithCore].int_instructions + XML->sys.core[ithCore].fp_instructions;
                        int_inst_window->stats_t.searchAc.access  = 2*(XML->sys.core[ithCore].int_instructions + XML->sys.core[ithCore].fp_instructions);
                        int_inst_window->rtp_stats                = int_inst_window->stats_t;
                }
    }

        //computation engine
        if (coredynp.core_ty==OOO)
        {
                int_inst_window->power_t.reset();
                fp_inst_window->power_t.reset();

                /* each instruction needs to write to scheduler, read out when all resources and source operands are ready
                 * two search ops with one for each source operand
                 *
                 */
                int_inst_window->power_t.readOp.dynamic  +=  int_inst_window->local_result.power.readOp.dynamic * int_inst_window->stats_t.readAc.access
                                        + int_inst_window->local_result.power.searchOp.dynamic * int_inst_window->stats_t.searchAc.access
                                        + int_inst_window->local_result.power.writeOp.dynamic  * int_inst_window->stats_t.writeAc.access
                                        + int_inst_window->stats_t.readAc.access * instruction_selection->power.readOp.dynamic;

                fp_inst_window->power_t.readOp.dynamic   +=  fp_inst_window->local_result.power.readOp.dynamic * fp_inst_window->stats_t.readAc.access
                                        + fp_inst_window->local_result.power.searchOp.dynamic * fp_inst_window->stats_t.searchAc.access
                                        + fp_inst_window->local_result.power.writeOp.dynamic * fp_inst_window->stats_t.writeAc.access
                                        + fp_inst_window->stats_t.writeAc.access * instruction_selection->power.readOp.dynamic;

                if (XML->sys.core[ithCore].ROB_size >0)
                {
                        ROB->power_t.reset();
                        ROB->power_t.readOp.dynamic   +=  ROB->local_result.power.readOp.dynamic*ROB->stats_t.readAc.access +
                                                ROB->stats_t.writeAc.access*ROB->local_result.power.writeOp.dynamic;
                }




        }
        else if (coredynp.multithreaded)
        {
                int_inst_window->power_t.reset();
                int_inst_window->power_t.readOp.dynamic  +=  int_inst_window->local_result.power.readOp.dynamic * int_inst_window->stats_t.readAc.access
                                                  + int_inst_window->local_result.power.searchOp.dynamic * int_inst_window->stats_t.searchAc.access
                                          + int_inst_window->local_result.power.writeOp.dynamic  * int_inst_window->stats_t.writeAc.access
                                          + int_inst_window->stats_t.writeAc.access * instruction_selection->power.readOp.dynamic;
        }

        //assign values
        if (is_tdp)
        {
                if (coredynp.core_ty==OOO)
                {
                        int_inst_window->power = int_inst_window->power_t + (int_inst_window->local_result.power +instruction_selection->power) *pppm_lkg;
                        fp_inst_window->power = fp_inst_window->power_t + (fp_inst_window->local_result.power +instruction_selection->power) *pppm_lkg;
                        power	   = power + int_inst_window->power + fp_inst_window->power;
                        if (XML->sys.core[ithCore].ROB_size >0)
                        {
                                ROB->power = ROB->power_t + ROB->local_result.power*pppm_lkg;
                                power	   = power + ROB->power;
                        }

                }
                else if (coredynp.multithreaded)
                {
                        //			set_pppm(pppm_t, XML->sys.core[ithCore].issue_width,1, 1, 1);
                        int_inst_window->power = int_inst_window->power_t + (int_inst_window->local_result.power +instruction_selection->power) *pppm_lkg;
                        power	   = power + int_inst_window->power;
        }

     }
    else
    {//rtp
                if (coredynp.core_ty==OOO)
                {
                        int_inst_window->rt_power = int_inst_window->power_t + (int_inst_window->local_result.power +instruction_selection->power) *pppm_lkg;
                        fp_inst_window->rt_power  = fp_inst_window->power_t + (fp_inst_window->local_result.power +instruction_selection->power) *pppm_lkg;
                        rt_power	              = rt_power + int_inst_window->rt_power + fp_inst_window->rt_power;
                        if (XML->sys.core[ithCore].ROB_size >0)
                        {
                                ROB->rt_power = ROB->power_t + ROB->local_result.power*pppm_lkg;
                                rt_power	              = rt_power + ROB->rt_power;
                        }

                }
                else if (coredynp.multithreaded)
                {
                        //			set_pppm(pppm_t, XML->sys.core[ithCore].issue_width,1, 1, 1);
                        int_inst_window->rt_power = int_inst_window->power_t + (int_inst_window->local_result.power +instruction_selection->power) *pppm_lkg;
                        rt_power	              = rt_power + int_inst_window->rt_power;
        }
    }
//	set_pppm(pppm_t, XML->sys.core[ithCore].issue_width,1, 1, 1);
//	cout<<"Scheduler power="<<power.readOp.dynamic<<"leakage="<<power.readOp.leakage<<endl;
//	cout<<"IW="<<int_inst_window->local_result.power.searchOp.dynamic * int_inst_window->stats_t.readAc.access +
//    + int_inst_window->local_result.power.writeOp.dynamic * int_inst_window->stats_t.writeAc.access<<"leakage="<<int_inst_window->local_result.power.readOp.leakage<<endl;
//	cout<<"selection"<<instruction_selection->power.readOp.dynamic<<"leakage"<<instruction_selection->power.readOp.leakage<<endl;
}

void SchedulerU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;


        if (is_tdp)
        {
                if (coredynp.core_ty==OOO)
                {
                        cout << indent_str << "Instruction Window:" << endl;
                        cout << indent_str_next << "Area = " << int_inst_window->area.get_area()*1e-6<< " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << int_inst_window->power.readOp.dynamic*clockRate << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? int_inst_window->power.readOp.longer_channel_leakage:int_inst_window->power.readOp.leakage) <<" W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << int_inst_window->power.readOp.gate_leakage << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << int_inst_window->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        cout << indent_str << "FP Instruction Window:" << endl;
                        cout << indent_str_next << "Area = " << fp_inst_window->area.get_area()*1e-6  << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << fp_inst_window->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? fp_inst_window->power.readOp.longer_channel_leakage:fp_inst_window->power.readOp.leakage ) << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << fp_inst_window->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << fp_inst_window->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        if (XML->sys.core[ithCore].ROB_size >0)
                        {
                                cout << indent_str<<"ROB:" << endl;
                                cout << indent_str_next << "Area = " << ROB->area.get_area() *1e-6 << " mm^2" << endl;
                                cout << indent_str_next << "Peak Dynamic = " << ROB->power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? ROB->power.readOp.longer_channel_leakage:ROB->power.readOp.leakage)  << " W" << endl;
                                cout << indent_str_next << "Gate Leakage = " << ROB->power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "Runtime Dynamic = " << ROB->rt_power.readOp.dynamic/executionTime << " W" << endl;
                                cout <<endl;
                        }
                }
                else if (coredynp.multithreaded)
                {
                        cout << indent_str << "Instruction Window:" << endl;
                        cout << indent_str_next << "Area = " << int_inst_window->area.get_area()*1e-6<< " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << int_inst_window->power.readOp.dynamic*clockRate << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? int_inst_window->power.readOp.longer_channel_leakage:int_inst_window->power.readOp.leakage) <<" W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << int_inst_window->power.readOp.gate_leakage << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << int_inst_window->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                }
        }
        else
        {
                if (coredynp.core_ty==OOO)
                {
                        cout << indent_str_next << "Instruction Window    Peak Dynamic = " << int_inst_window->rt_power.readOp.dynamic*clockRate << " W" << endl;
                        cout << indent_str_next << "Instruction Window    Subthreshold Leakage = " << int_inst_window->rt_power.readOp.leakage <<" W" << endl;
                        cout << indent_str_next << "Instruction Window    Gate Leakage = " << int_inst_window->rt_power.readOp.gate_leakage << " W" << endl;
                        cout << indent_str_next << "FP Instruction Window   Peak Dynamic = " << fp_inst_window->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "FP Instruction Window   Subthreshold Leakage = " << fp_inst_window->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "FP Instruction Window   Gate Leakage = " << fp_inst_window->rt_power.readOp.gate_leakage  << " W" << endl;
                        if (XML->sys.core[ithCore].ROB_size >0)
                        {
                                cout << indent_str_next << "ROB   Peak Dynamic = " << ROB->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "ROB   Subthreshold Leakage = " << ROB->rt_power.readOp.leakage  << " W" << endl;
                                cout << indent_str_next << "ROB   Gate Leakage = " << ROB->rt_power.readOp.gate_leakage  << " W" << endl;
                        }
                }
                else if (coredynp.multithreaded)
                {
                        cout << indent_str_next << "Instruction Window    Peak Dynamic = " << int_inst_window->rt_power.readOp.dynamic*clockRate << " W" << endl;
                        cout << indent_str_next << "Instruction Window    Subthreshold Leakage = " << int_inst_window->rt_power.readOp.leakage <<" W" << endl;
                        cout << indent_str_next << "Instruction Window    Gate Leakage = " << int_inst_window->rt_power.readOp.gate_leakage << " W" << endl;
                }
        }

}

void LoadStoreU::computeEnergy(bool is_tdp)
{
        if (!exist) return;
        if (is_tdp)
            {
                //init stats for Peak
                dcache.caches->stats_t.readAc.access  = 0.67*dcache.caches->l_ip.num_rw_ports*coredynp.LSU_duty_cycle;
                dcache.caches->stats_t.readAc.miss    = 0;
                dcache.caches->stats_t.readAc.hit     = dcache.caches->stats_t.readAc.access - dcache.caches->stats_t.readAc.miss;
                dcache.caches->stats_t.writeAc.access = 0.33*dcache.caches->l_ip.num_rw_ports*coredynp.LSU_duty_cycle;
                dcache.caches->stats_t.writeAc.miss   = 0;
                dcache.caches->stats_t.writeAc.hit    = dcache.caches->stats_t.writeAc.access -	dcache.caches->stats_t.writeAc.miss;
                dcache.caches->tdp_stats = dcache.caches->stats_t;

                dcache.missb->stats_t.readAc.access  = dcache.missb->l_ip.num_search_ports;
                dcache.missb->stats_t.writeAc.access = dcache.missb->l_ip.num_search_ports;
                dcache.missb->tdp_stats = dcache.missb->stats_t;

                dcache.ifb->stats_t.readAc.access  = dcache.ifb->l_ip.num_search_ports;
                dcache.ifb->stats_t.writeAc.access = dcache.ifb->l_ip.num_search_ports;
                dcache.ifb->tdp_stats = dcache.ifb->stats_t;

                dcache.prefetchb->stats_t.readAc.access  = dcache.prefetchb->l_ip.num_search_ports;
                dcache.prefetchb->stats_t.writeAc.access = dcache.ifb->l_ip.num_search_ports;
                dcache.prefetchb->tdp_stats = dcache.prefetchb->stats_t;
                if (cache_p==Write_back)
                {
                        dcache.wbb->stats_t.readAc.access  = dcache.wbb->l_ip.num_search_ports;
                        dcache.wbb->stats_t.writeAc.access = dcache.wbb->l_ip.num_search_ports;
                        dcache.wbb->tdp_stats = dcache.wbb->stats_t;
                }

                LSQ->stats_t.readAc.access = LSQ->stats_t.writeAc.access = LSQ->l_ip.num_search_ports*coredynp.LSU_duty_cycle;
                LSQ->tdp_stats = LSQ->stats_t;
                if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].load_buffer_size >0))
                {
                        LoadQ->stats_t.readAc.access = LoadQ->stats_t.writeAc.access = LoadQ->l_ip.num_search_ports*coredynp.LSU_duty_cycle;
                        LoadQ->tdp_stats = LoadQ->stats_t;
                }
            }
            else
            {
                //init stats for Runtime Dynamic (RTP)
                dcache.caches->stats_t.readAc.access  = XML->sys.core[ithCore].dcache.read_accesses;
                dcache.caches->stats_t.readAc.miss    = XML->sys.core[ithCore].dcache.read_misses;
                dcache.caches->stats_t.readAc.hit     = dcache.caches->stats_t.readAc.access - dcache.caches->stats_t.readAc.miss;
                dcache.caches->stats_t.writeAc.access = XML->sys.core[ithCore].dcache.write_accesses;
                dcache.caches->stats_t.writeAc.miss   = XML->sys.core[ithCore].dcache.write_misses;
                dcache.caches->stats_t.writeAc.hit    = dcache.caches->stats_t.writeAc.access -	dcache.caches->stats_t.writeAc.miss;
                dcache.caches->rtp_stats = dcache.caches->stats_t;

                if (cache_p==Write_back)
                {
                        dcache.missb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
                        dcache.missb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
                        dcache.missb->rtp_stats = dcache.missb->stats_t;

                        dcache.ifb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
                        dcache.ifb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
                        dcache.ifb->rtp_stats = dcache.ifb->stats_t;

                        dcache.prefetchb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
                        dcache.prefetchb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
                        dcache.prefetchb->rtp_stats = dcache.prefetchb->stats_t;

                        dcache.wbb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
                        dcache.wbb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
                        dcache.wbb->rtp_stats = dcache.wbb->stats_t;
                }
                else
                {
                        dcache.missb->stats_t.readAc.access  = dcache.caches->stats_t.readAc.miss;
                        dcache.missb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
                        dcache.missb->rtp_stats = dcache.missb->stats_t;

                        dcache.ifb->stats_t.readAc.access  = dcache.caches->stats_t.readAc.miss;
                        dcache.ifb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
                        dcache.ifb->rtp_stats = dcache.ifb->stats_t;

                        dcache.prefetchb->stats_t.readAc.access  = dcache.caches->stats_t.readAc.miss;
                        dcache.prefetchb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
                        dcache.prefetchb->rtp_stats = dcache.prefetchb->stats_t;
                }

                LSQ->stats_t.readAc.access  = (XML->sys.core[ithCore].load_instructions + XML->sys.core[ithCore].store_instructions)*2;//flush overhead considered
                LSQ->stats_t.writeAc.access = (XML->sys.core[ithCore].load_instructions + XML->sys.core[ithCore].store_instructions)*2;
                LSQ->rtp_stats = LSQ->stats_t;

                if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].load_buffer_size >0))
                {
                        LoadQ->stats_t.readAc.access  = XML->sys.core[ithCore].load_instructions + XML->sys.core[ithCore].store_instructions;
                        LoadQ->stats_t.writeAc.access = XML->sys.core[ithCore].load_instructions + XML->sys.core[ithCore].store_instructions;
                        LoadQ->rtp_stats = LoadQ->stats_t;
                }

            }

        dcache.power_t.reset();
        LSQ->power_t.reset();
    dcache.power_t.readOp.dynamic	+= (dcache.caches->stats_t.readAc.hit*dcache.caches->local_result.power.readOp.dynamic+
                dcache.caches->stats_t.readAc.miss*dcache.caches->local_result.power.readOp.dynamic+
                dcache.caches->stats_t.writeAc.miss*dcache.caches->local_result.tag_array2->power.readOp.dynamic+
                dcache.caches->stats_t.writeAc.access*dcache.caches->local_result.power.writeOp.dynamic);

    if (cache_p==Write_back)
    {//write miss will generate a write later
        dcache.power_t.readOp.dynamic	+= dcache.caches->stats_t.writeAc.miss*dcache.caches->local_result.power.writeOp.dynamic;
    }

    dcache.power_t.readOp.dynamic	+=  dcache.missb->stats_t.readAc.access*dcache.missb->local_result.power.searchOp.dynamic +
            dcache.missb->stats_t.writeAc.access*dcache.missb->local_result.power.writeOp.dynamic;//each access to missb involves a CAM and a write
    dcache.power_t.readOp.dynamic	+=  dcache.ifb->stats_t.readAc.access*dcache.ifb->local_result.power.searchOp.dynamic +
            dcache.ifb->stats_t.writeAc.access*dcache.ifb->local_result.power.writeOp.dynamic;
    dcache.power_t.readOp.dynamic	+=  dcache.prefetchb->stats_t.readAc.access*dcache.prefetchb->local_result.power.searchOp.dynamic +
            dcache.prefetchb->stats_t.writeAc.access*dcache.prefetchb->local_result.power.writeOp.dynamic;
    if (cache_p==Write_back)
    {
        dcache.power_t.readOp.dynamic	+=  dcache.wbb->stats_t.readAc.access*dcache.wbb->local_result.power.searchOp.dynamic
                        + dcache.wbb->stats_t.writeAc.access*dcache.wbb->local_result.power.writeOp.dynamic;
    }

    if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].load_buffer_size >0))
    {
        LoadQ->power_t.reset();
        LoadQ->power_t.readOp.dynamic  +=  LoadQ->stats_t.readAc.access*(LoadQ->local_result.power.searchOp.dynamic+ LoadQ->local_result.power.readOp.dynamic)+
                LoadQ->stats_t.writeAc.access*LoadQ->local_result.power.writeOp.dynamic;//every memory access invloves at least two operations on LoadQ

        LSQ->power_t.readOp.dynamic  +=  LSQ->stats_t.readAc.access*(LSQ->local_result.power.searchOp.dynamic + LSQ->local_result.power.readOp.dynamic)
                        + LSQ->stats_t.writeAc.access*LSQ->local_result.power.writeOp.dynamic;//every memory access invloves at least two operations on LSQ

    }
    else
    {
        LSQ->power_t.readOp.dynamic  +=  LSQ->stats_t.readAc.access*(LSQ->local_result.power.searchOp.dynamic + LSQ->local_result.power.readOp.dynamic)
                        + LSQ->stats_t.writeAc.access*LSQ->local_result.power.writeOp.dynamic;//every memory access invloves at least two operations on LSQ

    }

    if (is_tdp)
    {
//    	dcache.power = dcache.power_t + (dcache.caches->local_result.power)*pppm_lkg +
//    			(dcache.missb->local_result.power +
//    			dcache.ifb->local_result.power +
//    			dcache.prefetchb->local_result.power +
//    			dcache.wbb->local_result.power)*pppm_Isub;
        dcache.power = dcache.power_t + (dcache.caches->local_result.power +
                        dcache.missb->local_result.power +
                        dcache.ifb->local_result.power +
                        dcache.prefetchb->local_result.power) *pppm_lkg;
        if (cache_p==Write_back)
        {
                dcache.power = dcache.power + dcache.wbb->local_result.power*pppm_lkg;
        }

        LSQ->power = LSQ->power_t + LSQ->local_result.power *pppm_lkg;
        power     = power + dcache.power + LSQ->power;

        if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].load_buffer_size >0))
        {
                LoadQ->power = LoadQ->power_t + LoadQ->local_result.power *pppm_lkg;
                power     = power + LoadQ->power;
        }
    }
    else
    {
//    	dcache.rt_power = dcache.power_t + (dcache.caches->local_result.power +
//    			dcache.missb->local_result.power +
//    			dcache.ifb->local_result.power +
//    			dcache.prefetchb->local_result.power +
//    			dcache.wbb->local_result.power)*pppm_lkg;
        dcache.rt_power = dcache.power_t + (dcache.caches->local_result.power +
                        dcache.missb->local_result.power +
                        dcache.ifb->local_result.power +
                        dcache.prefetchb->local_result.power )*pppm_lkg;

        if (cache_p==Write_back)
        {
                dcache.rt_power = dcache.rt_power + dcache.wbb->local_result.power*pppm_lkg;
        }

        LSQ->rt_power = LSQ->power_t + LSQ->local_result.power *pppm_lkg;
        rt_power     = rt_power + dcache.rt_power + LSQ->rt_power;

        if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].load_buffer_size >0))
        {
                LoadQ->rt_power = LoadQ->power_t + LoadQ->local_result.power *pppm_lkg;
                rt_power     = rt_power + LoadQ->rt_power;
        }
    }
}


void LoadStoreU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;


        if (is_tdp)
        {
                cout << indent_str << "Data Cache:" << endl;
                cout << indent_str_next << "Area = " << dcache.area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << dcache.power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? dcache.power.readOp.longer_channel_leakage:dcache.power.readOp.leakage )<<" W" << endl;
                cout << indent_str_next << "Gate Leakage = " << dcache.power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << dcache.rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                if (coredynp.core_ty==Inorder)
                {
                        cout << indent_str << "Load/Store Queue:" << endl;
                        cout << indent_str_next << "Area = " << LSQ->area.get_area()*1e-6  << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << LSQ->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? LSQ->power.readOp.longer_channel_leakage:LSQ->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << LSQ->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << LSQ->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                }
                else

                {
                        if (XML->sys.core[ithCore].load_buffer_size >0)
                        {
                                cout << indent_str << "LoadQ:" << endl;
                                cout << indent_str_next << "Area = " << LoadQ->area.get_area() *1e-6 << " mm^2" << endl;
                                cout << indent_str_next << "Peak Dynamic = " << LoadQ->power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? LoadQ->power.readOp.longer_channel_leakage:LoadQ->power.readOp.leakage)  << " W" << endl;
                                cout << indent_str_next << "Gate Leakage = " << LoadQ->power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "Runtime Dynamic = " << LoadQ->rt_power.readOp.dynamic/executionTime << " W" << endl;
                                cout <<endl;
                        }
                        cout << indent_str<< "StoreQ:" << endl;
                        cout << indent_str_next << "Area = " << LSQ->area.get_area()  *1e-6<< " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << LSQ->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? LSQ->power.readOp.longer_channel_leakage:LSQ->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << LSQ->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << LSQ->rt_power.readOp.dynamic/executionTime<< " W" << endl;
                        cout <<endl;
                }
        }
        else
        {
                cout << indent_str_next << "Data Cache    Peak Dynamic = " << dcache.rt_power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Data Cache    Subthreshold Leakage = " << dcache.rt_power.readOp.leakage <<" W" << endl;
                cout << indent_str_next << "Data Cache    Gate Leakage = " << dcache.rt_power.readOp.gate_leakage << " W" << endl;
                if (coredynp.core_ty==Inorder)
                {
                        cout << indent_str_next << "Load/Store Queue   Peak Dynamic = " << LSQ->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Load/Store Queue   Subthreshold Leakage = " << LSQ->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "Load/Store Queue   Gate Leakage = " << LSQ->rt_power.readOp.gate_leakage  << " W" << endl;
                }
                else
                {
                        cout << indent_str_next << "LoadQ   Peak Dynamic = " << LoadQ->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "LoadQ   Subthreshold Leakage = " << LoadQ->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "LoadQ   Gate Leakage = " << LoadQ->rt_power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "StoreQ   Peak Dynamic = " << LSQ->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "StoreQ   Subthreshold Leakage = " << LSQ->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "StoreQ   Gate Leakage = " << LSQ->rt_power.readOp.gate_leakage  << " W" << endl;
                }
        }

}

void MemManU::computeEnergy(bool is_tdp)
{

        if (!exist) return;
        if (is_tdp)
    {
        //init stats for Peak
        itlb->stats_t.readAc.access  = itlb->l_ip.num_search_ports;
        itlb->stats_t.readAc.miss    = 0;
        itlb->stats_t.readAc.hit     = itlb->stats_t.readAc.access - itlb->stats_t.readAc.miss;
        itlb->tdp_stats = itlb->stats_t;

        dtlb->stats_t.readAc.access  = dtlb->l_ip.num_search_ports*coredynp.LSU_duty_cycle;
        dtlb->stats_t.readAc.miss    = 0;
        dtlb->stats_t.readAc.hit     = dtlb->stats_t.readAc.access - dtlb->stats_t.readAc.miss;
        dtlb->tdp_stats = dtlb->stats_t;
     }
    else
    {
        //init stats for Runtime Dynamic (RTP)
        itlb->stats_t.readAc.access  = XML->sys.core[ithCore].itlb.total_accesses;
        itlb->stats_t.readAc.miss    = XML->sys.core[ithCore].itlb.total_misses;
        itlb->stats_t.readAc.hit     = itlb->stats_t.readAc.access - itlb->stats_t.readAc.miss;
        itlb->rtp_stats = itlb->stats_t;

        dtlb->stats_t.readAc.access  = XML->sys.core[ithCore].dtlb.total_accesses;
        dtlb->stats_t.readAc.miss    = XML->sys.core[ithCore].dtlb.total_misses;
        dtlb->stats_t.readAc.hit     = dtlb->stats_t.readAc.access - dtlb->stats_t.readAc.miss;
        dtlb->rtp_stats = dtlb->stats_t;
    }

    itlb->power_t.reset();
    dtlb->power_t.reset();
        itlb->power_t.readOp.dynamic +=  itlb->stats_t.readAc.access*itlb->local_result.power.searchOp.dynamic//FA spent most power in tag, so use total access not hits
                              +itlb->stats_t.readAc.miss*itlb->local_result.power.writeOp.dynamic;
        dtlb->power_t.readOp.dynamic +=  dtlb->stats_t.readAc.access*dtlb->local_result.power.searchOp.dynamic//FA spent most power in tag, so use total access not hits
                              +dtlb->stats_t.readAc.miss*dtlb->local_result.power.writeOp.dynamic;

        if (is_tdp)
            {
                itlb->power = itlb->power_t + itlb->local_result.power *pppm_lkg;
                dtlb->power = dtlb->power_t + dtlb->local_result.power *pppm_lkg;
                power     = power + itlb->power + dtlb->power;
            }
            else
            {
                        itlb->rt_power = itlb->power_t + itlb->local_result.power *pppm_lkg;
                        dtlb->rt_power = dtlb->power_t + dtlb->local_result.power *pppm_lkg;
                        rt_power     = rt_power + itlb->rt_power + dtlb->rt_power;
            }
}

void MemManU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;




        if (is_tdp)
        {
                cout << indent_str << "Itlb:" << endl;
                cout << indent_str_next << "Area = " << itlb->area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << itlb->power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? itlb->power.readOp.longer_channel_leakage:itlb->power.readOp.leakage) <<" W" << endl;
                cout << indent_str_next << "Gate Leakage = " << itlb->power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << itlb->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                cout << indent_str<< "Dtlb:" << endl;
                cout << indent_str_next << "Area = " << dtlb->area.get_area()*1e-6  << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << dtlb->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? dtlb->power.readOp.longer_channel_leakage:dtlb->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << dtlb->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << dtlb->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
        }
        else
        {
                cout << indent_str_next << "Itlb    Peak Dynamic = " << itlb->rt_power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Itlb    Subthreshold Leakage = " << itlb->rt_power.readOp.leakage <<" W" << endl;
                cout << indent_str_next << "Itlb    Gate Leakage = " << itlb->rt_power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Dtlb   Peak Dynamic = " << dtlb->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Dtlb   Subthreshold Leakage = " << dtlb->rt_power.readOp.leakage  << " W" << endl;
                cout << indent_str_next << "Dtlb   Gate Leakage = " << dtlb->rt_power.readOp.gate_leakage  << " W" << endl;
        }

}

void RegFU::computeEnergy(bool is_tdp)
{
/*
 * Architecture RF and physical RF cannot be present at the same time.
 * Therefore, the RF stats can only refer to either ARF or PRF;
 * And the same stats can be used for both.
 */
        if (!exist) return;
        if (is_tdp)
    {
        //init stats for Peak
        IRF->stats_t.readAc.access  = coredynp.issueW*2*(coredynp.ALU_duty_cycle*1.1+
                        (coredynp.num_muls>0?coredynp.MUL_duty_cycle:0))*coredynp.num_pipelines;
        IRF->stats_t.writeAc.access  = coredynp.issueW*(coredynp.ALU_duty_cycle*1.1+
                        (coredynp.num_muls>0?coredynp.MUL_duty_cycle:0))*coredynp.num_pipelines;
        //Rule of Thumb: about 10% RF related instructions do not need to access ALUs
        IRF->tdp_stats = IRF->stats_t;

        FRF->stats_t.readAc.access  = FRF->l_ip.num_rd_ports*coredynp.FPU_duty_cycle*1.05*coredynp.num_fp_pipelines;
        FRF->stats_t.writeAc.access  = FRF->l_ip.num_wr_ports*coredynp.FPU_duty_cycle*1.05*coredynp.num_fp_pipelines;
        FRF->tdp_stats = FRF->stats_t;
        if (coredynp.regWindowing)
        {
                RFWIN->stats_t.readAc.access  = 0;//0.5*RFWIN->l_ip.num_rw_ports;
                RFWIN->stats_t.writeAc.access  = 0;//0.5*RFWIN->l_ip.num_rw_ports;
                RFWIN->tdp_stats = RFWIN->stats_t;
        }
     }
    else
    {
        //init stats for Runtime Dynamic (RTP)
        IRF->stats_t.readAc.access  = XML->sys.core[ithCore].int_regfile_reads;//TODO: no diff on archi and phy
        IRF->stats_t.writeAc.access  = XML->sys.core[ithCore].int_regfile_writes;
        IRF->rtp_stats = IRF->stats_t;

        FRF->stats_t.readAc.access  = XML->sys.core[ithCore].float_regfile_reads;
        FRF->stats_t.writeAc.access  = XML->sys.core[ithCore].float_regfile_writes;
        FRF->rtp_stats = FRF->stats_t;
        if (coredynp.regWindowing)
        {
                RFWIN->stats_t.readAc.access  = XML->sys.core[ithCore].function_calls*16;
                RFWIN->stats_t.writeAc.access  = XML->sys.core[ithCore].function_calls*16;
                RFWIN->rtp_stats = RFWIN->stats_t;

                IRF->stats_t.readAc.access  = XML->sys.core[ithCore].int_regfile_reads +
                     XML->sys.core[ithCore].function_calls*16;
                IRF->stats_t.writeAc.access  = XML->sys.core[ithCore].int_regfile_writes +
                     XML->sys.core[ithCore].function_calls*16;
                IRF->rtp_stats = IRF->stats_t;

                FRF->stats_t.readAc.access  = XML->sys.core[ithCore].float_regfile_reads +
                     XML->sys.core[ithCore].function_calls*16;;
                FRF->stats_t.writeAc.access  = XML->sys.core[ithCore].float_regfile_writes+
                     XML->sys.core[ithCore].function_calls*16;;
                FRF->rtp_stats = FRF->stats_t;
        }
    }
        IRF->power_t.reset();
        FRF->power_t.reset();
        IRF->power_t.readOp.dynamic  +=  (IRF->stats_t.readAc.access*IRF->local_result.power.readOp.dynamic
                        +IRF->stats_t.writeAc.access*IRF->local_result.power.writeOp.dynamic);
        FRF->power_t.readOp.dynamic  +=  (FRF->stats_t.readAc.access*FRF->local_result.power.readOp.dynamic
                        +FRF->stats_t.writeAc.access*FRF->local_result.power.writeOp.dynamic);
        if (coredynp.regWindowing)
        {
                RFWIN->power_t.reset();
                RFWIN->power_t.readOp.dynamic   +=  (RFWIN->stats_t.readAc.access*RFWIN->local_result.power.readOp.dynamic +
                                RFWIN->stats_t.writeAc.access*RFWIN->local_result.power.writeOp.dynamic);
        }

        if (is_tdp)
        {
                IRF->power  =  IRF->power_t + IRF->local_result.power *coredynp.pppm_lkg_multhread;
                FRF->power  =  FRF->power_t + FRF->local_result.power *coredynp.pppm_lkg_multhread;
                power	    =  power + (IRF->power + FRF->power);
                if (coredynp.regWindowing)
                {
                        RFWIN->power = RFWIN->power_t + RFWIN->local_result.power *pppm_lkg;
                        power        = power + RFWIN->power;
                }
        }
        else
        {
                IRF->rt_power  =  IRF->power_t + IRF->local_result.power *coredynp.pppm_lkg_multhread;
                FRF->rt_power  =  FRF->power_t + FRF->local_result.power *coredynp.pppm_lkg_multhread;
                rt_power	   =  rt_power + (IRF->power_t + FRF->power_t);
                if (coredynp.regWindowing)
                {
                        RFWIN->rt_power = RFWIN->power_t + RFWIN->local_result.power *pppm_lkg;
                        rt_power        = rt_power + RFWIN->rt_power;
                }
        }
}


void RegFU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;

        if (is_tdp)
        {	cout << indent_str << "Integer RF:" << endl;
                cout << indent_str_next << "Area = " << IRF->area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << IRF->power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? IRF->power.readOp.longer_channel_leakage:IRF->power.readOp.leakage) <<" W" << endl;
                cout << indent_str_next << "Gate Leakage = " << IRF->power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << IRF->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                cout << indent_str<< "Floating Point RF:" << endl;
                cout << indent_str_next << "Area = " << FRF->area.get_area()*1e-6  << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << FRF->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? FRF->power.readOp.longer_channel_leakage:FRF->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << FRF->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << FRF->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                if (coredynp.regWindowing)
                {
                        cout << indent_str << "Register Windows:" << endl;
                        cout << indent_str_next << "Area = " << RFWIN->area.get_area() *1e-6 << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << RFWIN->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? RFWIN->power.readOp.longer_channel_leakage:RFWIN->power.readOp.leakage)  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << RFWIN->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << RFWIN->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                }
        }
        else
        {
                cout << indent_str_next << "Integer RF    Peak Dynamic = " << IRF->rt_power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Integer RF    Subthreshold Leakage = " << IRF->rt_power.readOp.leakage <<" W" << endl;
                cout << indent_str_next << "Integer RF    Gate Leakage = " << IRF->rt_power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Floating Point RF   Peak Dynamic = " << FRF->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Floating Point RF   Subthreshold Leakage = " << FRF->rt_power.readOp.leakage  << " W" << endl;
                cout << indent_str_next << "Floating Point RF   Gate Leakage = " << FRF->rt_power.readOp.gate_leakage  << " W" << endl;
                if (coredynp.regWindowing)
                {
                        cout << indent_str_next << "Register Windows   Peak Dynamic = " << RFWIN->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Register Windows   Subthreshold Leakage = " << RFWIN->rt_power.readOp.leakage  << " W" << endl;
                        cout << indent_str_next << "Register Windows   Gate Leakage = " << RFWIN->rt_power.readOp.gate_leakage  << " W" << endl;
                }
        }
}


void EXECU::computeEnergy(bool is_tdp)
{
        if (!exist) return;
        double pppm_t[4]    = {1,1,1,1};
//	rfu->power.reset();
//	rfu->rt_power.reset();
//	scheu->power.reset();
//	scheu->rt_power.reset();
//	exeu->power.reset();
//	exeu->rt_power.reset();

        rfu->computeEnergy(is_tdp);
        scheu->computeEnergy(is_tdp);
        exeu->computeEnergy(is_tdp);
        if (coredynp.num_fpus >0)
        {
                fp_u->computeEnergy(is_tdp);
        }
        if (coredynp.num_muls >0)
        {
                mul->computeEnergy(is_tdp);
        }

        if (is_tdp)
        {
                set_pppm(pppm_t, 2*coredynp.ALU_cdb_duty_cycle, 2, 2, 2*coredynp.ALU_cdb_duty_cycle);//2 means two source operands needs to be passed for each int instruction.
                bypass.power = bypass.power + intTagBypass->power*pppm_t + int_bypass->power*pppm_t;
                if (coredynp.num_muls >0)
                {
                        set_pppm(pppm_t, 2*coredynp.MUL_cdb_duty_cycle, 2, 2, 2*coredynp.MUL_cdb_duty_cycle);//2 means two source operands needs to be passed for each int instruction.
                        bypass.power = bypass.power + intTag_mul_Bypass->power*pppm_t + int_mul_bypass->power*pppm_t;
                        power      = power + mul->power;
                }
                if (coredynp.num_fpus>0)
                {
                        set_pppm(pppm_t, 3*coredynp.FPU_cdb_duty_cycle, 3, 3, 3*coredynp.FPU_cdb_duty_cycle);//3 means three source operands needs to be passed for each fp instruction.
                        bypass.power = bypass.power + fp_bypass->power*pppm_t  + fpTagBypass->power*pppm_t ;
                        power      = power + fp_u->power;
                }

                power      = power + rfu->power + exeu->power + bypass.power + scheu->power;
        }
        else
        {
                set_pppm(pppm_t, XML->sys.core[ithCore].cdb_alu_accesses, 2, 2, XML->sys.core[ithCore].cdb_alu_accesses);
                bypass.rt_power = bypass.rt_power + intTagBypass->power*pppm_t;
                bypass.rt_power = bypass.rt_power + int_bypass->power*pppm_t;

                if (coredynp.num_muls >0)
                {
                        set_pppm(pppm_t, XML->sys.core[ithCore].cdb_mul_accesses, 2, 2, XML->sys.core[ithCore].cdb_mul_accesses);//2 means two source operands needs to be passed for each int instruction.
                        bypass.rt_power = bypass.rt_power + intTag_mul_Bypass->power*pppm_t + int_mul_bypass->power*pppm_t;
                        rt_power      = rt_power + mul->rt_power;
                }

                if (coredynp.num_fpus>0)
                {
                        set_pppm(pppm_t, XML->sys.core[ithCore].cdb_fpu_accesses, 3, 3, XML->sys.core[ithCore].cdb_fpu_accesses);
                        bypass.rt_power = bypass.rt_power + fp_bypass->power*pppm_t;
                        bypass.rt_power = bypass.rt_power + fpTagBypass->power*pppm_t;
                        rt_power      = rt_power + fp_u->rt_power;
                }
                rt_power      = rt_power + rfu->rt_power + exeu->rt_power + bypass.rt_power + scheu->rt_power;
        }
}

void EXECU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        if (!exist) return;
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;


//	cout << indent_str_next << "Results Broadcast Bus Area = " << bypass->area.get_area() *1e-6 << " mm^2" << endl;
        if (is_tdp)
        {
                cout << indent_str << "Register Files:" << endl;
                cout << indent_str_next << "Area = " << rfu->area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << rfu->power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? rfu->power.readOp.longer_channel_leakage:rfu->power.readOp.leakage) <<" W" << endl;
                cout << indent_str_next << "Gate Leakage = " << rfu->power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << rfu->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                if (plevel>3){
                        rfu->displayEnergy(indent+4,is_tdp);
                }
                cout << indent_str << "Instruction Scheduler:" << endl;
                cout << indent_str_next << "Area = " << scheu->area.get_area()*1e-6  << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << scheu->power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? scheu->power.readOp.longer_channel_leakage:scheu->power.readOp.leakage)  << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << scheu->power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << scheu->rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
                if (plevel>3){
                        scheu->displayEnergy(indent+4,is_tdp);
                }
                exeu->displayEnergy(indent,is_tdp);
                if (coredynp.num_fpus>0)
                {
                        fp_u->displayEnergy(indent,is_tdp);
                }
                if (coredynp.num_muls >0)
                {
                        mul->displayEnergy(indent,is_tdp);
                }
                cout << indent_str << "Results Broadcast Bus:" << endl;
                cout << indent_str_next << "Area Overhead = " << bypass.area.get_area()*1e-6  << " mm^2" << endl;
                cout << indent_str_next << "Peak Dynamic = " << bypass.power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Subthreshold Leakage = "
                        << (long_channel? bypass.power.readOp.longer_channel_leakage:bypass.power.readOp.leakage ) << " W" << endl;
                cout << indent_str_next << "Gate Leakage = " << bypass.power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Runtime Dynamic = " << bypass.rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout <<endl;
        }
        else
        {
                cout << indent_str_next << "Register Files    Peak Dynamic = " << rfu->rt_power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str_next << "Register Files    Subthreshold Leakage = " << rfu->rt_power.readOp.leakage <<" W" << endl;
                cout << indent_str_next << "Register Files    Gate Leakage = " << rfu->rt_power.readOp.gate_leakage << " W" << endl;
                cout << indent_str_next << "Instruction Sheduler   Peak Dynamic = " << scheu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Instruction Sheduler   Subthreshold Leakage = " << scheu->rt_power.readOp.leakage  << " W" << endl;
                cout << indent_str_next << "Instruction Sheduler   Gate Leakage = " << scheu->rt_power.readOp.gate_leakage  << " W" << endl;
                cout << indent_str_next << "Results Broadcast Bus   Peak Dynamic = " << bypass.rt_power.readOp.dynamic*clockRate  << " W" << endl;
                cout << indent_str_next << "Results Broadcast Bus   Subthreshold Leakage = " << bypass.rt_power.readOp.leakage  << " W" << endl;
                cout << indent_str_next << "Results Broadcast Bus   Gate Leakage = " << bypass.rt_power.readOp.gate_leakage  << " W" << endl;
        }

}

void Core::computeEnergy(bool is_tdp)
{
        //power_point_product_masks
        double pppm_t[4]    = {1,1,1,1};
    double rtp_pipeline_coe;
    double num_units = 4.0;
        if (is_tdp)
        {
                ifu->computeEnergy(is_tdp);
                lsu->computeEnergy(is_tdp);
                mmu->computeEnergy(is_tdp);
                exu->computeEnergy(is_tdp);

                if (coredynp.core_ty==OOO)
                {
                        num_units = 5.0;
                        rnu->computeEnergy(is_tdp);
                        set_pppm(pppm_t, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
                        if (rnu->exist)
                        {
                                rnu->power = rnu->power + corepipe->power*pppm_t;
                                power     = power + rnu->power;
                        }
                }

                if (ifu->exist)
                {
                        set_pppm(pppm_t, coredynp.num_pipelines/num_units*coredynp.IFU_duty_cycle, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
//			cout << "IFU = " << ifu->power.readOp.dynamic*clockRate  << " W" << endl;
                        ifu->power = ifu->power + corepipe->power*pppm_t;
//			cout << "IFU = " << ifu->power.readOp.dynamic*clockRate  << " W" << endl;
//			cout << "1/4 pipe = " << corepipe->power.readOp.dynamic*clockRate/num_units  << " W" << endl;
                        power     = power + ifu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
                }
                if (lsu->exist)
                {
                        set_pppm(pppm_t, coredynp.num_pipelines/num_units*coredynp.LSU_duty_cycle, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
                        lsu->power = lsu->power + corepipe->power*pppm_t;
//			cout << "LSU = " << lsu->power.readOp.dynamic*clockRate  << " W" << endl;
                        power     = power + lsu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
                }
                if (exu->exist)
                {
                        set_pppm(pppm_t, coredynp.num_pipelines/num_units*coredynp.ALU_duty_cycle, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
                        exu->power = exu->power + corepipe->power*pppm_t;
//			cout << "EXE = " << exu->power.readOp.dynamic*clockRate  << " W" << endl;
                        power     = power + exu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
                }
                if (mmu->exist)
                {
                        set_pppm(pppm_t, coredynp.num_pipelines/num_units*(0.5+0.5*coredynp.LSU_duty_cycle), coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
                        mmu->power = mmu->power + corepipe->power*pppm_t;
//			cout << "MMU = " << mmu->power.readOp.dynamic*clockRate  << " W" << endl;
                        power     = power +  mmu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
                }

                power     = power +  undiffCore->power;

                if (XML->sys.Private_L2)
                {

                        l2cache->computeEnergy(is_tdp);
                        set_pppm(pppm_t,l2cache->cachep.clockRate/clockRate, 1,1,1);
                        //l2cache->power = l2cache->power*pppm_t;
                        power = power  + l2cache->power*pppm_t;
                }
        }
        else
        {
                ifu->computeEnergy(is_tdp);
                lsu->computeEnergy(is_tdp);
                mmu->computeEnergy(is_tdp);
                exu->computeEnergy(is_tdp);
                if (coredynp.core_ty==OOO)
                {
                        num_units = 5.0;
                        rnu->computeEnergy(is_tdp);
                set_pppm(pppm_t, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
                        if (rnu->exist)
                        {
                rnu->rt_power = rnu->rt_power + corepipe->power*pppm_t;

                        rt_power      = rt_power + rnu->rt_power;
                        }
                }
                else
                {
                        if (XML->sys.homogeneous_cores==1)
                        {
                                rtp_pipeline_coe = coredynp.pipeline_duty_cycle * XML->sys.total_cycles * XML->sys.number_of_cores;
                        }
                        else
                        {
                                rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.total_cycles;
                        }
                set_pppm(pppm_t, coredynp.num_pipelines*rtp_pipeline_coe/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
                }

                if (ifu->exist)
                {
                        ifu->rt_power = ifu->rt_power + corepipe->power*pppm_t;
                        rt_power     = rt_power + ifu->rt_power ;
                }
                if (lsu->exist)
                {
                        lsu->rt_power = lsu->rt_power + corepipe->power*pppm_t;
                        rt_power     = rt_power  + lsu->rt_power;
                }
                if (exu->exist)
                {
                        exu->rt_power = exu->rt_power + corepipe->power*pppm_t;
                        rt_power     = rt_power  + exu->rt_power;
                }
                if (mmu->exist)
                {
                        mmu->rt_power = mmu->rt_power + corepipe->power*pppm_t;
                        rt_power     = rt_power +  mmu->rt_power ;
                }

                rt_power     = rt_power +  undiffCore->power;
//		cout << "EXE = " << exu->power.readOp.dynamic*clockRate  << " W" << endl;
                if (XML->sys.Private_L2)
                {

                        l2cache->computeEnergy(is_tdp);
                        //set_pppm(pppm_t,1/l2cache->cachep.executionTime, 1,1,1);
                        //l2cache->rt_power = l2cache->rt_power*pppm_t;
                        rt_power = rt_power  + l2cache->rt_power;
                }
        }

}

void Core::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;
        if (is_tdp)
        {
                cout << "Core:" << endl;
                cout << indent_str << "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic*clockRate << " W" << endl;
                cout << indent_str << "Subthreshold Leakage = "
                        << (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
                //cout << indent_str << "Subthreshold Leakage = " << power.readOp.longer_channel_leakage <<" W" << endl;
                cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
                cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
                cout<<endl;
                if (ifu->exist)
                {
                        cout << indent_str << "Instruction Fetch Unit:" << endl;
                        cout << indent_str_next << "Area = " << ifu->area.get_area()*1e-6<< " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << ifu->power.readOp.dynamic*clockRate << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? ifu->power.readOp.longer_channel_leakage:ifu->power.readOp.leakage) <<" W" << endl;
                        //cout << indent_str_next << "Subthreshold Leakage = " << ifu->power.readOp.longer_channel_leakage <<" W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << ifu->power.readOp.gate_leakage << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << ifu->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        if (plevel >2){
                                ifu->displayEnergy(indent+4,plevel,is_tdp);
                        }
                }
                if (coredynp.core_ty==OOO)
                {
                        if (rnu->exist)
                        {
                                cout << indent_str<< "Renaming Unit:" << endl;
                                cout << indent_str_next << "Area = " << rnu->area.get_area()*1e-6  << " mm^2" << endl;
                                cout << indent_str_next << "Peak Dynamic = " << rnu->power.readOp.dynamic*clockRate  << " W" << endl;
                                cout << indent_str_next << "Subthreshold Leakage = "
                                        << (long_channel? rnu->power.readOp.longer_channel_leakage:rnu->power.readOp.leakage)  << " W" << endl;
                                //cout << indent_str_next << "Subthreshold Leakage = " << rnu->power.readOp.longer_channel_leakage  << " W" << endl;
                                cout << indent_str_next << "Gate Leakage = " << rnu->power.readOp.gate_leakage  << " W" << endl;
                                cout << indent_str_next << "Runtime Dynamic = " << rnu->rt_power.readOp.dynamic/executionTime << " W" << endl;
                                cout <<endl;
                                if (plevel >2){
                                        rnu->displayEnergy(indent+4,plevel,is_tdp);
                                }
                        }

                }
                if (lsu->exist)
                {
                        cout << indent_str<< "Load Store Unit:" << endl;
                        cout << indent_str_next << "Area = " << lsu->area.get_area()*1e-6  << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << lsu->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? lsu->power.readOp.longer_channel_leakage:lsu->power.readOp.leakage ) << " W" << endl;
                        //cout << indent_str_next << "Subthreshold Leakage = " << lsu->power.readOp.longer_channel_leakage  << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << lsu->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << lsu->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        if (plevel >2){
                                lsu->displayEnergy(indent+4,plevel,is_tdp);
                        }
                }
                if (mmu->exist)
                {
                        cout << indent_str<< "Memory Management Unit:" << endl;
                        cout << indent_str_next << "Area = " << mmu->area.get_area() *1e-6 << " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << mmu->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? mmu->power.readOp.longer_channel_leakage:mmu->power.readOp.leakage)   << " W" << endl;
                        //cout << indent_str_next << "Subthreshold Leakage = " << mmu->power.readOp.longer_channel_leakage   << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << mmu->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << mmu->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        if (plevel >2){
                                mmu->displayEnergy(indent+4,plevel,is_tdp);
                        }
                }
                if (exu->exist)
                {
                        cout << indent_str<< "Execution Unit:" << endl;
                        cout << indent_str_next << "Area = " << exu->area.get_area()  *1e-6<< " mm^2" << endl;
                        cout << indent_str_next << "Peak Dynamic = " << exu->power.readOp.dynamic*clockRate  << " W" << endl;
                        cout << indent_str_next << "Subthreshold Leakage = "
                                << (long_channel? exu->power.readOp.longer_channel_leakage:exu->power.readOp.leakage)   << " W" << endl;
                        //cout << indent_str_next << "Subthreshold Leakage = " << exu->power.readOp.longer_channel_leakage << " W" << endl;
                        cout << indent_str_next << "Gate Leakage = " << exu->power.readOp.gate_leakage  << " W" << endl;
                        cout << indent_str_next << "Runtime Dynamic = " << exu->rt_power.readOp.dynamic/executionTime << " W" << endl;
                        cout <<endl;
                        if (plevel >2){
                                exu->displayEnergy(indent+4,plevel,is_tdp);
                        }
                }
//		if (plevel >2)
//		{
//			if (undiffCore->exist)
//			{
//				cout << indent_str << "Undifferentiated Core" << endl;
//				cout << indent_str_next << "Area = " << undiffCore->area.get_area()*1e-6<< " mm^2" << endl;
//				cout << indent_str_next << "Peak Dynamic = " << undiffCore->power.readOp.dynamic*clockRate << " W" << endl;
////				cout << indent_str_next << "Subthreshold Leakage = " << undiffCore->power.readOp.leakage <<" W" << endl;
//				cout << indent_str_next << "Subthreshold Leakage = "
//								<< (long_channel? undiffCore->power.readOp.longer_channel_leakage:undiffCore->power.readOp.leakage)   << " W" << endl;
//				cout << indent_str_next << "Gate Leakage = " << undiffCore->power.readOp.gate_leakage << " W" << endl;
//				//		cout << indent_str_next << "Runtime Dynamic = " << undiffCore->rt_power.readOp.dynamic/executionTime << " W" << endl;
//				cout <<endl;
//			}
//		}
                if (XML->sys.Private_L2)
                {

                        l2cache->displayEnergy(4,is_tdp);
                }

        }
        else
        {
//		cout << indent_str_next << "Instruction Fetch Unit    Peak Dynamic = " << ifu->rt_power.readOp.dynamic*clockRate << " W" << endl;
//		cout << indent_str_next << "Instruction Fetch Unit    Subthreshold Leakage = " << ifu->rt_power.readOp.leakage <<" W" << endl;
//		cout << indent_str_next << "Instruction Fetch Unit    Gate Leakage = " << ifu->rt_power.readOp.gate_leakage << " W" << endl;
//		cout << indent_str_next << "Load Store Unit   Peak Dynamic = " << lsu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Load Store Unit   Subthreshold Leakage = " << lsu->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Load Store Unit   Gate Leakage = " << lsu->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "Memory Management Unit   Peak Dynamic = " << mmu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Memory Management Unit   Subthreshold Leakage = " << mmu->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Memory Management Unit   Gate Leakage = " << mmu->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "Execution Unit   Peak Dynamic = " << exu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Execution Unit   Subthreshold Leakage = " << exu->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Execution Unit   Gate Leakage = " << exu->rt_power.readOp.gate_leakage  << " W" << endl;
        }
}
InstFetchU ::~InstFetchU(){

        if (!exist) return;
        if(IB) 	                   {delete IB; IB = 0;}
        if(ID_inst) 	           {delete ID_inst; ID_inst = 0;}
        if(ID_operand) 	           {delete ID_operand; ID_operand = 0;}
        if(ID_misc) 	           {delete ID_misc; ID_misc = 0;}
        if (coredynp.predictionW>0)
        {
                if(BTB) 	               {delete BTB; BTB = 0;}
                if(BPT) 	               {delete BPT; BPT = 0;}
        }
}

BranchPredictor ::~BranchPredictor(){

        if (!exist) return;
        if(globalBPT) 	           {delete globalBPT; globalBPT = 0;}
        if(localBPT) 	           {delete localBPT; localBPT = 0;}
    if(L1_localBPT) 	       {delete L1_localBPT; L1_localBPT = 0;}
    if(L2_localBPT) 	       {delete L2_localBPT; L2_localBPT = 0;}
    if(chooser) 	           {delete chooser; chooser = 0;}
    if(RAS) 	               {delete RAS; RAS = 0;}
        }

RENAMINGU ::~RENAMINGU(){

        if (!exist) return;
        if(iFRAT ) 	               {delete iFRAT; iFRAT = 0;}
    if(fFRAT ) 	               {delete fFRAT; fFRAT =0;}
    if(iRRAT)                  {delete iRRAT; iRRAT = 0;}
    if(iFRAT)                  {delete iFRAT; iFRAT = 0;}
    if(ifreeL)                 {delete ifreeL;ifreeL= 0;}
    if(ffreeL)                 {delete ffreeL;ffreeL= 0;}
    if(idcl)                   {delete idcl;  idcl = 0;}
    if(fdcl)                   {delete fdcl;  fdcl = 0;}
    if(RAHT)                   {delete RAHT;  RAHT = 0;}
        }

LoadStoreU ::~LoadStoreU(){

        if (!exist) return;
        if(LSQ) 	               {delete LSQ; LSQ = 0;}
        }

MemManU ::~MemManU(){

        if (!exist) return;
        if(itlb) 	               {delete itlb; itlb = 0;}
    if(dtlb) 	               {delete dtlb; dtlb = 0;}
        }

RegFU ::~RegFU(){

        if (!exist) return;
        if(IRF) 	               {delete IRF; IRF = 0;}
    if(FRF) 	               {delete FRF; FRF = 0;}
    if(RFWIN) 	               {delete RFWIN; RFWIN = 0;}
        }

SchedulerU ::~SchedulerU(){

        if (!exist) return;
        if(int_inst_window) 	   {delete int_inst_window; int_inst_window = 0;}
        if(fp_inst_window) 	       {delete int_inst_window; int_inst_window = 0;}
        if(ROB) 	               {delete ROB; ROB = 0;}
    if(instruction_selection)  {delete instruction_selection;instruction_selection = 0;}
        }

EXECU ::~EXECU(){

        if (!exist) return;
        if(int_bypass) 	           {delete int_bypass; int_bypass = 0;}
    if(intTagBypass) 	       {delete intTagBypass; intTagBypass =0;}
    if(int_mul_bypass) 	       {delete int_mul_bypass; int_mul_bypass = 0;}
    if(intTag_mul_Bypass) 	   {delete intTag_mul_Bypass; intTag_mul_Bypass =0;}
    if(fp_bypass) 	           {delete fp_bypass;fp_bypass = 0;}
    if(fpTagBypass) 	       {delete fpTagBypass;fpTagBypass = 0;}
    if(fp_u)                   {delete fp_u;fp_u = 0;}
    if(exeu)                   {delete exeu;exeu = 0;}
    if(mul)                    {delete mul;mul = 0;}
    if(rfu)                    {delete rfu;rfu = 0;}
        if(scheu) 	               {delete scheu; scheu = 0;}
        }

Core ::~Core(){

        if(ifu) 	               {delete ifu; ifu = 0;}
        if(lsu) 	               {delete lsu; lsu = 0;}
        if(rnu) 	               {delete rnu; rnu = 0;}
        if(mmu) 	               {delete mmu; mmu = 0;}
        if(exu) 	               {delete exu; exu = 0;}
    if(corepipe) 	           {delete corepipe; corepipe = 0;}
    if(undiffCore)             {delete undiffCore;undiffCore = 0;}
    if(l2cache)                {delete l2cache;l2cache = 0;}
        }

void Core::set_core_param()
{
        coredynp.opt_local = XML->sys.core[ithCore].opt_local;
        coredynp.x86 = XML->sys.core[ithCore].x86;
        coredynp.Embedded = XML->sys.Embedded;
        coredynp.core_ty   = (enum Core_type)XML->sys.core[ithCore].machine_type;
        coredynp.rm_ty     = (enum Renaming_type)XML->sys.core[ithCore].rename_scheme;
    coredynp.fetchW    = XML->sys.core[ithCore].fetch_width;
    coredynp.decodeW   = XML->sys.core[ithCore].decode_width;
    coredynp.issueW    = XML->sys.core[ithCore].issue_width;
    coredynp.peak_issueW   = XML->sys.core[ithCore].peak_issue_width;
    coredynp.commitW       = XML->sys.core[ithCore].commit_width;
    coredynp.peak_commitW  = XML->sys.core[ithCore].peak_issue_width;
    coredynp.predictionW   = XML->sys.core[ithCore].prediction_width;
    coredynp.fp_issueW     = XML->sys.core[ithCore].fp_issue_width;
    coredynp.fp_decodeW    = XML->sys.core[ithCore].fp_issue_width;
    coredynp.num_alus      = XML->sys.core[ithCore].ALU_per_core;
    coredynp.num_fpus      = XML->sys.core[ithCore].FPU_per_core;
    coredynp.num_muls      = XML->sys.core[ithCore].MUL_per_core;


    coredynp.num_hthreads	     = XML->sys.core[ithCore].number_hardware_threads;
    coredynp.multithreaded       = coredynp.num_hthreads>1? true:false;
    coredynp.instruction_length  = XML->sys.core[ithCore].instruction_length;
    coredynp.pc_width            = XML->sys.virtual_address_width;

        coredynp.opcode_length       = XML->sys.core[ithCore].opcode_width;
    coredynp.micro_opcode_length = XML->sys.core[ithCore].micro_opcode_width;
    coredynp.num_pipelines       = XML->sys.core[ithCore].pipelines_per_core[0];
    coredynp.pipeline_stages     = XML->sys.core[ithCore].pipeline_depth[0];
    coredynp.num_fp_pipelines    = XML->sys.core[ithCore].pipelines_per_core[1];
    coredynp.fp_pipeline_stages  = XML->sys.core[ithCore].pipeline_depth[1];
    coredynp.int_data_width      = int(ceil(XML->sys.machine_bits/32.0))*32;
    coredynp.fp_data_width       = coredynp.int_data_width;
    coredynp.v_address_width     = XML->sys.virtual_address_width;
    coredynp.p_address_width     = XML->sys.physical_address_width;

        coredynp.scheu_ty         = (enum Scheduler_type)XML->sys.core[ithCore].instruction_window_scheme;
        coredynp.arch_ireg_width  =  int(ceil(log2(XML->sys.core[ithCore].archi_Regs_IRF_size)));
        coredynp.arch_freg_width  =  int(ceil(log2(XML->sys.core[ithCore].archi_Regs_FRF_size)));
        coredynp.num_IRF_entry    = XML->sys.core[ithCore].archi_Regs_IRF_size;
        coredynp.num_FRF_entry    = XML->sys.core[ithCore].archi_Regs_FRF_size;
        coredynp.pipeline_duty_cycle = XML->sys.core[ithCore].pipeline_duty_cycle;
        coredynp.total_cycles        = XML->sys.core[ithCore].total_cycles;
        coredynp.busy_cycles         = XML->sys.core[ithCore].busy_cycles;
        coredynp.idle_cycles         = XML->sys.core[ithCore].idle_cycles;

        //Max power duty cycle for peak power estimation
//	if (coredynp.core_ty==OOO)
//	{
//		coredynp.IFU_duty_cycle = 1;
//		coredynp.LSU_duty_cycle = 1;
//		coredynp.MemManU_I_duty_cycle =1;
//		coredynp.MemManU_D_duty_cycle =1;
//		coredynp.ALU_duty_cycle =1;
//		coredynp.MUL_duty_cycle =1;
//		coredynp.FPU_duty_cycle =1;
//		coredynp.ALU_cdb_duty_cycle =1;
//		coredynp.MUL_cdb_duty_cycle =1;
//		coredynp.FPU_cdb_duty_cycle =1;
//	}
//	else
//	{
                coredynp.IFU_duty_cycle = XML->sys.core[ithCore].IFU_duty_cycle;
                coredynp.BR_duty_cycle = XML->sys.core[ithCore].BR_duty_cycle;
                coredynp.LSU_duty_cycle = XML->sys.core[ithCore].LSU_duty_cycle;
                coredynp.MemManU_I_duty_cycle = XML->sys.core[ithCore].MemManU_I_duty_cycle;
                coredynp.MemManU_D_duty_cycle = XML->sys.core[ithCore].MemManU_D_duty_cycle;
                coredynp.ALU_duty_cycle = XML->sys.core[ithCore].ALU_duty_cycle;
                coredynp.MUL_duty_cycle = XML->sys.core[ithCore].MUL_duty_cycle;
                coredynp.FPU_duty_cycle = XML->sys.core[ithCore].FPU_duty_cycle;
                coredynp.ALU_cdb_duty_cycle = XML->sys.core[ithCore].ALU_cdb_duty_cycle;
                coredynp.MUL_cdb_duty_cycle = XML->sys.core[ithCore].MUL_cdb_duty_cycle;
                coredynp.FPU_cdb_duty_cycle = XML->sys.core[ithCore].FPU_cdb_duty_cycle;
//	}


        if (!((coredynp.core_ty==OOO)||(coredynp.core_ty==Inorder)))
        {
                cout<<"Invalid Core Type"<<endl;
                exit(0);
        }
//	if (coredynp.core_ty==OOO)
//	{
//		cout<<"OOO processor models are being updated and will be available in next release"<<endl;
//		exit(0);
//	}
        if (!((coredynp.scheu_ty==PhysicalRegFile)||(coredynp.scheu_ty==ReservationStation)))
        {
                cout<<"Invalid OOO Scheduler Type"<<endl;
                exit(0);
        }

        if (!((coredynp.rm_ty ==RAMbased)||(coredynp.rm_ty ==CAMbased)))
        {
                cout<<"Invalid OOO Renaming Type"<<endl;
                exit(0);
        }

if (coredynp.core_ty==OOO)
{
        if (coredynp.scheu_ty==PhysicalRegFile)
        {
          coredynp.phy_ireg_width  =  int(ceil(log2(XML->sys.core[ithCore].phy_Regs_IRF_size)));
          coredynp.phy_freg_width  =  int(ceil(log2(XML->sys.core[ithCore].phy_Regs_FRF_size)));
          coredynp.num_ifreelist_entries = coredynp.num_IRF_entry  = XML->sys.core[ithCore].phy_Regs_IRF_size;
          coredynp.num_ffreelist_entries = coredynp.num_FRF_entry  = XML->sys.core[ithCore].phy_Regs_FRF_size;
        }
        else if (coredynp.scheu_ty==ReservationStation)
        {//ROB serves as Phy RF in RS based OOO
      coredynp.phy_ireg_width  =  int(ceil(log2(XML->sys.core[ithCore].ROB_size)));
          coredynp.phy_freg_width  =  int(ceil(log2(XML->sys.core[ithCore].ROB_size)));
          coredynp.num_ifreelist_entries = XML->sys.core[ithCore].ROB_size;
          coredynp.num_ffreelist_entries = XML->sys.core[ithCore].ROB_size;

        }

}
        coredynp.globalCheckpoint   =  32;//best check pointing entries for a 4~8 issue OOO should be 16~48;See TR for reference.
        coredynp.perThreadState     =  8;
        coredynp.instruction_length = 32;
        coredynp.clockRate          =  XML->sys.core[ithCore].clock_rate;
        coredynp.clockRate          *= 1e6;
        coredynp.regWindowing= (XML->sys.core[ithCore].register_windows_size>0&&coredynp.core_ty==Inorder)?true:false;
        coredynp.executionTime = XML->sys.total_cycles/coredynp.clockRate;
        set_pppm(coredynp.pppm_lkg_multhread, 0, coredynp.num_hthreads, coredynp.num_hthreads, 0);
}
