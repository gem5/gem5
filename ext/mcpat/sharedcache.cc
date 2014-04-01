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
#include <cstring>
#include <iostream>

#include "XML_Parse.h"
#include "arbiter.h"
#include "array.h"
#include "basic_circuit.h"
#include "const.h"
#include "io.h"
#include "logic.h"
#include "parameter.h"
#include "sharedcache.h"

SharedCache::SharedCache(ParseXML* XML_interface, int ithCache_, InputParameter* interface_ip_, enum cache_level cacheL_)
:XML(XML_interface),
 ithCache(ithCache_),
 interface_ip(*interface_ip_),
 cacheL(cacheL_),
 dir_overhead(0)
{
  int idx;
  int tag, data;
  bool is_default, debug;
  enum Device_ty device_t;
  enum Core_type  core_t;
  double size, line, assoc, banks;
  if (cacheL==L2 && XML->sys.Private_L2)
  {
          device_t=Core_device;
      core_t = (enum Core_type)XML->sys.core[ithCache].machine_type;
  }
  else
  {
          device_t=LLC_device;
          core_t = Inorder;
  }

  debug           = false;
  is_default=true;//indication for default setup
  if (XML->sys.Embedded)
                {
                interface_ip.wt                  =Global_30;
                interface_ip.wire_is_mat_type = 0;
                interface_ip.wire_os_mat_type = 1;
                }
        else
                {
                interface_ip.wt                  =Global;
                interface_ip.wire_is_mat_type = 2;
                interface_ip.wire_os_mat_type = 2;
                }
  set_cache_param();

  //All lower level cache are physically indexed and tagged.
  size                             = cachep.capacity;
  line                             = cachep.blockW;
  assoc                            = cachep.assoc;
  banks                            = cachep.nbanks;
  if ((cachep.dir_ty==ST&& cacheL==L1Directory)||(cachep.dir_ty==ST&& cacheL==L2Directory))
  {
          assoc = 0;
          tag   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          interface_ip.num_search_ports    = 1;
  }
  else
  {
          idx    					 	   = debug?9:int(ceil(log2(size/line/assoc)));
          tag							   = debug?51:XML->sys.physical_address_width-idx-int(ceil(log2(line))) + EXTRA_TAG_BITS;
          interface_ip.num_search_ports    = 0;
          if (cachep.dir_ty==SBT)
          {
                  dir_overhead = ceil(XML->sys.number_of_cores/8.0)*8/(cachep.blockW*8);
                  line = cachep.blockW*(1+ dir_overhead) ;
                  size = cachep.capacity*(1+ dir_overhead);

          }
  }
//  if (XML->sys.first_level_dir==2)
//	  tag += int(XML->sys.domain_size + 5);
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.cache_sz            = (int)size;
  interface_ip.line_sz             = (int)line;
  interface_ip.assoc               = (int)assoc;
  interface_ip.nbanks              = (int)banks;
  interface_ip.out_w               = interface_ip.line_sz*8/2;
  interface_ip.access_mode         = 1;
  interface_ip.throughput          = cachep.throughput;
  interface_ip.latency             = cachep.latency;
  interface_ip.is_cache			 = true;
  interface_ip.pure_ram			 = false;
  interface_ip.pure_cam          = false;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports        = 1;//lower level cache usually has one port.
  interface_ip.num_rd_ports        = 0;
  interface_ip.num_wr_ports        = 0;
  interface_ip.num_se_rd_ports     = 0;
//  interface_ip.force_cache_config  =true;
//  interface_ip.ndwl = 4;
//  interface_ip.ndbl = 8;
//  interface_ip.nspd = 1;
//  interface_ip.ndcm =1 ;
//  interface_ip.ndsam1 =1;
//  interface_ip.ndsam2 =1;
  unicache.caches = new ArrayST(&interface_ip, cachep.name + "cache", device_t, true, core_t);
  unicache.area.set_area(unicache.area.get_area()+ unicache.caches->local_result.area);
  area.set_area(area.get_area()+ unicache.caches->local_result.area);
  interface_ip.force_cache_config  =false;

  if (!((cachep.dir_ty==ST&& cacheL==L1Directory)||(cachep.dir_ty==ST&& cacheL==L2Directory)))
  {
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = (XML->sys.physical_address_width) + int(ceil(log2(size/line))) + unicache.caches->l_ip.line_sz;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
          interface_ip.cache_sz            = cachep.missb_size*interface_ip.line_sz;
          interface_ip.assoc               = 0;
          interface_ip.is_cache			   = true;
          interface_ip.pure_ram			   = false;
          interface_ip.pure_cam            = false;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8/2;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = cachep.throughput;//means cycle time
          interface_ip.latency             = cachep.latency;//means access time
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 1;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          interface_ip.num_search_ports    = 1;
          unicache.missb = new ArrayST(&interface_ip, cachep.name + "MissB", device_t, true, core_t);
          unicache.area.set_area(unicache.area.get_area()+ unicache.missb->local_result.area);
          area.set_area(area.get_area()+ unicache.missb->local_result.area);
          //fill buffer
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = unicache.caches->l_ip.line_sz;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
          interface_ip.cache_sz            = data*cachep.fu_size ;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8/2;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          =  cachep.throughput;
          interface_ip.latency             =  cachep.latency;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 1;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          unicache.ifb = new ArrayST(&interface_ip, cachep.name + "FillB", device_t, true, core_t);
          unicache.area.set_area(unicache.area.get_area()+ unicache.ifb->local_result.area);
          area.set_area(area.get_area()+ unicache.ifb->local_result.area);
          //prefetch buffer
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;//check with previous entries to decide wthether to merge.
          data							   = unicache.caches->l_ip.line_sz;//separate queue to prevent from cache polution.
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
          interface_ip.cache_sz            = cachep.prefetchb_size*interface_ip.line_sz;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8/2;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = cachep.throughput;
          interface_ip.latency             = cachep.latency;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 1;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          unicache.prefetchb = new ArrayST(&interface_ip, cachep.name + "PrefetchB", device_t, true, core_t);
          unicache.area.set_area(unicache.area.get_area()+ unicache.prefetchb->local_result.area);
          area.set_area(area.get_area()+ unicache.prefetchb->local_result.area);
          //WBB
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = unicache.caches->l_ip.line_sz;
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;
          interface_ip.cache_sz            = cachep.wbb_size*interface_ip.line_sz;
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1;
          interface_ip.out_w               = interface_ip.line_sz*8/2;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = cachep.throughput;
          interface_ip.latency             = cachep.latency;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 1;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          unicache.wbb = new ArrayST(&interface_ip, cachep.name + "WBB", device_t, true, core_t);
          unicache.area.set_area(unicache.area.get_area()+ unicache.wbb->local_result.area);
          area.set_area(area.get_area()+ unicache.wbb->local_result.area);
  }
  //  //pipeline
//  interface_ip.pipeline_stages = int(ceil(llCache.caches.local_result.access_time/llCache.caches.local_result.cycle_time));
//  interface_ip.per_stage_vector = llCache.caches.l_ip.out_w + llCache.caches.l_ip.tag_w ;
//  pipeLogicCache.init_pipeline(is_default, &interface_ip);
//  pipeLogicCache.compute_pipeline();

  /*
  if (!((XML->sys.number_of_dir_levels==1 && XML->sys.first_level_dir ==1)
                  ||(XML->sys.number_of_dir_levels==1 && XML->sys.first_level_dir ==2)))//not single level IC and DIC
  {
  //directory Now assuming one directory per bank, TODO:should change it later
  size                             = XML->sys.L2directory.L2Dir_config[0];
  line                             = XML->sys.L2directory.L2Dir_config[1];
  assoc                            = XML->sys.L2directory.L2Dir_config[2];
  banks                            = XML->sys.L2directory.L2Dir_config[3];
  tag							   = debug?51:XML->sys.physical_address_width + EXTRA_TAG_BITS;//TODO: a little bit over estimate
  interface_ip.specific_tag        = 0;
  interface_ip.tag_w               = tag;
  interface_ip.cache_sz            = XML->sys.L2directory.L2Dir_config[0];
  interface_ip.line_sz             = XML->sys.L2directory.L2Dir_config[1];
  interface_ip.assoc               = XML->sys.L2directory.L2Dir_config[2];
  interface_ip.nbanks              = XML->sys.L2directory.L2Dir_config[3];
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;//debug?0:XML->sys.core[ithCore].icache.icache_config[5];
  interface_ip.throughput          = XML->sys.L2directory.L2Dir_config[4]/clockRate;
  interface_ip.latency             = XML->sys.L2directory.L2Dir_config[5]/clockRate;
  interface_ip.is_cache			 = true;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;//lower level cache usually has one port.
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;

  strcpy(directory.caches.name,"L2 Directory");
  directory.caches.init_cache(&interface_ip);
  directory.caches.optimize_array();
  directory.area += directory.caches.local_result.area;
  //output_data_csv(directory.caches.local_result);
  ///cout<<"area="<<area<<endl;

  //miss buffer Each MSHR contains enough state to handle one or more accesses of any type to a single memory line.
  //Due to the generality of the MSHR mechanism, the amount of state involved is non-trivial,
  //including the address, pointers to the cache entry and destination register, written data, and various other pieces of state.
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data							   = (XML->sys.physical_address_width) + int(ceil(log2(size/line))) + directory.caches.l_ip.line_sz;
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
  interface_ip.cache_sz            = XML->sys.L2[ithCache].buffer_sizes[0]*interface_ip.line_sz;
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          = XML->sys.L2[ithCache].L2_config[4]/clockRate;//means cycle time
  interface_ip.latency             = XML->sys.L2[ithCache].L2_config[5]/clockRate;//means access time
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory.missb.name,"directoryMissB");
  directory.missb.init_cache(&interface_ip);
  directory.missb.optimize_array();
  directory.area += directory.missb.local_result.area;
  //output_data_csv(directory.missb.local_result);
  ///cout<<"area="<<area<<endl;

  //fill buffer
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data							   = directory.caches.l_ip.line_sz;
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz            = data*XML->sys.L2[ithCache].buffer_sizes[1];
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          =  XML->sys.L2[ithCache].L2_config[4]/clockRate;
  interface_ip.latency             =  XML->sys.L2[ithCache].L2_config[5]/clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory.ifb.name,"directoryFillB");
  directory.ifb.init_cache(&interface_ip);
  directory.ifb.optimize_array();
  directory.area += directory.ifb.local_result.area;
  //output_data_csv(directory.ifb.local_result);
  ///cout<<"area="<<area<<endl;

  //prefetch buffer
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;//check with previous entries to decide wthether to merge.
  data							   = directory.caches.l_ip.line_sz;//separate queue to prevent from cache polution.
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz            = XML->sys.L2[ithCache].buffer_sizes[2]*interface_ip.line_sz;
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          = XML->sys.L2[ithCache].L2_config[4]/clockRate;
  interface_ip.latency             = XML->sys.L2[ithCache].L2_config[5]/clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory.prefetchb.name,"directoryPrefetchB");
  directory.prefetchb.init_cache(&interface_ip);
  directory.prefetchb.optimize_array();
  directory.area += directory.prefetchb.local_result.area;
  //output_data_csv(directory.prefetchb.local_result);
  ///cout<<"area="<<area<<endl;

  //WBB
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data							   = directory.caches.l_ip.line_sz;
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = data;
  interface_ip.cache_sz            = XML->sys.L2[ithCache].buffer_sizes[3]*interface_ip.line_sz;
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          = XML->sys.L2[ithCache].L2_config[4]/clockRate;
  interface_ip.latency             = XML->sys.L2[ithCache].L2_config[4]/clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory.wbb.name,"directoryWBB");
  directory.wbb.init_cache(&interface_ip);
  directory.wbb.optimize_array();
  directory.area += directory.wbb.local_result.area;
  }

  if (XML->sys.number_of_dir_levels ==2 && XML->sys.first_level_dir==0)
  {
  //first level directory
  size                             = XML->sys.L2directory.L2Dir_config[0]*XML->sys.domain_size/128;
  line                             = int(ceil(XML->sys.domain_size/8.0));
  assoc                            = XML->sys.L2directory.L2Dir_config[2];
  banks                            = XML->sys.L2directory.L2Dir_config[3];
  tag							   = debug?51:XML->sys.physical_address_width + EXTRA_TAG_BITS;//TODO: a little bit over estimate
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.cache_sz            = XML->sys.L2directory.L2Dir_config[0];
  interface_ip.line_sz             = XML->sys.L2directory.L2Dir_config[1];
  interface_ip.assoc               = XML->sys.L2directory.L2Dir_config[2];
  interface_ip.nbanks              = XML->sys.L2directory.L2Dir_config[3];
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;//debug?0:XML->sys.core[ithCore].icache.icache_config[5];
  interface_ip.throughput          = XML->sys.L2directory.L2Dir_config[4]/clockRate;
  interface_ip.latency             = XML->sys.L2directory.L2Dir_config[5]/clockRate;
  interface_ip.is_cache			 = true;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;//lower level cache usually has one port.
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;

  strcpy(directory1.caches.name,"first level Directory");
  directory1.caches.init_cache(&interface_ip);
  directory1.caches.optimize_array();
  directory1.area += directory1.caches.local_result.area;
  //output_data_csv(directory.caches.local_result);
  ///cout<<"area="<<area<<endl;

  //miss buffer Each MSHR contains enough state to handle one or more accesses of any type to a single memory line.
  //Due to the generality of the MSHR mechanism, the amount of state involved is non-trivial,
  //including the address, pointers to the cache entry and destination register, written data, and various other pieces of state.
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data							   = (XML->sys.physical_address_width) + int(ceil(log2(size/line))) + directory1.caches.l_ip.line_sz;
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
  interface_ip.cache_sz            = XML->sys.L2[ithCache].buffer_sizes[0]*interface_ip.line_sz;
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          = XML->sys.L2[ithCache].L2_config[4]/clockRate;//means cycle time
  interface_ip.latency             = XML->sys.L2[ithCache].L2_config[5]/clockRate;//means access time
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory1.missb.name,"directory1MissB");
  directory1.missb.init_cache(&interface_ip);
  directory1.missb.optimize_array();
  directory1.area += directory1.missb.local_result.area;
  //output_data_csv(directory.missb.local_result);
  ///cout<<"area="<<area<<endl;

  //fill buffer
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data							   = directory1.caches.l_ip.line_sz;
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz            = data*XML->sys.L2[ithCache].buffer_sizes[1];
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          =  XML->sys.L2[ithCache].L2_config[4]/clockRate;
  interface_ip.latency             =  XML->sys.L2[ithCache].L2_config[5]/clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory1.ifb.name,"directory1FillB");
  directory1.ifb.init_cache(&interface_ip);
  directory1.ifb.optimize_array();
  directory1.area += directory1.ifb.local_result.area;
  //output_data_csv(directory.ifb.local_result);
  ///cout<<"area="<<area<<endl;

  //prefetch buffer
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;//check with previous entries to decide wthether to merge.
  data							   = directory1.caches.l_ip.line_sz;//separate queue to prevent from cache polution.
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz            = XML->sys.L2[ithCache].buffer_sizes[2]*interface_ip.line_sz;
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          = XML->sys.L2[ithCache].L2_config[4]/clockRate;
  interface_ip.latency             = XML->sys.L2[ithCache].L2_config[5]/clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory1.prefetchb.name,"directory1PrefetchB");
  directory1.prefetchb.init_cache(&interface_ip);
  directory1.prefetchb.optimize_array();
  directory1.area += directory1.prefetchb.local_result.area;
  //output_data_csv(directory.prefetchb.local_result);
  ///cout<<"area="<<area<<endl;

  //WBB
  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data							   = directory1.caches.l_ip.line_sz;
  interface_ip.specific_tag        = 1;
  interface_ip.tag_w               = tag;
  interface_ip.line_sz             = data;
  interface_ip.cache_sz            = XML->sys.L2[ithCache].buffer_sizes[3]*interface_ip.line_sz;
  interface_ip.assoc               = 0;
  interface_ip.nbanks              = 1;
  interface_ip.out_w               = interface_ip.line_sz*8;
  interface_ip.access_mode         = 0;
  interface_ip.throughput          = XML->sys.L2[ithCache].L2_config[4]/clockRate;
  interface_ip.latency             = XML->sys.L2[ithCache].L2_config[5]/clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power  = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports    = 1;
  interface_ip.num_rd_ports    = 0;
  interface_ip.num_wr_ports    = 0;
  interface_ip.num_se_rd_ports = 0;
  strcpy(directory1.wbb.name,"directoryWBB");
  directory1.wbb.init_cache(&interface_ip);
  directory1.wbb.optimize_array();
  directory1.area += directory1.wbb.local_result.area;
  }

  if (XML->sys.first_level_dir==1)//IC
  {
          tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
          data							   = int(ceil(XML->sys.domain_size/8.0));
          interface_ip.specific_tag        = 1;
          interface_ip.tag_w               = tag;
          interface_ip.line_sz             = data;
          interface_ip.cache_sz            = XML->sys.domain_size*data*XML->sys.L2[ithCache].L2_config[0]/XML->sys.L2[ithCache].L2_config[1];
          interface_ip.assoc               = 0;
          interface_ip.nbanks              = 1024;
          interface_ip.out_w               = interface_ip.line_sz*8;
          interface_ip.access_mode         = 0;
          interface_ip.throughput          = XML->sys.L2[ithCache].L2_config[4]/clockRate;
          interface_ip.latency             = XML->sys.L2[ithCache].L2_config[5]/clockRate;
          interface_ip.obj_func_dyn_energy = 0;
          interface_ip.obj_func_dyn_power  = 0;
          interface_ip.obj_func_leak_power = 0;
          interface_ip.obj_func_cycle_t    = 1;
          interface_ip.num_rw_ports    = 1;
          interface_ip.num_rd_ports    = 0;
          interface_ip.num_wr_ports    = 0;
          interface_ip.num_se_rd_ports = 0;
          strcpy(inv_dir.caches.name,"inv_dir");
          inv_dir.caches.init_cache(&interface_ip);
          inv_dir.caches.optimize_array();
          inv_dir.area = inv_dir.caches.local_result.area;

  }
*/
//  //pipeline
//  interface_ip.pipeline_stages = int(ceil(directory.caches.local_result.access_time/directory.caches.local_result.cycle_time));
//  interface_ip.per_stage_vector = directory.caches.l_ip.out_w + directory.caches.l_ip.tag_w ;
//  pipeLogicDirectory.init_pipeline(is_default, &interface_ip);
//  pipeLogicDirectory.compute_pipeline();
//
//  //clock power
//  clockNetwork.init_wire_external(is_default, &interface_ip);
//  clockNetwork.clk_area           =area*1.1;//10% of placement overhead. rule of thumb
//  clockNetwork.end_wiring_level   =5;//toplevel metal
//  clockNetwork.start_wiring_level =5;//toplevel metal
//  clockNetwork.num_regs           = pipeLogicCache.tot_stage_vector + pipeLogicDirectory.tot_stage_vector;
//  clockNetwork.optimize_wire();

}


void SharedCache::computeEnergy(bool is_tdp)
{
        double homenode_data_access = (cachep.dir_ty==SBT)? 0.9:1.0;
        if (is_tdp)
        {
                if (!((cachep.dir_ty==ST&& cacheL==L1Directory)||(cachep.dir_ty==ST&& cacheL==L2Directory)))
                {
                        //init stats for Peak
                        unicache.caches->stats_t.readAc.access  = .67*unicache.caches->l_ip.num_rw_ports*cachep.duty_cycle*homenode_data_access;
                        unicache.caches->stats_t.readAc.miss    = 0;
                        unicache.caches->stats_t.readAc.hit     = unicache.caches->stats_t.readAc.access - unicache.caches->stats_t.readAc.miss;
                        unicache.caches->stats_t.writeAc.access = .33*unicache.caches->l_ip.num_rw_ports*cachep.duty_cycle*homenode_data_access;
                        unicache.caches->stats_t.writeAc.miss   = 0;
                        unicache.caches->stats_t.writeAc.hit    = unicache.caches->stats_t.writeAc.access -	unicache.caches->stats_t.writeAc.miss;
                        unicache.caches->tdp_stats = unicache.caches->stats_t;

                        if (cachep.dir_ty==SBT)
                        {
                                homenode_stats_t.readAc.access  = .67*unicache.caches->l_ip.num_rw_ports*cachep.dir_duty_cycle*(1-homenode_data_access);
                                homenode_stats_t.readAc.miss    = 0;
                                homenode_stats_t.readAc.hit     = homenode_stats_t.readAc.access - homenode_stats_t.readAc.miss;
                                homenode_stats_t.writeAc.access  = .67*unicache.caches->l_ip.num_rw_ports*cachep.dir_duty_cycle*(1-homenode_data_access);
                                homenode_stats_t.writeAc.miss   = 0;
                                homenode_stats_t.writeAc.hit    = homenode_stats_t.writeAc.access -	homenode_stats_t.writeAc.miss;
                                homenode_tdp_stats = homenode_stats_t;
                        }

                        unicache.missb->stats_t.readAc.access  = unicache.missb->l_ip.num_search_ports;
                        unicache.missb->stats_t.writeAc.access = unicache.missb->l_ip.num_search_ports;
                        unicache.missb->tdp_stats = unicache.missb->stats_t;

                        unicache.ifb->stats_t.readAc.access  = unicache.ifb->l_ip.num_search_ports;
                        unicache.ifb->stats_t.writeAc.access = unicache.ifb->l_ip.num_search_ports;
                        unicache.ifb->tdp_stats = unicache.ifb->stats_t;

                        unicache.prefetchb->stats_t.readAc.access  = unicache.prefetchb->l_ip.num_search_ports;
                        unicache.prefetchb->stats_t.writeAc.access = unicache.ifb->l_ip.num_search_ports;
                        unicache.prefetchb->tdp_stats = unicache.prefetchb->stats_t;

                        unicache.wbb->stats_t.readAc.access  = unicache.wbb->l_ip.num_search_ports;
                        unicache.wbb->stats_t.writeAc.access = unicache.wbb->l_ip.num_search_ports;
                        unicache.wbb->tdp_stats = unicache.wbb->stats_t;
                }
                else
                {
                        unicache.caches->stats_t.readAc.access  = unicache.caches->l_ip.num_search_ports*cachep.duty_cycle;
                        unicache.caches->stats_t.readAc.miss    = 0;
                        unicache.caches->stats_t.readAc.hit     = unicache.caches->stats_t.readAc.access - unicache.caches->stats_t.readAc.miss;
                        unicache.caches->stats_t.writeAc.access = 0;
                        unicache.caches->stats_t.writeAc.miss   = 0;
                        unicache.caches->stats_t.writeAc.hit    = unicache.caches->stats_t.writeAc.access -	unicache.caches->stats_t.writeAc.miss;
                        unicache.caches->tdp_stats = unicache.caches->stats_t;

                }

        }
        else
        {
                //init stats for runtime power (RTP)
                if (cacheL==L2)
                {
                        unicache.caches->stats_t.readAc.access  = XML->sys.L2[ithCache].read_accesses;
                        unicache.caches->stats_t.readAc.miss    = XML->sys.L2[ithCache].read_misses;
                        unicache.caches->stats_t.readAc.hit     = unicache.caches->stats_t.readAc.access - unicache.caches->stats_t.readAc.miss;
                        unicache.caches->stats_t.writeAc.access = XML->sys.L2[ithCache].write_accesses;
                        unicache.caches->stats_t.writeAc.miss   = XML->sys.L2[ithCache].write_misses;
                        unicache.caches->stats_t.writeAc.hit    = unicache.caches->stats_t.writeAc.access -	unicache.caches->stats_t.writeAc.miss;
                        unicache.caches->rtp_stats = unicache.caches->stats_t;

                        if (cachep.dir_ty==SBT)
                        {
                                homenode_rtp_stats.readAc.access  = XML->sys.L2[ithCache].homenode_read_accesses;
                                homenode_rtp_stats.readAc.miss    = XML->sys.L2[ithCache].homenode_read_misses;
                                homenode_rtp_stats.readAc.hit     = homenode_rtp_stats.readAc.access - homenode_rtp_stats.readAc.miss;
                                homenode_rtp_stats.writeAc.access = XML->sys.L2[ithCache].homenode_write_accesses;
                                homenode_rtp_stats.writeAc.miss   = XML->sys.L2[ithCache].homenode_write_misses;
                                homenode_rtp_stats.writeAc.hit    = homenode_rtp_stats.writeAc.access -	homenode_rtp_stats.writeAc.miss;
                        }
                }
                else if (cacheL==L3)
                {
                        unicache.caches->stats_t.readAc.access  = XML->sys.L3[ithCache].read_accesses;
                        unicache.caches->stats_t.readAc.miss    = XML->sys.L3[ithCache].read_misses;
                        unicache.caches->stats_t.readAc.hit     = unicache.caches->stats_t.readAc.access - unicache.caches->stats_t.readAc.miss;
                        unicache.caches->stats_t.writeAc.access = XML->sys.L3[ithCache].write_accesses;
                        unicache.caches->stats_t.writeAc.miss   = XML->sys.L3[ithCache].write_misses;
                        unicache.caches->stats_t.writeAc.hit    = unicache.caches->stats_t.writeAc.access -	unicache.caches->stats_t.writeAc.miss;
                        unicache.caches->rtp_stats = unicache.caches->stats_t;

                        if (cachep.dir_ty==SBT)
                        {
                                homenode_rtp_stats.readAc.access  = XML->sys.L3[ithCache].homenode_read_accesses;
                                homenode_rtp_stats.readAc.miss    = XML->sys.L3[ithCache].homenode_read_misses;
                                homenode_rtp_stats.readAc.hit     = homenode_rtp_stats.readAc.access - homenode_rtp_stats.readAc.miss;
                                homenode_rtp_stats.writeAc.access = XML->sys.L3[ithCache].homenode_write_accesses;
                                homenode_rtp_stats.writeAc.miss   = XML->sys.L3[ithCache].homenode_write_misses;
                                homenode_rtp_stats.writeAc.hit    = homenode_rtp_stats.writeAc.access -	homenode_rtp_stats.writeAc.miss;
                        }
                }
                else if (cacheL==L1Directory)
                {
                        unicache.caches->stats_t.readAc.access  = XML->sys.L1Directory[ithCache].read_accesses;
                        unicache.caches->stats_t.readAc.miss    = XML->sys.L1Directory[ithCache].read_misses;
                        unicache.caches->stats_t.readAc.hit     = unicache.caches->stats_t.readAc.access - unicache.caches->stats_t.readAc.miss;
                        unicache.caches->stats_t.writeAc.access = XML->sys.L1Directory[ithCache].write_accesses;
                        unicache.caches->stats_t.writeAc.miss   = XML->sys.L1Directory[ithCache].write_misses;
                        unicache.caches->stats_t.writeAc.hit    = unicache.caches->stats_t.writeAc.access -	unicache.caches->stats_t.writeAc.miss;
                        unicache.caches->rtp_stats = unicache.caches->stats_t;
                }
                else if (cacheL==L2Directory)
                {
                        unicache.caches->stats_t.readAc.access  = XML->sys.L2Directory[ithCache].read_accesses;
                        unicache.caches->stats_t.readAc.miss    = XML->sys.L2Directory[ithCache].read_misses;
                        unicache.caches->stats_t.readAc.hit     = unicache.caches->stats_t.readAc.access - unicache.caches->stats_t.readAc.miss;
                        unicache.caches->stats_t.writeAc.access = XML->sys.L2Directory[ithCache].write_accesses;
                        unicache.caches->stats_t.writeAc.miss   = XML->sys.L2Directory[ithCache].write_misses;
                        unicache.caches->stats_t.writeAc.hit    = unicache.caches->stats_t.writeAc.access -	unicache.caches->stats_t.writeAc.miss;
                        unicache.caches->rtp_stats = unicache.caches->stats_t;
                }
                if (!((cachep.dir_ty==ST&& cacheL==L1Directory)||(cachep.dir_ty==ST&& cacheL==L2Directory)))
                {   //Assuming write back and write-allocate cache

                        unicache.missb->stats_t.readAc.access  = unicache.caches->stats_t.writeAc.miss ;
                        unicache.missb->stats_t.writeAc.access = unicache.caches->stats_t.writeAc.miss;
                        unicache.missb->rtp_stats = unicache.missb->stats_t;

                        unicache.ifb->stats_t.readAc.access  = unicache.caches->stats_t.writeAc.miss;
                        unicache.ifb->stats_t.writeAc.access = unicache.caches->stats_t.writeAc.miss;
                        unicache.ifb->rtp_stats = unicache.ifb->stats_t;

                        unicache.prefetchb->stats_t.readAc.access  = unicache.caches->stats_t.writeAc.miss;
                        unicache.prefetchb->stats_t.writeAc.access = unicache.caches->stats_t.writeAc.miss;
                        unicache.prefetchb->rtp_stats = unicache.prefetchb->stats_t;

                        unicache.wbb->stats_t.readAc.access  = unicache.caches->stats_t.writeAc.miss;
                        unicache.wbb->stats_t.writeAc.access = unicache.caches->stats_t.writeAc.miss;
                        if (cachep.dir_ty==SBT)
                        {
                                unicache.missb->stats_t.readAc.access  += homenode_rtp_stats.writeAc.miss;
                                unicache.missb->stats_t.writeAc.access += homenode_rtp_stats.writeAc.miss;
                                unicache.missb->rtp_stats = unicache.missb->stats_t;

                                unicache.missb->stats_t.readAc.access  += homenode_rtp_stats.writeAc.miss;
                                unicache.missb->stats_t.writeAc.access += homenode_rtp_stats.writeAc.miss;
                                unicache.missb->rtp_stats = unicache.missb->stats_t;

                                unicache.ifb->stats_t.readAc.access  += homenode_rtp_stats.writeAc.miss;
                                unicache.ifb->stats_t.writeAc.access += homenode_rtp_stats.writeAc.miss;
                                unicache.ifb->rtp_stats = unicache.ifb->stats_t;

                                unicache.prefetchb->stats_t.readAc.access  += homenode_rtp_stats.writeAc.miss;
                                unicache.prefetchb->stats_t.writeAc.access += homenode_rtp_stats.writeAc.miss;
                                unicache.prefetchb->rtp_stats = unicache.prefetchb->stats_t;

                                unicache.wbb->stats_t.readAc.access  += homenode_rtp_stats.writeAc.miss;
                                unicache.wbb->stats_t.writeAc.access += homenode_rtp_stats.writeAc.miss;
                        }
                        unicache.wbb->rtp_stats = unicache.wbb->stats_t;

                }

        }

        unicache.power_t.reset();
        if (!((cachep.dir_ty==ST&& cacheL==L1Directory)||(cachep.dir_ty==ST&& cacheL==L2Directory)))
        {
                unicache.power_t.readOp.dynamic	+= (unicache.caches->stats_t.readAc.hit*unicache.caches->local_result.power.readOp.dynamic+
                                unicache.caches->stats_t.readAc.miss*unicache.caches->local_result.tag_array2->power.readOp.dynamic+
                                unicache.caches->stats_t.writeAc.miss*unicache.caches->local_result.tag_array2->power.writeOp.dynamic+
                                unicache.caches->stats_t.writeAc.access*unicache.caches->local_result.power.writeOp.dynamic);//write miss will also generate a write later

                if (cachep.dir_ty==SBT)
                {
                        unicache.power_t.readOp.dynamic	+= homenode_stats_t.readAc.hit * (unicache.caches->local_result.data_array2->power.readOp.dynamic*dir_overhead +
                                                unicache.caches->local_result.tag_array2->power.readOp.dynamic) +
                                        homenode_stats_t.readAc.miss*unicache.caches->local_result.tag_array2->power.readOp.dynamic +
                                        homenode_stats_t.writeAc.miss*unicache.caches->local_result.tag_array2->power.readOp.dynamic +
                                homenode_stats_t.writeAc.hit*(unicache.caches->local_result.data_array2->power.writeOp.dynamic*dir_overhead +
                                                        unicache.caches->local_result.tag_array2->power.readOp.dynamic+
                                        homenode_stats_t.writeAc.miss*unicache.caches->local_result.power.writeOp.dynamic);//write miss on dynamic home node will generate a replacement write on whole cache block


                }

                unicache.power_t.readOp.dynamic	+=  unicache.missb->stats_t.readAc.access*unicache.missb->local_result.power.searchOp.dynamic +
                unicache.missb->stats_t.writeAc.access*unicache.missb->local_result.power.writeOp.dynamic;//each access to missb involves a CAM and a write
                unicache.power_t.readOp.dynamic	+=  unicache.ifb->stats_t.readAc.access*unicache.ifb->local_result.power.searchOp.dynamic +
                unicache.ifb->stats_t.writeAc.access*unicache.ifb->local_result.power.writeOp.dynamic;
                unicache.power_t.readOp.dynamic	+=  unicache.prefetchb->stats_t.readAc.access*unicache.prefetchb->local_result.power.searchOp.dynamic +
                unicache.prefetchb->stats_t.writeAc.access*unicache.prefetchb->local_result.power.writeOp.dynamic;
                unicache.power_t.readOp.dynamic	+=  unicache.wbb->stats_t.readAc.access*unicache.wbb->local_result.power.searchOp.dynamic +
                unicache.wbb->stats_t.writeAc.access*unicache.wbb->local_result.power.writeOp.dynamic;
        }
        else
        {
                unicache.power_t.readOp.dynamic	+= (unicache.caches->stats_t.readAc.access*unicache.caches->local_result.power.searchOp.dynamic+
                                unicache.caches->stats_t.writeAc.access*unicache.caches->local_result.power.writeOp.dynamic);
        }

        if (is_tdp)
        {
                unicache.power = unicache.power_t + (unicache.caches->local_result.power)*pppm_lkg;
                if (!((cachep.dir_ty==ST&& cacheL==L1Directory)||(cachep.dir_ty==ST&& cacheL==L2Directory)))
                {
                        unicache.power = unicache.power+
                        (unicache.missb->local_result.power +
                                        unicache.ifb->local_result.power +
                                        unicache.prefetchb->local_result.power +
                                        unicache.wbb->local_result.power)*pppm_lkg;
                }
                power     = power + unicache.power;
//		cout<<"unicache.caches->local_result.power.readOp.dynamic"<<unicache.caches->local_result.power.readOp.dynamic<<endl;
//		cout<<"unicache.caches->local_result.power.writeOp.dynamic"<<unicache.caches->local_result.power.writeOp.dynamic<<endl;
        }
        else
        {
                unicache.rt_power = unicache.power_t + (unicache.caches->local_result.power)*pppm_lkg;
                if (!((cachep.dir_ty==ST&& cacheL==L1Directory)||(cachep.dir_ty==ST&& cacheL==L2Directory)))
                {
                        (unicache.rt_power = unicache.rt_power +
                                        unicache.missb->local_result.power +
                                        unicache.ifb->local_result.power +
                                        unicache.prefetchb->local_result.power +
                                        unicache.wbb->local_result.power)*pppm_lkg;
                }
                rt_power     = rt_power + unicache.rt_power;
        }
}

void SharedCache::displayEnergy(uint32_t indent,bool is_tdp)
{
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;

        if (is_tdp)
        {
                cout << (XML->sys.Private_L2? indent_str:"")<< cachep.name << endl;
                cout << indent_str << "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic*cachep.clockRate << " W" << endl;
                cout << indent_str << "Subthreshold Leakage = "
                        << (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
                //cout << indent_str << "Subthreshold Leakage = " << power.readOp.longer_channel_leakage <<" W" << endl;
                cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
                cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic/cachep.executionTime << " W" << endl;
                cout <<endl;
        }
        else
        {
        }
}

//void SharedCache::computeMaxPower()
//{
//  //Compute maximum power and runtime power.
//  //When computing runtime power, McPAT gets or reasons out the statistics based on XML input.
//  maxPower		= 0.0;
//  //llCache,itlb
//  llCache.maxPower   = 0.0;
//  llCache.maxPower	+=  (llCache.caches.l_ip.num_rw_ports*(0.67*llCache.caches.local_result.power.readOp.dynamic+0.33*llCache.caches.local_result.power.writeOp.dynamic)
//                        +llCache.caches.l_ip.num_rd_ports*llCache.caches.local_result.power.readOp.dynamic+llCache.caches.l_ip.num_wr_ports*llCache.caches.local_result.power.writeOp.dynamic
//                        +llCache.caches.l_ip.num_se_rd_ports*llCache.caches.local_result.power.readOp.dynamic)*clockRate;
//  ///cout<<"llCache.maxPower=" <<llCache.maxPower<<endl;
//
//  llCache.maxPower	+=  llCache.missb.l_ip.num_search_ports*llCache.missb.local_result.power.searchOp.dynamic*clockRate;
//  ///cout<<"llCache.maxPower=" <<llCache.maxPower<<endl;
//
//  llCache.maxPower	+=  llCache.ifb.l_ip.num_search_ports*llCache.ifb.local_result.power.searchOp.dynamic*clockRate;
//  ///cout<<"llCache.maxPower=" <<llCache.maxPower<<endl;
//
//  llCache.maxPower	+=  llCache.prefetchb.l_ip.num_search_ports*llCache.prefetchb.local_result.power.searchOp.dynamic*clockRate;
//  ///cout<<"llCache.maxPower=" <<llCache.maxPower<<endl;
//
//  llCache.maxPower	+=  llCache.wbb.l_ip.num_search_ports*llCache.wbb.local_result.power.searchOp.dynamic*clockRate;
//  //llCache.maxPower *=  scktRatio; //TODO: this calculation should be self-contained
//  ///cout<<"llCache.maxPower=" <<llCache.maxPower<<endl;
//
////  directory_power =  (directory.caches.l_ip.num_rw_ports*(0.67*directory.caches.local_result.power.readOp.dynamic+0.33*directory.caches.local_result.power.writeOp.dynamic)
////                        +directory.caches.l_ip.num_rd_ports*directory.caches.local_result.power.readOp.dynamic+directory.caches.l_ip.num_wr_ports*directory.caches.local_result.power.writeOp.dynamic
////                        +directory.caches.l_ip.num_se_rd_ports*directory.caches.local_result.power.readOp.dynamic)*clockRate;
//
//  L2Tot.power.readOp.dynamic = llCache.maxPower;
//  L2Tot.power.readOp.leakage = llCache.caches.local_result.power.readOp.leakage +
//                               llCache.missb.local_result.power.readOp.leakage +
//                               llCache.ifb.local_result.power.readOp.leakage +
//                               llCache.prefetchb.local_result.power.readOp.leakage +
//                               llCache.wbb.local_result.power.readOp.leakage;
//
//  L2Tot.area.set_area(llCache.area*1.1*1e-6);//placement and routing overhead
//
//  if (XML->sys.number_of_dir_levels==1)
//  {
//	  if (XML->sys.first_level_dir==0)
//	  {
//		  directory.maxPower   = 0.0;
//		  directory.maxPower	+=  (directory.caches.l_ip.num_rw_ports*(0.67*directory.caches.local_result.power.readOp.dynamic+0.33*directory.caches.local_result.power.writeOp.dynamic)
//		                        +directory.caches.l_ip.num_rd_ports*directory.caches.local_result.power.readOp.dynamic+directory.caches.l_ip.num_wr_ports*directory.caches.local_result.power.writeOp.dynamic
//		                        +directory.caches.l_ip.num_se_rd_ports*directory.caches.local_result.power.readOp.dynamic)*clockRate;
//		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//		  directory.maxPower	+=  directory.missb.l_ip.num_search_ports*directory.missb.local_result.power.searchOp.dynamic*clockRate;
//		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//		  directory.maxPower	+=  directory.ifb.l_ip.num_search_ports*directory.ifb.local_result.power.searchOp.dynamic*clockRate;
//		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//		  directory.maxPower	+=  directory.prefetchb.l_ip.num_search_ports*directory.prefetchb.local_result.power.searchOp.dynamic*clockRate;
//		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//		  directory.maxPower	+=  directory.wbb.l_ip.num_search_ports*directory.wbb.local_result.power.searchOp.dynamic*clockRate;
//
//		  cc.power.readOp.dynamic = directory.maxPower*scktRatio*8;//8 is the memory controller counts
//		  cc.power.readOp.leakage = directory.caches.local_result.power.readOp.leakage +
//                                     directory.missb.local_result.power.readOp.leakage +
//                                     directory.ifb.local_result.power.readOp.leakage +
//                                     directory.prefetchb.local_result.power.readOp.leakage +
//                                     directory.wbb.local_result.power.readOp.leakage;
//
//		  cc.power.readOp.leakage *=8;
//
//		  cc.area.set_area(directory.area*8);
//		  cout<<"CC area="<<cc.area.get_area()*1e-6<<endl;
//		  cout<<"CC Power="<<cc.power.readOp.dynamic<<endl;
//		  ccTot.area.set_area(cc.area.get_area()*1e-6);
//		  ccTot.power = cc.power;
//		  cout<<"DC energy per access" << cc.power.readOp.dynamic/clockRate/8;
//	  }
//	  else if (XML->sys.first_level_dir==1)
//	  {
//		  inv_dir.maxPower = inv_dir.caches.local_result.power.searchOp.dynamic*clockRate*XML->sys.domain_size;
//		  cc.power.readOp.dynamic  = inv_dir.maxPower*scktRatio*64/XML->sys.domain_size;
//		  cc.power.readOp.leakage  = inv_dir.caches.local_result.power.readOp.leakage*inv_dir.caches.l_ip.nbanks*64/XML->sys.domain_size;
//
//		  cc.area.set_area(inv_dir.area*64/XML->sys.domain_size);
//		  cout<<"CC area="<<cc.area.get_area()*1e-6<<endl;
//		  cout<<"CC Power="<<cc.power.readOp.dynamic<<endl;
//		  ccTot.area.set_area(cc.area.get_area()*1e-6);
//		  cout<<"DC energy per access" << cc.power.readOp.dynamic/clockRate/8;
//		  ccTot.power = cc.power;
//	  }
//  }
//
//  else if (XML->sys.number_of_dir_levels==2)
//  {
//
//	  		  directory.maxPower   = 0.0;
//	  		  directory.maxPower	+=  (directory.caches.l_ip.num_rw_ports*(0.67*directory.caches.local_result.power.readOp.dynamic+0.33*directory.caches.local_result.power.writeOp.dynamic)
//	  		                        +directory.caches.l_ip.num_rd_ports*directory.caches.local_result.power.readOp.dynamic+directory.caches.l_ip.num_wr_ports*directory.caches.local_result.power.writeOp.dynamic
//	  		                        +directory.caches.l_ip.num_se_rd_ports*directory.caches.local_result.power.readOp.dynamic)*clockRate;
//	  		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//	  		  directory.maxPower	+=  directory.missb.l_ip.num_search_ports*directory.missb.local_result.power.searchOp.dynamic*clockRate;
//	  		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//	  		  directory.maxPower	+=  directory.ifb.l_ip.num_search_ports*directory.ifb.local_result.power.searchOp.dynamic*clockRate;
//	  		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//	  		  directory.maxPower	+=  directory.prefetchb.l_ip.num_search_ports*directory.prefetchb.local_result.power.searchOp.dynamic*clockRate;
//	  		  ///cout<<"directory.maxPower=" <<directory.maxPower<<endl;
//
//	  		  directory.maxPower	+=  directory.wbb.l_ip.num_search_ports*directory.wbb.local_result.power.searchOp.dynamic*clockRate;
//
//	  		  cc.power.readOp.dynamic = directory.maxPower*scktRatio*8;//8 is the memory controller counts
//			  cc.power.readOp.leakage = directory.caches.local_result.power.readOp.leakage +
//	                                     directory.missb.local_result.power.readOp.leakage +
//	                                     directory.ifb.local_result.power.readOp.leakage +
//	                                     directory.prefetchb.local_result.power.readOp.leakage +
//	                                     directory.wbb.local_result.power.readOp.leakage;
//			  cc.power.readOp.leakage *=8;
//	  		  cc.area.set_area(directory.area*8);
//
//	  		if (XML->sys.first_level_dir==0)
//	  		{
//	  		  directory1.maxPower   = 0.0;
//	  		  directory1.maxPower	+=  (directory1.caches.l_ip.num_rw_ports*(0.67*directory1.caches.local_result.power.readOp.dynamic+0.33*directory1.caches.local_result.power.writeOp.dynamic)
//	  				  +directory1.caches.l_ip.num_rd_ports*directory1.caches.local_result.power.readOp.dynamic+directory1.caches.l_ip.num_wr_ports*directory1.caches.local_result.power.writeOp.dynamic
//	  				  +directory1.caches.l_ip.num_se_rd_ports*directory1.caches.local_result.power.readOp.dynamic)*clockRate;
//	  		  ///cout<<"directory1.maxPower=" <<directory1.maxPower<<endl;
//
//	  		  directory1.maxPower	+=  directory1.missb.l_ip.num_search_ports*directory1.missb.local_result.power.searchOp.dynamic*clockRate;
//	  		  ///cout<<"directory1.maxPower=" <<directory1.maxPower<<endl;
//
//	  		  directory1.maxPower	+=  directory1.ifb.l_ip.num_search_ports*directory1.ifb.local_result.power.searchOp.dynamic*clockRate;
//	  		  ///cout<<"directory1.maxPower=" <<directory1.maxPower<<endl;
//
//	  		  directory1.maxPower	+=  directory1.prefetchb.l_ip.num_search_ports*directory1.prefetchb.local_result.power.searchOp.dynamic*clockRate;
//	  		  ///cout<<"directory1.maxPower=" <<directory1.maxPower<<endl;
//
//	  		  directory1.maxPower	+=  directory1.wbb.l_ip.num_search_ports*directory1.wbb.local_result.power.searchOp.dynamic*clockRate;
//
//	  		  cc1.power.readOp.dynamic = directory1.maxPower*scktRatio*64/XML->sys.domain_size;
//			  cc1.power.readOp.leakage = directory1.caches.local_result.power.readOp.leakage +
//	                                     directory1.missb.local_result.power.readOp.leakage +
//	                                     directory1.ifb.local_result.power.readOp.leakage +
//	                                     directory1.prefetchb.local_result.power.readOp.leakage +
//	                                     directory1.wbb.local_result.power.readOp.leakage;
//			  cc1.power.readOp.leakage *= 64/XML->sys.domain_size;
//	  		  cc1.area.set_area(directory1.area*64/XML->sys.domain_size);
//
//	  		  cout<<"CC area="<<(cc.area.get_area()+cc1.area.get_area())*1e-6<<endl;
//			  cout<<"CC Power="<<cc.power.readOp.dynamic + cc1.power.readOp.dynamic <<endl;
//			  ccTot.area.set_area((cc.area.get_area()+cc1.area.get_area())*1e-6);
//			  ccTot.power = cc.power + cc1.power;
//	  	  }
//	  	  else if (XML->sys.first_level_dir==1)
//	  	  {
//	  		  inv_dir.maxPower = inv_dir.caches.local_result.power.searchOp.dynamic*clockRate*XML->sys.domain_size;
//	  		  cc1.power.readOp.dynamic = inv_dir.maxPower*scktRatio*(64/XML->sys.domain_size);
//	  		  cc1.power.readOp.leakage  = inv_dir.caches.local_result.power.readOp.leakage*inv_dir.caches.l_ip.nbanks*XML->sys.domain_size;
//
//	  		  cc1.area.set_area(inv_dir.area*64/XML->sys.domain_size);
//			  cout<<"CC area="<<(cc.area.get_area()+cc1.area.get_area())*1e-6<<endl;
//			  cout<<"CC Power="<<cc.power.readOp.dynamic + cc1.power.readOp.dynamic <<endl;
//			  ccTot.area.set_area((cc.area.get_area()+cc1.area.get_area())*1e-6);
//			  ccTot.power = cc.power + cc1.power;
//
//	  	  }
//	  	  else if (XML->sys.first_level_dir==2)
//	  	  {
//			  cout<<"CC area="<<cc.area.get_area()*1e-6<<endl;
//			  cout<<"CC Power="<<cc.power.readOp.dynamic<<endl;
//			  ccTot.area.set_area(cc.area.get_area()*1e-6);
//			  ccTot.power = cc.power;
//	  	  }
//  }
//
//cout<<"L2cache size="<<L2Tot.area.get_area()*1e-6<<endl;
//cout<<"L2cache dynamic power="<<L2Tot.power.readOp.dynamic<<endl;
//cout<<"L2cache laeakge power="<<L2Tot.power.readOp.leakage<<endl;
//
//  ///cout<<"llCache.maxPower=" <<llCache.maxPower<<endl;
//
//
//  maxPower          +=  llCache.maxPower;
//  ///cout<<"maxpower=" <<maxPower<<endl;
//
////  maxPower	  +=  pipeLogicCache.power.readOp.dynamic*clockRate;
////  ///cout<<"pipeLogic.power="<<pipeLogicCache.power.readOp.dynamic*clockRate<<endl;
////  ///cout<<"maxpower=" <<maxPower<<endl;
////
////  maxPower	  +=  pipeLogicDirectory.power.readOp.dynamic*clockRate;
////  ///cout<<"pipeLogic.power="<<pipeLogicDirectory.power.readOp.dynamic*clockRate<<endl;
////  ///cout<<"maxpower=" <<maxPower<<endl;
////
////  //clock power
////  maxPower += clockNetwork.total_power.readOp.dynamic*clockRate;
////  ///cout<<"clockNetwork.total_power="<<clockNetwork.total_power.readOp.dynamic*clockRate<<endl;
////  ///cout<<"maxpower=" <<maxPower<<endl;
//
//}

void SharedCache::set_cache_param()
{
        if (cacheL==L2)
        {
                cachep.name = "L2";
                cachep.clockRate       = XML->sys.L2[ithCache].clockrate;
                cachep.clockRate       *= 1e6;
                cachep.executionTime = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);
                interface_ip.data_arr_ram_cell_tech_type    = XML->sys.L2[ithCache].device_type;//long channel device LSTP
                interface_ip.data_arr_peri_global_tech_type = XML->sys.L2[ithCache].device_type;
                interface_ip.tag_arr_ram_cell_tech_type     = XML->sys.L2[ithCache].device_type;
                interface_ip.tag_arr_peri_global_tech_type  = XML->sys.L2[ithCache].device_type;
                cachep.capacity      = XML->sys.L2[ithCache].L2_config[0];
                cachep.blockW        = XML->sys.L2[ithCache].L2_config[1];
                cachep.assoc         = XML->sys.L2[ithCache].L2_config[2];
                cachep.nbanks        = XML->sys.L2[ithCache].L2_config[3];
                cachep.throughput    = XML->sys.L2[ithCache].L2_config[4]/cachep.clockRate;
                cachep.latency       = XML->sys.L2[ithCache].L2_config[5]/cachep.clockRate;
                cachep.missb_size    = XML->sys.L2[ithCache].buffer_sizes[0];
                cachep.fu_size       = XML->sys.L2[ithCache].buffer_sizes[1];
                cachep.prefetchb_size= XML->sys.L2[ithCache].buffer_sizes[2];
                cachep.wbb_size      = XML->sys.L2[ithCache].buffer_sizes[3];
                cachep.duty_cycle    = XML->sys.L2[ithCache].duty_cycle;
                if (!XML->sys.L2[ithCache].merged_dir)
                {
                        cachep.dir_ty = NonDir;
                }
                else
                {
                        cachep.dir_ty = SBT;
                        cachep.dir_duty_cycle  = XML->sys.L2[ithCache].dir_duty_cycle;
                }
        }
        else if (cacheL==L3)
        {
                cachep.name = "L3";
                cachep.clockRate       = XML->sys.L3[ithCache].clockrate;
                cachep.clockRate       *= 1e6;
                cachep.executionTime   = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);
                interface_ip.data_arr_ram_cell_tech_type    = XML->sys.L3[ithCache].device_type;//long channel device LSTP
                interface_ip.data_arr_peri_global_tech_type = XML->sys.L3[ithCache].device_type;
                interface_ip.tag_arr_ram_cell_tech_type     = XML->sys.L3[ithCache].device_type;
                interface_ip.tag_arr_peri_global_tech_type  = XML->sys.L3[ithCache].device_type;
                cachep.capacity      = XML->sys.L3[ithCache].L3_config[0];
                cachep.blockW        = XML->sys.L3[ithCache].L3_config[1];
                cachep.assoc         = XML->sys.L3[ithCache].L3_config[2];
                cachep.nbanks        = XML->sys.L3[ithCache].L3_config[3];
                cachep.throughput    = XML->sys.L3[ithCache].L3_config[4]/cachep.clockRate;
                cachep.latency       = XML->sys.L3[ithCache].L3_config[5]/cachep.clockRate;
                cachep.missb_size    = XML->sys.L3[ithCache].buffer_sizes[0];
                cachep.fu_size       = XML->sys.L3[ithCache].buffer_sizes[1];
                cachep.prefetchb_size= XML->sys.L3[ithCache].buffer_sizes[2];
                cachep.wbb_size      = XML->sys.L3[ithCache].buffer_sizes[3];
                cachep.duty_cycle    = XML->sys.L3[ithCache].duty_cycle;
                if (!XML->sys.L2[ithCache].merged_dir)
                {
                        cachep.dir_ty = NonDir;
                }
                else
                {
                        cachep.dir_ty = SBT;
                        cachep.dir_duty_cycle  = XML->sys.L2[ithCache].dir_duty_cycle;
                }
        }
        else if (cacheL==L1Directory)
                {
                        cachep.name = "First Level Directory";
                        cachep.dir_ty = (enum Dir_type) XML->sys.L1Directory[ithCache].Directory_type;
                        cachep.clockRate       = XML->sys.L1Directory[ithCache].clockrate;
                        cachep.clockRate       *= 1e6;
                        cachep.executionTime   = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);
                        interface_ip.data_arr_ram_cell_tech_type    = XML->sys.L1Directory[ithCache].device_type;//long channel device LSTP
                        interface_ip.data_arr_peri_global_tech_type = XML->sys.L1Directory[ithCache].device_type;
                        interface_ip.tag_arr_ram_cell_tech_type     = XML->sys.L1Directory[ithCache].device_type;
                        interface_ip.tag_arr_peri_global_tech_type  = XML->sys.L1Directory[ithCache].device_type;
                        cachep.capacity      = XML->sys.L1Directory[ithCache].Dir_config[0];
                        cachep.blockW        = XML->sys.L1Directory[ithCache].Dir_config[1];
                        cachep.assoc         = XML->sys.L1Directory[ithCache].Dir_config[2];
                        cachep.nbanks        = XML->sys.L1Directory[ithCache].Dir_config[3];
                        cachep.throughput    = XML->sys.L1Directory[ithCache].Dir_config[4]/cachep.clockRate;
                        cachep.latency       = XML->sys.L1Directory[ithCache].Dir_config[5]/cachep.clockRate;
                        cachep.missb_size    = XML->sys.L1Directory[ithCache].buffer_sizes[0];
                        cachep.fu_size       = XML->sys.L1Directory[ithCache].buffer_sizes[1];
                        cachep.prefetchb_size= XML->sys.L1Directory[ithCache].buffer_sizes[2];
                        cachep.wbb_size      = XML->sys.L1Directory[ithCache].buffer_sizes[3];
                        cachep.duty_cycle    = XML->sys.L1Directory[ithCache].duty_cycle;
                }
        else if (cacheL==L2Directory)
                {
                        cachep.name = "Second Level Directory";
                        cachep.dir_ty = (enum Dir_type) XML->sys.L2Directory[ithCache].Directory_type;
                        cachep.clockRate       = XML->sys.L2Directory[ithCache].clockrate;
                        cachep.clockRate       *= 1e6;
                        cachep.executionTime   = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);
                        interface_ip.data_arr_ram_cell_tech_type    = XML->sys.L2Directory[ithCache].device_type;//long channel device LSTP
                        interface_ip.data_arr_peri_global_tech_type = XML->sys.L2Directory[ithCache].device_type;
                        interface_ip.tag_arr_ram_cell_tech_type     = XML->sys.L2Directory[ithCache].device_type;
                        interface_ip.tag_arr_peri_global_tech_type  = XML->sys.L2Directory[ithCache].device_type;
                        cachep.capacity      = XML->sys.L2Directory[ithCache].Dir_config[0];
                        cachep.blockW        = XML->sys.L2Directory[ithCache].Dir_config[1];
                        cachep.assoc         = XML->sys.L2Directory[ithCache].Dir_config[2];
                        cachep.nbanks        = XML->sys.L2Directory[ithCache].Dir_config[3];
                        cachep.throughput    = XML->sys.L2Directory[ithCache].Dir_config[4]/cachep.clockRate;
                        cachep.latency       = XML->sys.L2Directory[ithCache].Dir_config[5]/cachep.clockRate;
                        cachep.missb_size    = XML->sys.L2Directory[ithCache].buffer_sizes[0];
                        cachep.fu_size       = XML->sys.L2Directory[ithCache].buffer_sizes[1];
                        cachep.prefetchb_size= XML->sys.L2Directory[ithCache].buffer_sizes[2];
                        cachep.wbb_size      = XML->sys.L2Directory[ithCache].buffer_sizes[3];
                        cachep.duty_cycle    = XML->sys.L2Directory[ithCache].duty_cycle;
                }
        //cachep.cache_duty_cycle=cachep.dir_duty_cycle = 0.35;
}

