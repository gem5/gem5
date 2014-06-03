/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
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
 * Authors: Joel Hestness
 *          Yasuko Eckert
 *
 ***************************************************************************/

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

#include "arbiter.h"
#include "array.h"
#include "basic_circuit.h"
#include "cachearray.h"
#include "cacheunit.h"
#include "common.h"
#include "const.h"
#include "io.h"
#include "logic.h"
#include "parameter.h"

bool CacheUnit::is_cache = true;
bool CacheUnit::pure_cam = false;
bool CacheUnit::opt_local = true;
bool CacheUnit::force_cache_config = false;

CacheUnit::CacheUnit(XMLNode* _xml_data, InputParameter* _interface_ip)
        : dir_overhead(0), McPATComponent(_xml_data, _interface_ip) {

    int tag;
    int data;

    name = "Cache Unit";
    CacheArray* arrayPtr = NULL;

    set_cache_param_from_xml_data();

    //All lower level cache are physically indexed and tagged.
    double size;
    double line;
    double assoc;
    double banks;
    size                             = cache_params.capacity;
    line                             = cache_params.blockW;
    assoc                            = cache_params.assoc;
    banks                            = cache_params.nbanks;
    if ((cache_params.dir_ty == ST &&
         cache_params.cache_level == L1Directory) ||
        (cache_params.dir_ty == ST &&
         cache_params.cache_level == L2Directory)) {
        tag = physical_address_width + EXTRA_TAG_BITS;
    } else {
        tag = physical_address_width - int(ceil(log2(size / line / assoc))) -
            int(ceil(log2(line))) + EXTRA_TAG_BITS;

        if (cache_params.dir_ty == SBT) {
            dir_overhead = ceil(cache_params.num_cores / BITS_PER_BYTE) *
                BITS_PER_BYTE / (line * BITS_PER_BYTE);
            line *= (1 + dir_overhead);
            size *= (1 + dir_overhead);
        }
    }

    interface_ip.cache_sz = (int)size;
    interface_ip.line_sz = (int)line;
    interface_ip.assoc = (int)assoc;
    interface_ip.nbanks = (int)banks;
    interface_ip.specific_tag = tag > 0;
    interface_ip.tag_w = tag;

    if (cache_params.cache_level == L1) {
        interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    } else {
        interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE / 2;
    }

    interface_ip.access_mode = cache_params.cache_access_mode;
    interface_ip.throughput= cache_params.throughput;
    interface_ip.latency = cache_params.latency;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.is_cache = is_cache;
    interface_ip.pure_ram = cache_params.pure_ram;
    interface_ip.pure_cam = pure_cam;
    interface_ip.num_rw_ports = cache_params.cache_rw_ports;
    interface_ip.num_rd_ports = cache_params.cache_rd_ports;
    interface_ip.num_wr_ports = cache_params.cache_wr_ports;
    interface_ip.num_se_rd_ports = cache_params.cache_se_rd_ports;
    interface_ip.num_search_ports = cache_params.cache_search_ports;

    arrayPtr = new CacheArray(xml_data, &interface_ip, "Data and Tag Arrays",
                              cache_params.device_ty, clockRate, opt_local,
                              cache_params.core_ty);
    children.push_back(arrayPtr);

    // This is for calculating TDP, which depends on the number of
    // available ports
    int num_tdp_ports = arrayPtr->l_ip.num_rw_ports +
        arrayPtr->l_ip.num_rd_ports + arrayPtr->l_ip.num_wr_ports;

    // Set new array stats for calculating TDP and runtime power
    arrayPtr->tdp_stats.reset();
    arrayPtr->tdp_stats.readAc.access = cache_stats.tdp_read_access_scalar *
        num_tdp_ports * cache_stats.duty_cycle *
        cache_stats.homenode_access_scalar;
    arrayPtr->tdp_stats.readAc.miss = 0;
    arrayPtr->tdp_stats.readAc.hit = arrayPtr->tdp_stats.readAc.access -
        arrayPtr->tdp_stats.readAc.miss;
    arrayPtr->tdp_stats.writeAc.access = cache_stats.tdp_write_access_scalar *
        num_tdp_ports * cache_stats.duty_cycle *
        cache_stats.homenode_access_scalar;
    arrayPtr->tdp_stats.writeAc.miss = 0;
    arrayPtr->tdp_stats.writeAc.hit = arrayPtr->tdp_stats.writeAc.access -
        arrayPtr->tdp_stats.writeAc.miss;
    arrayPtr->tdp_stats.searchAc.access = 0;
    arrayPtr->tdp_stats.searchAc.miss = 0;
    arrayPtr->tdp_stats.searchAc.hit = 0;

    arrayPtr->rtp_stats.reset();
    if (cache_stats.use_detailed_stats) {
        arrayPtr->rtp_stats.dataReadAc.access =
            cache_stats.num_data_array_reads;
        arrayPtr->rtp_stats.dataWriteAc.access =
            cache_stats.num_data_array_writes;
        arrayPtr->rtp_stats.tagReadAc.access =
            cache_stats.num_tag_array_reads;
        arrayPtr->rtp_stats.tagWriteAc.access =
            cache_stats.num_tag_array_writes;
    } else {
        // This code makes assumptions. For instance, it assumes that
        // tag and data arrays are accessed in parallel on a read request and
        // this is a write-allocate cache. It also ignores any coherence
        // requests. Using detailed stats as above can avoid the ambiguity
        // that is introduced here
        arrayPtr->rtp_stats.dataReadAc.access =
            cache_stats.read_accesses + cache_stats.write_misses;
        arrayPtr->rtp_stats.dataWriteAc.access =
            cache_stats.write_accesses + cache_stats.read_misses;
        arrayPtr->rtp_stats.tagReadAc.access =
            cache_stats.read_accesses + cache_stats.write_accesses;
        arrayPtr->rtp_stats.tagWriteAc.access =
            cache_stats.read_misses + cache_stats.write_misses;
    }

    // Set SBT stats if this is an SBT directory type
    if (dir_overhead > 0) {
        arrayPtr->setSBTDirOverhead(dir_overhead);

        // TDP stats
        arrayPtr->sbt_tdp_stats.readAc.access =
            cache_stats.tdp_read_access_scalar *
            num_tdp_ports * cache_stats.dir_duty_cycle *
            (1 - cache_stats.homenode_access_scalar);
        arrayPtr->sbt_tdp_stats.readAc.miss = 0;
        arrayPtr->sbt_tdp_stats.readAc.hit =
            arrayPtr->sbt_tdp_stats.readAc.access -
            arrayPtr->sbt_tdp_stats.readAc.miss;
        arrayPtr->sbt_tdp_stats.writeAc.access =
            cache_stats.tdp_sbt_write_access_scalar *
            num_tdp_ports * cache_stats.dir_duty_cycle *
            (1 - cache_stats.homenode_access_scalar);
        arrayPtr->sbt_tdp_stats.writeAc.miss = 0;
        arrayPtr->sbt_tdp_stats.writeAc.hit =
            arrayPtr->sbt_tdp_stats.writeAc.access -
            arrayPtr->sbt_tdp_stats.writeAc.miss;

        // Runtime power stats
        arrayPtr->sbt_rtp_stats.readAc.access =
            cache_stats.homenode_read_accesses;
        arrayPtr->sbt_rtp_stats.readAc.miss =
            cache_stats.homenode_read_misses;
        arrayPtr->sbt_rtp_stats.readAc.access =
            cache_stats.homenode_read_accesses -
            cache_stats.homenode_read_misses;
        arrayPtr->sbt_rtp_stats.writeAc.access =
            cache_stats.homenode_write_accesses;
        arrayPtr->sbt_rtp_stats.writeAc.miss =
            cache_stats.homenode_write_misses;
        arrayPtr->sbt_rtp_stats.writeAc.hit =
            cache_stats.homenode_write_accesses -
            cache_stats.homenode_write_misses;
    }

    interface_ip.force_cache_config = force_cache_config;
    if (!((cache_params.dir_ty == ST &&
           cache_params.cache_level == L1Directory) ||
          (cache_params.dir_ty == ST &&
           cache_params.cache_level== L2Directory))) {
        // Miss Buffer
        tag = physical_address_width + EXTRA_TAG_BITS;
        data = (physical_address_width) +
            int(ceil(log2(size / cache_params.blockW))) +
            (cache_params.blockW * BITS_PER_BYTE);
        line = int(ceil(data / BITS_PER_BYTE));
        size = cache_params.missb_size * line;

        interface_ip.cache_sz = size;
        interface_ip.line_sz = line;
        interface_ip.assoc = cache_params.missb_assoc;
        interface_ip.nbanks = cache_params.missb_banks;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;

        if (cache_params.cache_level == L1) {
            interface_ip.out_w = line * BITS_PER_BYTE;
        } else {
            interface_ip.out_w = line * BITS_PER_BYTE / 2;
        }

        interface_ip.access_mode = cache_params.miss_buff_access_mode;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.is_cache = is_cache;
        interface_ip.pure_ram = cache_params.pure_ram;
        interface_ip.pure_cam = pure_cam;
        interface_ip.throughput = cache_params.throughput;
        interface_ip.latency = cache_params.latency;
        interface_ip.num_rw_ports = cache_params.miss_buff_rw_ports;
        interface_ip.num_rd_ports = cache_params.miss_buff_rd_ports;
        interface_ip.num_wr_ports = cache_params.miss_buff_wr_ports;
        interface_ip.num_se_rd_ports = cache_params.miss_buff_se_rd_ports;
        interface_ip.num_search_ports = cache_params.miss_buff_search_ports;

        arrayPtr = new CacheArray(xml_data, &interface_ip, "Miss Buffer",
                                  cache_params.device_ty, clockRate, opt_local,
                                  cache_params.core_ty);
        children.push_back(arrayPtr);

        arrayPtr->tdp_stats.reset();
        arrayPtr->tdp_stats.readAc.access = 0;
        arrayPtr->tdp_stats.writeAc.access = arrayPtr->l_ip.num_search_ports;
        arrayPtr->tdp_stats.searchAc.access = arrayPtr->l_ip.num_search_ports;

        arrayPtr->rtp_stats.reset();
        arrayPtr->rtp_stats.readAc.access =
            cache_stats.read_misses + cache_stats.write_misses;
        arrayPtr->rtp_stats.writeAc.access =
            cache_stats.read_misses + cache_stats.write_misses;
        arrayPtr->rtp_stats.searchAc.access = 0;

        if (cache_params.dir_ty == SBT) {
            arrayPtr->rtp_stats.readAc.access +=
                cache_stats.homenode_write_misses;
            arrayPtr->rtp_stats.writeAc.access +=
                cache_stats.homenode_write_misses;
        }

        // Fill Buffer
        tag = physical_address_width + EXTRA_TAG_BITS;
        data = cache_params.blockW;

        interface_ip.cache_sz = data * cache_params.fu_size;
        interface_ip.line_sz = data;
        interface_ip.assoc = cache_params.fu_assoc;
        interface_ip.nbanks = cache_params.fu_banks;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;

        if (cache_params.cache_level == L1) {
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
        } else {
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE / 2;
        }

        interface_ip.access_mode = cache_params.fetch_buff_access_mode;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.is_cache = is_cache;
        interface_ip.pure_cam = pure_cam;
        interface_ip.throughput = cache_params.throughput;
        interface_ip.latency = cache_params.latency;
        interface_ip.num_rw_ports = cache_params.fetch_buff_rw_ports;
        interface_ip.num_rd_ports = cache_params.fetch_buff_rd_ports;
        interface_ip.num_wr_ports = cache_params.fetch_buff_wr_ports;
        interface_ip.num_se_rd_ports = cache_params.fetch_buff_se_rd_ports;
        interface_ip.num_search_ports = cache_params.fetch_buff_search_ports;
        arrayPtr = new CacheArray(xml_data, &interface_ip, "Fill Buffer",
                                  cache_params.device_ty, clockRate, opt_local,
                                  cache_params.core_ty);
        children.push_back(arrayPtr);

        arrayPtr->tdp_stats.reset();
        arrayPtr->tdp_stats.readAc.access = 0;
        arrayPtr->tdp_stats.writeAc.access = arrayPtr->l_ip.num_search_ports;
        arrayPtr->tdp_stats.searchAc.access = arrayPtr->l_ip.num_search_ports;

        arrayPtr->rtp_stats.reset();
        arrayPtr->rtp_stats.readAc.access =
            cache_stats.read_misses + cache_stats.write_misses;
        arrayPtr->rtp_stats.writeAc.access =
            cache_stats.read_misses + cache_stats.write_misses;
        arrayPtr->rtp_stats.searchAc.access = 0;

        if (cache_params.dir_ty == SBT) {
            arrayPtr->rtp_stats.readAc.access +=
                cache_stats.homenode_write_misses;
            arrayPtr->rtp_stats.writeAc.access +=
                cache_stats.homenode_write_misses;
        }

        // Prefetch Buffer
        tag = physical_address_width + EXTRA_TAG_BITS;
        line = cache_params.blockW;

        interface_ip.cache_sz = cache_params.prefetchb_size * line;
        interface_ip.line_sz = line;
        interface_ip.assoc = cache_params.prefetchb_assoc;
        interface_ip.nbanks = cache_params.prefetchb_banks;
        interface_ip.specific_tag = tag > 0;
        interface_ip.tag_w = tag;

        if (cache_params.cache_level == L1) {
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
        } else {
            interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE / 2;
        }

        interface_ip.access_mode = cache_params.prefetch_buff_access_mode;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.is_cache = is_cache;
        interface_ip.pure_ram = cache_params.pure_ram;
        interface_ip.pure_cam = pure_cam;
        interface_ip.throughput = cache_params.throughput;
        interface_ip.latency = cache_params.latency;
        interface_ip.num_rw_ports = cache_params.pf_buff_rw_ports;
        interface_ip.num_rd_ports = cache_params.pf_buff_rd_ports;
        interface_ip.num_wr_ports = cache_params.pf_buff_wr_ports;
        interface_ip.num_se_rd_ports = cache_params.pf_buff_se_rd_ports;
        interface_ip.num_search_ports = cache_params.pf_buff_search_ports;
        arrayPtr = new CacheArray(xml_data, &interface_ip, "Prefetch Buffer",
                                  cache_params.device_ty, clockRate, opt_local,
                                  cache_params.core_ty);
        children.push_back(arrayPtr);

        arrayPtr->tdp_stats.reset();
        arrayPtr->tdp_stats.readAc.access = 0;
        arrayPtr->tdp_stats.writeAc.access = arrayPtr->l_ip.num_search_ports;
        arrayPtr->tdp_stats.searchAc.access = arrayPtr->l_ip.num_search_ports;

        arrayPtr->rtp_stats.reset();
        arrayPtr->rtp_stats.readAc.access = cache_stats.read_misses;
        arrayPtr->rtp_stats.writeAc.access = cache_stats.read_misses;
        arrayPtr->rtp_stats.searchAc.access = 0;

        if (cache_params.dir_ty == SBT) {
            arrayPtr->rtp_stats.readAc.access +=
                cache_stats.homenode_write_misses;
            arrayPtr->rtp_stats.writeAc.access +=
                cache_stats.homenode_write_misses;
        }

        // Writeback Buffer
        if (cache_params.wbb_size > 0) {
            tag = physical_address_width + EXTRA_TAG_BITS;
            line = cache_params.blockW;

            interface_ip.cache_sz = cache_params.wbb_size * line;
            interface_ip.line_sz = line;
            interface_ip.assoc = cache_params.wbb_assoc;
            interface_ip.nbanks = cache_params.wbb_banks;
            interface_ip.specific_tag = tag > 0;
            interface_ip.tag_w = tag;

            if (cache_params.cache_level == L1) {
                interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
            } else {
                interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE / 2;
            }

            interface_ip.access_mode = cache_params.writeback_buff_access_mode;
            interface_ip.obj_func_dyn_energy = 0;
            interface_ip.obj_func_dyn_power = 0;
            interface_ip.obj_func_leak_power = 0;
            interface_ip.obj_func_cycle_t = 1;
            interface_ip.is_cache = is_cache;
            interface_ip.pure_ram = cache_params.pure_ram;
            interface_ip.pure_cam = pure_cam;
            interface_ip.throughput = cache_params.throughput;
            interface_ip.latency = cache_params.latency;
            interface_ip.num_rw_ports = cache_params.wb_buff_rw_ports;
            interface_ip.num_rd_ports = cache_params.wb_buff_rd_ports;
            interface_ip.num_wr_ports = cache_params.wb_buff_wr_ports;
            interface_ip.num_se_rd_ports = cache_params.wb_buff_se_rd_ports;
            interface_ip.num_search_ports = cache_params.wb_buff_search_ports;
            arrayPtr = new CacheArray(xml_data, &interface_ip,
                                      "Writeback Buffer",
                                      cache_params.device_ty, clockRate,
                                      opt_local, cache_params.core_ty);
            children.push_back(arrayPtr);

            arrayPtr->tdp_stats.reset();
            arrayPtr->tdp_stats.readAc.access = 0;
            arrayPtr->tdp_stats.writeAc.access =
                arrayPtr->l_ip.num_search_ports;
            arrayPtr->tdp_stats.searchAc.access =
                arrayPtr->l_ip.num_search_ports;

            arrayPtr->rtp_stats.reset();
            arrayPtr->rtp_stats.readAc.access = cache_stats.write_misses;
            arrayPtr->rtp_stats.writeAc.access = cache_stats.write_misses;
            arrayPtr->rtp_stats.searchAc.access = 0;

            if (cache_params.dir_ty == SBT) {
                arrayPtr->rtp_stats.readAc.access +=
                    cache_stats.homenode_write_misses;
                arrayPtr->rtp_stats.writeAc.access +=
                    cache_stats.homenode_write_misses;
            }
        }
    }
}

void CacheUnit::computeEnergy() {
    McPATComponent::computeEnergy();
}

void CacheUnit::set_cache_param_from_xml_data() {
    int level, type;

    // Initialization... move this?
    memset(&cache_params, 0, sizeof(CacheParameters));
    memset(&cache_stats, 0, sizeof(CacheStatistics));

    // By default, use the core clock frequency. This can be changed by
    // setting the clockrate param in the XML definition of the CacheUnit
    clockRate = target_core_clockrate;
    XMLCSTR comp_name = xml_data->getAttribute("name");
    if (comp_name) {
        name = comp_name;
    }

    int num_children = xml_data->nChildNode("param");
    int i;
    int tech_type;
    int mat_type;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_INT_IF("level", level);
        ASSIGN_FP_IF("size", cache_params.capacity);
        ASSIGN_FP_IF("block_size", cache_params.blockW);
        ASSIGN_FP_IF("assoc", cache_params.assoc);
        ASSIGN_FP_IF("num_banks", cache_params.nbanks);
        ASSIGN_FP_IF("latency", cache_params.latency);
        ASSIGN_FP_IF("throughput", cache_params.throughput);
        ASSIGN_INT_IF("miss_buffer_size", cache_params.missb_size);
        ASSIGN_INT_IF("fetch_buffer_size", cache_params.fu_size);
        ASSIGN_INT_IF("prefetch_buffer_size", cache_params.prefetchb_size);
        ASSIGN_INT_IF("writeback_buffer_size", cache_params.wbb_size);
        ASSIGN_INT_IF("miss_buffer_assoc", cache_params.missb_assoc);
        ASSIGN_INT_IF("fetch_buffer_assoc", cache_params.fu_assoc);
        ASSIGN_INT_IF("prefetch_buffer_assoc", cache_params.prefetchb_assoc);
        ASSIGN_INT_IF("writeback_buffer_assoc", cache_params.wbb_assoc);
        ASSIGN_INT_IF("miss_buffer_banks", cache_params.missb_banks);
        ASSIGN_INT_IF("fetch_buffer_banks", cache_params.fu_banks);
        ASSIGN_INT_IF("prefetch_buffer_banks", cache_params.prefetchb_banks);
        ASSIGN_INT_IF("writeback_buffer_banks", cache_params.wbb_banks);
        ASSIGN_ENUM_IF("cache_access_mode",
                       cache_params.cache_access_mode, Access_mode);
        ASSIGN_ENUM_IF("miss_buff_access_mode",
                       cache_params.miss_buff_access_mode, Access_mode);
        ASSIGN_ENUM_IF("fetch_buff_access_mode",
                       cache_params.fetch_buff_access_mode, Access_mode);
        ASSIGN_ENUM_IF("prefetch_buff_access_mode",
                       cache_params.prefetch_buff_access_mode, Access_mode);
        ASSIGN_ENUM_IF("writeback_buff_access_mode",
                       cache_params.writeback_buff_access_mode, Access_mode);
        ASSIGN_INT_IF("cache_rw_ports", cache_params.cache_rw_ports);
        ASSIGN_INT_IF("cache_rd_ports", cache_params.cache_rd_ports);
        ASSIGN_INT_IF("cache_wr_ports", cache_params.cache_wr_ports);
        ASSIGN_INT_IF("cache_se_rd_ports", cache_params.cache_se_rd_ports);
        ASSIGN_INT_IF("cache_search_ports", cache_params.cache_search_ports);
        ASSIGN_INT_IF("miss_buff_rw_ports", cache_params.miss_buff_rw_ports);
        ASSIGN_INT_IF("miss_buff_rd_ports", cache_params.miss_buff_rd_ports);
        ASSIGN_INT_IF("miss_buff_wr_ports", cache_params.miss_buff_wr_ports);
        ASSIGN_INT_IF("miss_buff_se_rd_ports" ,
                      cache_params.miss_buff_se_rd_ports);
        ASSIGN_INT_IF("miss_buff_search_ports",
                      cache_params.miss_buff_search_ports);
        ASSIGN_INT_IF("fetch_buff_rw_ports", cache_params.fetch_buff_rw_ports);
        ASSIGN_INT_IF("fetch_buff_rd_ports", cache_params.fetch_buff_rd_ports);
        ASSIGN_INT_IF("fetch_buff_wr_ports", cache_params.fetch_buff_wr_ports);
        ASSIGN_INT_IF("fetch_buff_se_rd_ports",
                      cache_params.fetch_buff_se_rd_ports);
        ASSIGN_INT_IF("fetch_buff_search_ports",
                      cache_params.fetch_buff_search_ports);
        ASSIGN_INT_IF("pf_buff_rw_ports", cache_params.pf_buff_rw_ports);
        ASSIGN_INT_IF("pf_buff_rd_ports", cache_params.pf_buff_rd_ports);
        ASSIGN_INT_IF("pf_buff_wr_ports", cache_params.pf_buff_wr_ports);
        ASSIGN_INT_IF("pf_buff_se_rd_ports", cache_params.pf_buff_se_rd_ports);
        ASSIGN_INT_IF("pf_buff_search_ports",
                      cache_params.pf_buff_search_ports);
        ASSIGN_INT_IF("wb_buff_rw_ports", cache_params.wb_buff_rw_ports);
        ASSIGN_INT_IF("wb_buff_rd_ports", cache_params.wb_buff_rd_ports);
        ASSIGN_INT_IF("wb_buff_wr_ports", cache_params.wb_buff_wr_ports);
        ASSIGN_INT_IF("wb_buff_se_rd_ports", cache_params.wb_buff_se_rd_ports);
        ASSIGN_INT_IF("wb_buff_search_ports",
                      cache_params.wb_buff_search_ports);
        ASSIGN_FP_IF("clockrate", cache_params.clockRate);
        ASSIGN_INT_IF("pure_ram", cache_params.pure_ram);
        ASSIGN_INT_IF("tech_type", tech_type);
        ASSIGN_ENUM_IF("Directory_type", cache_params.dir_ty, Dir_type);
        ASSIGN_ENUM_IF("device_type", cache_params.device_ty, Device_ty);
        ASSIGN_ENUM_IF("core_type", cache_params.core_ty, Core_type);
        ASSIGN_INT_IF("num_cores", cache_params.num_cores);
        ASSIGN_INT_IF("wire_mat_type", mat_type);
        ASSIGN_ENUM_IF("wire_type", interface_ip.wt, Wire_type);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    // Change from MHz to Hz
    cache_params.clockRate *= 1e6;
    if (cache_params.clockRate > 0) {
        clockRate = cache_params.clockRate;
    }

    interface_ip.data_arr_ram_cell_tech_type    = tech_type;
    interface_ip.data_arr_peri_global_tech_type = tech_type;
    interface_ip.tag_arr_ram_cell_tech_type     = tech_type;
    interface_ip.tag_arr_peri_global_tech_type  = tech_type;

    interface_ip.wire_is_mat_type = mat_type;
    interface_ip.wire_os_mat_type = mat_type;

    switch(level) {
      case 1:
        cache_params.cache_level = L1;
        break;
      case 2:
        cache_params.cache_level = L2;
        break;
      case 3:
        cache_params.cache_level = L3;
        break;
      case 4:
        cache_params.cache_level = L1Directory;
        break;
      case 5:
        cache_params.cache_level = L2Directory;
        break;

      default:
        fprintf(stderr, "ERROR: Unrecognized cache level in %s: %d\n",
                name.c_str(), level);
        exit(1);
    }

    cache_stats.use_detailed_stats = false;

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("num_data_array_reads", cache_stats.num_data_array_reads);
        ASSIGN_FP_IF("num_data_array_writes",
                     cache_stats.num_data_array_writes);
        ASSIGN_FP_IF("num_tag_array_reads", cache_stats.num_tag_array_reads);
        ASSIGN_FP_IF("num_tag_array_writes", cache_stats.num_tag_array_writes);
        ASSIGN_FP_IF("duty_cycle", cache_stats.duty_cycle);
        ASSIGN_FP_IF("read_accesses", cache_stats.read_accesses);
        ASSIGN_FP_IF("write_accesses", cache_stats.write_accesses);
        ASSIGN_FP_IF("read_misses", cache_stats.read_misses);
        ASSIGN_FP_IF("write_misses", cache_stats.write_misses);
        ASSIGN_FP_IF("conflicts", cache_stats.conflicts);
        ASSIGN_INT_IF("homenode_read_accesses",
                      cache_stats.homenode_read_accesses);
        ASSIGN_INT_IF("homenode_write_accesses",
                      cache_stats.homenode_write_accesses);
        ASSIGN_INT_IF("homenode_read_misses",
                      cache_stats.homenode_read_misses);
        ASSIGN_INT_IF("homenode_write_misses",
                      cache_stats.homenode_write_misses);
        ASSIGN_FP_IF("homenode_access_scalar",
                     cache_stats.homenode_access_scalar);
        ASSIGN_FP_IF("tdp_read_access_scalar",
                     cache_stats.tdp_read_access_scalar);
        ASSIGN_FP_IF("tdp_write_access_scalar",
                     cache_stats.tdp_write_access_scalar);
        ASSIGN_FP_IF("tdp_sbt_write_access_scalar",
                     cache_stats.tdp_sbt_write_access_scalar);
        ASSIGN_FP_IF("dir_duty_cycle",
                     cache_stats.dir_duty_cycle);

        else {
            warnUnrecognizedStat(node_name);
        }
    }

    if (cache_stats.num_data_array_reads > 0 ||
        cache_stats.num_data_array_writes > 0 ||
        cache_stats.num_tag_array_reads > 0 ||
        cache_stats.num_tag_array_writes > 0) {
        cache_stats.use_detailed_stats = true;
        calculate_runtime_data_and_tag = true;
    }
}
