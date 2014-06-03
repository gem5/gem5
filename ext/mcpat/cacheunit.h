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

#ifndef CACHEUNIT_H_
#define CACHEUNIT_H_

#include "area.h"
#include "array.h"
#include "basic_components.h"
#include "logic.h"
#include "parameter.h"

class CacheParameters {
public:
    enum Dir_type dir_ty;
    double clockRate;
    double capacity;
    double blockW;
    double assoc;
    double nbanks;
    double throughput;
    double latency;
    int missb_size;
    int fu_size;
    int prefetchb_size;
    int wbb_size;
    int missb_assoc;
    int fu_assoc;
    int prefetchb_assoc;
    int wbb_assoc;
    int missb_banks;
    int fu_banks;
    int prefetchb_banks;
    int wbb_banks;
    enum Access_mode cache_access_mode;
    enum Access_mode miss_buff_access_mode;
    enum Access_mode fetch_buff_access_mode;
    enum Access_mode prefetch_buff_access_mode;
    enum Access_mode writeback_buff_access_mode;
    int cache_rw_ports;
    int cache_rd_ports;
    int cache_wr_ports;
    int cache_se_rd_ports;
    int cache_search_ports;
    int miss_buff_rw_ports;
    int miss_buff_rd_ports;
    int miss_buff_wr_ports;
    int miss_buff_se_rd_ports;
    int miss_buff_search_ports;
    int fetch_buff_rw_ports;
    int fetch_buff_rd_ports;
    int fetch_buff_wr_ports;
    int fetch_buff_se_rd_ports;
    int fetch_buff_search_ports;
    int pf_buff_rw_ports;
    int pf_buff_rd_ports;
    int pf_buff_wr_ports;
    int pf_buff_se_rd_ports;
    int pf_buff_search_ports;
    int wb_buff_rw_ports;
    int wb_buff_rd_ports;
    int wb_buff_wr_ports;
    int wb_buff_se_rd_ports;
    int wb_buff_search_ports;
    bool pure_ram;
    enum CacheLevel cache_level;
    enum Device_ty device_ty;
    enum Core_type core_ty;
    int num_cores;
};

class CacheStatistics {
public:
    // Duty cycle is used for estimating TDP. It should reflect the highest
    // sustainable rate of access to the cache unit in execution of a benchmark
    // Default should be 1.0: one access per cycle
    double duty_cycle;
    // This duty cycle is only used for SBT directory types
    double dir_duty_cycle;
    // The following two stats are also used for estimating TDP.
    double tdp_read_access_scalar;
    double tdp_write_access_scalar;
    // There are 2 ways to calculate dynamic power from activity statistics:
    // Default is false
    bool use_detailed_stats;
    // 1) Count the number and type of accesses to each cache array
    //    splitting data and tag arrays (use_detailed_stats = true).
    //    These are extremely detailed statistics.
    //    read_misses and write_misses are still required for this method for
    //    various buffers associated with this cache.
    double num_data_array_reads;
    double num_data_array_writes;
    double num_tag_array_reads;
    double num_tag_array_writes;
    // 2) Count the number and type of access to the cache unit and
    //    use them to extrapolate the number of accesses to the other
    //    subcomponents (cache arrays and buffers)
    double read_accesses;
    double write_accesses;
    double read_misses;
    double write_misses;
    double conflicts;
    // The following is only used for SBT directory types
    int homenode_read_accesses;
    int homenode_write_accesses;
    int homenode_read_misses;
    int homenode_write_misses;
    double homenode_access_scalar;
    double tdp_sbt_write_access_scalar;
};

class CacheUnit : public McPATComponent {
public:
    static bool is_cache;
    static bool pure_cam;
    // This is used for CacheArray objects
    static bool opt_local;
    static bool force_cache_config;

    int ithCache;
    CacheParameters cache_params;
    CacheStatistics cache_stats;
    Cache_type cacheType;
    bool calculate_runtime_data_and_tag;
    double dir_overhead;

    double scktRatio;

    // TODO: REMOVE _interface_ip... It promotes a mess. Find a better way...
    CacheUnit(XMLNode* _xml_data, InputParameter* _interface_ip);
    void set_cache_param_from_xml_data();
    void computeEnergy();
    ~CacheUnit() {};
};

#endif /* CACHEUNIT_H_ */
