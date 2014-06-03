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

#ifndef CACHEARRAY_H_
#define CACHEARRAY_H_

#include <iostream>
#include <string>

#include "basic_components.h"
#include "cacti_interface.h"
#include "component.h"
#include "const.h"
#include "parameter.h"

class CacheArray : public McPATComponent {
public:
    static double area_efficiency_threshold;

    // These are used for the CACTI interface.
    static int ed;
    static int delay_wt;
    static int cycle_time_wt;
    static int area_wt;
    static int dynamic_power_wt;
    static int leakage_power_wt;
    static int delay_dev;
    static int cycle_time_dev;
    static int area_dev;
    static int dynamic_power_dev;
    static int leakage_power_dev;
    static int cycle_time_dev_threshold;

    InputParameter l_ip;
    enum Device_ty device_ty;
    bool opt_local;
    enum Core_type core_ty;
    bool is_default;
    uca_org_t local_result;

    // These are only used for static bank tag (SBT) directory type.
    double sbt_dir_overhead;
    // Set this to contain SBT peak power stats
    statsDef sbt_tdp_stats;
    // Set this to contain SBT runtime power stats
    statsDef sbt_rtp_stats;

    CacheArray(XMLNode* _xml_data, const InputParameter *configure_interface,
            string _name, enum Device_ty device_ty_, double _clockRate = 0.0f,
            bool opt_local_ = true,
            enum Core_type core_ty_ = Inorder, bool _is_default = true);
    void computeArea();
    void computeEnergy();
    void compute_base_power();
    void setSBTDirOverhead(double overhead) { sbt_dir_overhead = overhead; }
    ~CacheArray();

  private:
    double computeSBTDynEnergy(statsDef *sbt_stats_ptr);
};

extern inline
double CacheArray::computeSBTDynEnergy(statsDef *sbt_stats_p) {
    if (sbt_dir_overhead == 0) {
        return 0;
    }

    // Write miss on dynamic home node will generate a replacement write on
    // whole cache block
    double dynamic =
        sbt_stats_p->readAc.hit *
        (local_result.data_array2->power.readOp.dynamic * sbt_dir_overhead +
         local_result.tag_array2->power.readOp.dynamic) +
        sbt_stats_p->readAc.miss *
        local_result.tag_array2->power.readOp.dynamic +
        sbt_stats_p->writeAc.miss *
        local_result.tag_array2->power.readOp.dynamic +
        sbt_stats_p->writeAc.hit *
        (local_result.data_array2->power.writeOp.dynamic * sbt_dir_overhead +
         local_result.tag_array2->power.readOp.dynamic+
         sbt_stats_p->writeAc.miss *
         local_result.power.writeOp.dynamic);
    return dynamic;
}

#endif /* CACHEARRAY_H_ */
