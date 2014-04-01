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

#ifndef SHAREDCACHE_H_
#define SHAREDCACHE_H_
#include <vector>

#include "XML_Parse.h"
#include "area.h"
#include "array.h"
#include "basic_components.h"
#include "logic.h"
#include "parameter.h"

class SharedCache :public Component{
  public:
    ParseXML * XML;
    int ithCache;
        InputParameter interface_ip;
        enum cache_level cacheL;
    DataCache unicache;//Shared cache
    CacheDynParam cachep;
    statsDef   homenode_tdp_stats;
    statsDef   homenode_rtp_stats;
    statsDef   homenode_stats_t;
    double	   dir_overhead;
    //	cache_processor llCache,directory, directory1, inv_dir;

    //pipeline pipeLogicCache, pipeLogicDirectory;
    //clock_network				clockNetwork;
    double scktRatio, executionTime;
    //   Component L2Tot, cc, cc1, ccTot;

    SharedCache(ParseXML *XML_interface, int ithCache_, InputParameter* interface_ip_,enum cache_level cacheL_ =L2);
    void set_cache_param();
        void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,bool is_tdp=true);
    ~SharedCache(){};
};

class CCdir :public Component{
  public:
    ParseXML * XML;
    int ithCache;
        InputParameter interface_ip;
    DataCache dc;//Shared cache
    ArrayST * shadow_dir;
//	cache_processor llCache,directory, directory1, inv_dir;

    //pipeline pipeLogicCache, pipeLogicDirectory;
    //clock_network				clockNetwork;
    double scktRatio, clockRate, executionTime;
    Component L2Tot, cc, cc1, ccTot;

    CCdir(ParseXML *XML_interface, int ithCache_, InputParameter* interface_ip_);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,bool is_tdp=true);
    ~CCdir();
};

#endif /* SHAREDCACHE_H_ */
