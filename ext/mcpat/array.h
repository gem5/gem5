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

#ifndef ARRAY_H_
#define ARRAY_H_

#include <iostream>
#include <string>

#include "basic_components.h"
#include "cacti_interface.h"
#include "component.h"
#include "const.h"
#include "parameter.h"

using namespace std;

class ArrayST :public Component{
 public:
  ArrayST(){};
  ArrayST(const InputParameter *configure_interface, string _name, enum Device_ty device_ty_, bool opt_local_=true, enum Core_type core_ty_=Inorder,  bool _is_default=true);

  InputParameter l_ip;
  string         name;
  enum Device_ty device_ty;
  bool opt_local;
  enum Core_type core_ty;
  bool           is_default;
  uca_org_t      local_result;

  statsDef       tdp_stats;
  statsDef       rtp_stats;
  statsDef       stats_t;
  powerDef       power_t;

  virtual void optimize_array();
  virtual void compute_base_power();
  virtual ~ArrayST();

  void leakage_feedback(double temperature);
};

class InstCache :public Component{
public:
  ArrayST* caches;
  ArrayST* missb;
  ArrayST* ifb;
  ArrayST* prefetchb;
  powerDef power_t;//temp value holder for both (max) power and runtime power
  InstCache(){caches=0;missb=0;ifb=0;prefetchb=0;};
  ~InstCache(){
          if (caches)    {//caches->local_result.cleanup();
                                          delete caches; caches=0;}
          if (missb)     {//missb->local_result.cleanup();
                                          delete missb; missb=0;}
          if (ifb)       {//ifb->local_result.cleanup();
                                          delete ifb; ifb=0;}
          if (prefetchb) {//prefetchb->local_result.cleanup();
                                          delete prefetchb; prefetchb=0;}
   };
};

class DataCache :public InstCache{
public:
  ArrayST* wbb;
  DataCache(){wbb=0;};
  ~DataCache(){
          if (wbb) {//wbb->local_result.cleanup();
                                delete wbb; wbb=0;}
   };
};

#endif /* TLB_H_ */
