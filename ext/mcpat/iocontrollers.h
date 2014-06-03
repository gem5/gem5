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
#ifndef IOCONTROLLERS_H_
#define IOCONTROLLERS_H_

#include <vector>

#include "array.h"
#include "basic_components.h"
#include "parameter.h"

class NIUController : public McPATComponent {
  public:
    NIUParameters niup;
    NIUStatistics nius;

    NIUController(XMLNode* _xml_data, InputParameter* interface_ip_);
    void set_niu_param();
    void computeArea();
    void computeEnergy();
    ~NIUController(){};
};

class PCIeController : public McPATComponent {
  public:
    PCIeParameters pciep;
    PCIeStatistics pcies;

    PCIeController(XMLNode* _xml_data, InputParameter* interface_ip_);
    void set_pcie_param();
    void computeArea();
    void computeEnergy();
    ~PCIeController(){};
};

class FlashController : public McPATComponent {
  public:
    MCParameters fcp;
    MCStatistics fcs;

    FlashController(XMLNode* _xml_data, InputParameter* interface_ip_);
    void set_fc_param();
    void computeArea();
    void computeEnergy();
    ~FlashController(){};
};

#endif /* IOCONTROLLERS_H_ */
