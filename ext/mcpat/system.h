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

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "arbiter.h"
#include "area.h"
#include "array.h"
#include "basic_components.h"
#include "bus_interconnect.h"
#include "cachecontroller.h"
#include "cacheunit.h"
#include "core.h"
#include "decoder.h"
#include "iocontrollers.h"
#include "memoryctrl.h"
#include "noc.h"
#include "parameter.h"
#include "router.h"

class System : public McPATComponent {
public:
    InputParameter interface_ip;

    int device_type;
    double core_tech_node;
    int interconnect_projection_type;
    int temperature;

    System(XMLNode* _xml_data);
    void set_proc_param();
    // TODO: make this recursively compute energy on subcomponents
    void displayData(uint32_t indent = 0, int plevel = 100);
    void displayDeviceType(int device_type_, uint32_t indent = 0);
    void displayInterconnectType(int interconnect_type_, uint32_t indent = 0);
    ~System();
};

#endif /* SYSTEM_H_ */
