/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
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
 *
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
 */

#include "mem/ruby/network/orion/power_static.hh"

#if (PARM_TECH_POINT == 18)
double NMOS_TAB[1] = {20.5e-9};
double PMOS_TAB[1] = {9.2e-9};
double NAND2_TAB[4] = {6.4e-10, 20.4e-9, 12.6e-9, 18.4e-9};
double NOR2_TAB[4] ={40.9e-9, 8.32e-9, 9.2e-9, 2.3e-10};
#elif (PARM_TECH_POINT == 10)
double NMOS_TAB[1] = {22.7e-9};
double PMOS_TAB[1] = {18.0e-9};
double NAND2_TAB[4] = {1.2e-9, 22.6e-9, 11.4e-9, 35.9e-9};
double NOR2_TAB[4] ={45.1e-9, 11.5e-9, 17.9e-9, 1.8e-9};
#elif (PARM_TECH_POINT == 7)
double NMOS_TAB[1] = {118.1e-9};
double PMOS_TAB[1] = {135.2e-9};
double NAND2_TAB[4] = {19.7e-9, 115.3e-9, 83.0e-9, 267.6e-9};
double NOR2_TAB[4] ={232.4e-9, 79.6e-9, 127.9e-9, 12.3e-9};
#endif
