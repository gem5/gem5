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

#include <cassert>
#include <cmath>
#include <iostream>

#include "basic_components.h"

double longer_channel_device_reduction(
                enum Device_ty device_ty,
                enum Core_type core_ty)
{

        double longer_channel_device_percentage_core;
        double longer_channel_device_percentage_uncore;
        double longer_channel_device_percentage_llc;

        double long_channel_device_reduction;

        longer_channel_device_percentage_llc    = 1.0;
        longer_channel_device_percentage_uncore = 0.82;
        if (core_ty==OOO)
        {
                longer_channel_device_percentage_core   = 0.56;//0.54 Xeon Tulsa //0.58 Nehelam
                //longer_channel_device_percentage_uncore = 0.76;//0.85 Nehelam

        }
        else
        {
                longer_channel_device_percentage_core   = 0.8;//0.8;//Niagara
                //longer_channel_device_percentage_uncore = 0.9;//Niagara
        }

        if (device_ty==Core_device)
        {
                long_channel_device_reduction = (1- longer_channel_device_percentage_core)
                + longer_channel_device_percentage_core * g_tp.peri_global.long_channel_leakage_reduction;
        }
        else if (device_ty==Uncore_device)
        {
                long_channel_device_reduction = (1- longer_channel_device_percentage_uncore)
                + longer_channel_device_percentage_uncore * g_tp.peri_global.long_channel_leakage_reduction;
        }
        else if (device_ty==LLC_device)
        {
                long_channel_device_reduction = (1- longer_channel_device_percentage_llc)
                + longer_channel_device_percentage_llc * g_tp.peri_global.long_channel_leakage_reduction;
        }
        else
        {
                cout<<"unknown device category"<<endl;
                exit(0);
        }

        return long_channel_device_reduction;
}

statsComponents operator+(const statsComponents & x, const statsComponents & y)
{
        statsComponents z;

        z.access = x.access + y.access;
        z.hit    = x.hit + y.hit;
        z.miss   = x.miss  + y.miss;

        return z;
}

statsComponents operator*(const statsComponents & x, double const * const y)
{
        statsComponents z;

        z.access = x.access*y[0];
        z.hit    = x.hit*y[1];
        z.miss   = x.miss*y[2];

        return z;
}

statsDef operator+(const statsDef & x, const statsDef & y)
{
        statsDef z;

        z.readAc   = x.readAc  + y.readAc;
        z.writeAc  = x.writeAc + y.writeAc;
        z.searchAc  = x.searchAc + y.searchAc;
        return z;
}

statsDef operator*(const statsDef & x, double const * const y)
{
        statsDef z;

        z.readAc   = x.readAc*y;
        z.writeAc  = x.writeAc*y;
        z.searchAc  = x.searchAc*y;
        return z;
}
