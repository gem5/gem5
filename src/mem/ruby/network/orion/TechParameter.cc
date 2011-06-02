/*
 * Copyright (c) 2009 Princeton University
 * Copyright (c) 2009 The Regents of the University of California
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
 *
 * Authors:  Hangsheng Wang (Orion 1.0, Princeton)
 *           Xinping Zhu (Orion 1.0, Princeton)
 *           Xuning Chen (Orion 1.0, Princeton)
 *           Bin Li (Orion 2.0, Princeton)
 *           Kambiz Samadi (Orion 2.0, UC San Diego)
 */

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "mem/ruby/network/orion/OrionConfig.hh"
#include "mem/ruby/network/orion/TechParameter.hh"

using namespace std;

TechParameter::TechParameter(
  const OrionConfig* orion_cfg_ptr_
)
{
    m_orion_cfg_ptr = orion_cfg_ptr_;
    uint32_t tech_node = m_orion_cfg_ptr->get<uint32_t>("TECH_NODE");
    switch(tech_node)
    {
        case 800:
        case 400:
        case 350:
        case 250:
        case 180:
        case 110:
        case 90:
        case 65:
        case 45:
        case 32:
            m_tech_node = tech_node;
            break;
        default:
            cerr << "Invalid technology node (" << tech_node << ")" << endl;
            exit(1);
    }

    const string& transistor_type_str = m_orion_cfg_ptr->get<string>("TRANSISTOR_TYPE");
    if(transistor_type_str == "LVT")
    {
        m_transistor_type = LVT;
    }
    else if(transistor_type_str == "NVT")
    {
        m_transistor_type = NVT;
    }
    else if(transistor_type_str == "HVT")
    {
        m_transistor_type = HVT;
    }
    else
    {
        cerr << "Invalid transistor type (" << transistor_type_str << ")" << endl;
        exit(1);
    }

    m_vdd = m_orion_cfg_ptr->get<double>("VDD");
    m_freq = m_orion_cfg_ptr->get<double>("FREQUENCY");
    m_period = 1 / m_freq;

    const string& wire_layer_type_str = m_orion_cfg_ptr->get<string>("WIRE_LAYER_TYPE");
    if (wire_layer_type_str == "LOCAL")
    {
        m_wire_layer_type = LOCAL;
    }
    else if (wire_layer_type_str == "INTERMEDIATE")
    {
        m_wire_layer_type = INTERMEDIATE;
    }
    else if (wire_layer_type_str == "GLOBAL")
    {
        m_wire_layer_type = GLOBAL;
    }
    else
    {
        cerr << "Invalid wire layer type (" << wire_layer_type_str << ")" << endl;
        exit(1);
    }

    init();
}

TechParameter::~TechParameter()
{}

void TechParameter::init()
{
    m_af = 5.000000e-01;
    m_max_n = 8;
    m_max_subarrays = 8;
    m_max_spd = 8;
    m_vth_outdr_nor = 4.310000e-01;
    m_vth_comp_inv = 4.370000e-01;
    m_bit_out = 64;
    m_ruu_issue_width = 4;
    m_amp_idsat = 5.000000e-04;
    m_vs_inv = 4.560000e-01;
    m_gen_power_factor = 1.310000e+00;
    m_vth_nand_60x90 = 5.610000e-01;
    m_fudge_factor = 1.000000e+00;
    m_vth_outdrive = 4.250000e-01;
    m_vth_muxdrv1 = 4.370000e-01;
    m_vth_muxdrv2 = 4.860000e-01;
    m_normalize_scale = 6.488730e-10;
    m_vth_muxdrv3 = 4.370000e-01;
    m_address_bits = 64;
    m_ruu_size = 16;
    m_vth_nor_12x4x1 = 5.030000e-01;
    m_vth_nor_12x4x2 = 4.520000e-01;
    m_vth_outdr_inv = 4.370000e-01;
    m_vth_nor_12x4x3 = 4.170000e-01;
    m_vth_eval_inv = 2.670000e-01;
    m_vth_nor_12x4x4 = 3.900000e-01;
    m_res_ialu = 4;
    m_vth_outdr_nand = 4.410000e-01;
    m_vth_inv_100x60 = 4.380000e-01;

    if((m_tech_node >= 110))
    {
        init_tech_110_800();
    }
    else
    {
        init_tech_32_90();
    }
    return;
}

void TechParameter::init_tech_110_800()
{
    m_Cgatepass = 1.450000e-15;
    m_Cpdiffarea = 6.060000e-16; 
    m_Cpdiffside = 2.400000e-16; 
    m_Cndiffside = 2.400000e-16;
    m_Cndiffarea = 6.600000e-16; 
    m_Cnoverlap = 1.320000e-16;
    m_Cpoverlap = 1.210000e-16;
    m_Cgate = 9.040000e-15;
    m_Cpdiffovlp = 1.380000e-16;
    m_Cndiffovlp = 1.380000e-16;
    m_Cnoxideovlp = 2.230000e-16;
    m_Cpoxideovlp = 3.380000e-16;

    //TODO -- data not imported
    return;
}

void TechParameter::init_tech_32_90()
{
    switch(m_transistor_type)
    {
        case LVT:
            m_Cgatepass = 1.5225000e-14;
            m_Cpdiffarea = 6.05520000e-15;
            m_Cpdiffside = 2.38380000e-15;
            m_Cndiffside = 2.8500000e-16; 
            m_Cndiffarea = 5.7420000e-15;
            m_Cnoverlap = 1.320000e-16;
            m_Cpoverlap = 1.210000e-16;
            m_Cgate = 7.8648000e-14;
            m_Cpdiffovlp = 1.420000e-16;
            m_Cndiffovlp = 1.420000e-16;
            m_Cnoxideovlp = 2.580000e-16;
            m_Cpoxideovlp = 3.460000e-16;
            break;
        case NVT:
            m_Cgatepass = 8.32500e-15;
            m_Cpdiffarea = 3.330600e-15;
            m_Cpdiffside = 1.29940000e-15;
            m_Cndiffside = 2.5500000e-16;
            m_Cndiffarea = 2.9535000e-15;
            m_Cnoverlap = 1.270000e-16;
            m_Cpoverlap = 1.210000e-16;
            m_Cgate = 3.9664000e-14;
            m_Cpdiffovlp = 1.31000e-16;
            m_Cndiffovlp = 1.310000e-16;
            m_Cnoxideovlp = 2.410000e-16;  
            m_Cpoxideovlp = 3.170000e-16;
            break;
        case HVT:
            m_Cgatepass = 1.45000e-15;
            m_Cpdiffarea = 6.06000e-16;
            m_Cpdiffside = 2.150000e-16;
            m_Cndiffside = 2.25000e-16;  
            m_Cndiffarea = 1.650000e-16;
            m_Cnoverlap = 1.220000e-16;
            m_Cpoverlap = 1.210000e-16;
            m_Cgate = 6.8000e-16;
            m_Cpdiffovlp = 1.20000e-16;
            m_Cndiffovlp = 1.20000e-16;
            m_Cnoxideovlp = 2.230000e-16;
            m_Cpoxideovlp = 2.880000e-16;
            break;
        default:
            cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
            exit(1);
    }

    m_Vbitpre = m_vdd;
    m_Vbitsense = 0.08;
    m_EnergyFactor = m_vdd*m_vdd;
    m_SenseEnergyFactor = m_vdd*m_vdd/2;
    m_SenseEnergyFactor2 = (m_Vbitpre-m_Vbitsense)*(m_Vbitpre-m_Vbitsense);
    m_SenseEnergyFactor3 = m_Vbitsense*m_Vbitsense;

    if((m_tech_node == 90) || (m_tech_node == 65))
    {
        m_SCALE_T = 1;
        m_SCALE_M = 1;
        m_SCALE_S = 1;
        m_SCALE_W = 1;
        m_SCALE_H = 1;
        m_SCALE_BW = 1;
        m_SCALE_Crs = 1;
    }
    else if(m_tech_node == 45)
    {
        switch(m_transistor_type)
        {
            case LVT:
                m_SCALE_T = 0.9123404;
                m_SCALE_M = 0.6442105;
                m_SCALE_S = 2.3352694;
                m_SCALE_W = 0.51;
                m_SCALE_H = 0.88;
                m_SCALE_BW = 0.73;
                m_SCALE_Crs = 0.7;
                break;
            case NVT:
                m_SCALE_T = 0.8233582;
                m_SCALE_M = 0.6442105;
                m_SCALE_S = 2.1860558;
                m_SCALE_W = 0.51;
                m_SCALE_H = 0.88;
                m_SCALE_BW = 0.73;
                m_SCALE_Crs = 0.7;
                break;
            case HVT:
                m_SCALE_T = 0.73437604;
                m_SCALE_M = 0.6442105;
                m_SCALE_S = 2.036842;
                m_SCALE_W = 0.51;
                m_SCALE_H = 0.88;
                m_SCALE_BW = 0.73;
                m_SCALE_Crs = 0.7;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }
    }
    else if(m_tech_node == 32)
    {
        switch(m_transistor_type)
        {
            case LVT:
                m_SCALE_T = 0.7542128;
                m_SCALE_M = 0.4863158;
                m_SCALE_S = 2.9692334;
                m_SCALE_W = 0.26;
                m_SCALE_H = 0.77;
                m_SCALE_BW = 0.53;
                m_SCALE_Crs = 0.49;
                break;
            case NVT:
                m_SCALE_T = 0.6352095;
                m_SCALE_M = 0.4863158;
                m_SCALE_S = 3.1319851;
                m_SCALE_W = 0.26;
                m_SCALE_H = 0.77;
                m_SCALE_BW = 0.53;
                m_SCALE_Crs = 0.49;
                break;
            case HVT:
                m_SCALE_T = 0.5162063;
                m_SCALE_M = 0.4863158;
                m_SCALE_S = 3.294737;
                m_SCALE_W = 0.26;
                m_SCALE_H = 0.77;
                m_SCALE_BW = 0.53;
                m_SCALE_Crs = 0.49;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }
    }
    else
    {
        cerr << "Invalid technology node (" << m_tech_node << ")" << endl;
        exit(1);
    }

    if(m_tech_node == 90)
    {
        m_LSCALE = 0.125;
        m_MSCALE = (m_LSCALE * .624 / .2250);

        /* bit width of RAM cell in um */
        m_BitWidth = 2.0;

        /* bit height of RAM cell in um */
        m_BitHeight = 2.0;

        m_Cout = 6.25e-14;

        m_BitlineSpacing = 1.1;
        m_WordlineSpacing = 1.1;

        m_RegCellHeight = 2.8;
        m_RegCellWidth = 1.9;

        m_Cwordmetal = 1.936e-15;
        m_Cbitmetal = 3.872e-15;

        m_Cmetal = m_Cbitmetal/16;
        m_CM2metal = m_Cbitmetal/16;
        m_CM3metal = m_Cbitmetal/16;

        /* minimal spacing metal cap per unit length */
        m_CCmetal = 0.18608e-15;
        m_CCM2metal = 0.18608e-15;
        m_CCM3metal = 0.18608e-15;
        /* 2x minimal spacing metal cap per unit length */
        m_CC2metal = 0.12529e-15;
        m_CC2M2metal = 0.12529e-15;
        m_CC2M3metal = 0.12529e-15;
        /* 3x minimal spacing metal cap per unit length */
        m_CC3metal = 0.11059e-15;
        m_CC3M2metal = 0.11059e-15;
        m_CC3M3metal = 0.11059e-15;

        /* corresponds to clock network*/
        m_Clockwire = 404.8e-12;
        m_Reswire = 36.66e3;
        m_invCap = 3.816e-14;
        m_Resout = 213.6;

        /* um */
        m_Leff = 0.1;
        /* length unit in um */
        m_Lamda = m_Leff * 0.5;

        /* fF/um */
        m_Cpolywire = 2.6317875e-15;

        /* ohms*um of channel width */
        m_Rnchannelstatic = 3225;

        /* ohms*um of channel width */
        m_Rpchannelstatic = 7650;

        //derived from Cacti 5.3
        switch(m_transistor_type)
        {
            case LVT:
                m_Rnchannelon = 1716;
                m_Rpchannelon = 4202;
                break;
            case NVT:
                m_Rnchannelon = 4120;
                m_Rpchannelon = 10464;
                break;
            case HVT:
                m_Rnchannelon = 4956;
                m_Rpchannelon = 12092;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }


        m_Rbitmetal = 1.38048;
        m_Rwordmetal = 0.945536;

        switch(m_transistor_type)
        {
            case LVT:
                m_Vt = 0.237;
                break;
            case NVT:
                m_Vt = 0.307;
                break;
            case HVT:
                m_Vt = 0.482;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }

        /* transistor widths in um = as described in Cacti 1.0 tech report, appendix 1;*/
        switch(m_transistor_type)
        {
            case LVT:
                m_Wdecdrivep = 12.50;
                m_Wdecdriven = 6.25;
                m_Wdec3to8n = 11.25;
                m_Wdec3to8p = 7.5;
                m_WdecNORn = 0.30;
                m_WdecNORp = 1.5;
                m_Wdecinvn = 0.63;
                m_Wdecinvp = 1.25;
                m_Wdff = 12.29;

                m_Wworddrivemax = 12.50;
                m_Wmemcella = 0.35;
                m_Wmemcellr = 0.50;
                m_Wmemcellw = 0.26;
                m_Wmemcellbscale = 2;
                m_Wbitpreequ = 1.25;

                m_Wbitmuxn = 1.25;
                m_WsenseQ1to4 = 0.55;
                m_Wcompinvp1 = 1.25;
                m_Wcompinvn1 = 0.75;
                m_Wcompinvp2 = 2.50;
                m_Wcompinvn2 = 1.50;
                m_Wcompinvp3 = 5.15;
                m_Wcompinvn3 = 3.25;
                m_Wevalinvp = 2.50;
                m_Wevalinvn = 9.45;

                m_Wcompn = 1.25;
                m_Wcompp = 3.75;
                m_Wcomppreequ = 5.15;
                m_Wmuxdrv12n = 3.75;
                m_Wmuxdrv12p = 6.25;
                m_WmuxdrvNANDn = 2.50;
                m_WmuxdrvNANDp = 10.33;
                m_WmuxdrvNORn = 7.33;
                m_WmuxdrvNORp = 10.66;
                m_Wmuxdrv3n = 24.85;
                m_Wmuxdrv3p = 60.25;
                m_Woutdrvseln = 1.55;
                m_Woutdrvselp = 2.33;
                m_Woutdrvnandn = 3.27;
                m_Woutdrvnandp = 1.25;
                m_Woutdrvnorn = 0.75;
                m_Woutdrvnorp = 5.33;
                m_Woutdrivern = 6.16;
                m_Woutdriverp = 9.77;
                m_Wbusdrvn = 6.16;
                m_Wbusdrvp = 10.57;

                m_Wcompcellpd2 = 0.33;
                m_Wcompdrivern = 50.95;
                m_Wcompdriverp = 102.67;
                m_Wcomparen2 = 5.13;
                m_Wcomparen1 = 2.5;
                m_Wmatchpchg = 1.25;
                m_Wmatchinvn = 1.33;
                m_Wmatchinvp = 2.77;
                m_Wmatchnandn = 2.33;
                m_Wmatchnandp = 1.76;
                m_Wmatchnorn = 2.66;
                m_Wmatchnorp = 1.15;

                m_WSelORn = 1.25;
                m_WSelORprequ = 5.15;
                m_WSelPn = 1.86;
                m_WSelPp = 1.86;
                m_WSelEnn = 0.63;
                m_WSelEnp = 1.25;

                m_Wsenseextdrv1p = 5.15;
                m_Wsenseextdrv1n = 3.05;
                m_Wsenseextdrv2p = 25.20;
                m_Wsenseextdrv2n = 15.65;
                break;
            case NVT:
                m_Wdecdrivep = 11.57;
                m_Wdecdriven = 5.74;
                m_Wdec3to8n = 10.31;
                m_Wdec3to8p = 6.87;
                m_WdecNORn = 0.28;
                m_WdecNORp = 1.38;
                m_Wdecinvn = 0.58;
                m_Wdecinvp = 1.15;
                m_Wdff = 6.57;

                m_Wworddrivemax = 11.57;
                m_Wmemcella = 0.33;
                m_Wmemcellr = 0.46;
                m_Wmemcellw = 0.24;
                m_Wmemcellbscale = 2;
                m_Wbitpreequ = 1.15;

                m_Wbitmuxn = 1.15;
                m_WsenseQ1to4 = 0.49;
                m_Wcompinvp1 = 1.17;
                m_Wcompinvn1 = 0.69;
                m_Wcompinvp2 = 2.29;
                m_Wcompinvn2 = 1.38;
                m_Wcompinvp3 = 4.66;
                m_Wcompinvn3 = 2.88;
                m_Wevalinvp = 2.29;
                m_Wevalinvn = 8.89;

                m_Wcompn = 1.15;
                m_Wcompp = 3.44;
                m_Wcomppreequ = 4.66;
                m_Wmuxdrv12n = 3.44;
                m_Wmuxdrv12p = 5.74;
                m_WmuxdrvNANDn = 2.29;
                m_WmuxdrvNANDp = 9.33;
                m_WmuxdrvNORn = 6.79;
                m_WmuxdrvNORp = 9.49;
                m_Wmuxdrv3n = 22.83;
                m_Wmuxdrv3p = 55.09;
                m_Woutdrvseln = 1.40;
                m_Woutdrvselp = 2.21;
                m_Woutdrvnandn = 2.89;
                m_Woutdrvnandp = 1.15;
                m_Woutdrvnorn = 0.69;
                m_Woutdrvnorp = 4.75;
                m_Woutdrivern = 5.58;
                m_Woutdriverp = 9.05;
                m_Wbusdrvn = 5.58;
                m_Wbusdrvp = 9.45;

                m_Wcompcellpd2 = 0.29;
                m_Wcompdrivern = 46.28;
                m_Wcompdriverp = 92.94;
                m_Wcomparen2 = 4.65;
                m_Wcomparen1 = 2.29;
                m_Wmatchpchg = 1.15;
                m_Wmatchinvn = 1.19;
                m_Wmatchinvp = 2.43;
                m_Wmatchnandn = 2.21;
                m_Wmatchnandp = 1.42;
                m_Wmatchnorn = 2.37;
                m_Wmatchnorp = 1.10;

                m_WSelORn = 1.15;
                m_WSelORprequ = 4.66;
                m_WSelPn = 1.45;
                m_WSelPp = 1.71;
                m_WSelEnn = 0.58;
                m_WSelEnp = 1.15;

                m_Wsenseextdrv1p = 4.66;
                m_Wsenseextdrv1n = 2.78;
                m_Wsenseextdrv2p = 23.02;
                m_Wsenseextdrv2n = 14.07;
                break;
            case HVT:
                m_Wdecdrivep = 10.64;
                m_Wdecdriven = 5.23;
                m_Wdec3to8n = 9.36;
                m_Wdec3to8p = 6.24;
                m_WdecNORn = 0.25;
                m_WdecNORp = 1.25;
                m_Wdecinvn = 0.52;
                m_Wdecinvp = 1.04;
                m_Wdff = 5.43;

                m_Wworddrivemax = 10.64;
                m_Wmemcella = 0.25;
                m_Wmemcellr = 0.42;
                m_Wmemcellw = 0.22;
                m_Wmemcellbscale = 2;
                m_Wbitpreequ = 1.04;

                m_Wbitmuxn = 1.04;
                m_WsenseQ1to4 = 0.42;
                m_Wcompinvp1 = 1.08;
                m_Wcompinvn1 = 0.62;
                m_Wcompinvp2 = 2.08;
                m_Wcompinvn2 = 1.25;
                m_Wcompinvp3 = 4.16;
                m_Wcompinvn3 = 2.50;
                m_Wevalinvp = 2.08;
                m_Wevalinvn = 8.32;

                m_Wcompn = 1.04;
                m_Wcompp = 3.12;
                m_Wcomppreequ = 4.16;
                m_Wmuxdrv12n = 3.12;
                m_Wmuxdrv12p = 5.23;
                m_WmuxdrvNANDn = 2.08;
                m_WmuxdrvNANDp = 8.32;
                m_WmuxdrvNORn = 6.24;
                m_WmuxdrvNORp = 8.32;
                m_Wmuxdrv3n = 20.80;
                m_Wmuxdrv3p = 49.92;
                m_Woutdrvseln = 1.25;
                m_Woutdrvselp = 2.08;
                m_Woutdrvnandn = 2.50;
                m_Woutdrvnandp = 1.04;
                m_Woutdrvnorn = 0.62;
                m_Woutdrvnorp = 4.16;
                m_Woutdrivern = 4.99;
                m_Woutdriverp = 8.32;
                m_Wbusdrvn = 4.99;
                m_Wbusdrvp = 8.32;

                m_Wcompcellpd2 = 0.25;
                m_Wcompdrivern = 41.60;
                m_Wcompdriverp = 83.20;
                m_Wcomparen2 = 4.16;
                m_Wcomparen1 = 2.08;
                m_Wmatchpchg = 1.04;
                m_Wmatchinvn = 1.04;
                m_Wmatchinvp = 2.08;
                m_Wmatchnandn = 2.08;
                m_Wmatchnandp = 1.08;
                m_Wmatchnorn = 2.08;
                m_Wmatchnorp = 1.04;

                m_WSelORn = 1.04;
                m_WSelORprequ = 4.16;
                m_WSelPn = 1.04;
                m_WSelPp = 1.56;
                m_WSelEnn = 0.52;
                m_WSelEnp = 1.04;

                m_Wsenseextdrv1p = 4.16;
                m_Wsenseextdrv1n = 2.50;
                m_Wsenseextdrv2p = 20.83;
                m_Wsenseextdrv2n = 12.48;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }

        m_CamCellHeight = 4.095;/*derived from Cacti 5.3 */ 
        m_CamCellWidth = 3.51;/*derived from Cacti 5.3 */ 

        m_MatchlineSpacing = 0.75;
        m_TaglineSpacing = 0.75;

        m_CrsbarCellHeight = 2.94;
        m_CrsbarCellWidth = 2.94;

        m_krise = 0.5e-10;
        m_tsensedata = 0.725e-10;
        m_tsensetag = 0.325e-10;
        m_tfalldata = 0.875e-10;
        m_tfalltag = 0.875e-10;
        /*=============Above are the parameters for 90nm ========================*/
    }
    else if(m_tech_node <= 65)
    {
        /*=============Below are the parameters for 65nm ========================*/

        m_LSCALE = 0.087;
        m_MSCALE = m_LSCALE * .624 / .2250;

        /* bit width of RAM cell in um */
        m_BitWidth = 1.4;

        /* bit height of RAM cell in um */
        m_BitHeight = 1.4;

        m_Cout = 4.35e-14;

        /* Sizing of cells and spacings */
        m_BitlineSpacing = 0.8 * m_SCALE_BW;
        m_WordlineSpacing = 0.8 * m_SCALE_BW;

        m_RegCellHeight = 2.1 * m_SCALE_H;
        m_RegCellWidth = 1.4 * m_SCALE_W;

        m_Cwordmetal = 1.63e-15 * m_SCALE_M;
        m_Cbitmetal = 3.27e-15 * m_SCALE_M;

        m_Cmetal = m_Cbitmetal/16;
        m_CM2metal = m_Cbitmetal/16;
        m_CM3metal = m_Cbitmetal/16;

        // minimum spacing
        m_CCmetal = 0.146206e-15;
        m_CCM2metal = 0.146206e-15;
        m_CCM3metal = 0.146206e-15;
        // 2x minimum spacing
        m_CC2metal = 0.09844e-15;
        m_CC2M2metal = 0.09844e-15;
        m_CC2M3metal = 0.09844e-15;
        // 3x minimum spacing
        m_CC3metal = 0.08689e-15;
        m_CC3M2metal = 0.08689e-15;
        m_CC3M3metal = 0.08689e-15;


        /* corresponds to clock network*/
        m_Clockwire = 323.4e-12 * m_SCALE_M;
        m_Reswire = 61.11e3 * 1.0/m_SCALE_M;
        m_invCap = 3.12e-14;
        m_Resout = 361.00;

        /* um */
        m_Leff = 0.0696;
        /* length unit in um */
        m_Lamda = m_Leff * 0.5;

        /* fF/um */
        m_Cpolywire = 1.832e-15;
        /* ohms*um of channel width */
        m_Rnchannelstatic = 2244.6;

        /* ohms*um of channel width */
        m_Rpchannelstatic = 5324.4;

        switch(m_transistor_type)
        {
            case LVT:
                m_Rnchannelon = 1370;
                m_Rpchannelon = 3301;
                break;
            case NVT:
                m_Rnchannelon = 2540;
                m_Rpchannelon = 5791;
                break;
            case HVT:
                m_Rnchannelon = 4530;
                m_Rpchannelon = 10101;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }

        m_Rbitmetal = 1.92644;/* derived from Cacti 5.3 */  
        m_Rwordmetal = 1.31948;/* derived from Cacti 5.3 */

        switch(m_transistor_type)
        {
            case LVT:
                m_Vt = 0.195;
                break;
            case NVT:
                m_Vt = 0.285;
                break;
            case HVT:
                m_Vt = 0.524;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }

        /* transistor widths in um for 65nm. = as described in Cacti 1.0 tech report, appendix 1;*/
        switch(m_transistor_type)
        {
            case LVT:
                m_Wdecdrivep = 8.27;
                m_Wdecdriven = 6.70;
                m_Wdec3to8n = 2.33;
                m_Wdec3to8p = 2.33;
                m_WdecNORn = 1.50;
                m_WdecNORp = 3.82;
                m_Wdecinvn = 8.46;
                m_Wdecinvp = 10.93;
                m_Wdff = 8.6;

                m_Wworddrivemax = 9.27;
                m_Wmemcella = 0.2225;
                m_Wmemcellr = 0.3708;
                m_Wmemcellw = 0.1947;
                m_Wmemcellbscale = 1.87;
                m_Wbitpreequ = 0.927;

                m_Wbitmuxn = 0.927;
                m_WsenseQ1to4 = 0.371;
                m_Wcompinvp1 = 0.927;
                m_Wcompinvn1 = 0.5562;
                m_Wcompinvp2 = 1.854;
                m_Wcompinvn2 = 1.1124;
                m_Wcompinvp3 = 3.708;
                m_Wcompinvn3 = 2.2248;
                m_Wevalinvp = 1.854;
                m_Wevalinvn = 7.416;


                m_Wcompn = 1.854;
                m_Wcompp = 2.781;
                m_Wcomppreequ = 3.712;
                m_Wmuxdrv12n = 2.785;
                m_Wmuxdrv12p = 4.635;
                m_WmuxdrvNANDn = 1.860;
                m_WmuxdrvNANDp = 7.416;
                m_WmuxdrvNORn = 5.562;
                m_WmuxdrvNORp = 7.416;
                m_Wmuxdrv3n = 18.54;
                m_Wmuxdrv3p = 44.496;
                m_Woutdrvseln = 1.112;
                m_Woutdrvselp = 1.854;
                m_Woutdrvnandn = 2.225;
                m_Woutdrvnandp = 0.927;
                m_Woutdrvnorn = 0.5562;
                m_Woutdrvnorp = 3.708;
                m_Woutdrivern = 4.450;
                m_Woutdriverp = 7.416;
                m_Wbusdrvn = 4.450;
                m_Wbusdrvp = 7.416;

                m_Wcompcellpd2 = 0.222;
                m_Wcompdrivern = 37.08;
                m_Wcompdriverp = 74.20;
                m_Wcomparen2 = 3.708;
                m_Wcomparen1 = 1.854;
                m_Wmatchpchg = 0.927;
                m_Wmatchinvn = 0.930;
                m_Wmatchinvp = 1.854;
                m_Wmatchnandn = 1.854;
                m_Wmatchnandp = 0.927;
                m_Wmatchnorn = 1.860;
                m_Wmatchnorp = 0.930;

                m_WSelORn = 0.930;
                m_WSelORprequ = 3.708;
                m_WSelPn = 0.927;
                m_WSelPp = 1.391;
                m_WSelEnn = 0.434;
                m_WSelEnp = 0.930;

                m_Wsenseextdrv1p = 3.708;
                m_Wsenseextdrv1n = 2.225;
                m_Wsenseextdrv2p = 18.54;
                m_Wsenseextdrv2n = 11.124;
                break;
            case NVT:
                m_Wdecdrivep = 6.7;
                m_Wdecdriven = 4.7;
                m_Wdec3to8n = 1.33;
                m_Wdec3to8p = 1.33;
                m_WdecNORn = 1.20;
                m_WdecNORp = 2.62;
                m_Wdecinvn = 1.46;
                m_Wdecinvp = 3.93;
                m_Wdff = 4.6;

                m_Wworddrivemax = 9.225;
                m_Wmemcella = 0.221;
                m_Wmemcellr = 0.369;
                m_Wmemcellw = 0.194;
                m_Wmemcellbscale = 1.87;
                m_Wbitpreequ = 0.923;

                m_Wbitmuxn = 0.923;
                m_WsenseQ1to4 = 0.369;
                m_Wcompinvp1 = 0.924;
                m_Wcompinvn1 = 0.554;
                m_Wcompinvp2 = 1.845;
                m_Wcompinvn2 = 1.107;
                m_Wcompinvp3 = 3.69;
                m_Wcompinvn3 = 2.214;
                m_Wevalinvp = 1.842;
                m_Wevalinvn = 7.368;

                m_Wcompn = 1.845;
                m_Wcompp = 2.768;
                m_Wcomppreequ = 3.692;
                m_Wmuxdrv12n = 2.773;
                m_Wmuxdrv12p = 4.618;
                m_WmuxdrvNANDn = 1.848;
                m_WmuxdrvNANDp = 7.38;
                m_WmuxdrvNORn = 5.535;
                m_WmuxdrvNORp = 7.380;
                m_Wmuxdrv3n = 18.45;
                m_Wmuxdrv3p = 44.28;
                m_Woutdrvseln = 1.105;
                m_Woutdrvselp = 1.842;
                m_Woutdrvnandn = 2.214;
                m_Woutdrvnandp = 0.923;
                m_Woutdrvnorn = 0.554;
                m_Woutdrvnorp = 3.69;
                m_Woutdrivern = 4.428;
                m_Woutdriverp = 7.380;
                m_Wbusdrvn = 4.421;
                m_Wbusdrvp = 7.368;

                m_Wcompcellpd2 = 0.221;
                m_Wcompdrivern = 36.84;
                m_Wcompdriverp = 73.77;
                m_Wcomparen2 = 3.684;
                m_Wcomparen1 = 1.842;
                m_Wmatchpchg = 0.921;
                m_Wmatchinvn = 0.923;
                m_Wmatchinvp = 1.852;
                m_Wmatchnandn = 1.852;
                m_Wmatchnandp = 0.921;
                m_Wmatchnorn = 1.845;
                m_Wmatchnorp = 0.923;

                m_WSelORn = 0.923;
                m_WSelORprequ = 3.684;
                m_WSelPn = 0.921;
                m_WSelPp = 1.382;
                m_WSelEnn = 0.446;
                m_WSelEnp = 0.923;

                m_Wsenseextdrv1p = 3.684;
                m_Wsenseextdrv1n = 2.211;
                m_Wsenseextdrv2p = 18.42;
                m_Wsenseextdrv2n = 11.052;
                break;
            case HVT:
                m_Wdecdrivep = 3.11;
                m_Wdecdriven = 1.90;
                m_Wdec3to8n = 1.33;
                m_Wdec3to8p = 1.33;
                m_WdecNORn = 0.90;
                m_WdecNORp = 1.82;
                m_Wdecinvn = 0.46;
                m_Wdecinvp = 0.93;
                m_Wdff = 3.8;

                m_Wworddrivemax = 9.18;
                m_Wmemcella = 0.220;
                m_Wmemcellr = 0.367;
                m_Wmemcellw = 0.193;
                m_Wmemcellbscale = 1.87;
                m_Wbitpreequ = 0.918;

                m_Wbitmuxn = 0.918;
                m_WsenseQ1to4 = 0.366;
                m_Wcompinvp1 = 0.920;
                m_Wcompinvn1 = 0.551;
                m_Wcompinvp2 = 1.836;
                m_Wcompinvn2 = 1.102;
                m_Wcompinvp3 = 3.672;
                m_Wcompinvn3 = 2.203;
                m_Wevalinvp = 1.83;
                m_Wevalinvn = 7.32;

                m_Wcompn = 1.836;
                m_Wcompp = 2.754;
                m_Wcomppreequ = 3.672;
                m_Wmuxdrv12n = 2.760;
                m_Wmuxdrv12p = 4.60;
                m_WmuxdrvNANDn = 1.836;
                m_WmuxdrvNANDp = 7.344;
                m_WmuxdrvNORn = 5.508;
                m_WmuxdrvNORp = 7.344;
                m_Wmuxdrv3n = 18.36;
                m_Wmuxdrv3p = 44.064;
                m_Woutdrvseln = 1.098;
                m_Woutdrvselp = 1.83;
                m_Woutdrvnandn = 2.203;
                m_Woutdrvnandp = 0.918;
                m_Woutdrvnorn = 0.551;
                m_Woutdrvnorp = 3.672;
                m_Woutdrivern = 4.406;
                m_Woutdriverp = 7.344;
                m_Wbusdrvn = 4.392;
                m_Wbusdrvp = 7.32;

                m_Wcompcellpd2 = 0.220;
                m_Wcompdrivern = 36.6;
                m_Wcompdriverp = 73.33;
                m_Wcomparen2 = 3.66;
                m_Wcomparen1 = 1.83;
                m_Wmatchpchg = 0.915;
                m_Wmatchinvn = 0.915;
                m_Wmatchinvp = 1.85;
                m_Wmatchnandn = 1.85;
                m_Wmatchnandp = 0.915;
                m_Wmatchnorn = 1.83;
                m_Wmatchnorp = 0.915;

                m_WSelORn = 0.915;
                m_WSelORprequ = 3.66;
                m_WSelPn = 0.915;
                m_WSelPp = 1.373;
                m_WSelEnn = 0.458;
                m_WSelEnp = 0.915;

                m_Wsenseextdrv1p = 3.66;
                m_Wsenseextdrv1n = 2.196;
                m_Wsenseextdrv2p = 18.3;
                m_Wsenseextdrv2n = 10.98;
                break;
            default:
                cerr << "Invalid transistor type (" << m_transistor_type << ")" << endl;
                exit(1);
        }

        m_CamCellHeight = 2.9575;/* derived from Cacti 5.3 */ 
        m_CamCellWidth = 2.535;/* derived from Cacti 5.3 */

        m_MatchlineSpacing = 0.522;
        m_TaglineSpacing = 0.522;

        m_CrsbarCellHeight = 2.06 * m_SCALE_Crs;
        m_CrsbarCellWidth = 2.06 * m_SCALE_Crs;

        m_krise = 0.348e-10;
        m_tsensedata = 0.5046e-10;
        m_tsensetag = 0.2262e-10;
        m_tfalldata = 0.609e-10;
        m_tfalltag = 0.6609e-10;
    }

    /*=======================PARAMETERS for Link===========================*/

    if(m_tech_node == 90)
    {
        if(m_wire_layer_type == LOCAL)
        {
            m_WireMinWidth            = 214e-9;
            m_WireMinSpacing          = 214e-9;
            m_WireMetalThickness      = 363.8e-9;
            m_WireBarrierThickness    = 10e-9;
            m_WireDielectricThickness = 363.8e-9;
            m_WireDielectricConstant  = 3.3;
        }
        else if(m_wire_layer_type == INTERMEDIATE)
        {
            m_WireMinWidth            = 275e-9;
            m_WireMinSpacing          = 275e-9;
            m_WireMetalThickness      = 467.5e-9;
            m_WireBarrierThickness    = 10e-9;
            m_WireDielectricThickness = 412.5e-9;
            m_WireDielectricConstant  = 3.3;
        }
        else if(m_wire_layer_type == GLOBAL)
        {
            m_WireMinWidth            = 410e-9;
            m_WireMinSpacing          = 410e-9;
            m_WireMetalThickness      = 861e-9;
            m_WireBarrierThickness    = 10e-9;
            m_WireDielectricThickness = 779e-9;
            m_WireDielectricConstant  = 3.3;
        }
    }
    else if(m_tech_node == 65)
    {
        if(m_wire_layer_type == LOCAL)
        {
            m_WireMinWidth 		        = 136e-9;
            m_WireMinSpacing 		      = 136e-9;
            m_WireMetalThickness	    = 231.2e-9;
            m_WireBarrierThickness	  = 4.8e-9;
            m_WireDielectricThickness	= 231.2e-9;
            m_WireDielectricConstant	= 2.85;
        }
        else if(m_wire_layer_type == INTERMEDIATE)
        {
            m_WireMinWidth            = 140e-9;
            m_WireMinSpacing          = 140e-9;
            m_WireMetalThickness      = 252e-9;
            m_WireBarrierThickness    = 5.2e-9;
            m_WireDielectricThickness = 224e-9;
            m_WireDielectricConstant  = 2.85;
        }
        else if(m_wire_layer_type == GLOBAL)
        {
            m_WireMinWidth            = 400e-9;
            m_WireMinSpacing          = 400e-9;
            m_WireMetalThickness      = 400e-9;
            m_WireBarrierThickness    = 5.2e-9;
            m_WireDielectricThickness = 790e-9;
            m_WireDielectricConstant  = 2.9;
        }
    }
    else if(m_tech_node == 45)
    {
        if(m_wire_layer_type == LOCAL)
        {
            m_WireMinWidth            = 45e-9;
            m_WireMinSpacing          = 45e-9;
            m_WireMetalThickness      = 129.6e-9;
            m_WireBarrierThickness    = 3.3e-9;
            m_WireDielectricThickness = 162e-9;
            m_WireDielectricConstant  = 2.0;
        }
        else if(m_wire_layer_type == INTERMEDIATE)
        {
            m_WireMinWidth            = 45e-9;
            m_WireMinSpacing          = 45e-9;
            m_WireMetalThickness      = 129.6e-9;
            m_WireBarrierThickness    = 3.3e-9;
            m_WireDielectricThickness = 72e-9;
            m_WireDielectricConstant  = 2.0;
        }
        else if(m_wire_layer_type == GLOBAL)
        {
            m_WireMinWidth            = 67.5e-9;
            m_WireMinSpacing          = 67.5e-9;
            m_WireMetalThickness      = 155.25e-9;
            m_WireBarrierThickness    = 3.3e-9;
            m_WireDielectricThickness = 141.75e-9;
            m_WireDielectricConstant  = 2.0;
        }
    }
    else if(m_tech_node == 32)
    {
        if(m_wire_layer_type == LOCAL)
        {
            m_WireMinWidth            = 32e-9;
            m_WireMinSpacing          = 32e-9;
            m_WireMetalThickness      = 60.8e-9;
            m_WireBarrierThickness    = 2.4e-9;
            m_WireDielectricThickness = 60.8e-9;
            m_WireDielectricConstant  = 1.9;
        }
        else if(m_wire_layer_type == INTERMEDIATE)
        {
            m_WireMinWidth            = 32e-9;
            m_WireMinSpacing          = 32e-9;
            m_WireMetalThickness      = 60.8e-9;
            m_WireBarrierThickness    = 2.4e-9;
            m_WireDielectricThickness = 54.4e-9;
            m_WireDielectricConstant  = 1.9;
        }
        else if(m_wire_layer_type == GLOBAL)
        {
            m_WireMinWidth            = 48e-9;
            m_WireMinSpacing          = 48e-9;
            m_WireMetalThickness      = 120e-9;
            m_WireBarrierThickness    = 2.4e-9;
            m_WireDielectricThickness = 110.4e-9;
            m_WireDielectricConstant  = 1.9;
        }
    }

    /*===================================================================*/
    /*parameters for insertion buffer for links at 90nm*/
    if(m_tech_node == 90)
    {
        m_BufferDriveResistance     =  5.12594e+03;
        m_BufferIntrinsicDelay      =  4.13985e-11;
        if(m_transistor_type == LVT)
        {
            m_BufferInputCapacitance    =  1.59e-15;
            m_BufferPMOSOffCurrent      =  116.2e-09;
            m_BufferNMOSOffCurrent      =  52.1e-09;
            m_ClockCap 		              = 2.7e-14;
        }
        else if(m_transistor_type == NVT)
        {
            m_BufferInputCapacitance    =  4.7e-15;
            m_BufferPMOSOffCurrent      =  67.6e-09;
            m_BufferNMOSOffCurrent      =  31.1e-09;
            m_ClockCap                  =  1.0e-14;
        }
        else if(m_transistor_type == HVT)
        {
            m_BufferInputCapacitance    =  15.0e-15;//9.5e-15
            m_BufferPMOSOffCurrent      =  19.2e-09;
            m_BufferNMOSOffCurrent      =  10.1e-09; 
            m_ClockCap                  =  0.3e-15;
        }
    }
    /*parameters for insertion buffer for links at 65nm*/
    else if(m_tech_node == 65)
    {
        m_BufferDriveResistance     = 6.77182e+03;
        m_BufferIntrinsicDelay	    = 3.31822e-11;
        if(m_transistor_type == LVT)
        {
            m_BufferPMOSOffCurrent	    = 317.2e-09;
            m_BufferNMOSOffCurrent	    = 109.7e-09;
            m_BufferInputCapacitance    = 1.3e-15;
            m_ClockCap                  = 2.6e-14;
        }
        else if(m_transistor_type == NVT)
        {
            m_BufferPMOSOffCurrent      = 113.1e-09;
            m_BufferNMOSOffCurrent      = 67.3e-09;
            m_BufferInputCapacitance    = 2.6e-15;
            m_ClockCap                  = 1.56e-14;
        }
        else if(m_transistor_type == HVT)
        {
            m_BufferPMOSOffCurrent      = 35.2e-09;
            m_BufferNMOSOffCurrent      = 18.4e-09;
            m_BufferInputCapacitance    = 7.8e-15;
            m_ClockCap                  = 0.9e-15;
        }
    }
    /*parameters for insertion buffer for links at 45nm*/
    else if(m_tech_node == 45)
    {
        m_BufferDriveResistance      = 7.3228e+03;
        m_BufferIntrinsicDelay       = 4.6e-11;
        if(m_transistor_type == LVT)
        {
            m_BufferInputCapacitance     = 1.25e-15;
            m_BufferPMOSOffCurrent       = 1086.75e-09;
            m_BufferNMOSOffCurrent       = 375.84e-09;
            m_ClockCap                   = 2.5e-14;
        }
        else if(m_transistor_type == NVT)
        {
            m_BufferInputCapacitance     = 2.5e-15;	
            m_BufferPMOSOffCurrent       = 382.3e-09;
            m_BufferNMOSOffCurrent       = 195.5e-09;
            m_ClockCap                   = 1.5e-14;
        }
        else if(m_transistor_type == HVT)
        {
            m_BufferInputCapacitance     = 7.5e-15;
            m_BufferPMOSOffCurrent       = 76.4e-09;
            m_BufferNMOSOffCurrent       = 39.1e-09;
            m_ClockCap                   = 0.84e-15;
        }
    }
    /*parameters for insertion buffer for links at 32nm*/
    else if(m_tech_node == 32)
    {
        m_BufferDriveResistance      = 10.4611e+03;
        m_BufferIntrinsicDelay       = 4.0e-11;
        if(m_transistor_type == LVT)
        {
            m_BufferPMOSOffCurrent       = 1630.08e-09;
            m_BufferNMOSOffCurrent       = 563.74e-09;
            m_BufferInputCapacitance     = 1.2e-15;
            m_ClockCap                   = 2.2e-14;
        }
        else if(m_transistor_type == NVT)
        {
            m_BufferPMOSOffCurrent       = 792.4e-09;
            m_BufferNMOSOffCurrent       = 405.1e-09;
            m_BufferInputCapacitance     = 2.4e-15;
            m_ClockCap                   = 1.44e-14;
        }
        else if(m_transistor_type == HVT)
        {
            m_BufferPMOSOffCurrent       = 129.9e-09;
            m_BufferNMOSOffCurrent       = 66.4e-09;
            m_BufferInputCapacitance     = 7.2e-15;
            m_ClockCap                   = 0.53e-15;
        }
    }

    /*======================Parameters for Area===========================*/
    if(m_tech_node == 90)
    {
        m_AreaNOR = 4.23;
        m_AreaINV = 2.82;
        m_AreaAND = 4.23;
        m_AreaDFF = 16.23;
        m_AreaMUX2 = 7.06;
        m_AreaMUX3 = 11.29;
        m_AreaMUX4 = 16.93;
    }
    else if(m_tech_node <= 65)
    {
        m_AreaNOR = 2.52 * m_SCALE_T;
        m_AreaINV = 1.44 * m_SCALE_T;
        m_AreaDFF = 8.28 * m_SCALE_T;
        m_AreaAND = 2.52 * m_SCALE_T;
        m_AreaMUX2 = 6.12 * m_SCALE_T;
        m_AreaMUX3 = 9.36 * m_SCALE_T;
        m_AreaMUX4 = 12.6 * m_SCALE_T;
    }

    if (m_tech_node == 90)
    {
        if (m_transistor_type == LVT)
        {
            m_NMOS_TAB[0] = 19.9e-9;
            m_PMOS_TAB[0] = 16.6e-9;
            m_NAND2_TAB[0] = 7.8e-9;
            m_NAND2_TAB[1] = 24.6e-9;
            m_NAND2_TAB[2] = 14.1e-9;
            m_NAND2_TAB[3] = 34.3e-9;
            m_NOR2_TAB[0] = 51.2e-9;
            m_NOR2_TAB[1] = 23.9e-9;
            m_NOR2_TAB[2] = 19.5e-9;
            m_NOR2_TAB[3] = 8.4e-9;
            m_DFF_TAB[0] = 219.7e-9;
        }
        else if (m_transistor_type == NVT)
        {
            m_NMOS_TAB[0] = 15.6e-9;
            m_PMOS_TAB[0] = 11.3e-9;
            m_NAND2_TAB[0] = 2.8e-9;
            m_NAND2_TAB[1] = 19.6e-9;
            m_NAND2_TAB[2] = 10.4e-9;
            m_NAND2_TAB[3] = 29.3e-9;
            m_NOR2_TAB[0] = 41.5e-9;
            m_NOR2_TAB[1] = 13.1e-9;
            m_NOR2_TAB[2] = 14.5e-9;
            m_NOR2_TAB[3] = 1.4e-9;
            m_DFF_TAB[0] = 194.7e-9;
        }
        else
        {
            m_NMOS_TAB[0] = 12.2e-9;
            m_PMOS_TAB[0] = 9.3e-9;
            m_NAND2_TAB[0] = 1.8e-9;
            m_NAND2_TAB[1] = 12.4e-9;
            m_NAND2_TAB[2] = 8.9e-9;
            m_NAND2_TAB[3] = 19.3e-9;
            m_NOR2_TAB[0] = 29.5e-9;
            m_NOR2_TAB[1] = 8.3e-9;
            m_NOR2_TAB[2] = 11.1e-9;
            m_NOR2_TAB[3] = 0.9e-9;
            m_DFF_TAB[0] = 194.7e-9; //FIXME-the same as NVT?
        }
    }
    else if (m_tech_node <= 65)
    {
        if (m_transistor_type == LVT)
        {
            m_NMOS_TAB[0] = 311.7e-9;
            m_PMOS_TAB[0] = 674.3e-9;
            m_NAND2_TAB[0] = 303.0e-9;
            m_NAND2_TAB[1] = 423.0e-9;
            m_NAND2_TAB[2] = 498.3e-9;
            m_NAND2_TAB[3] = 626.3e-9;
            m_NOR2_TAB[0] = 556.0e-9;
            m_NOR2_TAB[1] = 393.7e-9;
            m_NOR2_TAB[2] = 506.7e-9;
            m_NOR2_TAB[3] = 369.7e-9;
            m_DFF_TAB[0] = 970.4e-9;
        }
        else if (m_transistor_type == NVT)
        {
            m_NMOS_TAB[0] = 115.1e-9;
            m_PMOS_TAB[0] = 304.8e-9;
            m_NAND2_TAB[0] = 111.4e-9;
            m_NAND2_TAB[1] = 187.2e-9;
            m_NAND2_TAB[2] = 230.7e-9;
            m_NAND2_TAB[3] = 306.9e-9;
            m_NOR2_TAB[0] = 289.7e-9;
            m_NOR2_TAB[1] = 165.7e-9;
            m_NOR2_TAB[2] = 236.9e-9;
            m_NOR2_TAB[3] = 141.4e-9;
            m_DFF_TAB[0] = 400.3e-9;
        }
        else
        {
            m_NMOS_TAB[0] = 18.4e-9;
            m_PMOS_TAB[0] = 35.2e-9;
            m_NAND2_TAB[0] = 19.7e-9;
            m_NAND2_TAB[1] = 51.3e-9;
            m_NAND2_TAB[2] = 63.0e-9;
            m_NAND2_TAB[3] = 87.6e-9;
            m_NOR2_TAB[0] = 23.4e-9;
            m_NOR2_TAB[1] = 37.6e-9;
            m_NOR2_TAB[2] = 67.9e-9;
            m_NOR2_TAB[3] = 12.3e-9;
            m_DFF_TAB[0] = 231.3e-9;
        }
    }
    return;
}

//
// width - gate width in um (length is Leff)   
// wirelength - poly wire length going to gate in lambda
// return gate capacitance in Farads 
double TechParameter::calc_gatecap(
        double width_, 
        double wirelength_
        ) const
{
    return ((width_*m_Leff*m_Cgate + wirelength_*m_Cpolywire*m_Leff)*m_SCALE_T);
}

double TechParameter::calc_gatecappass(
        double width_,
        double wirelength_
        ) const
{
    return calc_gatecap(width_, wirelength_);
}

// Routine for calculating drain capacitances.  The draincap routine
// folds transistors larger than 10um */
//
// width - um   
// nchannel - whether n or p-channel (boolean)
// stack - number of transistors in series that are on
//
// return drain cap in Farads 
double TechParameter::calc_draincap(
        double width_,
        ChannelType ch_,
        uint32_t num_stack_
        ) const
{
    double Cdiffside, Cdiffarea, Coverlap, total_cap;

    if (ch_ == NCH)
    {
        Cdiffside = m_Cndiffside;
        Cdiffarea = m_Cndiffarea;
        Coverlap = m_Cnoverlap;
    }
    else
    {
        Cdiffside = m_Cpdiffside;
        Cdiffarea = m_Cpdiffarea;
        Coverlap = m_Cpoverlap;
    }

    if (width_ >= 10)
    {
        total_cap = (3.0*m_Leff*width_/2.0)*Cdiffarea + 6.0*m_Leff*Cdiffside + width_*Coverlap;
        total_cap += (double)(num_stack_ - 1)*(m_Leff*width_*Cdiffarea + 4.0*m_Leff*Cdiffside + 2.0*width_*Coverlap);
    }
    else
    {
        total_cap = (3.0*m_Leff*width_)*Cdiffarea + (6.0*m_Leff + width_)*Cdiffside + width_*Coverlap;
        total_cap += (double)(num_stack_ - 1)*(m_Leff*width_*Cdiffarea + 2.0*m_Leff*Cdiffside + 2.0*width_*Coverlap);
    }

    return (total_cap*m_SCALE_T);
}

double TechParameter::calc_restowidth(
        double res_, 
        ChannelType ch_
        ) const
{
    double restrans;

    restrans = (ch_ == NCH)? m_Rnchannelon : m_Rpchannelon;

    return (restrans / res_);
}

double TechParameter::calc_driver_psize(
        double cap_, 
        double rise_time_
        ) const
{
    double psize;
    double Rpdrive;

    Rpdrive = rise_time_ / (cap_ * log(m_vs_inv) * (-1.0));
    psize = calc_restowidth(Rpdrive, PCH);

    if (psize > m_Wworddrivemax)
    {
        psize = m_Wworddrivemax;
    }
    if (psize < 4.0 * m_LSCALE)
    {
        psize = 4.0 * m_LSCALE;
    }

    return psize;
}

