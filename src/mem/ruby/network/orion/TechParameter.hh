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

#ifndef __TECHPARAMETER_H__
#define __TECHPARAMETER_H__

#include <string>

#include "mem/ruby/network/orion/Type.hh"

class OrionConfig;

class TechParameter
{
  public:
    enum TransistorType
    {
      LVT = 0,
      NVT,
      HVT,
      NUM_TRANSISTOR_TYPE
    };
    enum WireLayerType
    {
      LOCAL = 0,
      INTERMEDIATE,
      GLOBAL,
      NUM_WIRE_LAYER_TYPE
    };
    enum ChannelType
    {
      NCH,
      PCH
    };

  public:
    TechParameter(const OrionConfig* orion_cfg_ptr_);
    ~TechParameter();

  public:
    double calc_gatecap(double width_, double wirelength_) const;
    double calc_gatecappass(double width_, double wirelength_) const;
    double calc_draincap(double width_, ChannelType ch_, uint32_t num_stack_) const;
    double calc_restowidth(double res_, ChannelType ch_) const;
    double calc_driver_psize(double cap_, double rise_time_) const;

  public:
    bool is_trans_type_hvt() const { return (m_transistor_type == HVT); }
    bool is_trans_type_nvt() const { return (m_transistor_type == NVT); }
    bool is_trans_type_lvt() const { return (m_transistor_type == LVT); }
    double get_tech_node() const { return m_tech_node; }
    double get_vdd() const { return m_vdd; }
    double get_freq() const { return m_freq; }
    double get_period() const { return m_period; }
    double get_amp_idsat() const { return m_amp_idsat; }

    double get_SCALE_S() const { return m_SCALE_S; }
    double get_MSCALE() const { return m_MSCALE; }

    double get_Lamda() const { return m_Lamda; }

    double get_Cmetal() const { return m_Cmetal; }
    double get_CM2metal() const { return m_CM2metal; }
    double get_CM3metal() const { return m_CM3metal; }
    double get_CCmetal() const { return m_CCmetal; }
    double get_CCM2metal() const { return m_CCM2metal; }
    double get_CCM3metal() const { return m_CCM3metal; }
    double get_CC2metal() const { return m_CC2metal; }
    double get_CC2M2metal() const { return m_CC2M2metal; }
    double get_CC2M3metal() const { return m_CC2M3metal; }
    double get_CC3metal() const { return m_CC3metal; }
    double get_CC3M2metal() const { return m_CC3M2metal; }
    double get_CC3M3metal() const { return m_CC3M3metal; }

    double get_EnergyFactor() const { return m_EnergyFactor; }
    double get_SenseEnergyFactor() const { return m_SenseEnergyFactor; }

    double get_Woutdrvseln() const { return m_Woutdrvseln; }
    double get_Woutdrvselp() const { return m_Woutdrvselp; }
    double get_Woutdrvnandn() const { return m_Woutdrvnandn; }
    double get_Woutdrvnandp() const { return m_Woutdrvnandp; }
    double get_Woutdrvnorn() const { return m_Woutdrvnorn; }
    double get_Woutdrvnorp() const { return m_Woutdrvnorp; }
    double get_Woutdrivern() const { return m_Woutdrivern; }
    double get_Woutdriverp() const { return m_Woutdriverp; }

    double get_WsenseQ1to4() const { return m_WsenseQ1to4; }
    double get_Wbitmuxn() const { return m_Wbitmuxn; }
    double get_Wmemcellr() const { return m_Wmemcellr; }
    double get_Wmemcellw() const { return m_Wmemcellw; }
    double get_Wmemcella() const { return m_Wmemcella; }
    double get_Wmemcellbscale() const { return m_Wmemcellbscale; }
    
    double get_RegCellHeight() const { return m_RegCellHeight; }
    double get_RegCellWidth() const { return m_RegCellWidth; }
    double get_WordlineSpacing() const { return m_WordlineSpacing; }
    double get_BitlineSpacing() const { return m_BitlineSpacing; }
    double get_BitWidth() const { return m_BitWidth; }

    double get_Wdecinvn() const { return m_Wdecinvn; }
    double get_Wdecinvp() const { return m_Wdecinvp; }
    double get_Wdec3to8n() const { return m_Wdec3to8n; }
    double get_Wdec3to8p() const { return m_Wdec3to8p; }
    double get_WdecNORn() const { return m_WdecNORn; }
    double get_WdecNORp() const { return m_WdecNORp; }
    double get_Wdecdriven() const { return m_Wdecdriven; }
    double get_Wdecdrivep() const { return m_Wdecdrivep; }

    double get_CrsbarCellWidth() const { return m_CrsbarCellWidth; }
    double get_CrsbarCellHeight() const { return m_CrsbarCellHeight; }

    double get_Wdff() const { return m_Wdff; }

    double get_WireMinWidth() const { return m_WireMinWidth; }
    double get_WireMinSpacing() const { return m_WireMinSpacing; }
    double get_WireBarrierThickness() const { return m_WireBarrierThickness; }
    double get_WireMetalThickness() const { return m_WireMetalThickness; }
    double get_WireDielectricThickness() const { return m_WireDielectricThickness; }
    double get_WireDielectricConstant() const { return m_WireDielectricConstant; }
    double get_BufferDriveResistance() const { return m_BufferDriveResistance; }
    double get_BufferInputCapacitance() const { return m_BufferInputCapacitance; }
    double get_BufferNMOSOffCurrent() const { return m_BufferNMOSOffCurrent; }
    double get_BufferPMOSOffCurrent() const { return m_BufferPMOSOffCurrent; }
    double get_ClockCap() const { return m_ClockCap; }
    double get_Clockwire() const { return m_Clockwire; }
    double get_Reswire() const { return m_Reswire; }

    double get_NMOS_TAB(uint32_t idx) const { return m_NMOS_TAB[idx]; }
    double get_PMOS_TAB(uint32_t idx) const { return m_PMOS_TAB[idx]; }
    double get_NAND2_TAB(uint32_t idx) const { return m_NAND2_TAB[idx]; }
    double get_NOR2_TAB(uint32_t idx) const { return m_NOR2_TAB[idx]; }
    double get_DFF_TAB(uint32_t idx) const { return m_DFF_TAB[idx]; }
  
  private:
    void init();
    void init_tech_110_800();
    void init_tech_32_90();
    void init_tech_90();

  private:
    const OrionConfig* m_orion_cfg_ptr;
    unsigned int m_tech_node;
    TransistorType m_transistor_type;
    double m_vdd;
    double m_freq;
    double m_period;
    WireLayerType m_wire_layer_type;

    double m_af;
    uint32_t m_max_n;
    uint32_t m_max_subarrays;
    uint32_t m_max_spd;
    double m_vth_outdr_nor;
    double m_vth_comp_inv;
    uint32_t m_bit_out;
    uint32_t m_ruu_issue_width;
    double m_amp_idsat;
    double m_vs_inv;
    double m_gen_power_factor;
    double m_vth_nand_60x90;
    double m_fudge_factor;
    double m_vth_outdrive;
    double m_vth_muxdrv1;
    double m_vth_muxdrv2;
    double m_normalize_scale;
    double m_vth_muxdrv3;
    uint32_t m_address_bits;
    uint32_t m_ruu_size;
    double m_vth_nor_12x4x1;
    double m_vth_nor_12x4x2;
    double m_vth_outdr_inv;
    double m_vth_nor_12x4x3;
    double m_vth_eval_inv;
    double m_vth_nor_12x4x4;
    uint32_t m_res_ialu;
    double m_vth_outdr_nand;
    double m_vth_inv_100x60;

    double m_Cpdiffarea;
    double m_Cpdiffside;
    double m_Cpoverlap;
    double m_Cndiffarea;
    double m_Cndiffside;
    double m_Cnoverlap;
    double m_Cgatepass;
    double m_Cgate;
    double m_Cpdiffovlp;
    double m_Cndiffovlp;
    double m_Cpoxideovlp;
    double m_Cnoxideovlp;

    double m_Vbitpre;
    double m_Vbitsense;
    double m_EnergyFactor;
    double m_SenseEnergyFactor;
    double m_SenseEnergyFactor3;
    double m_SenseEnergyFactor2;

    double m_SCALE_T;
    double m_SCALE_M;
    double m_SCALE_S;
    double m_SCALE_W;
    double m_SCALE_H;
    double m_SCALE_BW;
    double m_SCALE_Crs;

    double m_LSCALE;
    double m_MSCALE;

    double m_BitWidth;
    double m_BitHeight;
    double m_BitlineSpacing;
    double m_WordlineSpacing;

    double m_RegCellWidth;
    double m_RegCellHeight;

    double m_Cout;

    double m_Cwordmetal;
    double m_Cbitmetal;

    double m_Cmetal;
    double m_CM2metal;
    double m_CM3metal;

    double m_CCmetal;
    double m_CCM2metal;
    double m_CCM3metal;

    double m_CC2metal;
    double m_CC2M2metal;
    double m_CC2M3metal;

    double m_CC3metal;
    double m_CC3M2metal;
    double m_CC3M3metal;

    double m_Clockwire;
    double m_Reswire;
    double m_invCap;
    double m_Resout;

    double m_Leff;
    double m_Lamda;

    double m_Cpolywire;

    double m_Rnchannelstatic;
    double m_Rpchannelstatic;

    double m_Rnchannelon;
    double m_Rpchannelon;

    double m_Rbitmetal;
    double m_Rwordmetal;

    double m_Vt;

    double m_Wdecdrivep;
    double m_Wdecdriven;
    double m_Wdec3to8n;
    double m_Wdec3to8p;
    double m_WdecNORn;
    double m_WdecNORp;
    double m_Wdecinvn;
    double m_Wdecinvp;
    double m_Wdff;

    double m_Wworddrivemax;
    double m_Wmemcella;
    double m_Wmemcellr;
    double m_Wmemcellw;
    double m_Wmemcellbscale;
    double m_Wbitpreequ;

    double m_Wbitmuxn;
    double m_WsenseQ1to4;
    double m_Wcompinvp1;
    double m_Wcompinvn1;
    double m_Wcompinvp2;
    double m_Wcompinvn2;
    double m_Wcompinvp3;
    double m_Wcompinvn3;
    double m_Wevalinvp;
    double m_Wevalinvn;

    double m_Wcompn;
    double m_Wcompp;
    double m_Wcomppreequ;
    double m_Wmuxdrv12n;
    double m_Wmuxdrv12p;
    double m_WmuxdrvNANDn;
    double m_WmuxdrvNANDp;
    double m_WmuxdrvNORn;
    double m_WmuxdrvNORp;
    double m_Wmuxdrv3n;
    double m_Wmuxdrv3p;
    double m_Woutdrvseln;
    double m_Woutdrvselp;
    double m_Woutdrvnandn;
    double m_Woutdrvnandp;
    double m_Woutdrvnorn;
    double m_Woutdrvnorp;
    double m_Woutdrivern;
    double m_Woutdriverp;
    double m_Wbusdrvn;
    double m_Wbusdrvp;

    double m_Wcompcellpd2;
    double m_Wcompdrivern;
    double m_Wcompdriverp;
    double m_Wcomparen2;
    double m_Wcomparen1;
    double m_Wmatchpchg;
    double m_Wmatchinvn;
    double m_Wmatchinvp;
    double m_Wmatchnandn;
    double m_Wmatchnandp;
    double m_Wmatchnorn;
    double m_Wmatchnorp;

    double m_WSelORn;
    double m_WSelORprequ;
    double m_WSelPn;
    double m_WSelPp;
    double m_WSelEnn;
    double m_WSelEnp;

    double m_Wsenseextdrv1p;
    double m_Wsenseextdrv1n;
    double m_Wsenseextdrv2p;
    double m_Wsenseextdrv2n;

    double m_CamCellHeight;/*derived from Cacti 5.3 */ 
    double m_CamCellWidth;/*derived from Cacti 5.3 */ 

    double m_MatchlineSpacing;
    double m_TaglineSpacing;

    double m_CrsbarCellHeight;
    double m_CrsbarCellWidth;

    double m_krise;
    double m_tsensedata;
    double m_tsensetag;
    double m_tfalldata;
    double m_tfalltag;

    double m_WireMinWidth;
    double m_WireMinSpacing;
    double m_WireMetalThickness;
    double m_WireBarrierThickness;
    double m_WireDielectricThickness;
    double m_WireDielectricConstant;

    double m_BufferDriveResistance;
    double m_BufferIntrinsicDelay;
    double m_BufferInputCapacitance;
    double m_BufferPMOSOffCurrent;
    double m_BufferNMOSOffCurrent;
    double m_ClockCap;

    double m_AreaNOR;
    double m_AreaINV;
    double m_AreaAND;
    double m_AreaDFF;
    double m_AreaMUX2;
    double m_AreaMUX3;
    double m_AreaMUX4;

    double m_NMOS_TAB[1];
    double m_PMOS_TAB[1];
    double m_NAND2_TAB[4];
    double m_NOR2_TAB[4];
    double m_DFF_TAB[1];
};

#endif
 
