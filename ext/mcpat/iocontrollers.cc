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
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "XML_Parse.h"
#include "basic_circuit.h"
#include "basic_components.h"
#include "const.h"
#include "io.h"
#include "iocontrollers.h"
#include "logic.h"
#include "parameter.h"

/*
SUN Niagara 2 I/O power analysis:
total signal bits: 711
Total FBDIMM bits: (14+10)*2*8= 384
PCIe bits:         (8 + 8)*2 = 32
10Gb NIC:          (4*2+4*2)*2 = 32
Debug I/Os:        168
Other I/Os:        711- 32-32 - 384 - 168 = 95

According to "Implementation of an 8-Core, 64-Thread, Power-Efficient SPARC Server on a Chip"
90% of I/Os are SerDers (the calucaltion is 384+64/(711-168)=83% about the same as the 90% reported in the paper)
--> around 80Pins are common I/Os.
Common I/Os consumes 71mW/Gb/s according to Cadence ChipEstimate @65nm
Niagara 2 I/O clock is 1/4 of core clock. --> 87pin (<--((711-168)*17%)) * 71mW/Gb/s *0.25*1.4Ghz = 2.17W

Total dynamic power of FBDIMM, NIC, PCIe = 84*0.132 + 84*0.049*0.132 = 11.14 - 2.17 = 8.98
Further, if assuming I/O logic power is about 50% of I/Os then Total energy of FBDIMM, NIC, PCIe = 11.14 - 2.17*1.5 = 7.89
 */

/*
 * A bug in Cadence ChipEstimator: After update the clock rate in the clock tab, a user
 * need to re-select the IP clock (the same clk) and then click Estimate. if not reselect
 * the new clock rate may not be propogate into the IPs.
 *
 */

NIUController::NIUController(ParseXML *XML_interface,InputParameter* interface_ip_)
:XML(XML_interface),
 interface_ip(*interface_ip_)
 {
          local_result = init_interface(&interface_ip);

          double frontend_area, phy_area, mac_area, SerDer_area;
      double frontend_dyn, mac_dyn, SerDer_dyn;
      double frontend_gates, mac_gates, SerDer_gates;
          double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
          double NMOS_sizing, PMOS_sizing;

          set_niu_param();

          if (niup.type == 0) //high performance NIU
          {
                  //Area estimation based on average of die photo from Niagara 2 and Cadence ChipEstimate using 65nm.
                  mac_area = (1.53 + 0.3)/2 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //Area estimation based on average of die photo from Niagara 2, ISSCC "An 800mW 10Gb Ethernet Transceiver in 0.13μm CMOS"
                  //and"A 1.2-V-Only 900-mW 10 Gb Ethernet Transceiver and XAUI Interface With Robust VCO Tuning Technique" Frontend is PCS
                  frontend_area = (9.8 + (6 + 18)*65/130*65/130)/3 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //Area estimation based on average of die photo from Niagara 2 and Cadence ChipEstimate hard IP @65nm.
                  //SerDer is very hard to scale
                  SerDer_area = (1.39 + 0.36) * (interface_ip.F_sz_um/0.065);//* (interface_ip.F_sz_um/0.065);
                  phy_area = frontend_area + SerDer_area;
                  //total area
                  area.set_area((mac_area + frontend_area + SerDer_area)*1e6);
                  //Power
                  //Cadence ChipEstimate using 65nm (mac, front_end are all energy. E=P*T = P/F = 1.37/1Ghz = 1.37e-9);
                  mac_dyn      = 2.19e-9*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);//niup.clockRate; //2.19W@1GHz fully active according to Cadence ChipEstimate @65nm
                  //Cadence ChipEstimate using 65nm soft IP;
                  frontend_dyn = 0.27e-9*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);//niup.clockRate;
                  //according to "A 100mW 9.6Gb/s Transceiver in 90nm CMOS..." ISSCC 2006
                  //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
                  SerDer_dyn   = 0.01*10*sqrt(interface_ip.F_sz_um/0.09)*g_tp.peri_global.Vdd/1.2*g_tp.peri_global.Vdd/1.2;
                  SerDer_dyn   /= niup.clockRate;//covert to energy per clock cycle of whole NIU

                  //Cadence ChipEstimate using 65nm
                  mac_gates       = 111700;
                  frontend_gates  = 320000;
                  SerDer_gates    = 200000;
                  NMOS_sizing 	  = 5*g_tp.min_w_nmos_;
                  PMOS_sizing	  = 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r;


          }
          else
          {//Low power implementations are mostly from Cadence ChipEstimator; Ignore the multiple IP effect
                  // ---When there are multiple IP (same kind or not) selected, Cadence ChipEstimator results are not
                  // a simple summation of all IPs. Ignore this effect
                  mac_area      = 0.24 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  frontend_area = 0.1  * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);//Frontend is the PCS layer
                  SerDer_area   = 0.35 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //Compare 130um implementation in "A 1.2-V-Only 900-mW 10 Gb Ethernet Transceiver and XAUI Interface With Robust VCO Tuning Technique"
                  //and the ChipEstimator XAUI PHY hard IP, confirm that even PHY can scale perfectly with the technology
                  //total area
                  area.set_area((mac_area + frontend_area + SerDer_area)*1e6);
                  //Power
                  //Cadence ChipEstimate using 65nm (mac, front_end are all energy. E=P*T = P/F = 1.37/1Ghz = 1.37e-9);
                  mac_dyn      = 1.257e-9*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);//niup.clockRate; //2.19W@1GHz fully active according to Cadence ChipEstimate @65nm
                  //Cadence ChipEstimate using 65nm soft IP;
                  frontend_dyn = 0.6e-9*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);//niup.clockRate;
                  //SerDer_dyn is power not energy, scaling from 216mw/10Gb/s @130nm
                  SerDer_dyn   = 0.0216*10*(interface_ip.F_sz_um/0.13)*g_tp.peri_global.Vdd/1.2*g_tp.peri_global.Vdd/1.2;
                  SerDer_dyn   /= niup.clockRate;//covert to energy per clock cycle of whole NIU

                  mac_gates       = 111700;
                  frontend_gates  = 52000;
                  SerDer_gates    = 199260;

                  NMOS_sizing 	  = g_tp.min_w_nmos_;
                  PMOS_sizing	  = g_tp.min_w_nmos_*pmos_to_nmos_sizing_r;

          }

          power_t.readOp.dynamic = mac_dyn + frontend_dyn + SerDer_dyn;
          power_t.readOp.leakage = (mac_gates + frontend_gates + frontend_gates)*cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand)*g_tp.peri_global.Vdd;//unit W
          double long_channel_device_reduction = longer_channel_device_reduction(Uncore_device);
          power_t.readOp.longer_channel_leakage = power_t.readOp.leakage * long_channel_device_reduction;
          power_t.readOp.gate_leakage = (mac_gates + frontend_gates + frontend_gates)*cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand)*g_tp.peri_global.Vdd;//unit W
 }

void NIUController::computeEnergy(bool is_tdp)
{
        if (is_tdp)
    {


                power	= power_t;
        power.readOp.dynamic *= niup.duty_cycle;

    }
    else
    {
        rt_power = power_t;
        rt_power.readOp.dynamic *= niup.perc_load;
    }
}

void NIUController::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;

        if (is_tdp)
        {
                cout << "NIU:" << endl;
                cout << indent_str<< "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic*niup.clockRate  << " W" << endl;
                cout << indent_str<< "Subthreshold Leakage = "
                        << (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
                //cout << indent_str<< "Subthreshold Leakage = " << power.readOp.longer_channel_leakage <<" W" << endl;
                cout << indent_str<< "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
                cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic*niup.clockRate << " W" << endl;
                cout<<endl;
        }
        else
        {

        }

}

void NIUController::set_niu_param()
{
          niup.clockRate       = XML->sys.niu.clockrate;
          niup.clockRate       *= 1e6;
          niup.num_units       = XML->sys.niu.number_units;
          niup.duty_cycle      = XML->sys.niu.duty_cycle;
          niup.perc_load       = XML->sys.niu.total_load_perc;
          niup.type            = XML->sys.niu.type;
//	  niup.executionTime   = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);
}

PCIeController::PCIeController(ParseXML *XML_interface,InputParameter* interface_ip_)
:XML(XML_interface),
 interface_ip(*interface_ip_)
 {
          local_result = init_interface(&interface_ip);
          double frontend_area, phy_area, ctrl_area, SerDer_area;
      double ctrl_dyn, frontend_dyn, SerDer_dyn;
      double ctrl_gates,frontend_gates, SerDer_gates;
          double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
          double NMOS_sizing, PMOS_sizing;

          /* Assuming PCIe is bit-slice based architecture
           * This is the reason for /8 in both area and power calculation
           * to get per lane numbers
           */

          set_pcie_param();
          if (pciep.type == 0) //high performance NIU
          {
                  //Area estimation based on average of die photo from Niagara 2 and Cadence ChipEstimate @ 65nm.
                  ctrl_area = (5.2 + 0.5)/2 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //Area estimation based on average of die photo from Niagara 2, and Cadence ChipEstimate @ 65nm.
                  frontend_area = (5.2 + 0.1)/2 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //Area estimation based on average of die photo from Niagara 2 and Cadence ChipEstimate hard IP @65nm.
                  //SerDer is very hard to scale
                  SerDer_area = (3.03 + 0.36) * (interface_ip.F_sz_um/0.065);//* (interface_ip.F_sz_um/0.065);
                  phy_area = frontend_area + SerDer_area;
                  //total area
                  //Power
                  //Cadence ChipEstimate using 65nm the controller includes everything: the PHY, the data link and transaction layer
                  ctrl_dyn      = 3.75e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);
                  //	  //Cadence ChipEstimate using 65nm soft IP;
                  //	  frontend_dyn = 0.27e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);
                  //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
                  SerDer_dyn   = 0.01*4*(interface_ip.F_sz_um/0.09)*g_tp.peri_global.Vdd/1.2*g_tp.peri_global.Vdd/1.2;//PCIe 2.0 max per lane speed is 4Gb/s
                  SerDer_dyn   /= pciep.clockRate;//covert to energy per clock cycle

                  //power_t.readOp.dynamic = (ctrl_dyn)*pciep.num_channels;
                  //Cadence ChipEstimate using 65nm
                  ctrl_gates       = 900000/8*pciep.num_channels;
                  //	  frontend_gates   = 120000/8;
                  //	  SerDer_gates     = 200000/8;
                  NMOS_sizing 	  = 5*g_tp.min_w_nmos_;
                  PMOS_sizing	  = 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r;
          }
          else
          {
                  ctrl_area = 0.412 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //Area estimation based on average of die photo from Niagara 2, and Cadence ChipEstimate @ 65nm.
          SerDer_area = 0.36 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //total area
                  //Power
                  //Cadence ChipEstimate using 65nm the controller includes everything: the PHY, the data link and transaction layer
                  ctrl_dyn      = 2.21e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);
                  //	  //Cadence ChipEstimate using 65nm soft IP;
                  //	  frontend_dyn = 0.27e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);
                  //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
                  SerDer_dyn   = 0.01*4*(interface_ip.F_sz_um/0.09)*g_tp.peri_global.Vdd/1.2*g_tp.peri_global.Vdd/1.2;//PCIe 2.0 max per lane speed is 4Gb/s
                  SerDer_dyn   /= pciep.clockRate;//covert to energy per clock cycle

                  //Cadence ChipEstimate using 65nm
                  ctrl_gates       = 200000/8*pciep.num_channels;
                  //	  frontend_gates   = 120000/8;
                  SerDer_gates     = 200000/8*pciep.num_channels;
                  NMOS_sizing 	  = g_tp.min_w_nmos_;
                  PMOS_sizing	  = g_tp.min_w_nmos_*pmos_to_nmos_sizing_r;

          }
          area.set_area(((ctrl_area + (pciep.withPHY? SerDer_area:0))/8*pciep.num_channels)*1e6);
          power_t.readOp.dynamic = (ctrl_dyn + (pciep.withPHY? SerDer_dyn:0))*pciep.num_channels;
          power_t.readOp.leakage = (ctrl_gates + (pciep.withPHY? SerDer_gates:0))*cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand)*g_tp.peri_global.Vdd;//unit W
          double long_channel_device_reduction = longer_channel_device_reduction(Uncore_device);
          power_t.readOp.longer_channel_leakage = power_t.readOp.leakage * long_channel_device_reduction;
          power_t.readOp.gate_leakage = (ctrl_gates + (pciep.withPHY? SerDer_gates:0))*cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand)*g_tp.peri_global.Vdd;//unit W
 }

void PCIeController::computeEnergy(bool is_tdp)
{
        if (is_tdp)
    {


                power	= power_t;
        power.readOp.dynamic *= pciep.duty_cycle;

    }
    else
    {
        rt_power = power_t;
        rt_power.readOp.dynamic *= pciep.perc_load;
    }
}

void PCIeController::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;

        if (is_tdp)
        {
                cout << "PCIe:" << endl;
                cout << indent_str<< "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic*pciep.clockRate  << " W" << endl;
                cout << indent_str<< "Subthreshold Leakage = "
                        << (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
                //cout << indent_str<< "Subthreshold Leakage = " << power.readOp.longer_channel_leakage <<" W" << endl;
                cout << indent_str<< "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
                cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic*pciep.clockRate << " W" << endl;
                cout<<endl;
        }
        else
        {

        }

}

void PCIeController::set_pcie_param()
{
          pciep.clockRate       = XML->sys.pcie.clockrate;
          pciep.clockRate       *= 1e6;
          pciep.num_units       = XML->sys.pcie.number_units;
          pciep.num_channels    = XML->sys.pcie.num_channels;
          pciep.duty_cycle      = XML->sys.pcie.duty_cycle;
          pciep.perc_load       = XML->sys.pcie.total_load_perc;
          pciep.type            = XML->sys.pcie.type;
          pciep.withPHY         = XML->sys.pcie.withPHY;
//	  pciep.executionTime   = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);

}

FlashController::FlashController(ParseXML *XML_interface,InputParameter* interface_ip_)
:XML(XML_interface),
 interface_ip(*interface_ip_)
 {
          local_result = init_interface(&interface_ip);
          double frontend_area, phy_area, ctrl_area, SerDer_area;
      double ctrl_dyn, frontend_dyn, SerDer_dyn;
      double ctrl_gates,frontend_gates, SerDer_gates;
          double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
          double NMOS_sizing, PMOS_sizing;

          /* Assuming PCIe is bit-slice based architecture
           * This is the reason for /8 in both area and power calculation
           * to get per lane numbers
           */

          set_fc_param();
          if (fcp.type == 0) //high performance NIU
          {
                  cout<<"Current McPAT does not support high performance flash contorller since even low power designs are enough for maintain throughput"<<endl;
                  exit(0);
                  NMOS_sizing 	  = 5*g_tp.min_w_nmos_;
                  PMOS_sizing	  = 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r;
          }
          else
          {
                  ctrl_area   = 0.243 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
                  //Area estimation based on Cadence ChipEstimate @ 65nm: NANDFLASH-CTRL from CAST
          SerDer_area = 0.36/8 * (interface_ip.F_sz_um/0.065)* (interface_ip.F_sz_um/0.065);
          //based On PCIe PHY TSMC65GP from Cadence ChipEstimate @ 65nm, it support 8x lanes with each lane
          //speed up to 250MB/s (PCIe1.1x) This is already saturate the 200MB/s of the flash controller core above.
                  ctrl_gates      = 129267;
                  SerDer_gates    = 200000/8;
                  NMOS_sizing 	  = g_tp.min_w_nmos_;
                  PMOS_sizing	  = g_tp.min_w_nmos_*pmos_to_nmos_sizing_r;

                  //Power
                  //Cadence ChipEstimate using 65nm the controller 125mW for every 200MB/s This is power not energy!
                  ctrl_dyn      = 0.125*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);
                  //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
                  SerDer_dyn   = 0.01*1.6*(interface_ip.F_sz_um/0.09)*g_tp.peri_global.Vdd/1.2*g_tp.peri_global.Vdd/1.2;
                  //max  Per controller speed is 1.6Gb/s (200MB/s)
          }
          double number_channel = 1+(fcp.num_channels-1)*0.2;
          area.set_area((ctrl_area + (fcp.withPHY? SerDer_area:0))*1e6*number_channel);
          power_t.readOp.dynamic = (ctrl_dyn + (fcp.withPHY? SerDer_dyn:0))*number_channel;
          power_t.readOp.leakage = ((ctrl_gates + (fcp.withPHY? SerDer_gates:0))*number_channel)*cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand)*g_tp.peri_global.Vdd;//unit W
          double long_channel_device_reduction = longer_channel_device_reduction(Uncore_device);
          power_t.readOp.longer_channel_leakage = power_t.readOp.leakage * long_channel_device_reduction;
          power_t.readOp.gate_leakage = ((ctrl_gates + (fcp.withPHY? SerDer_gates:0))*number_channel)*cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand)*g_tp.peri_global.Vdd;//unit W
 }

void FlashController::computeEnergy(bool is_tdp)
{
        if (is_tdp)
    {


                power	= power_t;
        power.readOp.dynamic *= fcp.duty_cycle;

    }
    else
    {
        rt_power = power_t;
        rt_power.readOp.dynamic *= fcp.perc_load;
    }
}

void FlashController::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
        string indent_str(indent, ' ');
        string indent_str_next(indent+2, ' ');
        bool long_channel = XML->sys.longer_channel_device;

        if (is_tdp)
        {
                cout << "Flash Controller:" << endl;
                cout << indent_str<< "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
                cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic << " W" << endl;//no multiply of clock since this is power already
                cout << indent_str<< "Subthreshold Leakage = "
                        << (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
                //cout << indent_str<< "Subthreshold Leakage = " << power.readOp.longer_channel_leakage <<" W" << endl;
                cout << indent_str<< "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
                cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic << " W" << endl;
                cout<<endl;
        }
        else
        {

        }

}

void FlashController::set_fc_param()
{
//	  fcp.clockRate       = XML->sys.flashc.mc_clock;
//	  fcp.clockRate       *= 1e6;
          fcp.peakDataTransferRate = XML->sys.flashc.peak_transfer_rate;
          fcp.num_channels    = ceil(fcp.peakDataTransferRate/200);
          fcp.num_mcs         = XML->sys.flashc.number_mcs;
          fcp.duty_cycle      = XML->sys.flashc.duty_cycle;
          fcp.perc_load       = XML->sys.flashc.total_load_perc;
          fcp.type            = XML->sys.flashc.type;
          fcp.withPHY         = XML->sys.flashc.withPHY;
//	  flashcp.executionTime   = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);

}
