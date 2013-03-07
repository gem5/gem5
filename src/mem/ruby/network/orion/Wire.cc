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

#include "base/misc.hh"
#include "mem/ruby/network/orion/TechParameter.hh"
#include "mem/ruby/network/orion/Wire.hh"

using namespace std;

Wire::Wire(
  const string& wire_spacing_model_str_,
  const string& buf_scheme_str_,
  bool is_shielding_,
  const TechParameter* tech_param_ptr_
)
{
    set_width_spacing_model(wire_spacing_model_str_);
    set_buf_scheme(buf_scheme_str_);

    m_is_shielding = is_shielding_;

    m_tech_param_ptr = tech_param_ptr_;

    if (m_tech_param_ptr->get_tech_node() > 90)
    {
        cerr << "ERROR: Wire model only support tech node <= 90" << endl;
        exit(1);
    }

    init();
}

Wire::~Wire()
{}

// OPTIMUM K and H CALCULATION
// Computes the optimum number and size of repeaters for the link   
void Wire::calc_opt_buffering(
        int* k_,
        double* h_,
        double len_
        ) const
{
    double BufferDriveResistance = m_tech_param_ptr->get_BufferDriveResistance();
    double BufferInputCapacitance = m_tech_param_ptr->get_BufferInputCapacitance();
    switch(m_buf_scheme)
    {
        case MIN_DELAY:
            {
                if (m_is_shielding)
                {
                    double r = m_res_unit_len*len_;
                    double c_g = 2*m_gnd_cap_unit_len*len_;
                    double c_c = 2*m_couple_cap_unit_len*len_;
                    *k_ = (int) sqrt(((0.4*r*c_g)+(0.57*r*c_c))/
                            (0.7*BufferDriveResistance*BufferInputCapacitance)); //k is the number of buffers to be inserted
                    *h_ = sqrt(((0.7*BufferDriveResistance*c_g)+
                                (1.4*1.5*BufferDriveResistance*c_c))/(0.7*r*BufferInputCapacitance)); //the size of the buffers to be inserted
                    break;
                }
                else
                {
                    double r = m_res_unit_len*len_;
                    double c_g = 2*m_gnd_cap_unit_len*len_;
                    double c_c = 2*m_couple_cap_unit_len*len_;
                    *k_ = (int) sqrt(((0.4*r*c_g)+(1.51*r*c_c))/
                            (0.7*BufferDriveResistance*BufferInputCapacitance));
                    *h_ = sqrt(((0.7*BufferDriveResistance*c_g)+
                                (1.4*2.2*BufferDriveResistance*c_c))/(0.7*r*BufferInputCapacitance));
                    break;
                }
            }
        case STAGGERED:
            {
                double r = m_res_unit_len*len_;
                double c_g = 2*m_gnd_cap_unit_len*len_;
                double c_c = 2*m_couple_cap_unit_len*len_;

                *k_ = (int) sqrt(((0.4*r*c_g)+(0.57*r*c_c))/
                        (0.7*BufferDriveResistance*BufferInputCapacitance));
                *h_ = sqrt(((0.7*BufferDriveResistance*c_g)+
                            (1.4*1.5*BufferDriveResistance*c_c))/(0.7*r*BufferInputCapacitance));
                break;
            }
        default:
            {
                // make gcc happy although all the cases of the enum
                // are already covered
                *k_ = 0;
                *h_ = 0;
                break;
            }
    }
    return;
}

double Wire::calc_dynamic_energy(double len_) const
{

    double c_g = 2*m_gnd_cap_unit_len*len_;
    double c_c = 2*m_couple_cap_unit_len*len_;
    double cap_wire = c_g + c_c;

    int k;
    double h;

    calc_opt_buffering(&k, &h, len_);

    double BufferInputCapacitance = m_tech_param_ptr->get_BufferInputCapacitance();
    double cap_buf = ((double)k)*BufferInputCapacitance*h;

    double e_factor = m_tech_param_ptr->get_EnergyFactor();
    return ((cap_wire+cap_buf)*e_factor);
}

double Wire::calc_static_power(double len_) const
{
    int k;
    double h;
    calc_opt_buffering(&k, &h, len_);

    double BufferNMOSOffCurrent = m_tech_param_ptr->get_BufferNMOSOffCurrent();
    double BufferPMOSOffCurrent = m_tech_param_ptr->get_BufferPMOSOffCurrent();
    double i_static_nmos = BufferNMOSOffCurrent*h*k;
    double i_static_pmos = BufferPMOSOffCurrent*h*k;

    double vdd = m_tech_param_ptr->get_vdd();
    return (vdd*(i_static_pmos+i_static_nmos)/2);
}

void Wire::init()
{
    m_res_unit_len = calc_res_unit_len();
    m_gnd_cap_unit_len = calc_gnd_cap_unit_len();
    m_couple_cap_unit_len = calc_couple_cap_unit_len();
    return;
}

void Wire::set_width_spacing_model(
        const string& wire_spacing_model_str_
        )
{
    if (wire_spacing_model_str_ == string("SWIDTH_SSPACE"))
    {
        m_width_spacing_model = SWIDTH_SSPACE;
    }
    else if (wire_spacing_model_str_ == string("SWIDTH_DSPACE"))
    {
        m_width_spacing_model = SWIDTH_DSPACE;
    }
    else if (wire_spacing_model_str_ == string("DWIDTH_SSPACE"))
    {
        m_width_spacing_model = DWIDTH_SSPACE;
    }
    else if (wire_spacing_model_str_ == string("DWIDTH_DSPACE"))
    {
        m_width_spacing_model = DWIDTH_DSPACE;
    }
    else
    {
        cerr << "ERROR: Invalid wire width/spacing model" << endl;
        exit(1);
    }
    return;
}

void Wire::set_buf_scheme(
        const string& buf_scheme_str_
        )
{
    if (buf_scheme_str_ == string("MIN_DELAY"))
    {
        m_buf_scheme = MIN_DELAY;
    }
    else if (buf_scheme_str_ == string("STAGGERED"))
    {
        m_buf_scheme = STAGGERED;
    }
    else
    {
        cerr << "ERROR: Invalid wire buf scheme" << endl;
        exit(1);
    }
    return;
}

// The following function computes the wire resistance considering
// width-spacing combination and a width-dependent resistivity model
double Wire::calc_res_unit_len()
{
    double r = -1.0;
    double rho;
    // r, rho is in ohm.m

    double WireMinWidth = m_tech_param_ptr->get_WireMinWidth();
    double WireBarrierThickness = m_tech_param_ptr->get_WireBarrierThickness();
    double WireMetalThickness = m_tech_param_ptr->get_WireMetalThickness();

    switch(m_width_spacing_model)
    {
        case SWIDTH_SSPACE:
        case SWIDTH_DSPACE:
            rho = 2.202e-8 + (1.030e-15  / (WireMinWidth - 2*WireBarrierThickness));
            r = ((rho) / ((WireMinWidth - 2*WireBarrierThickness) *
                        (WireMetalThickness - WireBarrierThickness)));
            break;
        case DWIDTH_SSPACE:
        case DWIDTH_DSPACE:
            rho = 2.202e-8 + (1.030e-15  / (2*WireMinWidth - 2*WireBarrierThickness));
            r = ((rho) / ((2*WireMinWidth - 2*WireBarrierThickness) *
                        (WireMetalThickness - WireBarrierThickness)));
            break;
        default:
            warn("Orion: Width spacing model not found: %s\n", m_width_spacing_model);
            r = 1.0;
    }
    return r;
}

// COMPUTE WIRE CAPACITANCE using PTM models
double Wire::calc_gnd_cap_unit_len()
{
    // c_g is in F
    double c_g = -1.0;

    double WireMinWidth = m_tech_param_ptr->get_WireMinWidth();
    double WireMinSpacing = m_tech_param_ptr->get_WireMinSpacing();
    double WireMetalThickness = m_tech_param_ptr->get_WireMetalThickness();
    double WireDielectricThickness = m_tech_param_ptr->get_WireDielectricThickness();
    double WireDielectricConstant = m_tech_param_ptr->get_WireDielectricConstant();

    switch(m_width_spacing_model)
    {
        case SWIDTH_SSPACE:
            {
                double A = (WireMinWidth/WireDielectricThickness);
                double B = 2.04*pow((WireMinSpacing/(WireMinSpacing +
                                0.54*WireDielectricThickness)), 1.77);
                double C = pow((WireMetalThickness/(WireMetalThickness +
                                4.53*WireDielectricThickness)), 0.07);
                c_g = WireDielectricConstant*8.85e-12*(A+(B*C)); 
                break;
            }
        case SWIDTH_DSPACE:
            {
                double minSpacingNew = 2*WireMinSpacing + WireMinWidth;
                double A = (WireMinWidth/WireDielectricThickness);
                double B = 2.04*pow((minSpacingNew/(minSpacingNew +
                                0.54*WireDielectricThickness)), 1.77);
                double C = pow((WireMetalThickness/(WireMetalThickness +
                                4.53*WireDielectricThickness)), 0.07);
                c_g = WireDielectricConstant*8.85e-12*(A+(B*C));
                break;
            }
        case DWIDTH_SSPACE:
            {
                double minWidthNew = 2*WireMinWidth;
                double A = (minWidthNew/WireDielectricThickness);
                double B = 2.04*pow((WireMinSpacing/(WireMinSpacing +
                                0.54*WireDielectricThickness)), 1.77);
                double C = pow((WireMetalThickness/(WireMetalThickness +
                                4.53*WireDielectricThickness)), 0.07);
                c_g = WireDielectricConstant*8.85e-12*(A+(B*C)); 
                break;
            }
        case DWIDTH_DSPACE:
            {
                double minWidthNew = 2*WireMinWidth;
                double minSpacingNew = 2*WireMinSpacing;
                double A = (minWidthNew/WireDielectricThickness);
                double B = 2.04*pow((minSpacingNew/(minSpacingNew+
                                0.54*WireDielectricThickness)), 1.77);
                double C = pow((WireMetalThickness/(WireMetalThickness +
                                4.53*WireDielectricThickness)), 0.07);
                c_g = WireDielectricConstant*8.85e-12*(A+(B*C)); 
                break;
            }
        default:
            warn("Orion: Width spacing model not found: %s\n", m_width_spacing_model);
            c_g = 1.0;
    }

    return c_g;
}

// Computes the coupling capacitance considering cross-talk
double Wire::calc_couple_cap_unit_len()
{
    //c_c is in F
    double c_c = -1.0;

    double WireMinWidth = m_tech_param_ptr->get_WireMinWidth();
    double WireMinSpacing = m_tech_param_ptr->get_WireMinSpacing();
    double WireMetalThickness = m_tech_param_ptr->get_WireMetalThickness();
    double WireDielectricThickness = m_tech_param_ptr->get_WireDielectricThickness();
    double WireDielectricConstant = m_tech_param_ptr->get_WireDielectricConstant();

    switch(m_width_spacing_model)
    {
        case SWIDTH_SSPACE:
            {
                double A = 1.14*(WireMetalThickness/WireMinSpacing) *
                    exp(-4*WireMinSpacing/(WireMinSpacing + 8.01*WireDielectricThickness));
                double B = 2.37*pow((WireMinWidth/(WireMinWidth + 0.31*WireMinSpacing)), 0.28);
                double C = pow((WireDielectricThickness/(WireDielectricThickness +
                                8.96*WireMinSpacing)), 0.76) *
                    exp(-2*WireMinSpacing/(WireMinSpacing + 6*WireDielectricThickness));
                c_c = WireDielectricConstant*8.85e-12*(A + (B*C));
                break;
            }
        case SWIDTH_DSPACE:
            {
                double minSpacingNew = 2*WireMinSpacing + WireMinWidth;
                double A = 1.14*(WireMetalThickness/minSpacingNew) *
                    exp(-4*minSpacingNew/(minSpacingNew + 8.01*WireDielectricThickness));
                double B = 2.37*pow((WireMinWidth/(WireMinWidth + 0.31*minSpacingNew)), 0.28);
                double C = pow((WireDielectricThickness/(WireDielectricThickness +
                                8.96*minSpacingNew)), 0.76) *
                    exp(-2*minSpacingNew/(minSpacingNew + 6*WireDielectricThickness));
                c_c = WireDielectricConstant*8.85e-12*(A + (B*C));
                break;
            }
        case DWIDTH_SSPACE:
            {
                double minWidthNew = 2*WireMinWidth;
                double A = 1.14*(WireMetalThickness/WireMinSpacing) *
                    exp(-4*WireMinSpacing/(WireMinSpacing + 8.01*WireDielectricThickness));
                double B = 2.37*pow((2*minWidthNew/(2*minWidthNew + 0.31*WireMinSpacing)), 0.28);
                double C = pow((WireDielectricThickness/(WireDielectricThickness +
                                8.96*WireMinSpacing)), 0.76) *
                    exp(-2*WireMinSpacing/(WireMinSpacing + 6*WireDielectricThickness));
                c_c = WireDielectricConstant*8.85e-12*(A + (B*C));
                break;
            }
        case DWIDTH_DSPACE:
            {
                double minWidthNew = 2*WireMinWidth;
                double minSpacingNew = 2*WireMinSpacing;
                double A = 1.14*(WireMetalThickness/minSpacingNew) *
                    exp(-4*minSpacingNew/(minSpacingNew + 8.01*WireDielectricThickness));
                double B = 2.37*pow((minWidthNew/(minWidthNew + 0.31*minSpacingNew)), 0.28);
                double C = pow((WireDielectricThickness/(WireDielectricThickness +
                                8.96*minSpacingNew)), 0.76) *
                    exp(-2*minSpacingNew/(minSpacingNew + 6*WireDielectricThickness));
                c_c = WireDielectricConstant*8.85e-12*(A + (B*C));
                break;
            }
        default:
            warn("Orion: Width spacing model not found: %s\n", m_width_spacing_model);
            c_c = 1.0;
    }

    return c_c;
}

