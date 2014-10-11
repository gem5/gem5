/* Copyright (c) 2012 Massachusetts Institute of Technology
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "model/std_cells/StdCellLib.h"

#include <cmath>

#include "model/std_cells/StdCell.h"
#include "model/std_cells/INV.h"
#include "model/ModelGen.h"

namespace DSENT
{
    using std::pow;

    StdCellLib::StdCellLib(TechModel* tech_model_)
        : m_tech_model_(tech_model_)
    {
        m_std_cell_cache_ = new Map<double>();
        ASSERT((m_tech_model_ != NULL), "[Error] StdCellLib -> tech_model is NULL");
        createLib();
    }

    StdCellLib::~StdCellLib()
    {
        delete m_std_cell_cache_;
    }

    const TechModel* StdCellLib::getTechModel() const
    {
        return m_tech_model_;
    }

    StdCell* StdCellLib::createStdCell(const String& std_cell_name_, const String& instance_name_) const
    {
        // Create the standard cell
        StdCell* created_cell = ModelGen::createStdCell(std_cell_name_, instance_name_, getTechModel());
        // Grab the variants of each standard cell
        String driving_strength_str = getTechModel()->get("StdCell->AvailableSizes");
        // Set library properties for the standard cell
        created_cell->setPToNRatio(getPToNRatio());
        created_cell->setActiveHeight(getActiveHeight());
        created_cell->setTotalHeight(getTotalHeight());
        created_cell->setAvailableDrivingStrengths(driving_strength_str);
        return created_cell;
    }

    // Get PMOS to NMOS ratio
    double StdCellLib::getPToNRatio() const
    {
        return m_p_to_n_ratio_;
    }
    
    void StdCellLib::setPToNRatio(double p_to_n_ratio_)
    {
        m_p_to_n_ratio_ = p_to_n_ratio_;
    }
    
    // Get height of the standard cell taken by active transistors
    double StdCellLib::getActiveHeight() const
    {
        return m_active_height_;
    }
    
    void StdCellLib::setActiveHeight(double active_height_)
    {
        m_active_height_ = active_height_;
    }
    
    // Get total height of the standard cell including overheads
    double StdCellLib::getTotalHeight() const
    {
        return m_total_height_;
    }

    void StdCellLib::setTotalHeight(double total_height_)
    {
        m_total_height_ = total_height_;
    }
    
    void StdCellLib::createLib()
    {
        Log::printLine("Standard cell library creation for tech model " + getTechModel()->get("Name"));
        
        // Get technology parameters
        double nmos_eff_res = getTechModel()->get("Nmos->EffResWidth");
        double pmos_eff_res = getTechModel()->get("Pmos->EffResWidth");
        double gate_min_width = getTechModel()->get("Gate->MinWidth");
        
        // Create standard cell common parameters
        double pn_ratio = pmos_eff_res / nmos_eff_res;
        double nmos_unit_width = gate_min_width;
        double pmos_unit_width = gate_min_width * pn_ratio;
        
        // Derive the height of each cell in the standard cell library, as well as the max Nmos and Pmos widths
        double std_cell_total_height = getTechModel()->get("StdCell->Tracks").toDouble() * 
            (getTechModel()->get("Wire->Metal1->MinWidth").toDouble() + getTechModel()->get("Wire->Metal1->MinSpacing").toDouble());
        double std_cell_active_height = std_cell_total_height / getTechModel()->get("StdCell->HeightOverheadFactor").toDouble();

        Log::printLine("Standard cell P-to-N ratio (Beta) = " + (String) pn_ratio);
        Log::printLine("Standard cell NMOS unit width = " + (String) nmos_unit_width);
        Log::printLine("Standard cell PMOS unit width = " + (String) pmos_unit_width);
        Log::printLine("Standard cell active height = " + (String) std_cell_active_height);
        Log::printLine("Standard cell total height = " + (String) std_cell_total_height);
        
        setPToNRatio(pn_ratio);
        setActiveHeight(std_cell_active_height);
        setTotalHeight(std_cell_total_height);
        
        const vector<String>& cell_sizes = getTechModel()->get("StdCell->AvailableSizes").split("[,]");
        // Create cached standard cells
        for (unsigned int i = 0; i < cell_sizes.size(); ++i)
        {        
            StdCell* inv = createStdCell("INV", "CachedINV");           
            inv->cacheStdCell(this, cell_sizes[i].toDouble());
            delete inv;

            StdCell* nand2 = createStdCell("NAND2", "CachedNAND2");           
            nand2->cacheStdCell(this, cell_sizes[i].toDouble());
            delete nand2;

            StdCell* nor2 = createStdCell("NOR2", "CachedNOR2");           
            nor2->cacheStdCell(this, cell_sizes[i].toDouble());
            delete nor2;

            StdCell* mux2 = createStdCell("MUX2", "CachedMUX2");           
            mux2->cacheStdCell(this, cell_sizes[i].toDouble());
            delete mux2;

            StdCell* xor2 = createStdCell("XOR2", "CachedXOR2");           
            xor2->cacheStdCell(this, cell_sizes[i].toDouble());
            delete xor2;
            
            StdCell* addf = createStdCell("ADDF", "CachedADDF");           
            addf->cacheStdCell(this, cell_sizes[i].toDouble());
            delete addf;

            StdCell* dffq = createStdCell("DFFQ", "CachedDFFQ");           
            dffq->cacheStdCell(this, cell_sizes[i].toDouble());
            delete dffq;

            StdCell* latq = createStdCell("LATQ", "CachedLATQ");           
            latq->cacheStdCell(this, cell_sizes[i].toDouble());
            delete latq;

            StdCell* or2 = createStdCell("OR2", "CachedOR2");           
            or2->cacheStdCell(this, cell_sizes[i].toDouble());
            delete or2;

            StdCell* and2 = createStdCell("AND2", "CachedAND2");           
            and2->cacheStdCell(this, cell_sizes[i].toDouble());
            delete and2;
        }

        Log::printLine("Standard cell library creation - End");
        return;
    }

    StdCellLib* StdCellLib::clone() const
    {
        StdCellLib* new_lib = new StdCellLib(m_tech_model_);
        return new_lib;
    }

    const String StdCellLib::genDrivingStrengthString(const vector<double>& driving_strength_) const
    {
        String ret_str = "[";
        for(int i = 0; i < (int)driving_strength_.size() - 1; ++i)
        {
            ret_str += String(driving_strength_[i]) + ", ";
        }
        ret_str += String(driving_strength_[driving_strength_.size() - 1]);
        ret_str += "]";
        return ret_str;
    }
    
    Map<double>* StdCellLib::getStdCellCache() const
    {
        return m_std_cell_cache_;
    }
    
} // namespace DSENT

