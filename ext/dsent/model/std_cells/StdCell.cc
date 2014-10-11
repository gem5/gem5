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

#include "model/std_cells/StdCell.h"

#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalLoad.h"

#include <cmath>
#include <algorithm>

namespace DSENT
{
    StdCell::StdCell(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    StdCell::~StdCell()
    {

    }
    
    
    void StdCell::initParameters()
    {
        addParameterName("AvailableDrivingStrengths");
        return;
    }
    
    void StdCell::initProperties()
    {
        addPropertyName("DrivingStrength");
        return;
    }

    // Get PMOS to NMOS ratio
    double StdCell::getPToNRatio() const
    {
        return m_p_to_n_ratio_;
    }
    
    void StdCell::setPToNRatio(double p_to_n_ratio_)
    {
        m_p_to_n_ratio_ = p_to_n_ratio_;
    }
    
    // Get height of the standard cell taken by active transistors
    double StdCell::getActiveHeight() const
    {
        return m_active_height_;
    }
    
    void StdCell::setActiveHeight(double active_height_)
    {
        m_active_height_ = active_height_;
    }
    
    // Get total height of the standard cell including overheads
    double StdCell::getTotalHeight() const
    {
        return m_total_height_;
    }

    void StdCell::setTotalHeight(double total_height_)
    {
        m_total_height_ = total_height_;
    }

} // namespace DSENT

