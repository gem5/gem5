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

#ifndef __DSENT_MODEL_STD_CELLS_STDCELL_H__
#define __DSENT_MODEL_STD_CELLS_STDCELL_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class StdCell : public ElectricalModel
    {
        public:
            StdCell(const String& instance_name_, const TechModel* tech_model_);
            virtual ~StdCell();

        public:
            // Set a list of parameters needed to construct model
            virtual void initParameters();
            // Set a list of properties needed to update model
            virtual void initProperties();
            
            // Get PMOS to NMOS ratio
            double getPToNRatio() const;
            void setPToNRatio(double p_to_n_ratio_);
            // Get height of the standard cell taken by active transistors
            double getActiveHeight() const;
            void setActiveHeight(double active_height_);
            // Get total height of the standard cell including overheads
            double getTotalHeight() const;
            void setTotalHeight(double total_height_);

            // Construct the full model of the standard cell and cache
            // its contents to use for future copies of the standard cell
            virtual void cacheStdCell(StdCellLib* cell_lib_, double drive_strength_) = 0;
        
        protected:
            // Build the model, note that this is only available if the
            // standard cell has been cached (via cacheStdCellModel)
            virtual void constructModel() = 0;
            virtual void updateModel() = 0;
            
        private:
            // The PMOS to NMOS ratio
            double m_p_to_n_ratio_;
            // The height of the standard cell taken by active transitors
            double m_active_height_;
            // The total height of the standard cell including overheads
            double m_total_height_;            
        
    }; // class StdCell
} // namespace DSENT

#endif // __DSENT_MODEL_STD_CELLS_STDCELL_H__

