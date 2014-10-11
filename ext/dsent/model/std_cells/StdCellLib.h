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

#ifndef __DSENT_MODEL_STD_CELLS_STDCELLLIBS_H__
#define __DSENT_MODEL_STD_CELLS_STDCELLLIBS_H__

#include "util/CommonType.h"

namespace DSENT
{
    class TechModel;
    class StdCell;
    class LibertyFile;

    class StdCellLib
    {
        public:
            StdCellLib(TechModel* tech_model_);
            ~StdCellLib();

        public:
            // Get the technology model pointer
            const TechModel* getTechModel() const;
            // Create a standard cell by name and instance name
            StdCell* createStdCell(const String& std_cell_name_, const String& instance_name_) const;

            // Get PMOS to NMOS ratio
            double getPToNRatio() const;
            void setPToNRatio(double p_to_n_ratio_);
            // Get height of the standard cell taken by active transistors
            double getActiveHeight() const;
            void setActiveHeight(double active_height_);
            // Get total height of the standard cell including overheads
            double getTotalHeight() const;
            void setTotalHeight(double total_height_);
            // Get the standard cell library cache of values
            Map<double>* getStdCellCache() const;
            // Create a list of standard cells
            void createLib();

            // Return a copy of this instance
            StdCellLib* clone() const;            
            
        private:
            // Disabled copy constructor. Use clone to perform copy operation
            StdCellLib(const StdCellLib& std_cell_lib_);
            // Generate driving strength string
            const String genDrivingStrengthString(const vector<double>& driving_strength_) const;

        private:
            // Technology model pointer
            TechModel* m_tech_model_;
            // The PMOS to NMOS ratio
            double m_p_to_n_ratio_;
            // The height of the standard cell taken by active transitors
            double m_active_height_;
            // The total height of the standard cell including overheads
            double m_total_height_;
            // Std cell values cache
            Map<double>* m_std_cell_cache_;
            
    }; // class StdCellLib
} // namespace DSENT

#endif // __DSENT_MODEL_STD_CELLS_STDCELLLIBS_H__

