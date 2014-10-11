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

