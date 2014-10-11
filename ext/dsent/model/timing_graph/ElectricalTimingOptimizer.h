#ifndef __DSENT_MODEL_TIMING_GRAPH_ELECTRICAL_TIMING_OPTIMIZER_H__
#define __DSENT_MODEL_TIMING_GRAPH_ELECTRICAL_TIMING_OPTIMIZER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    // This model is only used to optimize the timing
    class ElectricalTimingOptimizer : public ElectricalModel
    {
        public:
            ElectricalTimingOptimizer(const String& instance_name_, const TechModel* tech_model_);
            virtual ~ElectricalTimingOptimizer();

        public:
            void setModel(ElectricalModel* model_);
            ElectricalModel* getModel();

        protected:
            // Build the optimizer
            virtual void constructModel();

        private:
            ElectricalModel* m_model_;
    }; // class ElectricalTimingOptimizer
} // namespace

#endif // __DSENT_MODEL_TIMING_GRAPH_ELECTRICAL_TIMING_OPTIMIZER_H__

