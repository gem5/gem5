#ifndef __DSENT_MODEL_NETWORK_ELECTRICAL_CLOS_H__
#define __DSENT_MODEL_NETWORK_ELECTRICAL_CLOS_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    /** 
     * \brief An electrical 3-stage clos network
     */
    class ElectricalClos : public ElectricalModel
    {
        public:
            ElectricalClos(const String& instance_name_, const TechModel* tech_model_);
            virtual ~ElectricalClos();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual ElectricalClos* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void propagateTransitionInfo();

    };
} // namespace DSENT

#endif // __DSENT_MODEL_NETWORK_ELECTRICAL_CLOS_H__

