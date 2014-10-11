#ifndef __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_H__
#define __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    /** \class Router
     * \param Input ports: In[0-9]*
     * \param Output ports: Out[0-9]*
     */
    class Router : public ElectricalModel
    {
        public:
            Router(const String& instance_name_, const TechModel* tech_model_);
            virtual ~Router();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual Router* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void propagateTransitionInfo();

        private:
            void createRouterInputPort();
            void createVirtualChannelAllocator();
            void createSwitchAllocator();
            void createCrossbar();
            void createClockTree();
            void createPipelineReg();

    }; // class Router
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_H__

