#ifndef __DSENT_MODEL_PORT_INFO_H__
#define __DSENT_MODEL_PORT_INFO_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"
#include "model/TransitionInfo.h"

namespace DSENT
{
    class PortInfo
    {
        public:
            PortInfo(const String& port_name_, const NetIndex& net_index_ = NetIndex(0, 0));
            ~PortInfo();

        public:
            // Get the port name
            const String& getPortName() const;
            // Get the net index
            const NetIndex& getNetIndex() const;
            // Set the transition information of this port
            void setTransitionInfo(const TransitionInfo& trans_info_);
            // Get the transition information of this port
            const TransitionInfo& getTransitionInfo() const;

        private:
            // Name of this port
            String m_port_name_;
            // Net index of the input port
            NetIndex m_net_index_;
            // Store the transition information of this port
            TransitionInfo m_tran_info_;
    }; // class PortInfo
} // namespace DSENT

#endif // __DSENT_MODEL_PORT_INFO_H__

