#ifndef __DSENT_MODEL_EVENT_INFO_H__
#define __DSENT_MODEL_EVENT_INFO_H__

#include "util/CommonType.h"
#include "model/TransitionInfo.h"

namespace DSENT
{
    class PortInfo;

    class EventInfo
    {
        public:
            EventInfo(const String& event_name_, const Map<PortInfo*>* port_infos_);
            ~EventInfo();

        public:
            const String& getEventName() const;
            void setTransitionInfo(const String& port_name_, const TransitionInfo& trans_info_);
            void setStaticTransitionInfo(const String& port_name_);
            void setRandomTransitionInfos();
            void setStaticTransitionInfos();
            const TransitionInfo& getTransitionInfo(const String& port_name_) const;

        private:
            String m_event_name_;
            Map<TransitionInfo>* m_trans_info_map_;
    }; // class EventInfo
} // namespace DSENT

#endif // __DSENT_MODEL_EVENT_INFO_H__

