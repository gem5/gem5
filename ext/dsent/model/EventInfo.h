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

