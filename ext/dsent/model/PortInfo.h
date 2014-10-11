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

