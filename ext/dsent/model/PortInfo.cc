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

#include "model/PortInfo.h"

namespace DSENT
{
    PortInfo::PortInfo(const String& port_name_, const NetIndex& net_index_)
        : m_port_name_(port_name_), m_net_index_(net_index_), m_tran_info_(TransitionInfo())
    {
    }

    PortInfo::~PortInfo()
    {
    }

    const String& PortInfo::getPortName() const
    {
        return m_port_name_;
    }

    const NetIndex& PortInfo::getNetIndex() const
    {
        return m_net_index_;
    }

    void PortInfo::setTransitionInfo(const TransitionInfo& trans_info_)
    {
        m_tran_info_ = trans_info_;
        return;
    }

    const TransitionInfo& PortInfo::getTransitionInfo() const
    {
        return m_tran_info_;
    }
} // namespace DSENT

