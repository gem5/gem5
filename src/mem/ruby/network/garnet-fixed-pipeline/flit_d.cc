/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * flit_d.C
 *
 * Niket Agarwal, Princeton University
 *
 * */

#include "flit_d.hh"

flit_d::flit_d(int id, int  vc, int vnet, int size, MsgPtr msg_ptr)
{
        m_size = size;
        m_msg_ptr = msg_ptr;
        m_enqueue_time = g_eventQueue_ptr->getTime();
        m_time = g_eventQueue_ptr->getTime();
        m_id = id;
        m_vnet = vnet;
        m_vc = vc;
        m_stage.first = I_;
        m_stage.second = m_time;

        if(size == 1)
        {
                m_type = HEAD_TAIL_;
                return;
        }
        if(id == 0)
                m_type = HEAD_;
        else if(id == (size - 1))
                m_type = TAIL_;
        else
                m_type = BODY_;
}

flit_d::flit_d(int vc, bool is_free_signal)
{
        m_id = 0;
        m_vc = vc;
        m_is_free_signal = is_free_signal;
        m_time = g_eventQueue_ptr->getTime();
}
/*
int flit_d::get_size()
{
        return m_size;
}
Time flit_d::get_enqueue_time()
{
        return m_enqueue_time;
}
int flit_d::get_id()
{
        return m_id;
}
Time flit_d::get_time()
{
        return m_time;
}
void flit_d::set_time(Time time)
{
        m_time = time;
}
int flit_d::get_vnet()
{
        return m_vnet;
}
int flit_d::get_vc()
{
        return m_vc;
}
void flit_d::set_vc(int vc)
{
        m_vc = vc;
}
MsgPtr& flit_d::get_msg_ptr()
{
        return m_msg_ptr;
}
flit_type flit_d::get_type()
{
        return m_type;
}
bool flit_d::is_stage(flit_stage t_stage)
{
        return ((m_stage.first == t_stage) && (g_eventQueue_ptr->getTime() >= m_stage.second));
}
bool flit_d::is_next_stage(flit_stage t_stage)
{
        return ((m_stage.first == t_stage) && ((g_eventQueue_ptr->getTime()+1) >= m_stage.second));
}
void flit_d::advance_stage(flit_stage t_stage)
{
        m_stage.first = t_stage;
        m_stage.second = g_eventQueue_ptr->getTime() + 1;
}
*/
void flit_d::print(ostream& out) const
{
        out << "[flit:: ";
        out << "Id=" << m_id << " ";
        out << "Type=" << m_type << " ";
        out << "Vnet=" << m_vnet << " ";
        out << "VC=" << m_vc << " ";
        out << "Enqueue Time=" << m_enqueue_time << " ";
        out << "]";
}
