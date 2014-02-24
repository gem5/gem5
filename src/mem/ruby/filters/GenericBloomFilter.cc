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

#include "base/str.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/filters/BlockBloomFilter.hh"
#include "mem/ruby/filters/BulkBloomFilter.hh"
#include "mem/ruby/filters/GenericBloomFilter.hh"
#include "mem/ruby/filters/H3BloomFilter.hh"
#include "mem/ruby/filters/LSB_CountingBloomFilter.hh"
#include "mem/ruby/filters/MultiBitSelBloomFilter.hh"
#include "mem/ruby/filters/MultiGrainBloomFilter.hh"
#include "mem/ruby/filters/NonCountingBloomFilter.hh"

using namespace std;

GenericBloomFilter::GenericBloomFilter(string config)
{
    string head, tail;
#ifndef NDEBUG
    bool success =
#endif
        split_first(config, head, tail, '_');
    assert(success);

    if (head == "LSB_Counting" ) {
        m_filter = new LSB_CountingBloomFilter(tail);
    } else if(head == "NonCounting" ) {
        m_filter = new NonCountingBloomFilter(tail);
    } else if(head == "Bulk" ) {
        m_filter = new BulkBloomFilter(tail);
    } else if(head == "Block") {
        m_filter = new BlockBloomFilter(tail);
    } else if(head == "Multigrain"){
        m_filter = new MultiGrainBloomFilter(tail);
    } else if(head == "MultiBitSel"){
        m_filter = new MultiBitSelBloomFilter(tail);
    } else if(head == "H3"){
        m_filter = new H3BloomFilter(tail);
    } else {
        assert(0);
    }
}

GenericBloomFilter::~GenericBloomFilter()
{
    delete m_filter;
}

void
GenericBloomFilter::clear()
{
    m_filter->clear();
}

void
GenericBloomFilter::increment(const Address& addr)
{
    m_filter->increment(addr);
}

void
GenericBloomFilter::decrement(const Address& addr)
{
    m_filter->decrement(addr);
}

void
GenericBloomFilter::merge(GenericBloomFilter * other_filter)
{
    m_filter->merge(other_filter->getFilter());
}

void
GenericBloomFilter::set(const Address& addr)
{
    m_filter->set(addr);
}

void
GenericBloomFilter::unset(const Address& addr)
{
    m_filter->unset(addr);
}

bool
GenericBloomFilter::isSet(const Address& addr)
{
    return m_filter->isSet(addr);
}

int
GenericBloomFilter::getCount(const Address& addr)
{
    return m_filter->getCount(addr);
}

int
GenericBloomFilter::getTotalCount()
{
    return m_filter->getTotalCount();
}

int
GenericBloomFilter::getIndex(const Address& addr)
{
    return m_filter->getIndex(addr);
}

int
GenericBloomFilter::readBit(const int index)
{
    return m_filter->readBit(index);
}

void
GenericBloomFilter::writeBit(const int index, const int value)
{
    m_filter->writeBit(index, value);
}

void
GenericBloomFilter::print(ostream& out) const
{
    return m_filter->print(out);
}


