/*
 * Copyright (c) 2009 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_STRUCTURES_MEMORYVECTOR_HH__
#define __MEM_RUBY_STRUCTURES_MEMORYVECTOR_HH__

#include "base/trace.hh"
#include "debug/RubyCacheTrace.hh"
#include "mem/ruby/common/Address.hh"

class DirectoryMemory;

/**
 *  MemoryVector holds memory data (DRAM only)
 */
class MemoryVector
{
  public:
    MemoryVector();
    MemoryVector(uint64 size);
    ~MemoryVector();
    friend class DirectoryMemory;

    void resize(uint64 size);  // destructive

    void write(const Address & paddr, uint8_t *data, int len);
    uint8_t *read(const Address & paddr, uint8_t *data, int len);
    uint32_t collatePages(uint8_t *&raw_data);
    void populatePages(uint8_t *raw_data);

  private:
    uint8_t *getBlockPtr(const PhysAddress & addr);

    uint64 m_size;
    uint8_t **m_pages;
    uint32_t m_num_pages;
    const uint32_t m_page_offset_mask;
    static const uint32_t PAGE_SIZE = 4096;
};

inline
MemoryVector::MemoryVector()
    : m_page_offset_mask(4095)
{
    m_size = 0;
    m_num_pages = 0;
    m_pages = NULL;
}

inline
MemoryVector::MemoryVector(uint64 size)
    : m_page_offset_mask(4095)
{
    resize(size);
}

inline
MemoryVector::~MemoryVector()
{
    for (int i = 0; i < m_num_pages; i++) {
        if (m_pages[i] != 0) {
            delete [] m_pages[i];
        }
    }
    delete [] m_pages;
}

inline void
MemoryVector::resize(uint64 size)
{
    if (m_pages != NULL){
        for (int i = 0; i < m_num_pages; i++) {
            if (m_pages[i] != 0) {
                delete [] m_pages[i];
            }
        }
        delete [] m_pages;
    }
    m_size = size;
    assert(size%PAGE_SIZE == 0);
    m_num_pages = size >> 12;
    m_pages = new uint8_t*[m_num_pages];
    memset(m_pages, 0, m_num_pages * sizeof(uint8_t*));
}

inline void
MemoryVector::write(const Address & paddr, uint8_t *data, int len)
{
    assert(paddr.getAddress() + len <= m_size);
    uint32_t page_num = paddr.getAddress() >> 12;
    if (m_pages[page_num] == 0) {
        bool all_zeros = true;
        for (int i = 0; i < len;i++) {
            if (data[i] != 0) {
                all_zeros = false;
                break;
            }
        }
        if (all_zeros)
            return;
        m_pages[page_num] = new uint8_t[PAGE_SIZE];
        memset(m_pages[page_num], 0, PAGE_SIZE);
        uint32_t offset = paddr.getAddress() & m_page_offset_mask;
        memcpy(&m_pages[page_num][offset], data, len);
    } else {
        memcpy(&m_pages[page_num][paddr.getAddress()&m_page_offset_mask],
               data, len);
    }
}

inline uint8_t*
MemoryVector::read(const Address & paddr, uint8_t *data, int len)
{
    assert(paddr.getAddress() + len <= m_size);
    uint32_t page_num = paddr.getAddress() >> 12;
    if (m_pages[page_num] == 0) {
        memset(data, 0, len);
    } else {
        memcpy(data, &m_pages[page_num][paddr.getAddress()&m_page_offset_mask],
               len);
    }
    return data;
}

inline uint8_t*
MemoryVector::getBlockPtr(const PhysAddress & paddr)
{
    uint32_t page_num = paddr.getAddress() >> 12;
    if (m_pages[page_num] == 0) {
        m_pages[page_num] = new uint8_t[PAGE_SIZE];
        memset(m_pages[page_num], 0, PAGE_SIZE);
    }
    return &m_pages[page_num][paddr.getAddress()&m_page_offset_mask];
}

/*!
 * Function for collating all the pages of the physical memory together.
 * In case a pointer for a page is NULL, this page needs only a single byte
 * to represent that the pointer is NULL. Otherwise, it needs 1 + PAGE_SIZE
 * bytes. The first represents that the page pointer is not NULL, and rest of
 * the bytes represent the data on the page.
 */

inline uint32_t
MemoryVector::collatePages(uint8_t *&raw_data)
{
    uint32_t num_zero_pages = 0;
    uint32_t data_size = 0;

    for (uint32_t i = 0;i < m_num_pages; ++i)
    {
        if (m_pages[i] == 0) num_zero_pages++;
    }

    raw_data = new uint8_t[sizeof(uint32_t) /* number of pages*/ +
                           m_num_pages /* whether the page is all zeros */ +
                           PAGE_SIZE * (m_num_pages - num_zero_pages)];

    /* Write the number of pages to be stored. */
    memcpy(raw_data, &m_num_pages, sizeof(uint32_t));
    data_size = sizeof(uint32_t);

    DPRINTF(RubyCacheTrace, "collating %d pages\n", m_num_pages);

    for (uint32_t i = 0;i < m_num_pages; ++i)
    {
        if (m_pages[i] == 0) {
            raw_data[data_size] = 0;
        } else {
            raw_data[data_size] = 1;
            memcpy(raw_data + data_size + 1, m_pages[i], PAGE_SIZE);
            data_size += PAGE_SIZE;
        }
        data_size += 1;
    }

    return data_size;
}

/*!
 * Function for populating the pages of the memory using the available raw
 * data. Each page has a byte associate with it, which represents whether the
 * page was NULL or not, when all the pages were collated. The function assumes
 * that the number of pages in the memory are same as those that were recorded
 * in the checkpoint.
 */
inline void
MemoryVector::populatePages(uint8_t *raw_data)
{
    uint32_t data_size = 0;
    uint32_t num_pages = 0;

    /* Read the number of pages that were stored. */
    memcpy(&num_pages, raw_data, sizeof(uint32_t));
    data_size = sizeof(uint32_t);
    assert(num_pages == m_num_pages);

    DPRINTF(RubyCacheTrace, "Populating %d pages\n", num_pages);

    for (uint32_t i = 0;i < m_num_pages; ++i)
    {
        assert(m_pages[i] == 0);
        if (raw_data[data_size] != 0) {
            m_pages[i] = new uint8_t[PAGE_SIZE];
            memcpy(m_pages[i], raw_data + data_size + 1, PAGE_SIZE);
            data_size += PAGE_SIZE;
        }
        data_size += 1;
    }
}

#endif // __MEM_RUBY_STRUCTURES_MEMORYVECTOR_HH__
