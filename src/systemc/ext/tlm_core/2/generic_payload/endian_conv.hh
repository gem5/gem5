/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/


#ifndef __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_ENDIAN_CONV_HH__
#define __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_ENDIAN_CONV_HH__

#include <cstring> // std::memset

#include "gp.hh"

namespace tlm
{

/*
Tranaction-Level Modelling
Endianness Helper Functions

DESCRIPTION
A set of functions for helping users to get the endianness
right in their TLM models of system initiators.  These functions are
for use within an initiator.  They can not be used as-is outside
an initiator because the extension used to store context will not work
if cascaded, and they do not respect the generic payload mutability
rules.  However this code may be easily copied and adapted for use
in bridges, etc..

These functions are not compulsory.  There are other legitimate ways to
achieve the same functionality.  If extra information is available at
compile time about the nature of an initiator's transactions, this can
be exploited to accelerate simulations by creating further functions
similar to those in this file.  In general a functional transaction can be
described in more than one way by a TLM-2 GP object.

The functions convert the endianness of a GP object, either on request or
response.  They should only be used when the initiator's endianness
does not match the host's endianness.  They assume 'arithmetic mode'
meaning that within a data word the byte order is always host-endian.
For non-arithmetic mode initiators they can be used with a data word
size of 1 byte.

All the functions are templates, for example:

template<class DATAWORD> inline void
  to_hostendian_generic(tlm_generic_payload *txn, int sizeof_databus)

The template parameter provides the data word width.  Having this as a class
makes it easy to use it for copy and swap operations within the functions.
If the assignment operator for this class is overloaded, the endianness
conversion function may not have the desired effect.

All the functions have the same signature except for different names.

The principle is that a function to_hostendian_convtype() is called when the
initiator-endian transaction is created, and the matching function
from_hostendian_convtype() is called when the transaction is completed, for
example before read data can be used.  In some cases the from_ function is
redundant but an empty function is provided anyway.  It is strongly
recommended that the from_ function is called, in case it ceases to be
redundant in future versions of this code.

No context needs to be managed outside the two functions, except that they
must be called with the same template parameter and the same bus width.

For initiator models that can not easily manage this context information,
a single entry point for the from_ function is provided, which will be
a little slower than calling the correct from_ function directly, as
it can not be inlined.

All functions assume power-of-2 bus and data word widths.

Functions offered:

0) A pair of functions that work for almost all TLM2 GP transactions.  The
only limitations are that data and bus widths should be powers of 2, and that
the data length should be an integer number of streaming widths and that the
streaming width should be an integer number of data words.
These functions always allocate new data and byte enable buffers and copy
data one byte at a time.
  tlm_to_hostendian_generic(tlm_generic_payload *txn, int sizeof_databus)
  tlm_from_hostendian_generic(tlm_generic_payload *txn, int sizeof_databus)

1) A pair of functions that work for all transactions regardless of data and
bus data sizes and address alignment except for the the following
limitations:
- byte-enables are supported only when byte-enable granularity is no finer
than the data word (every data word is wholly enabled or wholly disabled)
- byte-enable-length is not supported (if byte enables are present, the byte
enable length must be equal to the data length).
- streaming width is not supported
- data word wider than bus word is not supported
A new data buffer and a new byte enable buffer are always allocated.  Byte
enables are assumed to be needed even if not required for the original
(unconverted) transaction.  Data is copied to the new buffer on request
(for writes) or on response (for reads).  Copies are done word-by-word
where possible.
  tlm_to_hostendian_word(tlm_generic_payload *txn, int sizeof_databus)
  tlm_from_hostendian_word(tlm_generic_payload *txn, int sizeof_databus)

2) If the original transaction is both word and bus-aligned then this pair of
functions can be used.  It will complete faster than the generic function
because the data reordering function is much simpler and no address
conversion is required.
The following limitations apply:
- byte-enables are supported only when byte-enable granularity is no finer
than the data word (every data word is wholly enabled or wholly disabled)
- byte-enable-length is not supported (if byte enables are present, the byte
enable length must be equal to the data length).
- streaming width is not supported
- data word wider than bus word is not supported
- the transaction must be an integer number of bus words
- the address must be aligned to the bus width
  tlm_to_hostendian_aligned(tlm_generic_payload *txn, int sizeof_databus)
  tlm_from_hostendian_aligned(tlm_generic_payload *txn, int sizeof_databus)

3) For single word transactions that don't cross a bus word boundary it
is always safe to work in-place and the conversion is very simple.  Again,
streaming width and byte-enable length are not supported, and byte-enables
may not changes within a data word.
  tlm_to_hostendian_single(tlm_generic_payload *txn, int sizeof_databus)
  tlm_from_hostendian_single(tlm_generic_payload *txn, int sizeof_databus)

4) A single entry point for accessing the correct from_ function without
needing to store context.
  tlm_from_hostendian(tlm_generic_payload *txn)
*/

///////////////////////////////////////////////////////////////////////////////
// Generic Utilities

class tlm_endian_context;

class tlm_endian_context_pool
{
  public:
    tlm_endian_context *first;
    inline tlm_endian_context_pool();
    inline ~tlm_endian_context_pool();
    inline tlm_endian_context *pop();
    inline void push(tlm_endian_context *c);
};

static tlm_endian_context_pool global_tlm_endian_context_pool;

// an extension to keep the information needed for reconversion of response
class tlm_endian_context : public tlm_extension<tlm_endian_context>
{
  public:
    tlm_endian_context() : dbuf_size(0), bebuf_size(0) {}

    ~tlm_endian_context() {
        if (dbuf_size > 0)
            delete [] new_dbuf;
        if (bebuf_size > 0)
            delete [] new_bebuf;
    }

    sc_dt::uint64 address; // Used by generic, word.
    sc_dt::uint64 new_address; // Used by generic.
    unsigned char *data_ptr; // Used by generic, word, aligned.
    unsigned char *byte_enable; // Used by word.
    int length; // Used by generic, word.
    int stream_width; // Used by generic.

    // Used by common entry point on response.
    void (*from_f)(tlm_generic_payload *txn, unsigned int sizeof_databus);
    int sizeof_databus;

    // Reordering buffers for data and byte-enables.
    unsigned char *new_dbuf, *new_bebuf;
    int dbuf_size, bebuf_size;

    void
    establish_dbuf(int len)
    {
        if (len <= dbuf_size)
            return;
        if (dbuf_size > 0)
            delete [] new_dbuf;
        new_dbuf = new unsigned char[len];
        dbuf_size = len;
    }

    void
    establish_bebuf(int len)
    {
        if (len <= bebuf_size)
            return;
        if (bebuf_size > 0)
            delete [] new_bebuf;
        new_bebuf = new unsigned char[len];
        bebuf_size = len;
    }

    // Required for extension management.
    void free() { global_tlm_endian_context_pool.push(this); }
    tlm_extension_base *clone() const { return 0; }
    void copy_from(tlm_extension_base const &) { return; }

    // For pooling.
    tlm_endian_context *next;
};

// Assumptions about transaction contexts:
// 1) only the address attribute of a transaction
// is mutable.  all other attributes are unchanged from the request to
// response side conversion.
// 2) the conversion functions in this file do not respect the mutability
// rules and do not put the transaction back into its original state after
// completion.  so if the initiator has any cleaning up to do (eg of byte
// enable buffers), it needs to store its own context.  the transaction
// returned to the initiator may contain pointers to data and byte enable
// that can/must not be deleted.
// 3) the conversion functions in this file use an extension to store
// context information.  they do not remove this extension.  the initiator
// should not remove it unless it deletes the generic payload
// object.

inline tlm_endian_context *
establish_context(tlm_generic_payload *txn)
{
    tlm_endian_context *tc = txn->get_extension<tlm_endian_context>();
    if (tc == 0) {
        tc = global_tlm_endian_context_pool.pop();
        txn->set_extension(tc);
    }
    return tc;
}

inline tlm_endian_context_pool::tlm_endian_context_pool() : first(0) {}

inline tlm_endian_context_pool::~tlm_endian_context_pool()
{
    while (first != 0) {
        tlm_endian_context *next = first->next;
        delete first;
        first = next;
    }
}

tlm_endian_context *
tlm_endian_context_pool::pop()
{
    if (first == 0)
        return new tlm_endian_context;
    tlm_endian_context *r = first;
    first = first->next;
    return r;
}

void tlm_endian_context_pool::push(tlm_endian_context *c)
{
    c->next = first;
    first = c;
}


// A set of constants for efficient filling of byte enables.
template <class D>
class tlm_bool
{
  public:
    static D TLM_TRUE;
    static D TLM_FALSE;
    static D
    make_uchar_array(unsigned char c)
    {
        D d;
        unsigned char *tmp = (unsigned char *)(&d);
        for (ptrdiff_t i = 0; i != sizeof(D); i++)
            tmp[i] = c; // 64BITFIX negligable risk but easy fix.
        return d;
    }

    // Also provides an syntax-efficient tester, using a
    // copy constuctor and an implicit cast to boolean.
    tlm_bool(D &d) : b(*((unsigned char *)&d) != TLM_BYTE_DISABLED) {}
    operator bool() const { return b; }
  private:
    bool b;
};

template<class D>
D tlm_bool<D>::TLM_TRUE = tlm_bool<D>::make_uchar_array(TLM_BYTE_ENABLED);
template<class D>
D tlm_bool<D>::TLM_FALSE = tlm_bool<D>::make_uchar_array(TLM_BYTE_DISABLED);



inline void
copy_db0(unsigned char *src1, unsigned char *src2,
         unsigned char *dest1, unsigned char *dest2)
{
    *dest1 = *src1;
    *dest2 = *src2;
}

inline void
copy_dbtrue0(unsigned char *src1, unsigned char * /* src2 */,
             unsigned char *dest1, unsigned char *dest2)
{
    *dest1 = *src1;
    *dest2 = TLM_BYTE_ENABLED;
}

inline void
copy_btrue0(unsigned char * /* src1 */, unsigned char * /* src2 */,
            unsigned char * /* dest1 */, unsigned char *dest2)
{
    *dest2 = TLM_BYTE_ENABLED;
}

inline void
copy_b0(unsigned char * /* src1 */, unsigned char *src2,
        unsigned char * /* dest1 */, unsigned char *dest2)
{
    *dest2 = *src2;
}

inline void
copy_dbyb0(unsigned char *src1, unsigned char * /* src2 */,
           unsigned char *dest1, unsigned char *dest2)
{
    if (*dest2 == TLM_BYTE_ENABLED)
        *src1 = *dest1;
}


template <class D,
          void COPY(unsigned char *he_d, unsigned char *he_b,
                    unsigned char *ie_d, unsigned char *ie_b)>
inline void
loop_generic0(int new_len, int new_stream_width, int orig_stream_width,
              int sizeof_databus, sc_dt::uint64 orig_start_address,
              sc_dt::uint64 new_start_address, int be_length,
              unsigned char *ie_data, unsigned char *ie_be,
              unsigned char *he_data, unsigned char *he_be)
{
    for (int orig_sword = 0, new_sword = 0; new_sword < new_len;
            new_sword += new_stream_width, orig_sword += orig_stream_width) {
        sc_dt::uint64 ie_addr = orig_start_address;
        for (int orig_dword = orig_sword;
                orig_dword < orig_sword + orig_stream_width;
                orig_dword += sizeof(D)) {
            for (int curr_byte = orig_dword + sizeof(D) - 1;
                    curr_byte >= orig_dword; curr_byte--) {
                ptrdiff_t he_index = ((ie_addr++) ^ (sizeof_databus - 1)) -
                    new_start_address + new_sword; // 64BITFIX
                COPY(ie_data + curr_byte,
                        // 64BITRISK no risk of overflow, always positive.
                        ie_be + (curr_byte % be_length),
                        he_data + he_index, he_be + he_index);
            }
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
// function set (0): Response
///////////////////////////////////////////////////////////////////////////////

template <class DATAWORD>
inline void
tlm_from_hostendian_generic(tlm_generic_payload *txn,
                            unsigned int sizeof_databus)
{
    if (txn->is_read()) {
        tlm_endian_context *tc =
            txn->template get_extension<tlm_endian_context>();
        loop_generic0<DATAWORD, &copy_dbyb0>(txn->get_data_length(),
                txn->get_streaming_width(), tc->stream_width, sizeof_databus,
                tc->address, tc->new_address, txn->get_data_length(),
                tc->data_ptr, 0, txn->get_data_ptr(),
                txn->get_byte_enable_ptr());
    }
}


///////////////////////////////////////////////////////////////////////////////
// function set (0): Request
template <class DATAWORD>
inline void
tlm_to_hostendian_generic(tlm_generic_payload *txn,
        unsigned int sizeof_databus)
{
    tlm_endian_context *tc = establish_context(txn);
    tc->from_f = &(tlm_from_hostendian_generic<DATAWORD>);
    tc->sizeof_databus = sizeof_databus;

    // Calculate new size: nr stream words multiplied by big enough stream
    // width.
    int s_width = txn->get_streaming_width();
    int length = txn->get_data_length();
    if (s_width >= length)
        s_width = length;
    int nr_stream_words = length / s_width;

    // Find out in which bus word the stream word starts and ends.
    sc_dt::uint64 new_address = (txn->get_address() & ~(sizeof_databus - 1));
    sc_dt::uint64 end_address = ((txn->get_address() + s_width - 1) &
            ~(sizeof_databus - 1));

    int new_stream_width = end_address - new_address + sizeof_databus;
    int new_length = new_stream_width * nr_stream_words;

    // Store context.
    tc->data_ptr = txn->get_data_ptr();
    tc->address = txn->get_address();
    tc->new_address = new_address;
    tc->stream_width = s_width;
    unsigned char *orig_be = txn->get_byte_enable_ptr();
    int orig_be_length = txn->get_byte_enable_length();

    // Create data and byte-enable buffers.
    txn->set_address(new_address);
    tc->establish_dbuf(new_length);
    txn->set_data_ptr(tc->new_dbuf);
    tc->establish_bebuf(new_length);
    txn->set_byte_enable_ptr(tc->new_bebuf);
    std::memset(txn->get_byte_enable_ptr(), TLM_BYTE_DISABLED, new_length);
    txn->set_streaming_width(new_stream_width);
    txn->set_data_length(new_length);
    txn->set_byte_enable_length(new_length);

    // Copy data and/or byte enables.
    if (txn->is_write()) {
        if (orig_be == 0) {
            loop_generic0<DATAWORD, &copy_dbtrue0>(
                    new_length, new_stream_width, s_width, sizeof_databus,
                    tc->address, new_address, new_length, tc->data_ptr, 0,
                    txn->get_data_ptr(), txn->get_byte_enable_ptr());
        } else {
            loop_generic0<DATAWORD, &copy_db0>(new_length, new_stream_width,
                    s_width, sizeof_databus, tc->address, new_address,
                    orig_be_length, tc->data_ptr, orig_be,
                    txn->get_data_ptr(), txn->get_byte_enable_ptr());
        }
    } else {
        // Read transaction.
        if (orig_be == 0) {
            loop_generic0<DATAWORD, &copy_btrue0>(new_length,
                    new_stream_width, s_width, sizeof_databus, tc->address,
                    new_address, new_length, tc->data_ptr, 0,
                    txn->get_data_ptr(), txn->get_byte_enable_ptr());
        } else {
            loop_generic0<DATAWORD, &copy_b0>(new_length, new_stream_width,
                    s_width, sizeof_databus, tc->address, new_address,
                    orig_be_length, tc->data_ptr, orig_be,
                    txn->get_data_ptr(), txn->get_byte_enable_ptr());
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
// function set (1): Utilities
///////////////////////////////////////////////////////////////////////////////

template <class D>
inline void
copy_d1(unsigned char *src1, unsigned char *src2,
        unsigned char *dest1, unsigned char *dest2)
{
    *((D *)dest1) = *((D *)src1);
    *((D *)dest2) = tlm_bool<D>::TLM_TRUE;
}

template <class D>
inline void
copy_db1(unsigned char *src1, unsigned char *src2,
         unsigned char *dest1, unsigned char *dest2)
{
    *((D *)dest1) = *((D *)src1);
    *((D *)dest2) = *((D *)src2);
}

template <class D>
inline void
true_b1(unsigned char *src1, unsigned char *src2,
        unsigned char *dest1, unsigned char *dest2)
{
    *((D *)dest2) = tlm_bool<D>::TLM_TRUE;
}

template <class D>
inline void
copy_b1(unsigned char *src1, unsigned char *src2,
        unsigned char *dest1, unsigned char *dest2)
{
    *((D *)dest2) = *((D *)src2);
}

template <class D>
inline void
copy_dbyb1(unsigned char *src1, unsigned char *src2,
           unsigned char *dest1, unsigned char *dest2)
{
    if (*src2 != TLM_BYTE_DISABLED)
        *((D *)src1) = *((D *)dest1);
}

template <class D>
inline void
copy_dbytrue1(unsigned char *src1, unsigned char *src2,
              unsigned char *dest1, unsigned char *dest2)
{
    *((D *)src1) = *((D *)dest1);
}

template<class D>
inline void
false_b1(unsigned char *dest1)
{
    *((D *)dest1) = tlm_bool<D>::TLM_FALSE;
}

template<class D>
inline void
no_b1(unsigned char *dest1)
{}

template<class D,
         void COPY(unsigned char *src1, unsigned char *src2,
                   unsigned char *dest1, unsigned char *dest2),
         void COPYuchar(unsigned char *src1, unsigned char *src2,
                        unsigned char *dest1, unsigned char *dest2),
         void FILLFALSE(unsigned char *dest1),
         void FILLFALSEuchar(unsigned char *dest1)>
inline int
loop_word1(int bytes_left, int len0, int lenN, int sizeof_databus,
           unsigned char *start, unsigned char *end,
           unsigned char *src, unsigned char *bsrc,
           unsigned char *dest, unsigned char *bdest)
{
    ptrdiff_t d2b_src = bsrc - src; // 64BITFIX was int
    ptrdiff_t d2b_dest = bdest - dest; // 64BITFIX was int
    unsigned char *original_dest = dest;

    while (true) {
        // len0 bytes at start of a bus word.
        if ((src >= start) && (src < end)) {
            for (int i = 0; i < len0; i++) {
                COPYuchar(src, src + d2b_src, dest, dest + d2b_dest);
                src++;
                dest++;
            }
            bytes_left -= len0;
            if (bytes_left <= 0)
                return int(dest - original_dest);
        } else {
            for (int i = 0; i < len0; i++) {
                FILLFALSEuchar(dest + d2b_dest);
                src++;
                dest++;
            }
        }
        src -= 2 * sizeof(D);

        // Sequence of full data word fragments.
        for (unsigned int i = 1; i < sizeof_databus / sizeof(D); i++) {
            if ((src >= start) && (src < end)) {
                COPY(src, src + d2b_src, dest, dest + d2b_dest);
                bytes_left -= sizeof(D);
            } else {
                FILLFALSE(dest + d2b_dest);
            }
            dest += sizeof(D);
            if (bytes_left <= 0)
                return int(dest - original_dest);
            src -= sizeof(D);
        }

        // lenN bytes at end of bus word.
        if ((src >= start) && (src < end)) {
            for (int i = 0; i < lenN; i++) {
                COPYuchar(src, src + d2b_src, dest, dest + d2b_dest);
                src++;
                dest++;
            }
            bytes_left -= lenN;
            if (bytes_left <= 0)
                return int(dest - original_dest);
        } else {
            for (int i = 0; i < lenN; i++) {
                FILLFALSEuchar(dest + d2b_dest);
                src++;
                dest++;
            }
        }
        src += 2 * sizeof_databus;
    }
}


///////////////////////////////////////////////////////////////////////////////
// function set (1): Response
///////////////////////////////////////////////////////////////////////////////

template <class DATAWORD>
inline void
tlm_from_hostendian_word(tlm_generic_payload *txn, unsigned int sizeof_databus)
{
    if (txn->is_read()) {
        tlm_endian_context *tc =
            txn->template get_extension<tlm_endian_context>();
        sc_dt::uint64 b_mask = sizeof_databus - 1;
        int d_mask = sizeof(DATAWORD) - 1;
        int a_offset = static_cast<int>(tc->address & b_mask);
        int len0 = (sizeof_databus - a_offset) & d_mask;
        int lenN = sizeof(DATAWORD) - len0;
        unsigned char *d_start = tc->data_ptr;
        unsigned char *d_end =
            ptrdiff_t(tc->length) + d_start; // 64BITFIX probably redundant
        unsigned char *d =
            ptrdiff_t(((sizeof_databus - a_offset) & ~d_mask) + lenN) +
            d_start; // 64BITFIX probably redundant

        // Iterate over transaction copying data qualified by byte-enables.
        if (tc->byte_enable == 0) {
            loop_word1<DATAWORD, &copy_dbytrue1<DATAWORD>,
                       &copy_dbytrue1<unsigned char>, &no_b1<DATAWORD>,
                       &no_b1<unsigned char>>(
                               tc->length, len0, lenN, sizeof_databus,
                               d_start, d_end, d, 0, txn->get_data_ptr(), 0);
        } else {
            loop_word1<DATAWORD, &copy_dbyb1<DATAWORD>,
                       &copy_dbyb1<unsigned char>, &no_b1<DATAWORD>,
                       &no_b1<unsigned char>>(
                               tc->length, len0, lenN, sizeof_databus,
                               d_start, d_end, d,
                               tc->byte_enable - d_start + d,
                               txn->get_data_ptr(), 0);
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
// function set (1): Request
///////////////////////////////////////////////////////////////////////////////

template <class DATAWORD>
inline void
tlm_to_hostendian_word(tlm_generic_payload *txn, unsigned int sizeof_databus)
{
    tlm_endian_context *tc = establish_context(txn);
    tc->from_f = &(tlm_from_hostendian_word<DATAWORD>);
    tc->sizeof_databus = sizeof_databus;

    sc_dt::uint64 b_mask = sizeof_databus - 1;
    int d_mask = sizeof(DATAWORD) - 1;
    sc_dt::uint64 a_aligned = txn->get_address() & ~b_mask;
    int a_offset = static_cast<int>(txn->get_address() & b_mask);
    int len0 = (sizeof_databus - a_offset) & d_mask;
    int lenN = sizeof(DATAWORD) - len0;
    unsigned char *d_start = txn->get_data_ptr();
    unsigned char *d_end =
        ptrdiff_t(txn->get_data_length()) + d_start;
            // 64BITFIX probably redundant.
    unsigned char *d =
        ptrdiff_t(((sizeof_databus - a_offset) & ~d_mask) + lenN) + d_start;
            // 64BITFIX probably redundant.

    // Create new data and byte enable buffers.
    int long_enough = txn->get_data_length() + 2 * sizeof_databus;
    tc->establish_dbuf(long_enough);
    unsigned char *new_data = tc->new_dbuf;
    tc->establish_bebuf(long_enough);
    unsigned char *new_be = tc->new_bebuf;

    if (txn->is_read()) {
        tc->data_ptr = d_start;
        tc->address = txn->get_address();
        tc->byte_enable = txn->get_byte_enable_ptr();
        tc->length = txn->get_data_length();
        if (txn->get_byte_enable_ptr() == 0) {
            // Iterate over transaction creating new byte enables from all-true
            txn->set_data_length(
                    loop_word1<DATAWORD, &true_b1<DATAWORD>,
                               &true_b1<unsigned char>, &false_b1<DATAWORD>,
                               &false_b1<unsigned char>>(
                                   txn->get_data_length(), len0, lenN,
                                   sizeof_databus, d_start, d_end, d, 0,
                                   new_data, new_be));
        } else {
            // iterate over transaction copying byte enables
            txn->set_data_length(
                    loop_word1<DATAWORD, &copy_b1<DATAWORD>,
                               &copy_b1<unsigned char>, &false_b1<DATAWORD>,
                               &false_b1<unsigned char>>(
                                   txn->get_data_length(), len0, lenN,
                                   sizeof_databus, d_start, d_end, d,
                                   txn->get_byte_enable_ptr() - d_start + d,
                                   new_data, new_be));
        }
    } else {
        // WRITE
        if (txn->get_byte_enable_ptr() == 0) {
            // Iterate over transaction copying data and creating new
            // byte-enables.
            txn->set_data_length(
                    loop_word1<DATAWORD, &copy_d1<DATAWORD>,
                               &copy_d1<unsigned char>, &false_b1<DATAWORD>,
                               &false_b1<unsigned char>>(
                                   txn->get_data_length(), len0, lenN,
                                   sizeof_databus, d_start, d_end, d, 0,
                                   new_data, new_be));
        } else {
            // Iterate over transaction copying data and byte-enables.
            txn->set_data_length(
                    loop_word1<DATAWORD, &copy_db1<DATAWORD>,
                               &copy_db1<unsigned char>, &false_b1<DATAWORD>,
                                   &false_b1<unsigned char>>(
                                   txn->get_data_length(), len0, lenN,
                                   sizeof_databus, d_start, d_end, d,
                                   txn->get_byte_enable_ptr() - d_start + d,
                                   new_data, new_be));
        }
    }
    txn->set_byte_enable_length(txn->get_data_length());
    txn->set_streaming_width(txn->get_data_length());
    txn->set_data_ptr(new_data);
    txn->set_byte_enable_ptr(new_be);
    txn->set_address(a_aligned);
}



///////////////////////////////////////////////////////////////////////////////
// function set (2): Utilities
///////////////////////////////////////////////////////////////////////////////

template <class D>
inline void copy_d2(D *src1, D *src2, D *dest1, D *dest2) { *dest1 = *src1; }

template <class D>
inline void
copy_db2(D *src1, D *src2, D *dest1, D *dest2)
{
    *dest1 = *src1;
    *dest2 = *src2;
}

template <class D>
inline void
copy_dbyb2(D *src1, D *src2, D *dest1, D *dest2)
{
    if (tlm_bool<D>(*src2))
        *dest1 = *src1;
}

template <class D, void COPY(D *src1, D *src2, D *dest1, D *dest2)>
inline void
loop_aligned2(D *src1, D *src2, D *dest1, D *dest2, int words,
        int words_per_bus)
{
    // 64BITFIX was int and operands were cast to int.
    ptrdiff_t src1to2 = (char *)src2 - (char *)src1;
    // 64BITFIX was int and operands were cast to int.
    ptrdiff_t dest1to2 = (char *)dest2 - (char *)dest1;

    D *done = src1 + ptrdiff_t(words); // 64BITFIX.
    D *bus_start = src1;
    src1 += ptrdiff_t(words_per_bus - 1); // 64BITFIX.

    while (true) {
        COPY(src1, (D *)(src1to2 + (char *)src1), dest1,
                (D *)(dest1to2 + (char *)dest1)); // 64BITFIX.
        dest1++;
        if ((--src1) < bus_start) {
            bus_start += ptrdiff_t(words_per_bus); // 64BITFIX.
            if (bus_start == done)
                break;
            src1 = bus_start + ptrdiff_t(words_per_bus - 1); // 64BITFIX.
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
// function set (2): Response
///////////////////////////////////////////////////////////////////////////////

template <class DATAWORD>
inline void
tlm_from_hostendian_aligned(
        tlm_generic_payload *txn, unsigned int sizeof_databus)
{
    int words_per_bus = sizeof_databus / sizeof(DATAWORD);
    if (words_per_bus == 1)
        return;
    int words = (txn->get_data_length()) / sizeof(DATAWORD);
    tlm_endian_context *tc = txn->template get_extension<tlm_endian_context>();

    if (txn->get_byte_enable_ptr() == 0) {
        // no byte enables
        if (txn->is_read()) {
            // RD without byte enables. Copy data to original buffer.
            loop_aligned2<DATAWORD, &copy_d2<DATAWORD>>(
                    (DATAWORD *)(txn->get_data_ptr()), 0,
                    (DATAWORD *)(tc->data_ptr), 0, words, words_per_bus);
        }
    } else {
        // byte enables present
        if (txn->is_read()) {
            // RD with byte enables. Copy data qualified by byte-enables.
            loop_aligned2<DATAWORD, &copy_dbyb2<DATAWORD>>(
                    (DATAWORD *)(txn->get_data_ptr()),
                    (DATAWORD *)(txn->get_byte_enable_ptr()),
                    (DATAWORD *)(tc->data_ptr), 0, words, words_per_bus);
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
// function set (2): Request
///////////////////////////////////////////////////////////////////////////////

template <class DATAWORD>
inline void
tlm_to_hostendian_aligned(
        tlm_generic_payload *txn, unsigned int sizeof_databus)
{
    tlm_endian_context *tc = establish_context(txn);
    tc->from_f = &(tlm_from_hostendian_aligned<DATAWORD>);
    tc->sizeof_databus = sizeof_databus;

    int words_per_bus = sizeof_databus / sizeof(DATAWORD);
    if (words_per_bus == 1)
        return;
    int words = (txn->get_data_length()) / sizeof(DATAWORD);

    DATAWORD *original_be = (DATAWORD *)(txn->get_byte_enable_ptr());
    DATAWORD *original_data = (DATAWORD *)(txn->get_data_ptr());

    // Always allocate a new data buffer.
    tc->establish_dbuf(txn->get_data_length());
    txn->set_data_ptr(tc->new_dbuf);

    if (original_be == 0) {
        // No byte enables.
        if (txn->is_write()) {
            // WR no byte enables. Copy data.
            loop_aligned2<DATAWORD, &copy_d2<DATAWORD>>(
                    original_data, 0, (DATAWORD *)(txn->get_data_ptr()), 0,
                    words, words_per_bus);
        } else {
            // RD no byte enables. Save original data pointer.
            tc->data_ptr = (unsigned char *)original_data;
        }
    } else {
        // Byte enables present.
        // Allocate a new buffer for them.
        tc->establish_bebuf(txn->get_data_length());
        txn->set_byte_enable_ptr(tc->new_bebuf);
        txn->set_byte_enable_length(txn->get_data_length());

        if (txn->is_write()) {
            // WR with byte enables. Copy data and BEs.
            loop_aligned2<DATAWORD, &copy_db2<DATAWORD>>(
                    original_data, original_be,
                    (DATAWORD *)(txn->get_data_ptr()),
                    (DATAWORD *)(txn->get_byte_enable_ptr()),
                    words, words_per_bus);
        } else {
            // RD with byte enables. Save original data pointer.
            tc->data_ptr = (unsigned char *)original_data;
            // Copy byte enables to new buffer.
            loop_aligned2<DATAWORD, &copy_d2<DATAWORD>>(
                    original_be, 0, (DATAWORD *)(txn->get_byte_enable_ptr()),
                    0, words, words_per_bus);
        }
    }
}



///////////////////////////////////////////////////////////////////////////////
// function set (3): Response
///////////////////////////////////////////////////////////////////////////////

template <class DATAWORD>
inline void
tlm_from_hostendian_single(
        tlm_generic_payload *txn, unsigned int sizeof_databus)
{}


///////////////////////////////////////////////////////////////////////////////
// function set (3): Request
///////////////////////////////////////////////////////////////////////////////

template <class DATAWORD>
inline void
tlm_to_hostendian_single(tlm_generic_payload *txn, unsigned int sizeof_databus)
{
    tlm_endian_context *tc = establish_context(txn);
    tc->from_f = &(tlm_from_hostendian_single<DATAWORD>);
    tc->sizeof_databus = sizeof_databus;

    // Only need to change the address, always safe to work in-place.
    sc_dt::uint64 mask = sizeof_databus - 1;
    sc_dt::uint64 a = txn->get_address();
    txn->set_address((a & ~mask) |
            (sizeof_databus - (a & mask) - sizeof(DATAWORD)));
}



///////////////////////////////////////////////////////////////////////////////
// helper function which works for all responses
///////////////////////////////////////////////////////////////////////////////

inline void
tlm_from_hostendian(tlm_generic_payload *txn)
{
    tlm_endian_context *tc = txn->get_extension<tlm_endian_context>();
    (*(tc->from_f))(txn, tc->sizeof_databus);
}

}  // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_ENDIAN_CONV_HH__ */
