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

/*****************************************************************************

  sc_bv_base.h -- Arbitrary size bit vector class.

  Original Author: Gene Bushuyev, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_bv_base.h,v $
// Revision 1.3  2011/08/26 22:32:00  acg
//  Torsten Maehne: added parentheses to make opearator ordering more obvious.
//
// Revision 1.2  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef __SYSTEMC_EXT_DT_BIT_SC_BV_BASE_HH__
#define __SYSTEMC_EXT_DT_BIT_SC_BV_BASE_HH__

#include "../int/sc_length_param.hh"
#include "messages.hh"
#include "sc_bit_proxies.hh"
#include "sc_proxy.hh"

namespace sc_dt
{

// classes defined in this module
class sc_bv_base;


// ----------------------------------------------------------------------------
//  CLASS : sc_bv_base
//
//  Arbitrary size bit vector base class.
// ----------------------------------------------------------------------------

class sc_bv_base : public sc_proxy<sc_bv_base>
{
    friend class sc_lv_base;

    void init(int length_, bool init_value=false);
    void assign_from_string(const std::string &);

  public:
    // typedefs
    typedef sc_proxy<sc_bv_base> base_type;
    typedef base_type::value_type value_type;

    // constructors
    explicit sc_bv_base(int length_=sc_length_param().len()) :
        m_len(0), m_size(0), m_data(0)
    {
        init(length_);
    }

    explicit sc_bv_base(bool a, int length_=sc_length_param().len()) :
        m_len(0), m_size(0), m_data(0)
    {
        init(length_, a);
    }

    sc_bv_base(const char *a);
    sc_bv_base(const char *a, int length_);

    template <class X>
    sc_bv_base(const sc_proxy<X> &a) : m_len(0), m_size(0), m_data(0)
    {
        init(a.back_cast().length());
        base_type::assign_(a);
    }

    sc_bv_base(const sc_bv_base &a);

    // destructor
    virtual ~sc_bv_base() { delete [] m_data; }

    // assignment operators
    template <class X>
    sc_bv_base &
    operator = (const sc_proxy<X> &a)
    {
        assign_p_(*this, a);
        return *this;
    }

    sc_bv_base &
    operator = (const sc_bv_base &a)
    {
        assign_p_(*this, a);
        return *this;
    }

    sc_bv_base &operator = (const char *a);

    sc_bv_base &
    operator = (const bool *a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (const sc_logic *a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (const sc_unsigned &a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (const sc_signed &a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (const sc_uint_base &a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (const sc_int_base &a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (unsigned long a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (long a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (unsigned int a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (int a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (uint64 a)
    {
        base_type::assign_(a);
        return *this;
    }

    sc_bv_base &
    operator = (int64 a)
    {
        base_type::assign_(a);
        return *this;
    }

    // common methods
    int length() const { return m_len; }
    int size() const { return m_size; }

    value_type get_bit(int i) const;
    void set_bit(int i, value_type value);

    sc_digit get_word(int i) const { return m_data[i]; }

    void set_word(int i, sc_digit w) { m_data[i] = w; }

    sc_digit get_cword(int /*i*/) const { return SC_DIGIT_ZERO; }

    void set_cword(int i, sc_digit w);

    void clean_tail();

    // other methods
    bool is_01() const { return true; }

  protected:
    int m_len; // length in bits
    int m_size; // size of data array
    sc_digit *m_data; // data array
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// common methods
inline sc_bv_base::value_type
sc_bv_base::get_bit(int i) const
{
    int wi = i / SC_DIGIT_SIZE;
    int bi = i % SC_DIGIT_SIZE;
    return value_type((m_data[wi] >> bi) & SC_DIGIT_ONE);
}

inline void
sc_bv_base::set_bit(int i, value_type value)
{
    int wi = i / SC_DIGIT_SIZE;
    int bi = i % SC_DIGIT_SIZE;
    sc_digit mask = SC_DIGIT_ONE << bi;
    m_data[wi] |= mask; // set bit to 1
    m_data[wi] &= value << bi | ~mask;
}

inline void
sc_bv_base::set_cword(int /*i*/, sc_digit w)
{
    if (w) {
        SC_REPORT_WARNING(sc_core::SC_ID_SC_BV_CANNOT_CONTAIN_X_AND_Z_, 0);
    }
}

inline void
sc_bv_base::clean_tail()
{
    int wi = m_size - 1;
    int bi = m_len % SC_DIGIT_SIZE;
    if (bi != 0)
        m_data[wi] &= ~SC_DIGIT_ZERO >> (SC_DIGIT_SIZE - bi);
}

} // namespace sc_dt

#endif // __SYSTEMC_EXT_DT_BIT_SC_BV_BASE_HH__
