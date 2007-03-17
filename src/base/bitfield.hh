/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __BASE_BITFIELD_HH__
#define __BASE_BITFIELD_HH__

#include <inttypes.h>
//#include "base/misc.hh"

/**
 * Generate a 64-bit mask of 'nbits' 1s, right justified.
 */
inline uint64_t
mask(int nbits)
{
    return (nbits == 64) ? (uint64_t)-1LL : (1ULL << nbits) - 1;
}



/**
 * Extract the bitfield from position 'first' to 'last' (inclusive)
 * from 'val' and right justify it.  MSB is numbered 63, LSB is 0.
 */
template <class T>
inline
T
bits(T val, int first, int last)
{
    int nbits = first - last + 1;
    return (val >> last) & mask(nbits);
}

/**
 * Mask off the given bits in place like bits() but without shifting.
 * msb = 63, lsb = 0
 */
template <class T>
inline
T
mbits(T val, int first, int last)
{
    return val & (mask(first+1) & ~mask(last));
}

inline uint64_t
mask(int first, int last)
{
    return mbits((uint64_t)-1LL, first, last);
}

/**
 * Sign-extend an N-bit value to 64 bits.
 */
template <int N>
inline
int64_t
sext(uint64_t val)
{
    int sign_bit = bits(val, N-1, N-1);
    return sign_bit ? (val | ~mask(N)) : val;
}

/**
 * Return val with bits first to last set to bit_val
 */
template <class T, class B>
inline
T
insertBits(T val, int first, int last, B bit_val)
{
    T bmask = mask(first - last + 1) << last;
    return ((bit_val << last) & bmask) | (val & ~bmask);
}

/**
 * A convenience function to replace bits first to last of val with bit_val
 * in place.
 */
template <class T, class B>
inline
void
replaceBits(T& val, int first, int last, B bit_val)
{
    val = insertBits(val, first, last, bit_val);
}

/**
 * Returns the bit position of the MSB that is set in the input
 */
inline
int
findMsbSet(uint64_t val) {
    int msb = 0;
    if (!val)
        return 0;
    if (bits(val, 63,32)) { msb += 32; val >>= 32; }
    if (bits(val, 31,16)) { msb += 16; val >>= 16; }
    if (bits(val, 15,8))  { msb += 8;  val >>= 8;  }
    if (bits(val, 7,4))   { msb += 4;  val >>= 4;  }
    if (bits(val, 3,2))   { msb += 2;  val >>= 2;  }
    if (bits(val, 1,1))   { msb += 1; }
    return msb;
}

template<class Data, int first, int last=first>
class BitfieldBase
{
  protected:
    uint8_t __data[sizeof(Data)];

    //These are defined here so it can be specialized for Data, but can be
    //hidden by RO and WO variants.
    inline uint64_t getBits(int _first, int _last)
    {
        //build up the right bits from the byte array "data"
        //panic("Not yet implemented.\n");
        return 0;
    }

    inline void setBits(int _first, int _last, uint64_t val)
    {
        //Set the right bits from the byte array "data"
        //panic("Not yet implemented.\n");
    }
};

template<class Data, int first, int last=first>
class BitfieldNativeBase
{
  protected:
    Data __data;

    inline uint64_t getBits(int _first, int _last)
    {
        return bits(__data, first, last);
    }

    inline void setBits(int _first, int _last, uint64_t val)
    {
        replaceBits(__data, first, last, val);
    }
};

template <int first>
class BitfieldBase<uint64_t, first, first> :
        public BitfieldNativeBase<uint64_t, first, first>
{};

template <int first, int last>
class BitfieldBase<uint64_t, first, last> :
        public BitfieldNativeBase<uint64_t, first, last>
{};

template <int first>
class BitfieldBase<uint32_t, first, first> :
        public BitfieldNativeBase<uint32_t, first, first>
{};

template <int first, int last>
class BitfieldBase<uint32_t, first, last> :
        public BitfieldNativeBase<uint32_t, first, last>
{};

template <int first>
class BitfieldBase<uint16_t, first, first> :
        public BitfieldNativeBase<uint16_t, first, first>
{};

template <int first, int last>
class BitfieldBase<uint16_t, first, last> :
        public BitfieldNativeBase<uint16_t, first, last>
{};

template <int first>
class BitfieldBase<uint8_t, first, first> :
        public BitfieldNativeBase<uint8_t, first, first>
{};

template <int first, int last>
class BitfieldBase<uint8_t, first, last> :
        public BitfieldNativeBase<uint8_t, first, last>
{};

template<class Data, int first, int last=first>
class _BitfieldRO : public BitfieldBase<Data, first, last>
{
  public:
    operator const Data & ()
    {
        return this->getBits(first, last);
    }
};

template<class Data, int first, int last=first>
class _BitfieldWO : public BitfieldBase<Data, first, last>
{
  public:
    const Data & operator = (const Data & _data)
    {
        this->setBits(first, last, _data);
        return *this;
    }
};

template<class Data, int first, int last=first>
class _BitfieldRW : public BitfieldBase<Data, first, last>
{
  public:
    operator const Data ()
    {
        return this->getBits(first, last);
    }

    const Data operator = (const Data & _data)
    {
        this->setBits(first, last, _data);
        return *this;
    }
};

template <class Type, class Base>
class BitUnionOperators : public Base
{
  public:
    operator const Type ()
    {
        return Base::__data;
    }

    const Type operator = (const Type & _data)
    {
        Base::__data = _data;
    }

    bool operator < (const Base & base)
    {
        return Base::__data < base.__data;
    }

    bool operator == (const Base & base)
    {
        return Base::__data == base.__data;
    }
};

#define __BitUnion(type, name) \
    class __##name { \
      public: \
        typedef type __DataType; \
        union { \
            type __data;\

#define EndBitUnion(name) \
        }; \
    }; \
    typedef BitUnionOperators<__##name::__DataType, __##name> name;

#define __SubBitUnion(type, name) \
        union { \
            type __data; \
            inline operator const __DataType () \
            { return __data; } \
            \
            inline const __DataType operator = (const __DataType & _data) \
            { __data = _data; }

#define EndSubBitUnion(name) } name;

//Regular read/write bitfields
#define BitfieldRW(first, last) _BitfieldRW<__DataType, first, last>
#define SubBitUnionRW(name, first, last) \
        __SubBitUnion(BitfieldRW(first, last), name)
#define Bitfield(first, last) BitfieldRW(first, last)
#define SubBitUnion(name, first, last) SubBitUnionRW(name, first, last)

//Read only bitfields
#define BitfieldRO(first, last) _BitfieldRO<__DataType, first, last>
#define SubBitUnionRO(name, first, last) \
        __SubBitUnion(BitfieldRO(first, last), name)

//Write only bitfields
#define BitfieldWO(first, last) _BitfieldWO<__DataType, first, last>
#define SubBitUnionWO(name, first, last) \
        __SubBitUnion(BitfieldWO(first, last), name)

#define BitUnion(type, name) __BitUnion(type, name)

#define BitUnion64(name) __BitUnion(uint64_t, name)
#define BitUnion32(name) __BitUnion(uint32_t, name)
#define BitUnion16(name) __BitUnion(uint16_t, name)
#define BitUnion8(name) __BitUnion(uint8_t, name)

#endif // __BASE_BITFIELD_HH__
