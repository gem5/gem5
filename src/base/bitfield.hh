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

//	The following implements the BitUnion system of defining bitfields
//on top of an underlying class. This is done through the extensive use of
//both named and unnamed unions which all contain the same actual storage.
//Since they're unioned with each other, all of these storage locations
//overlap. This allows all of the bitfields to manipulate the same data
//without having to know about each other. More details are provided with the
//individual components.

//This namespace is for classes which implement the backend of the BitUnion
//stuff. Don't use any of this directly! Use the macros at the end instead.
namespace BitfieldBackend
{
    //A base class for all bitfields. It instantiates the actual storage,
    //and provides getBits and setBits functions for manipulating it. The
    //Data template parameter is type of the underlying storage.
    template<class Data>
    class BitfieldBase
    {
      protected:
        Data __data;

        //This function returns a range of bits from the underlying storage.
        //It relies on the "bits" function above. It's the user's
        //responsibility to make sure that there is a properly overloaded
        //version of this function for whatever type they want to overlay.
        inline uint64_t
        getBits(int first, int last)
        {
            return bits(__data, first, last);
        }

        //Similar to the above, but for settings bits with replaceBits.
        inline void
        setBits(int first, int last, uint64_t val)
        {
            replaceBits(__data, first, last, val);
        }
    };

    //A class which specializes a given base so that it can only be read
    //from. This is accomplished by only passing through the conversion
    //operator and explicitly making sure the assignment operator is blocked.
    template<class Type, class Base>
    class _BitfieldRO : public Base
    {
      private:
        const Type
        operator=(const Type & _data);

      public:
        operator const Type ()
        {
            return *((Base *)this);
        }
    };

    //Similar to the above, but only allows writing.
    template<class Type, class Base>
    class _BitfieldWO : public Base
    {
      private:
        operator const Type ();

      public:
        const Type operator=(const Type & _data)
        {
            *((Base *)this) = _data;
            return _data;
        }
    };

    //This class implements ordinary bitfields, that is a span of bits
    //who's msb is "first", and who's lsb is "last".
    template<class Data, int first, int last=first>
    class _Bitfield : public BitfieldBase<Data>
    {
      public:
        operator const Data ()
        {
            return this->getBits(first, last);
        }

        const Data
        operator=(const Data & _data)
        {
            this->setBits(first, last, _data);
            return _data;
        }
    };

    //When a BitUnion is set up, an underlying class is created which holds
    //the actual union. This class then inherits from it, and provids the
    //implementations for various operators. Setting things up this way
    //prevents having to redefine these functions in every different BitUnion
    //type. More operators could be implemented in the future, as the need
    //arises.
    template <class Type, class Base>
    class BitUnionOperators : public Base
    {
      public:
        operator const Type ()
        {
            return Base::__data;
        }

        const Type
        operator=(const Type & _data)
        {
            Base::__data = _data;
        }

        bool
        operator<(const Base & base)
        {
            return Base::__data < base.__data;
        }

        bool
        operator==(const Base & base)
        {
            return Base::__data == base.__data;
        }
    };
}

//This macro is a backend for other macros that specialize it slightly.
//First, it creates/extends a namespace "BitfieldUnderlyingClasses" and
//sticks the class which has the actual union in it, which
//BitfieldOperators above inherits from. Putting these classes in a special
//namespace ensures that there will be no collisions with other names as long
//as the BitUnion names themselves are all distinct and nothing else uses
//the BitfieldUnderlyingClasses namespace, which is unlikely. The class itself
//creates a typedef of the "type" parameter called __DataType. This allows
//the type to propagate outside of the macro itself in a controlled way.
//Finally, the base storage is defined which BitfieldOperators will refer to
//in the operators it defines. This macro is intended to be followed by
//bitfield definitions which will end up inside it's union. As explained
//above, these is overlayed the __data member in its entirety by each of the
//bitfields which are defined in the union, creating shared storage with no
//overhead.
#define __BitUnion(type, name) \
    namespace BitfieldUnderlyingClasses \
    { \
        class name; \
    } \
    class BitfieldUnderlyingClasses::name { \
      public: \
        typedef type __DataType; \
        union { \
            type __data;\

//This closes off the class and union started by the above macro. It is
//followed by a typedef which makes "name" refer to a BitfieldOperator
//class inheriting from the class and union just defined, which completes
//building up the type for the user.
#define EndBitUnion(name) \
        }; \
    }; \
    typedef BitfieldBackend::BitUnionOperators< \
        BitfieldUnderlyingClasses::name::__DataType, \
        BitfieldUnderlyingClasses::name> name;

//This sets up a bitfield which has other bitfields nested inside of it. The
//__data member functions like the "underlying storage" of the top level
//BitUnion. Like everything else, it overlays with the top level storage, so
//making it a regular bitfield type makes the entire thing function as a
//regular bitfield when referred to by itself. The operators are defined in
//the macro itself instead of a class for technical reasons. If someone
//determines a way to move them to one, please do so.
#define __SubBitUnion(type, name) \
        union { \
            type __data; \
            inline operator const __DataType () \
            { return __data; } \
            \
            inline const __DataType operator = (const __DataType & _data) \
            { __data = _data; }

//This closes off the union created above and gives it a name. Unlike the top
//level BitUnion, we're interested in creating an object instead of a type.
#define EndSubBitUnion(name) } name;

//The preprocessor will treat everything inside of parenthesis as a single
//argument even if it has commas in it. This is used to pass in templated
//classes which typically have commas to seperate their parameters.
#define wrap(guts) guts

//Read only bitfields
//This wraps another bitfield class inside a _BitfieldRO class using
//inheritance. As explained above, the _BitfieldRO class only passes through
//the conversion operator, so the underlying bitfield can then only be read
//from.
#define __BitfieldRO(base) \
    BitfieldBackend::_BitfieldRO<__DataType, base>
#define __SubBitUnionRO(name, base) \
    __SubBitUnion(wrap(_BitfieldRO<__DataType, base>), name)

//Write only bitfields
//Similar to above, but for making write only versions of bitfields with
//_BitfieldWO.
#define __BitfieldWO(base) \
    BitfieldBackend::_BitfieldWO<__DataType, base>
#define __SubBitUnionWO(name, base) \
    __SubBitUnion(wrap(_BitfieldWO<__DataType, base>), name)

//Regular bitfields
//This uses all of the above to define macros for read/write, read only, and
//write only versions of regular bitfields.
#define Bitfield(first, last) \
    BitfieldBackend::_Bitfield<__DataType, first, last>
#define SubBitUnion(name, first, last) \
    __SubBitUnion(Bitfield(first, last), name)
#define BitfieldRO(first, last) __BitfieldRO(Bitfield(first, last))
#define SubBitUnionRO(name, first, last) \
    __SubBitUnionRO(Bitfield(first, last), name)
#define BitfieldWO(first, last) __BitfieldWO(Bitfield(first, last))
#define SubBitUnionWO(name, first, last) \
    __SubBitUnionWO(Bitfield(first, last), name)

//Use this to define an arbitrary type overlayed with bitfields.
#define BitUnion(type, name) __BitUnion(type, name)

//Use this to define conveniently sized values overlayed with bitfields.
#define BitUnion64(name) __BitUnion(uint64_t, name)
#define BitUnion32(name) __BitUnion(uint32_t, name)
#define BitUnion16(name) __BitUnion(uint16_t, name)
#define BitUnion8(name) __BitUnion(uint8_t, name)

#endif // __BASE_BITFIELD_HH__
