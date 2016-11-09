/*
 * Copyright (c) 2007-2008 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#ifndef __BASE_BITUNION_HH__
#define __BASE_BITUNION_HH__

#include "base/bitfield.hh"

//      The following implements the BitUnion system of defining bitfields
//on top of an underlying class. This is done through the pervasive use of
//both named and unnamed unions which all contain the same actual storage.
//Since they're unioned with each other, all of these storage locations
//overlap. This allows all of the bitfields to manipulate the same data
//without having to have access to each other. More details are provided with
//the individual components.

//This namespace is for classes which implement the backend of the BitUnion
//stuff. Don't use any of these directly, except for the Bitfield classes in
//the *BitfieldTypes class(es).
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
        getBits(int first, int last) const
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

    //This class contains all the "regular" bitfield classes. It is inherited
    //by all BitUnions which give them access to those types.
    template<class Type>
    class RegularBitfieldTypes
    {
      protected:
        //This class implements ordinary bitfields, that is a span of bits
        //who's msb is "first", and who's lsb is "last".
        template<int first, int last=first>
        class Bitfield : public BitfieldBase<Type>
        {
            static_assert(first >= last,
                          "Bitfield ranges must be specified as <msb, lsb>");

          public:
            operator uint64_t () const
            {
                return this->getBits(first, last);
            }

            uint64_t
            operator=(const uint64_t _data)
            {
                this->setBits(first, last, _data);
                return _data;
            }

            uint64_t
            operator=(Bitfield<first, last> const & other)
            {
                return *this = (uint64_t)other;
            }
        };

        //A class which specializes the above so that it can only be read
        //from. This is accomplished explicitly making sure the assignment
        //operator is blocked. The conversion operator is carried through
        //inheritance. This will unfortunately need to be copied into each
        //bitfield type due to limitations with how templates work
        template<int first, int last=first>
        class BitfieldRO : public Bitfield<first, last>
        {
          private:
            uint64_t
            operator=(const uint64_t _data);

            uint64_t
            operator=(const Bitfield<first, last>& other);
        };

        //Similar to the above, but only allows writing.
        template<int first, int last=first>
        class BitfieldWO : public Bitfield<first, last>
        {
          private:
            operator uint64_t () const;

          public:
            using Bitfield<first, last>::operator=;
        };
    };

    //This class contains all the "regular" bitfield classes. It is inherited
    //by all BitUnions which give them access to those types.
    template<class Type>
    class SignedBitfieldTypes
    {
      protected:
        //This class implements ordinary bitfields, that is a span of bits
        //who's msb is "first", and who's lsb is "last".
        template<int first, int last=first>
        class SignedBitfield : public BitfieldBase<Type>
        {
          public:
            operator int64_t () const
            {
                return sext<first - last + 1>(this->getBits(first, last));
            }

            int64_t
            operator=(const int64_t _data)
            {
                this->setBits(first, last, _data);
                return _data;
            }

            int64_t
            operator=(SignedBitfield<first, last> const & other)
            {
                return *this = (int64_t)other;
            }
        };

        //A class which specializes the above so that it can only be read
        //from. This is accomplished explicitly making sure the assignment
        //operator is blocked. The conversion operator is carried through
        //inheritance. This will unfortunately need to be copied into each
        //bitfield type due to limitations with how templates work
        template<int first, int last=first>
        class SignedBitfieldRO : public SignedBitfield<first, last>
        {
          private:
            int64_t
            operator=(const int64_t _data);

            int64_t
            operator=(const SignedBitfield<first, last>& other);
        };

        //Similar to the above, but only allows writing.
        template<int first, int last=first>
        class SignedBitfieldWO : public SignedBitfield<first, last>
        {
          private:
            operator int64_t () const;

          public:
            using SignedBitfield<first, last>::operator=;
        };
    };

    template<class Type>
    class BitfieldTypes : public RegularBitfieldTypes<Type>,
                          public SignedBitfieldTypes<Type>
    {};

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
        BitUnionOperators(Type const & _data)
        {
            Base::__data = _data;
        }

        BitUnionOperators() {}

        operator const Type () const
        {
            return Base::__data;
        }

        Type
        operator=(Type const & _data)
        {
            Base::__data = _data;
            return _data;
        }

        Type
        operator=(BitUnionOperators const & other)
        {
            Base::__data = other;
            return Base::__data;
        }

        bool
        operator<(Base const & base) const
        {
            return Base::__data < base.__data;
        }

        bool
        operator==(Base const & base) const
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
    class BitfieldUnderlyingClasses##name : \
        public BitfieldBackend::BitfieldTypes<type> \
    { \
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
        BitfieldUnderlyingClasses##name::__DataType, \
        BitfieldUnderlyingClasses##name> name;

//This sets up a bitfield which has other bitfields nested inside of it. The
//__data member functions like the "underlying storage" of the top level
//BitUnion. Like everything else, it overlays with the top level storage, so
//making it a regular bitfield type makes the entire thing function as a
//regular bitfield when referred to by itself.
#define __SubBitUnion(fieldType, first, last, name) \
    class : public BitfieldBackend::BitfieldTypes<__DataType> \
    { \
      public: \
        union { \
            fieldType<first, last> __data;

//This closes off the union created above and gives it a name. Unlike the top
//level BitUnion, we're interested in creating an object instead of a type.
//The operators are defined in the macro itself instead of a class for
//technical reasons. If someone determines a way to move them to one, please
//do so.
#define EndSubBitUnion(name) \
        }; \
        inline operator __DataType () const \
        { return __data; } \
        \
        inline __DataType operator = (const __DataType & _data) \
        { return __data = _data;} \
    } name;

//Regular bitfields
//These define macros for read/write regular bitfield based subbitfields.
#define SubBitUnion(name, first, last) \
    __SubBitUnion(Bitfield, first, last, name)

//Regular bitfields
//These define macros for read/write regular bitfield based subbitfields.
#define SignedSubBitUnion(name, first, last) \
    __SubBitUnion(SignedBitfield, first, last, name)

//Use this to define an arbitrary type overlayed with bitfields.
#define BitUnion(type, name) __BitUnion(type, name)

//Use this to define conveniently sized values overlayed with bitfields.
#define BitUnion64(name) __BitUnion(uint64_t, name)
#define BitUnion32(name) __BitUnion(uint32_t, name)
#define BitUnion16(name) __BitUnion(uint16_t, name)
#define BitUnion8(name) __BitUnion(uint8_t, name)

#endif // __BASE_BITUNION_HH__
