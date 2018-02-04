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

#include <functional>
#include <type_traits>

#include "base/bitfield.hh"

//      The following implements the BitUnion system of defining bitfields
//on top of an underlying class. This is done through the pervasive use of
//both named and unnamed unions which all contain the same actual storage.
//Since they're unioned with each other, all of these storage locations
//overlap. This allows all of the bitfields to manipulate the same data
//without having to have access to each other. More details are provided with
//the individual components.

//This class wraps around another which defines getter/setter functions which
//manipulate the underlying data. The type of the underlying data and the type
//of the bitfield itself are inferred from the argument types of the setter
//function.
template<class Base>
class BitfieldTypeImpl : public Base
{
    static_assert(std::is_empty<Base>::value,
                  "Bitfield base class must be empty.");

  private:

    struct TypeDeducer
    {
        template<typename>
        struct T;

        template<typename C, typename Type1, typename Type2>
        struct T<void (C::*)(Type1 &, Type2)>
        {
            typedef Type1 Storage;
            typedef Type2 Type;
        };

        struct Wrapper : public Base
        {
            using Base::setter;
        };

        typedef typename T<decltype(&Wrapper::setter)>::Storage Storage;
        typedef typename T<decltype(&Wrapper::setter)>::Type Type;
    };

  protected:
    typedef typename TypeDeducer::Storage Storage;
    typedef typename TypeDeducer::Type Type;

    Type getter(const Storage &storage) const = delete;
    void setter(Storage &storage, Type val) = delete;

    Storage __storage;

    operator Type () const
    {
        return Base::getter(__storage);
    }

    Type
    operator=(const Type val)
    {
        Base::setter(__storage, val);
        return val;
    }

    Type
    operator=(BitfieldTypeImpl<Base> const & other)
    {
        return *this = (Type)other;
    }
};

//A wrapper for the above class which allows setting and getting.
template<class Base>
class BitfieldType : public BitfieldTypeImpl<Base>
{
  protected:
    using Impl = BitfieldTypeImpl<Base>;
    using typename Impl::Type;

  public:
    operator Type () const { return Impl::operator Type(); }
    Type operator=(const Type val) { return Impl::operator=(val); }
    Type
    operator=(BitfieldType<Base> const & other)
    {
        return Impl::operator=(other);
    }
};

//A wrapper which only supports getting.
template<class Base>
class BitfieldROType : public BitfieldTypeImpl<Base>
{
  public:
    using Impl = BitfieldTypeImpl<Base>;
    using typename Impl::Type;

    Type operator=(BitfieldROType<Base> const &other) = delete;
    operator Type () const { return Impl::operator Type(); }
};

//A wrapper which only supports setting.
template <class Base>
class BitfieldWOType : public BitfieldTypeImpl<Base>
{
  protected:
    using Impl = BitfieldTypeImpl<Base>;
    using typename Impl::Type;

  public:
    Type operator=(const Type val) { return Impl::operator=(val); }
    Type
    operator=(BitfieldWOType<Base> const & other)
    {
        return Impl::operator=(other);
    }
};

//This namespace is for classes which implement the backend of the BitUnion
//stuff. Don't use any of these directly.
namespace BitfieldBackend
{
    template<class Storage, int first, int last>
    class Unsigned
    {
        static_assert(first >= last,
                      "Bitfield ranges must be specified as <msb, lsb>");

      protected:
        uint64_t
        getter(const Storage &storage) const
        {
            return bits(storage, first, last);
        }

        void
        setter(Storage &storage, uint64_t val)
        {
            replaceBits(storage, first, last, val);
        }
    };

    template<class Storage, int first, int last>
    class Signed
    {
        static_assert(first >= last,
                      "Bitfield ranges must be specified as <msb, lsb>");

      protected:
        int64_t
        getter(const Storage &storage) const
        {
            return sext<first - last + 1>(bits(storage, first, last));
        }

        void
        setter(Storage &storage, int64_t val)
        {
            replaceBits(storage, first, last, val);
        }
    };

    //This class contains the basic bitfield types which are automatically
    //available within a BitUnion. They inherit their Storage type from the
    //containing BitUnion.
    template<class Storage>
    class BitfieldTypes
    {
      protected:

        template<int first, int last=first>
        using Bitfield = BitfieldType<Unsigned<Storage, first, last> >;
        template<int first, int last=first>
        using BitfieldRO =
                BitfieldROType<Unsigned<Storage, first, last> >;
        template<int first, int last=first>
        using BitfieldWO =
                BitfieldWOType<Unsigned<Storage, first, last> >;

        template<int first, int last=first>
        using SignedBitfield =
                BitfieldType<Signed<Storage, first, last> >;
        template<int first, int last=first>
        using SignedBitfieldRO =
                BitfieldROType<Signed<Storage, first, last> >;
        template<int first, int last=first>
        using SignedBitfieldWO =
                BitfieldWOType<Signed<Storage, first, last> >;
    };

    //When a BitUnion is set up, an underlying class is created which holds
    //the actual union. This class then inherits from it, and provids the
    //implementations for various operators. Setting things up this way
    //prevents having to redefine these functions in every different BitUnion
    //type. More operators could be implemented in the future, as the need
    //arises.
    template <class Base>
    class BitUnionOperators : public Base
    {
        static_assert(sizeof(Base) == sizeof(typename Base::__StorageType),
                      "BitUnion larger than its storage type.");

      public:
        BitUnionOperators(typename Base::__StorageType const &val)
        {
            Base::__storage = val;
        }

        BitUnionOperators() {}

        operator const typename Base::__StorageType () const
        {
            return Base::__storage;
        }

        typename Base::__StorageType
        operator=(typename Base::__StorageType const &val)
        {
            Base::__storage = val;
            return val;
        }

        typename Base::__StorageType
        operator=(BitUnionOperators const &other)
        {
            Base::__storage = other;
            return Base::__storage;
        }

        bool
        operator<(Base const &base) const
        {
            return Base::__storage < base.__storage;
        }

        bool
        operator==(Base const &base) const
        {
            return Base::__storage == base.__storage;
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
//creates a typedef of the "type" parameter called __StorageType. This allows
//the type to propagate outside of the macro itself in a controlled way.
//Finally, the base storage is defined which BitfieldOperators will refer to
//in the operators it defines. This macro is intended to be followed by
//bitfield definitions which will end up inside it's union. As explained
//above, these is overlayed the __storage member in its entirety by each of the
//bitfields which are defined in the union, creating shared storage with no
//overhead.
#define __BitUnion(type, name) \
    class BitfieldUnderlyingClasses##name : \
        public BitfieldBackend::BitfieldTypes<type> \
    { \
      protected: \
        typedef type __StorageType; \
        friend BitfieldBackend::BitUnionBaseType< \
            BitfieldBackend::BitUnionOperators< \
                BitfieldUnderlyingClasses##name> >; \
        friend BitfieldBackend::BitUnionBaseType< \
                BitfieldUnderlyingClasses##name>; \
      public: \
        union { \
            type __storage;

//This closes off the class and union started by the above macro. It is
//followed by a typedef which makes "name" refer to a BitfieldOperator
//class inheriting from the class and union just defined, which completes
//building up the type for the user.
#define EndBitUnion(name) \
        }; \
    }; \
    typedef BitfieldBackend::BitUnionOperators< \
        BitfieldUnderlyingClasses##name> name;

//This sets up a bitfield which has other bitfields nested inside of it. The
//__storage member functions like the "underlying storage" of the top level
//BitUnion. Like everything else, it overlays with the top level storage, so
//making it a regular bitfield type makes the entire thing function as a
//regular bitfield when referred to by itself.
#define __SubBitUnion(name, fieldType, ...) \
    class \
    { \
      public: \
        union { \
            fieldType<__VA_ARGS__> __storage;

//This closes off the union created above and gives it a name. Unlike the top
//level BitUnion, we're interested in creating an object instead of a type.
//The operators are defined in the macro itself instead of a class for
//technical reasons. If someone determines a way to move them to one, please
//do so.
#define EndSubBitUnion(name) \
        }; \
        inline operator __StorageType () const \
        { return __storage; } \
        \
        inline __StorageType operator = (const __StorageType & _storage) \
        { return __storage = _storage;} \
    } name;

//Regular bitfields
//These define macros for read/write regular bitfield based subbitfields.
#define SubBitUnion(name, first, last) \
    __SubBitUnion(name, Bitfield, first, last)

//Regular bitfields
//These define macros for read/write regular bitfield based subbitfields.
#define SignedSubBitUnion(name, first, last) \
    __SubBitUnion(name, SignedBitfield, first, last)

//Use this to define an arbitrary type overlayed with bitfields.
#define BitUnion(type, name) __BitUnion(type, name)

//Use this to define conveniently sized values overlayed with bitfields.
#define BitUnion64(name) __BitUnion(uint64_t, name)
#define BitUnion32(name) __BitUnion(uint32_t, name)
#define BitUnion16(name) __BitUnion(uint16_t, name)
#define BitUnion8(name) __BitUnion(uint8_t, name)


//These templates make it possible to define other templates related to
//BitUnions without having to refer to internal typedefs or the BitfieldBackend
//namespace.

//To build a template specialization which works for all BitUnions, accept a
//template argument T, and then use BitUnionType<T> as an argument in the
//template. To refer to the basic type the BitUnion wraps, use
//BitUnionBaseType<T>.

//For example:
//template <typename T>
//void func(BitUnionType<T> u) { BitUnionBaseType<T> b = u; }

//Also, BitUnionBaseType can be used on a BitUnion type directly.

template <typename T>
using BitUnionType = BitfieldBackend::BitUnionOperators<T>;

namespace BitfieldBackend
{
    template<typename T>
    struct BitUnionBaseType
    {
        typedef typename BitUnionType<T>::__StorageType Type;
    };

    template<typename T>
    struct BitUnionBaseType<BitUnionType<T> >
    {
        typedef typename BitUnionType<T>::__StorageType Type;
    };
}

template <typename T>
using BitUnionBaseType = typename BitfieldBackend::BitUnionBaseType<T>::Type;


//An STL style hash structure for hashing BitUnions based on their base type.
namespace std
{
    template <typename T>
    struct hash<BitUnionType<T> > : public hash<BitUnionBaseType<T> >
    {
        size_t
        operator() (const BitUnionType<T> &val) const
        {
            return hash<BitUnionBaseType<T> >::operator()(val);
        }
    };
}

#endif // __BASE_BITUNION_HH__
