/*
 * Copyright 2020 Google, Inc.
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

#ifndef __DEV_REG_BANK_HH__
#define __DEV_REG_BANK_HH__

#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <utility>

#include "base/bitfield.hh"
#include "base/debug.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "sim/byteswap.hh"
#include "sim/serialize_handlers.hh"

/*
 * Device models often have contiguous banks of registers which can each
 * have unique and arbitrary behavior when they are completely or partially
 * read or written. Historically it's been up to each model to map an access
 * which covers an arbitrary portion of that register bank down to individual
 * registers. It must handle cases where registers are only partially accessed,
 * or where multiple registers are accessed at the same time, or a combination
 * of both.
 *
 *
 * == RegisterBank ==
 *
 * The RegisterBank class(es), defined below, handle that mapping, and let the
 * device model focus on defining what each of the registers actually do when
 * read or written. Once it's set up, it has two primary interfaces which
 * access the registers it contains:
 *
 * void read(Addr addr, void *buf, Addr bytes);
 * void write(Addr addr, const void *buf, Addr bytes);
 *
 * These two methods will handle a read or write contained within the register
 * bank starting at address "addr". The data that will be written or has been
 * read is pointed to by "buf", and is "bytes" bytes long.
 *
 * These methods are virtual, so if you need to implement extra rules, like
 * for instance that registers can only be accessed one at a time, that
 * accesses have to be aligned, have to access complete registers, etc, that
 * can be added in a subclass.
 *
 * Additionally, each RegisterBank has a name and a base address which is
 * passed into the constructor. The meaning of the "base" value can be whatever
 * makes sense for your device, and is considered the lowest address contained
 * in the bank. The value could be the offset of this bank of registers within
 * the device itself, with the device's own offset subtracted out before read
 * or write are called. It could alternatively be the base address of the
 * entire device, with the address from accesses passed into read or write
 * unmodified.
 *
 * The base(), size() and name() methods can be used to access each of those
 * read only properties of the RegisterBank instance.
 *
 * To add actual registers to the RegisterBank (discussed below), you can use
 * either the addRegister method which adds a single register, or addRegisters
 * which adds an initializer list of them all at once. The register will be
 * appended to the end of the bank as they're added, contiguous to the
 * existing registers. The size of the bank is automatically accumulated as
 * registers are added.
 *
 * When adding a lot of registers, you might accidentally add an extra,
 * or accidentally skip one in a long list. Because the offset is handled
 * automatically, some of your registers might end up shifted higher or lower
 * than you expect. To help mitigate this, you can set what offset you expect
 * a register to have by specifying it as an offset, register pair.
 *
 * addRegisters({{0x1000, reg0}, reg1, reg2});
 *
 * If the register would end up at a different offset, gem5 will panic. You
 * can also leave off the register if you want to just check the offset, for
 * instance between groups of registers.
 *
 * addRegisters({reg0, reg1, reg2, 0x100c})
 *
 * While the RegisterBank itself doesn't have any data in it directly and so
 * has no endianness, it's very likely all the registers within it will have
 * the same endinanness. The bank itself therefore has a default endianness
 * which, unless specified otherwise, will be passed on to the register types
 * within it. The RegisterBank class is templated on its endianness. There are
 * RegisterBankLE and RegisterBankBE aliases to make it a little easier to
 * refer to one or the other version.
 *
 * A RegisterBank also has a reset() method which will (by default) call the
 * reset() method on each register within it. This method is virtual, and so
 * can be overridden if something additional or different needs to be done to
 * reset the hardware model.
 *
 *
 * == Register interface ==
 *
 * Every register in a RegisterBank needs to inherit, directly or indirectly,
 * from the RegisterBase class. Each register must have a name (for debugging),
 * and a well defined size. The following methods define the interface the
 * register bank uses to access the register, and where the register can
 * implement its special behaviors:
 *
 * void read(void *buf);
 * void read(void *buf, off_t offset, size_t bytes);
 *
 * void write(const void *buf);
 * void write(const void *buf, off_t offset, size_t bytes);
 *
 * The single argument versions of these methods completely overwrite the
 * register's contents with whatever is pointed to by buf.
 *
 * The version which also takes "offset" and "bytes" arguments reads or writes
 * only a portion of the register, starting "offset" bytes from the start of
 * the register, and writing or reading the next "bytes" bytes.
 *
 * Each register also needs to implement serialize or unserialize methods
 * which make it accessible to the checkpointing mechanism. If a register
 * doesn't need to be serialized (for instance if it has a fixed value) then
 * it still has to implement these methods, but they don't have to actually do
 * anything.
 *
 * Each register also has a "reset" method, which will reset the register as
 * if its containing device is being reset. By default, this will just restore
 * the initial value of the register, but can be overridden to implement
 * additional behavior like resetting other aspects of the device which are
 * controlled by the value of the register.
 *
 *
 * == Basic Register types ==
 *
 * Some simple register types have been defined which handle basic, common
 * behaviors found in many devices:
 *
 * = RegisterRaz and RegisterRao =
 *
 * RegisterRaz (read as zero) and RegisterRao (read as one) will ignore writes,
 * and will return all zeroes or ones, respectively, when read. These can have
 * arbitrary alignment and size, and can be used for, for instance,
 * unimplemented registers that still need to take up a certain amount of
 * space, or for gaps between registers which still need to handle accesses
 * even though they don't do anything or hold any data.
 *
 * For instance, a device might have several regions of registers which are
 * aligned on different boundaries, but which might not take up all of the
 * space in each region. The extra space can be filled with a RegisterRaz or
 * RegisterRao, making it possible to implement all the registers as a single
 * bank.
 *
 * If you need a register with a different fill pattern, you can subclass the
 * RegisterRoFill type and implement its "fill" method. This should behave
 * like the three argument form of the read() method, described above.
 *
 * = RegisterBuf and RegisterLBuf =
 *
 * These two types act like inert blobs of storage. They don't have any
 * special behavior and can have any arbitrary size like the RegisterRao and
 * RegisterRaz types above, but these registers actually store what's written
 * to them.
 *
 * The RegisterBuf type acts as an interface to a buffer stored elsewhere. That
 * makes it possible to, for instance, alias the same buffer to different parts
 * of the register space, or to expose some other object which needs to exist
 * outside of the register bank for some reason.
 *
 * The RegisterLBuf does the same thing, except it uses a local buffer it
 * manages. That makes it a little easier to work with if you don't need the
 * flexibility of the RegisterBuf type.
 *
 *
 * == Typed Registers ==
 *
 * The Register template class is for more complex registers with side effects,
 * and/or which hold structured data. The template arguments define what type
 * the register should hold, and also its endianness.
 *
 * = Access handlers =
 *
 * Instead of subclassing the Register<Data> type and redefining its read/write
 * methods, reads and writes are implemented using replaceable handlers with
 * these signatures:
 *
 * Data read(Register<Data> &reg);
 * Data partialRead(Register<Data> &reg, int first, int last);
 * void write(Register<Data> &reg, const Data &value);
 * void partialWrite(Register<Data> &reg, const Data &value,
 *                   int first, int last);
 *
 * The "partial" version of these handlers take "first" and "last" arguments
 * which specify what bits of the register to modify. They should be
 * interpreted like the same arguments in base/bitfield.hh. The endianness
 * of the register will have already been dealt with by the time the handler
 * is called.
 *
 * The read and partialRead handlers should generate whatever value reading the
 * register should return, based on (or not based on) the state of "reg". The
 * partial handler should keep the bits it returns in place. For example, if
 * bits 15-8 are read from a 16 bit register with the value 0x1234, it should
 * return 0x1200, not 0x0012.
 *
 * The write and partialWrite handlers work the same way, except in they write
 * instead of read. They are responsible for updating the value in reg in
 * whatever way and to whatever value is appropriate, based on
 * (or not based on) the value of "value" and the state of "reg".
 *
 * The default implementations of the read and write handlers simply return or
 * update the value stored in reg. The default partial read calls the read
 * handler (which may not be the default), and trims down the data as required.
 * The default partial write handler calls the read handler (which may not be
 * the default), updates the value as requested, and then calls the write
 * handler (which may not be the default).
 *
 * Overriding the partial read or write methods might be necessary if reads or
 * writes have side effects which should affect only the part of the register
 * read or written. For instance, there might be some status bits which will
 * be cleared when accessed. Only the bits which were actually accessed should
 * be affected, even if they're grouped together logically with the other bits
 * in a single register.
 *
 * To set your own handlers, you can use the "reader", "writer",
 * "partialReader", and "partialWriter" methods. Each of these takes a single
 * callable argument (lambda, functor, function pointer, etc.) which will
 * replace the current corresponding handler.
 *
 * These methods all return a reference to the current Register so that they
 * can be strung together without having to respecify what object you're
 * modifying over and over again.
 *
 * There are also versions of these which will set up methods on some object as
 * the handlers. These take a pointer to whatever object will handle the call,
 * and a member function pointer to the method that will actually implement
 * the handler. This can be used if, for instance, the registers are all
 * members of a RegisterBank subclass, and need to call methods on their
 * parent class to actually implement the behavior. These methods must have
 * the same signature as above, with the exception that they are methods and
 * not bare functions.
 *
 * When updating the register's value in custom write or partialWrite handlers,
 * be sure to use the "update" method which will honor read only bits. There
 * is an alternative form of update which also takes a custom bitmask, if you
 * need to update bits other than the normally writeable ones.
 *
 * Similarly, you can set a "resetter" handler which is responsible for
 * resetting the register. It takes a reference to the current Register, and
 * no other parameters. The "initialValue" accessor can retrieve the value the
 * register was constructed with. The register is simply set to this value
 * in the default resetter implementation.
 *
 * = Read only bits =
 *
 * Often registers have bits which are fixed and not affected by writes. To
 * specify which bits are writeable, use the "writeable" method which takes a
 * single argument the same type as the type of the register. It should hold a
 * bitmask where a 1 bit can be written, and a 0 cannot. Calling writeable with
 * no arguments will return the current bitmask.
 *
 * A shorthand "readonly" method marks all bits as read only.
 *
 * Both methods return a reference to the current Register so they can be
 * strung together into a sequence when configuring it.
 *
 * = Underlying data and serialization =
 *
 * The "get" method returns a reference to the underlying storage inside the
 * register. That can be used to manually update the entire register, even bits
 * which are normally read only, or for structured data, to access members of
 * the underlying data type.
 *
 * For instance, if the register holds a BitUnion, you could use the get()
 * method to access the bitfields within it:
 *
 * reg.get().bitfieldA = reg.get().bitfieldB;
 *
 * The serialize and unserialize methods for these types will pass through the
 * underlying data within the register. For instance, when serializing a
 * Register<Foo>, the value in the checkpoint will be the same as if you had
 * serialized a Foo directly, with the value stored in the register.
 *
 * = Aliases =
 *
 * Some convenient aliases have been defined for frequently used versions of
 * the Register class. These are
 *
 * Register(8|16|32|64)(LE|BE|)
 *
 * Where the underlying type of the register is a uint8_t, uint16_t, etc, and
 * the endianness is little endian, big endian, or whatever the default is for
 * the RegisterBank.
 */

namespace gem5
{

// Common bases to make it easier to identify both endiannesses at once.
class RegisterBankBase
{
  public:
    class RegisterBaseBase {};
};

template <ByteOrder BankByteOrder>
class RegisterBank : public RegisterBankBase
{
  public:
    // Static helper methods for implementing register types.
    template <typename Data>
    static constexpr Data
    readWithMask(const Data &value, const Data &bitmask)
    {
        return value & bitmask;
    }

    template <typename Data>
    static constexpr Data
    writeWithMask(const Data &old, const Data &value, const Data &bitmask)
    {
        return readWithMask(
                old, (Data)~bitmask) | readWithMask(value, bitmask);
    }

    class RegisterBase : public RegisterBankBase::RegisterBaseBase
    {
      protected:
        const std::string _name;
        size_t _size = 0;

      public:
        constexpr RegisterBase(const std::string &new_name, size_t new_size) :
            _name(new_name), _size(new_size)
        {}
        virtual ~RegisterBase() {}

        // Read the register's name.
        virtual const std::string &name() const { return _name; }

        // Read the register's size in bytes.
        size_t size() const { return _size; }

        // Perform a read on the register.
        virtual void read(void *buf) = 0;
        virtual void read(void *buf, off_t offset, size_t bytes) = 0;

        // Perform a write on the register.
        virtual void write(const void *buf) = 0;
        virtual void write(const void *buf, off_t offset, size_t bytes) = 0;

        // Methods for implementing serialization for checkpoints.
        virtual void serialize(std::ostream &os) const = 0;
        virtual bool unserialize(const std::string &s) = 0;

        // Reset the register.
        virtual void reset() = 0;
    };

    // Filler registers which return a fixed pattern.
    class RegisterRoFill : public RegisterBase
    {
      protected:
        constexpr RegisterRoFill(
                const std::string &new_name, size_t new_size) :
            RegisterBase(new_name, new_size)
        {}

        virtual void fill(void *buf, off_t offset, size_t bytes) = 0;

      public:
        // Ignore writes.
        void write(const void *buf) override {}
        void write(const void *buf, off_t offset, size_t bytes) override {}

        // Use fill() to handle reads.
        void read(void *buf) override { fill(buf, 0, this->size()); }
        void
        read(void *buf, off_t offset, size_t bytes) override
        {
            fill(buf, offset, bytes);
        }

        void serialize(std::ostream &os) const override {}
        bool unserialize(const std::string &s) override { return true; }

        // Resetting a read only register doesn't need to do anything.
        void reset() override {}
    };

    // Register which reads as all zeroes.
    class RegisterRaz : public RegisterRoFill
    {
      protected:
        void
        fill(void *buf, off_t offset, size_t bytes) override
        {
            bzero(buf, bytes);
        }

      public:
        RegisterRaz(const std::string &new_name, size_t new_size) :
            RegisterRoFill(new_name, new_size)
        {}
    };

    // Register which reads as all ones.
    class RegisterRao : public RegisterRoFill
    {
      protected:
        void
        fill(void *buf, off_t offset, size_t bytes) override
        {
            memset(buf, 0xff, bytes);
        }

      public:
        RegisterRao(const std::string &new_name, size_t new_size) :
            RegisterRoFill(new_name, new_size)
        {}
    };

    // Register which acts as a simple buffer.
    class RegisterBuf : public RegisterBase
    {
      private:
        void *_ptr = nullptr;

      public:
        RegisterBuf(const std::string &new_name, void *ptr, size_t bytes) :
            RegisterBase(new_name, bytes), _ptr(ptr)
        {}

        void write(const void *buf) override { write(buf, 0, this->size()); }
        void
        write(const void *buf, off_t offset, size_t bytes) override
        {
            assert(offset + bytes <= this->size());
            memcpy((uint8_t *)_ptr + offset, buf, bytes);
        }

        void read(void *buf) override { read(buf, 0, this->size()); }
        void
        read(void *buf, off_t offset, size_t bytes) override
        {
            assert(offset + bytes <= this->size());
            memcpy(buf, (uint8_t *)_ptr + offset, bytes);
        }

        // The buffer's owner is responsible for serializing it.
        void serialize(std::ostream &os) const override {}
        bool unserialize(const std::string &s) override { return true; }

        // Assume since the buffer is managed externally, it will be reset
        // externally.
        void reset() override {}

      protected:
        /**
         * This method exists so that derived classes that need to initialize
         * their buffers before they can be set can do so.
         *
         * @param buf The pointer to the backing buffer.
         */
        void
        setBuffer(void *buf)
        {
            assert(_ptr == nullptr);
            assert(buf != nullptr);
            _ptr = buf;
        }
    };

    // Same as above, but which keeps its storage locally.
    template <int BufBytes>
    class RegisterLBuf : public RegisterBuf
    {
      public:
        std::array<uint8_t, BufBytes> buffer;

        RegisterLBuf(const std::string &new_name) :
            RegisterBuf(new_name, nullptr, BufBytes)
        {
            this->setBuffer(buffer.data());
        }

        void
        serialize(std::ostream &os) const override
        {
            if (BufBytes)
                ShowParam<uint8_t>::show(os, buffer[0]);
            for (int i = 1; i < BufBytes; i++) {
                os << " ";
                ShowParam<uint8_t>::show(os, buffer[i]);
            }
        }

        bool
        unserialize(const std::string &s) override
        {
            std::vector<std::string> tokens;
            std::istringstream is(s);

            std::string token;
            while (is >> token)
                tokens.push_back(token);

            if (tokens.size() != BufBytes) {
                warn("Size mismatch unserialing %s, expected %d, got %d",
                        this->name(), BufBytes, tokens.size());
                return false;
            }

            for (int i = 0; i < BufBytes; i++) {
                if (!ParseParam<uint8_t>::parse(tokens[i], buffer[i]))
                    return false;
            }

            return true;
        }

        void reset() override { buffer = std::array<uint8_t, BufBytes>{}; }
    };

    template <typename Data, ByteOrder RegByteOrder=BankByteOrder>
    class Register : public RegisterBase
    {
      protected:
        using This = Register<Data, RegByteOrder>;

      public:
        using ReadFunc = std::function<Data (This &reg)>;
        using PartialReadFunc = std::function<
            Data (This &reg, int first, int last)>;
        using WriteFunc = std::function<void (This &reg, const Data &value)>;
        using PartialWriteFunc = std::function<
            void (This &reg, const Data &value, int first, int last)>;
        using ResetFunc = std::function<void (This &reg)>;

      private:
        Data _data = {};
        Data _resetData = {};
        Data _writeMask = mask(sizeof(Data) * 8);

        ReadFunc _reader = defaultReader;
        WriteFunc _writer = defaultWriter;
        PartialWriteFunc _partialWriter = defaultPartialWriter;
        PartialReadFunc _partialReader = defaultPartialReader;
        ResetFunc _resetter = defaultResetter;

      protected:
        static Data defaultReader(This &reg) { return reg.get(); }

        static Data
        defaultPartialReader(This &reg, int first, int last)
        {
            return mbits(reg._reader(reg), first, last);
        }

        static void
        defaultWriter(This &reg, const Data &value)
        {
            reg.update(value);
        }

        static void
        defaultPartialWriter(This &reg, const Data &value, int first, int last)
        {
            reg._writer(reg, writeWithMask<Data>(reg._reader(reg), value,
                                                 mask(first, last)));
        }

        static void
        defaultResetter(This &reg)
        {
            reg.get() = reg.initialValue();
        }

        constexpr Data
        htoreg(Data data)
        {
            switch (RegByteOrder) {
              case ByteOrder::big:
                return htobe(data);
              case ByteOrder::little:
                return htole(data);
              default:
                panic("Unrecognized byte order %d.", (unsigned)RegByteOrder);
            }
        }

        constexpr Data
        regtoh(Data data)
        {
            switch (RegByteOrder) {
              case ByteOrder::big:
                return betoh(data);
              case ByteOrder::little:
                return letoh(data);
              default:
                panic("Unrecognized byte order %d.", (unsigned)RegByteOrder);
            }
        }

      public:

        /*
         * Interface for setting up the register.
         */

        // Constructor which lets data default initialize itself.
        constexpr Register(const std::string &new_name) :
            RegisterBase(new_name, sizeof(Data))
        {}

        // Constructor and move constructor with an initial data value.
        constexpr Register(const std::string &new_name, const Data &new_data) :
            RegisterBase(new_name, sizeof(Data)), _data(new_data),
            _resetData(new_data)
        {}
        constexpr Register(const std::string &new_name,
                           const Data &&new_data) :
            RegisterBase(new_name, sizeof(Data)), _data(new_data),
            _resetData(new_data)
        {}

        // Set which bits of the register are writeable.
        constexpr This &
        writeable(const Data &new_mask)
        {
            _writeMask = new_mask;
            return *this;
        }

        // Set the register as read only.
        constexpr This &readonly() { return writeable(0); }

        // Set the callables which handles reads or writes.
        // The default reader just returns the register value.
        // The default writer uses the write mask to update the register value.
        constexpr This &
        reader(const ReadFunc &new_reader)
        {
            _reader = new_reader;
            return *this;
        }
        template <class Parent, class... Args>
        constexpr This &
        reader(Parent *parent, Data (Parent::*nr)(Args... args))
        {
            auto wrapper = [parent, nr](Args&&... args) -> Data {
                return (parent->*nr)(std::forward<Args>(args)...);
            };
            return reader(wrapper);
        }
        constexpr This &
        writer(const WriteFunc &new_writer)
        {
            _writer = new_writer;
            return *this;
        }
        template <class Parent, class... Args>
        constexpr This &
        writer(Parent *parent, void (Parent::*nw)(Args... args))
        {
            auto wrapper = [parent, nw](Args&&... args) {
                (parent->*nw)(std::forward<Args>(args)...);
            };
            return writer(wrapper);
        }

        // Set the callables which handle reads or writes. These may need to
        // be handled specially if, for instance, accessing bits outside of
        // the enables would have side effects that shouldn't happen.
        //
        // The default partial reader just uses the byte enables to mask off
        // bits that are not being read.
        //
        // The default partial writer reads the current value of the register,
        // uses the byte enables to update only the bytes that are changing,
        // and then writes the result back to the register.
        constexpr This &
        partialReader(const PartialReadFunc &new_reader)
        {
            _partialReader = new_reader;
            return *this;
        }
        template <class Parent, class... Args>
        constexpr This &
        partialReader(Parent *parent, Data (Parent::*nr)(Args... args))
        {
            auto wrapper = [parent, nr](Args&&... args) -> Data {
                return (parent->*nr)(std::forward<Args>(args)...);
            };
            return partialReader(wrapper);
        }
        constexpr This &
        partialWriter(const PartialWriteFunc &new_writer)
        {
            _partialWriter = new_writer;
            return *this;
        }
        template <class Parent, class... Args>
        constexpr This &
        partialWriter(Parent *parent, void (Parent::*nw)(Args... args))
        {
            auto wrapper = [parent, nw](Args&&... args) {
                return (parent->*nw)(std::forward<Args>(args)...);
            };
            return partialWriter(wrapper);
        }

        // Set the callables which handle resetting.
        //
        // The default resetter restores the initial value used in the
        // constructor.
        constexpr This &
        resetter(const ResetFunc &new_resetter)
        {
            _resetter = new_resetter;
            return *this;
        }
        template <class Parent, class... Args>
        constexpr This &
        resetter(Parent *parent, void (Parent::*nr)(Args... args))
        {
            auto wrapper = [parent, nr](Args&&... args) {
                return (parent->*nr)(std::forward<Args>(args)...);
            };
            return resetter(wrapper);
        }

        // An accessor which returns the initial value as set in the
        // constructor. This is intended to be used in a resetter function.
        const Data &initialValue() const { return _resetData; }

        // Reset the initial value, which is normally set in the constructor,
        // to the register's current value.
        void resetInitialValue() { _resetData = _data; }

        /*
         * Interface for accessing the register's state, for use by the
         * register's helper functions and the register bank.
         */

        const Data &writeable() const { return _writeMask; }

        // Directly access the underlying data value.
        const Data &get() const { return _data; }
        Data &get() { return _data; }

        // Update data while applying a mask.
        void
        update(const Data &new_data, const Data &bitmask)
        {
            _data = writeWithMask(_data, new_data, bitmask);
        }
        // This version uses the default write mask.
        void
        update(const Data &new_data)
        {
            _data = writeWithMask(_data, new_data, _writeMask);
        }


        /*
         * Interface for reading/writing the register, for use by the
         * register bank.
         */

        // Perform a read on the register.
        void
        read(void *buf) override
        {
            Data data = htoreg(_reader(*this));
            memcpy(buf, (uint8_t *)&data, sizeof(data));
        }

        void
        read(void *buf, off_t offset, size_t bytes) override
        {
            // Move the region we're reading to be little endian, since that's
            // what gem5 uses internally in BitUnions, masks, etc.
            const off_t host_off = (RegByteOrder != ByteOrder::little) ?
                sizeof(Data) - (offset + bytes) : offset;

            const int first = (host_off + bytes) * 8 - 1;
            const int last = host_off * 8;
            Data data = htoreg(_partialReader(*this, first, last));

            memcpy(buf, (uint8_t *)&data + offset, bytes);
        }

        // Perform a write on the register.
        void
        write(const void *buf) override
        {
            Data data;
            memcpy((uint8_t *)&data, buf, sizeof(data));
            data = regtoh(data);
            _writer(*this, data);
        }

        void
        write(const void *buf, off_t offset, size_t bytes) override
        {
            Data data = {};
            memcpy((uint8_t *)&data + offset, buf, bytes);

            data = regtoh(data);

            // Move the region we're reading to be little endian, since that's
            // what gem5 uses internally in BitUnions, masks, etc.
            const off_t host_off = (RegByteOrder != ByteOrder::little) ?
                sizeof(Data) - (offset + bytes) : offset;

            const int first = (host_off + bytes) * 8 - 1;
            const int last = host_off * 8;
            _partialWriter(*this, data, first, last);
        }

        // Serialize our data using existing mechanisms.
        void
        serialize(std::ostream &os) const override
        {
            ShowParam<Data>::show(os, get());
        }

        bool
        unserialize(const std::string &s) override
        {
            return ParseParam<Data>::parse(s, get());
        }

        // Reset our data to its initial value.
        void reset() override { _resetter(*this); }
    };

    // Allow gem5 models to set a debug flag to the register bank for logging
    // all full/partial read/write access to the registers. The register bank
    // would not log if the flag is not set.
    //
    // The debug flag is the one declared in the SConscript
    //
    // DebugFlag('HelloExample')
    //
    // Then the flag can be set in the register bank with:
    //
    // setDebugFlag(::gem5::debug::HelloExample)
    void setDebugFlag(const ::gem5::debug::SimpleFlag& flag)
    {
        _debug_flag = &flag;
    }

  private:
    std::map<Addr, std::reference_wrapper<RegisterBase>> _offsetMap;

    const ::gem5::debug::SimpleFlag* _debug_flag = nullptr;
    Addr _base = 0;
    Addr _size = 0;
    const std::string _name;

  public:

    using Register8 = Register<uint8_t>;
    using Register8LE = Register<uint8_t, ByteOrder::little>;
    using Register8BE = Register<uint8_t, ByteOrder::big>;
    using Register16 = Register<uint16_t>;
    using Register16LE = Register<uint16_t, ByteOrder::little>;
    using Register16BE = Register<uint16_t, ByteOrder::big>;
    using Register32 = Register<uint32_t>;
    using Register32LE = Register<uint32_t, ByteOrder::little>;
    using Register32BE = Register<uint32_t, ByteOrder::big>;
    using Register64 = Register<uint64_t>;
    using Register64LE = Register<uint64_t, ByteOrder::little>;
    using Register64BE = Register<uint64_t, ByteOrder::big>;


    constexpr RegisterBank(const std::string &new_name, Addr new_base) :
        _base(new_base), _name(new_name)
    {}

    virtual ~RegisterBank() {}

    class RegisterAdder
    {
      private:
        std::optional<Addr> offset;
        std::optional<RegisterBase *> reg;

      public:
        // Nothing special to do for this register.
        RegisterAdder(RegisterBase &new_reg) : reg(&new_reg) {}
        // Ensure that this register is added at a particular offset.
        RegisterAdder(Addr new_offset, RegisterBase &new_reg) :
            offset(new_offset), reg(&new_reg)
        {}
        // No register, just check that the offset is what we expect.
        RegisterAdder(Addr new_offset) : offset(new_offset) {}

        friend class RegisterBank;
    };

    void
    addRegisters(std::initializer_list<RegisterAdder> adders)
    {
        panic_if(std::empty(adders),
                "Adding an empty list of registers to %s?", name());
        for (auto &adder: adders) {
            const Addr offset = _base + _size;

            if (adder.reg) {
                auto *reg = adder.reg.value();
                if (adder.offset && adder.offset.value() != offset) {
                    panic(
                        "Expected offset of register %s.%s to be %#x, is %#x.",
                        name(), reg->name(), adder.offset.value(), offset);
                }
                _offsetMap.emplace(offset, *reg);
                _size += reg->size();
            } else if (adder.offset) {
                if (adder.offset.value() != offset) {
                    panic("Expected current offset of %s to be %#x, is %#x.",
                        name(), adder.offset.value(), offset);
                }
            }
        }
    }

    void addRegister(RegisterAdder reg) { addRegisters({reg}); }

    Addr base() const { return _base; }
    Addr size() const { return _size; }
    const std::string &name() const { return _name; }

    virtual void
    read(Addr addr, void *buf, Addr bytes)
    {
        uint8_t *ptr = (uint8_t *)buf;
        // Number of bytes we've transferred.
        Addr done = 0;

        panic_if(addr - base() + bytes > size(),
            "Out of bounds read in register bank %s, address %#x, size %d.",
            name(), addr, bytes);

        auto it = _offsetMap.lower_bound(addr);
        if (it == _offsetMap.end() || it->first > addr)
            it--;

        std::ostringstream ss;
        while (done != bytes) {
          RegisterBase &reg = it->second.get();
          const size_t reg_off = addr - it->first;
          const size_t reg_size = reg.size() - reg_off;
          const size_t reg_bytes = std::min(reg_size, bytes - done);

          if (reg_bytes != reg.size()) {
              if (_debug_flag) {
                  ccprintf(ss, "Read register %s, byte offset %d, size %d\n",
                          reg.name(), reg_off, reg_bytes);
              }
              reg.read(ptr + done, reg_off, reg_bytes);
          } else {
              if (_debug_flag) {
                  ccprintf(ss, "Read register %s\n", reg.name());
              }
              reg.read(ptr + done);
          }

          done += reg_bytes;
          addr += reg_bytes;
          ++it;
        }

        if (_debug_flag) {
            ::gem5::trace::getDebugLogger()->dprintf_flag(
                curTick(), name(), _debug_flag->name(), "%s", ss.str());
        }
    }

    virtual void
    write(Addr addr, const void *buf, Addr bytes)
    {
        const uint8_t *ptr = (const uint8_t *)buf;
        // Number of bytes we've transferred.
        Addr done = 0;

        panic_if(addr - base() + bytes > size(),
            "Out of bounds write in register bank %s, address %#x, size %d.",
            name(), addr, bytes);

        auto it = _offsetMap.lower_bound(addr);
        if (it == _offsetMap.end() || it->first > addr)
            it--;

        std::ostringstream ss;
        while (done != bytes) {
            RegisterBase &reg = it->second.get();
            const size_t reg_off = addr - it->first;
            const size_t reg_size = reg.size() - reg_off;
            const size_t reg_bytes = std::min(reg_size, bytes - done);

            if (reg_bytes != reg.size()) {
                if (_debug_flag) {
                    ccprintf(ss, "Write register %s, byte offset %d, size %d\n",
                              reg.name(), reg_off, reg_size);
                }
                reg.write(ptr + done, reg_off, reg_bytes);
            } else {
                if (_debug_flag) {
                  ccprintf(ss, "Write register %s\n", reg.name());
                }
                reg.write(ptr + done);
            }

            done += reg_bytes;
            addr += reg_bytes;
            ++it;
        }

        if (_debug_flag) {
            ::gem5::trace::getDebugLogger()->dprintf_flag(
                curTick(), name(), _debug_flag->name(), "%s", ss.str());
        }
    }

    // By default, reset all the registers in the bank.
    virtual void
    reset()
    {
        for (auto &it: _offsetMap)
            it.second.get().reset();
    }
};

using RegisterBankLE = RegisterBank<ByteOrder::little>;
using RegisterBankBE = RegisterBank<ByteOrder::big>;

// Delegate serialization to the individual RegisterBase subclasses.
template <class T>
struct ParseParam<T, std::enable_if_t<std::is_base_of_v<
    typename RegisterBankBase::RegisterBaseBase, T>>>
{
    static bool
    parse(const std::string &s, T &value)
    {
        return value.unserialize(s);
    }
};

template <class T>
struct ShowParam<T, std::enable_if_t<std::is_base_of_v<
    typename RegisterBankBase::RegisterBaseBase, T>>>
{
    static void
    show(std::ostream &os, const T &value)
    {
        value.serialize(os);
    }
};

} // namespace gem5

#endif // __DEV_REG_BANK_HH__
