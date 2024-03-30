/*
 * Copyright (c) 2018, 2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __ARCH_ARM_SEMIHOSTING_HH__
#define __ARCH_ARM_SEMIHOSTING_HH__

#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "arch/arm/regs/int.hh"
#include "arch/arm/utility.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "sim/core.hh"
#include "sim/guest_abi.hh"
#include "sim/pseudo_inst.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct ArmSemihostingParams;
class SerialDevice;

/**
 * Semihosting for AArch32 and AArch64
 *
 * This class implements the Arm semihosting interface. This interface
 * allows baremetal code access service, such as IO, from the
 * simulator. It is conceptually a simplified version of gem5's more
 * general syscall emulation mode.
 *
 * Exits calls (SYS_EXIT, SYS_EXIT_EXTENDED) from the guest get
 * translated into simualtion exits. Well-known exit codes are
 * translated to messages on the form 'semi:ADP_.*' while unknown
 * codes are returned in hex ('semi:0x..'). The subcode is reported in
 * the gem5 exit event.
 */
class ArmSemihosting : public SimObject
{
  public:
    enum
    {
        // Standard ARM immediate values which trigger semihosting.
        T32Imm = 0xAB,
        A32Imm = 0x123456,
        A64Imm = 0xF000,

        // The immediate value which enables gem5 semihosting calls. Use the
        // standard value for thumb.
        Gem5Imm = 0x5D57
    };

    static PortProxy &portProxy(ThreadContext *tc);

    struct AbiBase
    {
        template <typename Arg>
        class StateBase
        {
          private:
            Addr argPointer;
            ByteOrder endian;

          public:
            StateBase(const ThreadContext *tc, Addr arg_pointer) :
                argPointer(arg_pointer), endian(ArmISA::byteOrder(tc))
            {}

            /*
             * These two methods are used to both read an argument or its
             * address, and to move position on to the next location. Normally
             * State would be more passive, but since it behaves almost the
             * same no matter what the argument type is we can simplify and
             * consolidate a little bit by centralizing these methods.
             */

            // Return the address of an argument slot and move past it.
            Addr
            getAddr()
            {
                Addr addr = argPointer;
                argPointer += sizeof(Arg);
                return addr;
            }

            // Read the value in an argument slot and move past it.
            Arg
            get(ThreadContext *tc)
            {
                Arg arg = ArmSemihosting::portProxy(tc).read<Arg>(
                        argPointer, endian);
                argPointer += sizeof(Arg);
                return arg;
            }

            using ArgType = Arg;
        };
    };

    struct Abi64 : public AbiBase
    {
        using UintPtr = uint64_t;

        class State : public StateBase<uint64_t>
        {
          public:
            // For 64 bit semihosting, the params are pointer to by X1.
            explicit State(const ThreadContext *tc) :
                StateBase<uint64_t>(tc, tc->getReg(ArmISA::int_reg::X1))
            {}
        };
    };

    struct Abi32 : public AbiBase
    {
        using UintPtr = uint32_t;

        class State : public StateBase<uint64_t>
        {
          public:
            // For 32 bit semihosting, the params are pointer to by R1.
            explicit State(const ThreadContext *tc) :
                StateBase<uint64_t>(tc, tc->getReg(ArmISA::int_reg::R1))
            {}
        };
    };

    // Use this argument type when you need to modify an argument in place.
    // This will give you the address of the argument itself and the size of
    // each argument slot, rather than the actual value of the argument.
    struct InPlaceArg
    {
        Addr addr;
        size_t size;

        InPlaceArg(Addr _addr, size_t _size) : addr(_addr), size(_size) {}

        // A helper function to read the argument since the guest ABI mechanism
        // didn't do that for us.
        uint64_t
        read(ThreadContext *tc, ByteOrder endian)
        {
            auto &proxy = ArmSemihosting::portProxy(tc);
            if (size == 8)
                return proxy.read<uint64_t>(addr, endian);
            else if (size == 4)
                return proxy.read<uint32_t>(addr, endian);
            else
                panic("Unexpected semihosting argument size %d.", size);
        }

        // A helper function to write to the argument's slot in the params.
        void
        write(ThreadContext *tc, uint64_t val, ByteOrder endian)
        {
            auto &proxy = ArmSemihosting::portProxy(tc);
            if (size == 8)
                proxy.write<uint64_t>(addr, val, endian);
            else if (size == 4)
                proxy.write<uint32_t>(addr, val, endian);
            else
                panic("Unexpected semihosting argument size %d.", size);
        }
    };

    enum Operation
    {
        SYS_OPEN = 0x01,
        SYS_CLOSE = 0x02,
        SYS_WRITEC = 0x03,
        SYS_WRITE0 = 0x04,
        SYS_WRITE = 0x05,
        SYS_READ = 0x06,
        SYS_READC = 0x07,
        SYS_ISERROR = 0x08,
        SYS_ISTTY = 0x09,
        SYS_SEEK = 0x0A,
        SYS_FLEN = 0x0C,
        SYS_TMPNAM = 0x0D,
        SYS_REMOVE = 0x0E,
        SYS_RENAME = 0x0F,
        SYS_CLOCK = 0x10,
        SYS_TIME = 0x11,
        SYS_SYSTEM = 0x12,
        SYS_ERRNO = 0x13,
        SYS_GET_CMDLINE = 0x15,
        SYS_HEAPINFO = 0x16,
        SYS_EXIT = 0x18,
        SYS_EXIT_EXTENDED = 0x20,
        SYS_ELAPSED = 0x30,
        SYS_TICKFREQ = 0x31,

        MaxStandardOp = 0xFF,

        SYS_GEM5_PSEUDO_OP = 0x100
    };

    ArmSemihosting(const ArmSemihostingParams &p);

    /** Perform an Arm Semihosting call from aarch64 code. */
    bool call64(ThreadContext *tc, bool gem5_ops);
    /** Perform an Arm Semihosting call from aarch32 code. */
    bool call32(ThreadContext *tc, bool gem5_ops);

  public: // SimObject and related interfaces
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected: // Configuration
    const std::string cmdLine;
    const Addr memReserve;
    const Addr stackSize;

    /**
     * Base time when the simulation started. This is used to
     * calculate the time of date when the guest call SYS_TIME.
     */
    const time_t timeBase;

    /** Number of bits to right shift gem5 ticks to fit in a uint32_t */
    const unsigned tickShift;

  protected: // Internal state
    typedef uint64_t SemiErrno;
    SemiErrno semiErrno;

  protected: // File IO
    /**
     * Internal state for open files
     *
     * This class describes the internal state of a file opened
     * through the semihosting interface.
     *
     * A file instance is normally created using one of the
     * ArmSemihosting::FileBase::create() factory methods. These
     * methods handle some the magic file names in the Arm Semihosting
     * specification and instantiate the right implementation. For the
     * same, when unserializing a checkpoint, the create method must
     * be used to unserialize a new instance of a file descriptor.
     */
    class FileBase : public Serializable
    {
      public:
        FileBase(ArmSemihosting &_parent, const char *name, const char *_mode)
            : parent(_parent), _name(name), mode(_mode) {}
        virtual ~FileBase() {};

        FileBase() = delete;
        FileBase(FileBase &) = delete;

        static std::unique_ptr<FileBase> create(
            ArmSemihosting &parent, const std::string &fname,
            const char *mode);
        static std::unique_ptr<FileBase> create(
            ArmSemihosting &parent, CheckpointIn &cp, const std::string &sec);

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        const std::string &fileName() { return _name; }

      public:
        /** @{
         * Semihosting file IO interfaces
         *
         * These interfaces implement common IO functionality in the
         * Semihosting interface.
         *
         * All functions return a negative value that corresponds to a
         * UNIX errno value when they fail and >=0 on success.
         */

        /**
         * Open the the file.
         *
         * @return <0 on error (-errno), 0 on success.
         */
        virtual int64_t open() { return 0; }

        /**
         * Close the file.
         *
         * @return <0 on error (-errno), 0 on success.
         */
        virtual int64_t close() { return 0; }

        /**
         * Check if a file corresponds to a TTY device.
         *
         * @return True if the file is a TTY, false otherwise.
         */
        virtual bool isTTY() const { return false; }

        /**
         * Read data from file.
         *
         * @return <0 on error (-errno), bytes read on success (0 for EOF).
         */
        virtual int64_t read(uint8_t *buffer, uint64_t size);

        /**
         * Write data to file.
         *
         * @return <0 on error (-errno), bytes written on success.
         */
        virtual int64_t write(const uint8_t *buffer, uint64_t size);

        /**
         * Seek to an absolute position in the file.
         *
         * @param pos Byte offset from start of file.
         * @return <0 on error (-errno), 0 on success.
         */
        virtual int64_t seek(uint64_t pos);

        /**
         * Get the length of a file in bytes.
         *
         * @return <0 on error (-errno), length on success
         */
        virtual int64_t flen();

        /** @} */

      protected:
        ArmSemihosting &parent;
        std::string _name;
        std::string mode;
    };

    /** Implementation of the ':semihosting-features' magic file. */
    class FileFeatures : public FileBase
    {
      public:
        FileFeatures(ArmSemihosting &_parent,
                     const char *name, const char *mode);

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        int64_t read(uint8_t *buffer, uint64_t size) override;
        int64_t seek(uint64_t pos) override;

      protected:
        size_t pos;
    };

    class File : public FileBase
    {
      public:
        File(ArmSemihosting &_parent, const char *name, const char *mode);
        ~File();

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        int64_t open() override { return openImpl(false); }
        int64_t close() override;
        bool isTTY() const override;
        int64_t read(uint8_t *buffer, uint64_t size) override;
        int64_t write(const uint8_t *buffer, uint64_t size) override;
        int64_t seek(uint64_t pos) override;
        int64_t flen() override;

      protected:
        int64_t openImpl(bool unserialize);
        bool needClose() const { return !isTTY(); }

        FILE *file;
    };

    std::string filesRootDir;
    std::vector<std::unique_ptr<FileBase>> files;
    using Handle = size_t;
    FILE *stdin;
    FILE *stdout;
    FILE *stderr;

  protected: // Helper functions
    unsigned
    calcTickShift() const
    {
        int msb = findMsbSet(sim_clock::Frequency);
        return msb > 31 ? msb - 31 : 0;
    }
    uint64_t
    semiTick(Tick tick) const
    {
        return tick >> tickShift;
    }
    void semiExit(uint64_t code, uint64_t subcode);
    std::string readString(ThreadContext *tc, Addr ptr, size_t len);

  public:
    typedef std::pair<uint64_t, SemiErrno> RetErrno;

  private:
    static RetErrno
    retError(SemiErrno e)
    {
        return RetErrno((uint64_t)-1, e);
    }

    static RetErrno
    retOK(uint64_t r)
    {
        return RetErrno(r, 0);
    }

    /**
     * Semihosting call information structure.
     *
     * This structure describes how a semi-hosting call is
     * implemented. It contains debug information (e.g., the name of
     * the call), and a way to invoke it in a particular context.
     */
    struct SemiCall
    {
        /** Call name */
        const char *name;

        // A type for member functions implementing semihosting calls.
        template <typename ...Args>
        using Implementation =
            RetErrno (ArmSemihosting::*)(ThreadContext *tc, Args... args);

        // Since guest ABI doesn't know how to call member function pointers,
        // this template builds a wrapper that takes care of that.
        template <typename ...Args>
        static inline std::function<RetErrno(ThreadContext *tc, Args... args)>
        wrapImpl(ArmSemihosting *sh, Implementation<Args...> impl)
        {
            return [sh, impl](ThreadContext *tc, Args... args) {
                return (sh->*impl)(tc, args...);
            };
        }

        // A type for functions which dispatch semihosting calls through the
        // guest ABI mechanism.
        using Dispatcher =
            std::function<RetErrno(ArmSemihosting *sh, ThreadContext *tc)>;
        using Dumper = std::function<std::string(ThreadContext *tc)>;

        // Dispatchers for 32 and 64 bits.
        Dispatcher call32;
        Dispatcher call64;

        // Dumpers which print semihosting calls and their arguments.
        Dumper dump32;
        Dumper dump64;

        // A function which builds a dispatcher for a semihosting call.
        template <typename Abi, typename ...Args>
        static inline Dispatcher
        buildDispatcher(Implementation<Args...> impl)
        {
            // This lambda is the dispatcher we're building.
            return [impl](ArmSemihosting *sh, ThreadContext *tc) {
                auto wrapper = wrapImpl(sh, impl);
                return invokeSimcall<Abi>(tc, wrapper);
            };
        }

        // A function which builds a dumper for a semihosting call.
        template <typename Abi, typename ...Args>
        static inline Dumper
        buildDumper(const char *name, Implementation<Args...> impl)
        {
            // This lambda is the dumper we're building.
            return [name](ThreadContext *tc) -> std::string {
                return dumpSimcall<Abi, RetErrno, Args...>(name, tc);
            };
        }

        // When there's one implementation, use it for both 32 and 64 bits.
        template <typename ...Args>
        SemiCall(const char *_name, Implementation<Args...> common) :
            name(_name), call32(buildDispatcher<Abi32>(common)),
            call64(buildDispatcher<Abi64>(common)),
            dump32(buildDumper<Abi32>(_name, common)),
            dump64(buildDumper<Abi64>(_name, common))
        {}

        // When there are two, use one for 32 bits and one for 64 bits.
        template <typename ...Args32, typename ...Args64>
        SemiCall(const char *_name, Implementation<Args32...> impl32,
                 Implementation<Args64...> impl64) :
            name(_name), call32(buildDispatcher<Abi32>(impl32)),
            call64(buildDispatcher<Abi64>(impl64)),
            dump32(buildDumper<Abi32>(_name, impl32)),
            dump64(buildDumper<Abi64>(_name, impl64))
        {}
    };

    RetErrno callOpen(ThreadContext *tc, const Addr name_base,
                      int fmode, size_t name_size);
    RetErrno callClose(ThreadContext *tc, Handle handle);
    RetErrno callWriteC(ThreadContext *tc, InPlaceArg c);
    RetErrno callWrite0(ThreadContext *tc, InPlaceArg str);
    RetErrno callWrite(ThreadContext *tc, Handle handle,
                       Addr buffer, size_t size);
    RetErrno callRead(ThreadContext *tc, Handle handle,
                      Addr buffer, size_t size);
    RetErrno callReadC(ThreadContext *tc);
    RetErrno callIsError(ThreadContext *tc, int64_t status);
    RetErrno callIsTTY(ThreadContext *tc, Handle handle);
    RetErrno callSeek(ThreadContext *tc, Handle handle, uint64_t pos);
    RetErrno callFLen(ThreadContext *tc, Handle handle);
    RetErrno callTmpNam(ThreadContext *tc, Addr buffer,
                        uint64_t id, size_t size);
    RetErrno callRemove(ThreadContext *tc, Addr name_base, size_t name_size);
    RetErrno callRename(ThreadContext *tc, Addr from_addr, size_t from_size,
                        Addr to_addr, size_t to_size);
    RetErrno callClock(ThreadContext *tc);
    RetErrno callTime(ThreadContext *tc);
    RetErrno callSystem(ThreadContext *tc, Addr cmd_addr, size_t cmd_size);
    RetErrno callErrno(ThreadContext *tc);
    RetErrno callGetCmdLine(ThreadContext *tc, Addr addr, InPlaceArg size_arg);

    void gatherHeapInfo(ThreadContext *tc, bool aarch64,
                        Addr &heap_base, Addr &heap_limit,
                        Addr &stack_base, Addr &stack_limit);
    RetErrno callHeapInfo32(ThreadContext *tc, Addr block_addr);
    RetErrno callHeapInfo64(ThreadContext *tc, Addr block_addr);
    RetErrno callExit32(ThreadContext *tc, InPlaceArg code);
    RetErrno callExit64(ThreadContext *tc, uint64_t code, uint64_t subcode);
    RetErrno callExitExtended(ThreadContext *tc, uint64_t code,
                              uint64_t subcode);

    RetErrno callElapsed32(ThreadContext *tc, InPlaceArg low, InPlaceArg high);
    RetErrno callElapsed64(ThreadContext *tc, InPlaceArg ticks);
    RetErrno callTickFreq(ThreadContext *tc);

    RetErrno callGem5PseudoOp32(ThreadContext *tc, uint32_t encoded_func);
    RetErrno callGem5PseudoOp64(ThreadContext *tc, uint64_t encoded_func);

    template <typename Abi>
    void
    unrecognizedCall(ThreadContext *tc, const char *format, uint64_t op)
    {
        warn(format, op);
        std::function<RetErrno(ThreadContext *tc)> retErr =
            [](ThreadContext *tc) { return retError(EINVAL); };
        invokeSimcall<Abi>(tc, retErr);
    }

    static FILE *getSTDIO(const char *stream_name,
                          const std::string &name, const char *mode);

    static const std::map<uint32_t, SemiCall> calls;
    static const std::vector<const char *> fmodes;
    static const std::map<uint64_t, const char *> exitCodes;
    static const std::vector<uint8_t> features;
    static const std::map<const std::string, FILE *> stdioMap;

    // used in callTmpNam() to deterministically generate a temp filename
    uint16_t tmpNameIndex = 0;

};

std::ostream &operator << (
        std::ostream &os, const ArmSemihosting::InPlaceArg &ipa);

namespace guest_abi
{

template <typename Arg>
struct Argument<ArmSemihosting::Abi64, Arg,
    typename std::enable_if_t<
        (std::is_integral_v<Arg> ||
         std::is_same<Arg,pseudo_inst::GuestAddr>::value)>>
{
    static Arg
    get(ThreadContext *tc, ArmSemihosting::Abi64::State &state)
    {
        return (Arg)state.get(tc);
    }
};

template <typename Arg>
struct Argument<ArmSemihosting::Abi32, Arg,
    typename std::enable_if_t<
        (std::is_integral_v<Arg> ||
         std::is_same<Arg,pseudo_inst::GuestAddr>::value)>>
{
    static Arg
    get(ThreadContext *tc, ArmSemihosting::Abi32::State &state)
    {
        if (std::is_signed_v<Arg>) {
            return (Arg)sext<32>(state.get(tc));
        }
        else {
            return (Arg)state.get(tc);
        }
    }
};

template <typename Abi>
struct Argument<Abi, ArmSemihosting::InPlaceArg, typename std::enable_if_t<
    std::is_base_of_v<ArmSemihosting::AbiBase, Abi>>>
{
    static ArmSemihosting::InPlaceArg
    get(ThreadContext *tc, typename Abi::State &state)
    {
        return ArmSemihosting::InPlaceArg(
                state.getAddr(), sizeof(typename Abi::State::ArgType));
    }
};

template <>
struct Result<ArmSemihosting::Abi32, ArmSemihosting::RetErrno>
{
    static void
    store(ThreadContext *tc, const ArmSemihosting::RetErrno &err)
    {
        tc->setReg(ArmISA::int_reg::R0, err.first);
    }
};

template <>
struct Result<ArmSemihosting::Abi64, ArmSemihosting::RetErrno>
{
    static void
    store(ThreadContext *tc, const ArmSemihosting::RetErrno &err)
    {
        tc->setReg(ArmISA::int_reg::X0, err.first);
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_ARM_SEMIHOSTING_HH__
