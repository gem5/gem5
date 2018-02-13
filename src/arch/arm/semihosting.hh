/*
 * Copyright (c) 2018 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */
#ifndef __ARCH_ARM_SEMIHOSTING_HH__
#define __ARCH_ARM_SEMIHOSTING_HH__

#include <cstdio>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "sim/sim_object.hh"

struct ArmSemihostingParams;
class PortProxy;
class SerialDevice;
class ThreadContext;

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
    ArmSemihosting(const ArmSemihostingParams *p);

    /** Perform an Arm Semihosting call from aarch64 code. */
    uint64_t call64(ThreadContext *tc, uint32_t op, uint64_t param);
    /** Perform an Arm Semihosting call from aarch32 code. */
    uint32_t call32(ThreadContext *tc, uint32_t op, uint32_t param);

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

    std::vector<std::unique_ptr<FileBase>> files;

  protected: // Helper functions
    unsigned calcTickShift() const {
        int msb = findMsbSet(SimClock::Frequency);
        return msb > 31 ? msb - 31 : 0;
    }
    uint64_t semiTick(Tick tick) const {
        return tick >> tickShift;
    }
    void semiExit(uint64_t code, uint64_t subcode);
    PortProxy &physProxy(ThreadContext *tc);
    std::string readString(ThreadContext *tc, Addr ptr, size_t len);

    std::unique_ptr<PortProxy> physProxyS;

  private:
    typedef std::pair<uint64_t, SemiErrno> RetErrno;
    static constexpr RetErrno retError(SemiErrno e) {
        return RetErrno((uint64_t)-1, e);
    }

    static constexpr RetErrno retOK(uint64_t r) {
        return RetErrno(r, 0);
    }

    /**
     * Semihosting call information structure.
     *
     * This structure describes how a semi-hosting call is
     * implemented. It contains debug information (e.g., the name of
     * the call), a pointer to the implementation, and information
     * needed to read its parameters from guest memory.
     */
    struct SemiCall
    {
        /** Call name */
        const char *name;

        /**
         * Pointer to  call implementation
         *
         * @param tc ThreadContext pointer for caller
         * @param aarch64 True if in aarc64 mode, false otherwise.
         * @parma argv Argument vector. argv[0] always corresponds to
         *             the pointer to the argument list. Remaining
         *             entries are read as consecutive words starting
         *             at the address pointed to by argv[0].
         * @return a (return value, errno) pair
         */
        RetErrno (ArmSemihosting::*call)(ThreadContext *tc, bool aarch64,
                                         std::vector<uint64_t> &argv);

        /** Number of aarch32 arguments to read from guest memory. -1
         * if unimplemented.*/
        int argc32;
        /** Number of aarch32 arguments to read from guest memory. -1
         * if unimplemented.*/
        int argc64;

        /** Is call implemented in aarch32? */
        bool implemented32() const { return call && argc32 >= 0; }
        /** Is call implemented in aarch64? */
        bool implemented64() const { return call && argc64 >= 0; }
    };

#define SEMI_CALL(N)                                                    \
    RetErrno call ## N (ThreadContext *tc,                              \
                        bool aarch64, std::vector<uint64_t> &argv)

    SEMI_CALL(Open);
    SEMI_CALL(Close);
    SEMI_CALL(WriteC);
    SEMI_CALL(Write0);
    SEMI_CALL(Write);
    SEMI_CALL(Read);
    SEMI_CALL(ReadC);
    SEMI_CALL(IsError);
    SEMI_CALL(IsTTY);
    SEMI_CALL(Seek);
    SEMI_CALL(FLen);
    SEMI_CALL(TmpNam);
    SEMI_CALL(Remove);
    SEMI_CALL(Rename);
    SEMI_CALL(Clock);
    SEMI_CALL(Time);
    SEMI_CALL(System);
    SEMI_CALL(Errno);
    SEMI_CALL(GetCmdLine);
    SEMI_CALL(HeapInfo);
    SEMI_CALL(Exit);
    SEMI_CALL(ExitExtended);

    SEMI_CALL(Elapsed);
    SEMI_CALL(TickFreq);

#undef SEMI_CALL

    static const SemiCall *getCall(uint32_t op, bool aarch64);

    static const std::map<uint32_t, SemiCall> calls;
    static const std::vector<const char *> fmodes;
    static const std::map<uint64_t, const char *> exitCodes;
    static const std::vector<uint8_t> features;
};

#endif // __ARCH_ARM_SEMIHOSTING_HH__
