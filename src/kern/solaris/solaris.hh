/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#ifndef __SOLARIS_HH__
#define __SOLARIS_HH__

#include "base/types.hh"
#include "kern/operatingsystem.hh"

///
/// This class encapsulates the types, structures, constants,
/// functions, and syscall-number mappings specific to the Solaris
/// syscall interface.
///
class Solaris : public OperatingSystem
{

  public:

    //@{
    /// Basic Solaris types.
    typedef uint64_t size_t;
    typedef uint64_t off_t;
    typedef int64_t time_t;
    typedef int32_t uid_t;
    typedef int32_t gid_t;
    typedef uint64_t rlim_t;
    typedef uint64_t ino_t;
    typedef uint64_t dev_t;
    typedef uint32_t mode_t;
    typedef uint32_t nlink_t;
    //@}

    struct tgt_timespec {
        int64_t tv_sec;
        int64_t tv_nsec;
    };

    /// Stat buffer.  Note that we can't call it 'stat' since that
    /// gets #defined to something else on some systems.
    typedef struct {
        uint64_t        st_dev;         //!< device
        uint64_t        st_ino;         //!< inode
        uint32_t        st_mode;        //!< mode
        uint32_t        st_nlink;       //!< link count
        int32_t         st_uid;         //!< owner's user ID
        int32_t         st_gid;         //!< owner's group ID
        uint64_t        st_rdev;        //!< device number
        int64_t         st_size;        //!< file size in bytes
        //struct tgt_timespec   st_atimeX;      //!< time of last access
        //struct tgt_timespec   st_mtimeX;      //!< time of last modification
        //struct tgt_timespec   st_ctimeX;      //!< time of last status change
        int64_t st_atimeX, st_mtimeX, st_ctimeX;
        int32_t         st_blksize;     //!< optimal I/O block size
        int64_t         st_blocks;      //!< number of blocks allocated
        char            st_fstype[16];
    } tgt_stat;

    // same for stat64
    typedef struct {
        uint64_t        st_dev;         //!< device
        uint64_t        st_ino;         //!< inode
        uint32_t        st_mode;        //!< mode
        uint32_t        st_nlink;       //!< link count
        int32_t         st_uid;         //!< owner's user ID
        int32_t         st_gid;         //!< owner's group ID
        uint64_t        st_rdev;        //!< device number
        int64_t         st_size;        //!< file size in bytes
        //struct tgt_timespec   st_atimeX;      //!< time of last access
        //struct tgt_timespec   st_mtimeX;      //!< time of last modification
        //struct tgt_timespec   st_ctimeX;      //!< time of last status change
        int64_t st_atimeX, st_mtimeX, st_ctimeX;
        int32_t         st_blksize;     //!< optimal I/O block size
        int64_t         st_blocks;      //!< number of blocks allocated
        char            st_fstype[16];
    } tgt_stat64;

    /// Length of strings in struct utsname (plus 1 for null char).
    static const int _SYS_NMLN = 257;

    /// Interface struct for uname().
    typedef struct utsname {
        char sysname[_SYS_NMLN];        //!< System name.
        char nodename[_SYS_NMLN];       //!< Node name.
        char release[_SYS_NMLN];        //!< OS release.
        char version[_SYS_NMLN];        //!< OS version.
        char machine[_SYS_NMLN];        //!< Machine type.
    } utsname;

    // for *at syscalls
    static const int TGT_AT_FDCWD   = -100;

};  // class Solaris

#endif // __SOLARIS_HH__
