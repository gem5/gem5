/*
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Alec Roelke
 */

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>

#include "insttest.h"
#include "rv64i.h"

int main()
{
    using namespace std;
    using namespace insttest;

    // LUI
    expect<int64_t>(4096, []{return I::lui(1);}, "lui");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return I::lui(0x80000);}, "lui, negative");

    // AUIPC
    expect<bool>(true, []{return I::auipc(3);}, "auipc");

    // Jump (JAL, JALR)
    expect<bool>(true, []{return I::jal();}, "jal");
    expect<bool>(true, []{return I::jalr();}, "jalr");

    // BEQ
    expect<bool>(true, []{return I::beq(5, 5);}, "beq, equal");
    expect<bool>(false, []{return I::beq(numeric_limits<int64_t>::max(),
            numeric_limits<int64_t>::min());}, "beq, not equal");

    // BNE
    expect<bool>(false, []{return I::bne(5, 5);}, "bne, equal");
    expect<bool>(true, []{return I::bne(numeric_limits<int64_t>::max(),
            numeric_limits<int64_t>::min());}, "bne, not equal");

    // BLT
    expect<bool>(true, []{return I::blt(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::max());}, "blt, less");
    expect<bool>(false, []{return I::blt(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::min());}, "blt, equal");
    expect<bool>(false, []{return I::blt(numeric_limits<int64_t>::max(),
            numeric_limits<int64_t>::min());}, "blt, greater");

    // BGE
    expect<bool>(false, []{return I::bge(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::max());}, "bge, less");
    expect<bool>(true, []{return I::bge(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::min());}, "bge, equal");
    expect<bool>(true, []{return I::bge(numeric_limits<int64_t>::max(),
            numeric_limits<int64_t>::min());}, "bge, greater");

    // BLTU
    expect<bool>(true, []{return I::blt(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::max());}, "bltu, greater");
    expect<bool>(false, []{return I::blt(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::min());}, "bltu, equal");
    expect<bool>(false, []{return I::blt(numeric_limits<int64_t>::max(),
            numeric_limits<int64_t>::min());}, "bltu, less");

    // BGEU
    expect<bool>(false, []{return I::bge(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::max());}, "bgeu, greater");
    expect<bool>(true, []{return I::bge(numeric_limits<int64_t>::min(),
            numeric_limits<int64_t>::min());}, "bgeu, equal");
    expect<bool>(true, []{return I::bge(numeric_limits<int64_t>::max(),
            numeric_limits<int64_t>::min());}, "bgeu, less");

    // Load (LB, LH, LW, LBU, LHU)
    expect<int64_t>(7, []{return I::load<int8_t, int64_t>(0x07);},
            "lb, positive");
    expect<int64_t>(numeric_limits<int8_t>::min(),
            []{return I::load<int8_t, int64_t>(0x80);}, "lb, negative");
    expect<int64_t>(1792, []{return I::load<int16_t, int64_t>(0x0700);},
            "lh, positive");
    expect<int64_t>(numeric_limits<int16_t>::min(),
            []{return I::load<int16_t, int64_t>(0x8000);}, "lh, negative");
    expect<int64_t>(458752, []{return I::load<int32_t, int64_t>(0x00070000);},
            "lw, positive");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return I::load<int32_t, int64_t>(0x80000000);},
            "lw, negative");
    expect<uint64_t>(128, []{return I::load<uint8_t, uint64_t>(0x80);}, "lbu");
    expect<uint64_t>(32768, []{return I::load<uint16_t, uint64_t>(0x8000);},
            "lhu");

    // Store (SB, SH, SW)
    expect<uint8_t>(0xFF, []{return I::store<int8_t>(-1);}, "sb");
    expect<uint16_t>(0xFFFF, []{return I::store<int16_t>(-1);}, "sh");
    expect<uint32_t>(0xFFFFFFFF, []{return I::store<int32_t>(-1);}, "sw");

    // ADDI
    expect<int64_t>(1073742078, []{return I::addi(0x3FFFFFFF, 255);},
            "addi");
    expect<int64_t>(1, []{return I::addi(-1, 2);}, "addi, overflow");

    // SLTI
    expect<bool>(true, []{return I::slti(-1, 0);}, "slti, true");
    expect<bool>(false, []{return I::slti(0, -1);}, "slti, false");

    // SLTIU
    expect<bool>(false, []{return I::sltiu(-1, 0);}, "sltiu, false");
    expect<bool>(true, []{return I::sltiu(0, -1);}, "sltiu, true");
    expect<bool>(true, []{return I::sltiu(0xFFFF, -1);}, "sltiu, sext");

    // XORI
    expect<uint64_t>(0xFF, []{return I::xori(0xAA, 0x55);}, "xori (1)");
    expect<uint64_t>(0, []{return I::xori(0xAA, 0xAA);}, "xori (0)");

    // ORI
    expect<uint64_t>(0xFF, []{return I::ori(0xAA, 0x55);}, "ori (1)");
    expect<uint64_t>(0xAA, []{return I::ori(0xAA, 0xAA);}, "ori (A)");

    // ANDI
    expect<uint64_t>(0, []{return I::andi(-1, 0);}, "andi (0)");
    expect<uint64_t>(0x1234567812345678ULL,
            []{return I::andi(0x1234567812345678ULL, -1);}, "andi (1)");

    // SLLI
    expect<int64_t>(65280, []{return I::slli(255, 8);}, "slli, general");
    expect<int64_t>(numeric_limits<int64_t>::min(),
            []{return I::slli(255, 63);}, "slli, erase");

    // SRLI
    expect<int64_t>(255, []{return I::srli(65280, 8);}, "srli, general");
    expect<int64_t>(0, []{return I::srli(255, 8);}, "srli, erase");
    expect<int64_t>(1, []{return I::srli(numeric_limits<int64_t>::min(), 63);},
            "srli, negative");

    // SRAI
    expect<int64_t>(255, []{return I::srai(65280, 8);}, "srai, general");
    expect<int64_t>(0, []{return I::srai(255, 8);}, "srai, erase");
    expect<int64_t>(-1,
            []{return I::srai(numeric_limits<int64_t>::min(), 63);},
            "srai, negative");

    // ADD
    expect<int64_t>(1073742078, []{return I::add(0x3FFFFFFF, 255);}, "add");
    expect<int64_t>(-1,
            []{return I::add(0x7FFFFFFFFFFFFFFFLL, 0x8000000000000000LL);},
            "add, overflow");

    // SUB
    expect<int64_t>(65535, []{return I::sub(65536, 1);}, "sub");
    expect<int64_t>(-1,
            []{return I::sub(0x7FFFFFFFFFFFFFFFLL, 0x8000000000000000LL);},
            "sub, \"overflow\"");

    // SLL
    expect<int64_t>(65280, []{return I::sll(255, 8);}, "sll, general");
    expect<int64_t>(numeric_limits<int64_t>::min(),
            []{return I::sll(255, 63);}, "sll, erase");

    // SLT
    expect<bool>(true, []{return I::slt(-1, 0);}, "slt, true");
    expect<bool>(false, []{return I::slt(0, -1);}, "slt, false");

    // SLTU
    expect<bool>(false, []{return I::sltu(-1, 0);}, "sltu, false");
    expect<bool>(true, []{return I::sltu(0, -1);}, "sltu, true");

    // XOR
    expect<uint64_t>(-1,
            []{return I::xor_inst(0xAAAAAAAAAAAAAAAAULL,
                    0x5555555555555555ULL);},
            "xor (1)");
    expect<uint64_t>(0,
            []{return I::xor_inst(0xAAAAAAAAAAAAAAAAULL,
                    0xAAAAAAAAAAAAAAAAULL);},
            "xor (0)");

    // SRL
    expect<uint64_t>(255, []{return I::srl(65280, 8);}, "srl, general");
    expect<uint64_t>(0, []{return I::srl(255, 8);}, "srl, erase");
    expect<uint64_t>(1, []{return I::srl(numeric_limits<int64_t>::min(), 63);},
            "srl, negative");

    // SRA
    expect<int64_t>(255, []{return I::sra(65280, 8);}, "sra, general");
    expect<int64_t>(0, []{return I::sra(255, 8);}, "sra, erase");
    expect<int64_t>(-1, []{return I::sra(numeric_limits<int64_t>::min(), 63);},
            "sra, negative");

    // OR
    expect<uint64_t>(-1,
            []{return I::or_inst(0xAAAAAAAAAAAAAAAAULL,
                    0x5555555555555555ULL);},
            "or (1)");
    expect<uint64_t>(0xAAAAAAAAAAAAAAAAULL,
            []{return I::or_inst(0xAAAAAAAAAAAAAAAAULL,
                    0xAAAAAAAAAAAAAAAAULL);},
            "or (A)");

    // AND
    expect<uint64_t>(0, []{return I::and_inst(-1, 0);}, "and (0)");
    expect<uint64_t>(0x1234567812345678ULL,
            []{return I::and_inst(0x1234567812345678ULL, -1);}, "and (-1)");

    // FENCE/FENCE.I
    asm volatile("fence" : : );
    asm volatile("fence.i" : : );

    // ECALL
    char fname[] = "test.txt";
    char teststr[] = "this is a test";
    expect<bool>(true, [=]{
            int fd = open(fname, O_CREAT | O_WRONLY | O_TRUNC, 0644);
            if (fd < 0) {
                return false;
            }
            size_t n = write(fd, teststr, sizeof(teststr));
            cout << "Bytes written: " << n << endl;
            return close(fd) >= 0 && n > 0;
        }, "open, write");
    expect<int>(0, [=]{return access(fname, F_OK);}, "access F_OK");
    expect<int>(0, [=]{return access(fname, R_OK);}, "access R_OK");
    expect<int>(0, [=]{return access(fname, W_OK);}, "access W_OK");
    // gem5's implementation of access is incorrect; it should return
    // -1 on failure, not -errno.  Account for this using an inequality.
    expect<bool>(true, [=]{return access(fname, X_OK) != 0;}, "access X_OK");
    expect<bool>(true, [=]{
            struct stat stat_buf, fstat_buf;
            int s = stat(fname, &stat_buf);
            if (s < 0) {
                return false;
            } else {
                cout << "stat:" << endl;
                cout << "\tst_dev =\t" << stat_buf.st_dev << endl;
                cout << "\tst_ino =\t" << stat_buf.st_ino << endl;
                cout << "\tst_mode =\t" << stat_buf.st_mode << endl;
                cout << "\tst_nlink =\t" << stat_buf.st_nlink << endl;
                cout << "\tst_uid =\t" << stat_buf.st_uid << endl;
                cout << "\tst_gid =\t" << stat_buf.st_gid << endl;
                cout << "\tst_rdev =\t" << stat_buf.st_rdev << endl;
                cout << "\tst_size =\t" << stat_buf.st_size << endl;
                cout << "\tst_blksize =\t" << stat_buf.st_blksize << endl;
                cout << "\tst_blocks =\t" << stat_buf.st_blocks << endl;
            }
            int fd = open(fname, O_RDONLY);
            if (fd < 0) {
                return false;
            }
            int f = fstat(fd, &fstat_buf);
            if (f >= 0) {
                cout << "fstat:" << endl;
                cout << "\tst_dev =\t" << fstat_buf.st_dev << endl;
                cout << "\tst_ino =\t" << fstat_buf.st_ino << endl;
                cout << "\tst_mode =\t" << fstat_buf.st_mode << endl;
                cout << "\tst_nlink =\t" << fstat_buf.st_nlink << endl;
                cout << "\tst_uid =\t" << fstat_buf.st_uid << endl;
                cout << "\tst_gid =\t" << fstat_buf.st_gid << endl;
                cout << "\tst_rdev =\t" << fstat_buf.st_rdev << endl;
                cout << "\tst_size =\t" << fstat_buf.st_size << endl;
                cout << "\tst_blksize =\t" << fstat_buf.st_blksize << endl;
                cout << "\tst_blocks =\t" << fstat_buf.st_blocks << endl;
            }
            return close(fd) >= 0 && f >= 0;
        }, "open, stat");
    expect<bool>(true, [=]{
            int fd = open(fname, O_RDONLY);
            if (fd < 0) {
                return false;
            }
            char in[128];
            size_t n = read(fd, in, sizeof(in));
            cout << "Bytes read: " << n << endl;
            cout << "String read: " << in << endl;
            int cl = close(fd);
            int un = unlink(fname);
            return n > 0 && cl >= 0 && un >= 0 && strcmp(teststr, in) == 0;
        }, "open, read, unlink");
    expect<bool>(true, []{
            struct tms buf;
            clock_t t = times(&buf);
            cout << "times:" << endl;
            cout << "\ttms_utime =\t" << buf.tms_utime << endl;
            cout << "\ttms_stime =\t" << buf.tms_stime << endl;
            cout << "\ttms_cutime =\t" << buf.tms_cutime << endl;
            cout << "\ttms_cstime =\t" << buf.tms_cstime << endl;
            return t > 0;
        }, "times");
    expect<int>(0, []{
            struct timeval time;
            int res = gettimeofday(&time, nullptr);
            cout << "timeval:" << endl;
            cout << "\ttv_sec =\t" << time.tv_sec << endl;
            cout << "\ttv_usec =\t" << time.tv_usec << endl;
            return res;
        }, "gettimeofday");

    // EBREAK not tested because it only makes sense in FS mode or when
    // using gdb

    // ERET not tested because it only makes sense in FS mode and will cause
    // a panic when used in SE mode

    // CSRs (RDCYCLE, RDTIME, RDINSTRET)
    expect<bool>(true, []{
                uint64_t cycles = 0;
                asm("rdcycle %0" : "=r" (cycles));
                cout << "Cycles: " << cycles << endl;
                return cycles > 0;
            }, "rdcycle");
    expect<bool>(true, []{
                uint64_t time = 0;
                asm("rdtime %0" : "=r" (time));
                cout << "Time: " << time << endl;
                return time > 0;
            }, "rdtime");
    expect<bool>(true, []{
                uint64_t instret = 0;
                asm("rdinstret %0" : "=r" (instret));
                cout << "Instructions Retired: " << instret << endl;
                return instret > 0;
            }, "rdinstret");

    // 64-bit memory (LWU, LD, SD)
    expect<int64_t>(0xFFFFFFFF, []{return I::load<uint32_t, uint64_t>(-1);},
            "lwu");
    expect<int64_t>(30064771072,
            []{return I::load<int64_t, int64_t>(30064771072);}, "ld");
    expect<uint64_t>(-1, []{return I::store<int64_t>(-1);}, "sd");

    // ADDIW
    expect<int64_t>(268435710, []{return I::addiw(0x0FFFFFFF, 255);}, "addiw");
    expect<int64_t>(-2147481602, []{return I::addiw(0x7FFFFFFF, 0x7FF);},
            "addiw, overflow");
    expect<int64_t>(0, []{return I::addiw(0x7FFFFFFFFFFFFFFFLL, 1);},
            "addiw, truncate");

    // SLLIW
    expect<int64_t>(65280, []{return I::slliw(255, 8);}, "slliw, general");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return I::slliw(255, 31);}, "slliw, erase");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return I::slliw(0xFFFFFFFF00800000LL, 8);}, "slliw, truncate");

    // SRLIW
    expect<int64_t>(255, []{return I::srliw(65280, 8);}, "srliw, general");
    expect<int64_t>(0, []{return I::srliw(255, 8);}, "srliw, erase");
    expect<int64_t>(1,
            []{return I::srliw(numeric_limits<int32_t>::min(), 31);},
            "srliw, negative");
    expect<int64_t>(1, []{return I::srliw(0xFFFFFFFF80000000LL, 31);},
            "srliw, truncate");

    // SRAIW
    expect<int64_t>(255, []{return I::sraiw(65280, 8);}, "sraiw, general");
    expect<int64_t>(0, []{return I::sraiw(255, 8);}, "sraiw, erase");
    expect<int64_t>(-1,
            []{return I::sraiw(numeric_limits<int32_t>::min(), 31);},
            "sraiw, negative");
    expect<int64_t>(-1, []{return I::sraiw(0x0000000180000000LL, 31);},
            "sraiw, truncate");

    // ADDW
    expect<int64_t>(1073742078, []{return I::addw(0x3FFFFFFF, 255);}, "addw");
    expect<int64_t>(-1, []{return I::addw(0x7FFFFFFF, 0x80000000);},
            "addw, overflow");
    expect<int64_t>(65536, []{return I::addw(0xFFFFFFFF0000FFFFLL, 1);},
            "addw, truncate");

    // SUBW
    expect<int64_t>(65535, []{return I::subw(65536, 1);}, "subw");
    expect<int64_t>(-1, []{return I::subw(0x7FFFFFFF, 0x80000000);},
            "subw, \"overflow\"");
    expect<int64_t>(0,
            []{return I::subw(0xAAAAAAAAFFFFFFFFULL, 0x55555555FFFFFFFFULL);},
            "subw, truncate");

    // SLLW
    expect<int64_t>(65280, []{return I::sllw(255, 8);}, "sllw, general");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return I::sllw(255, 31);}, "sllw, erase");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return I::sllw(0xFFFFFFFF00008000LL, 16);}, "sllw, truncate");

    // SRLW
    expect<uint64_t>(255, []{return I::srlw(65280, 8);}, "srlw, general");
    expect<uint64_t>(0, []{return I::srlw(255, 8);}, "srlw, erase");
    expect<uint64_t>(1,
            []{return I::srlw(numeric_limits<int32_t>::min(), 31);},
            "srlw, negative");
    expect<uint64_t>(1, []{return I::srlw(0x0000000180000000LL, 31);},
            "srlw, truncate");

    // SRAW
    expect<int64_t>(255, []{return I::sraw(65280, 8);}, "sraw, general");
    expect<int64_t>(0, []{return I::sraw(255, 8);}, "sraw, erase");
    expect<int64_t>(-1,
            []{return I::sraw(numeric_limits<int32_t>::min(), 31);},
            "sraw, negative");
    expect<int64_t>(1, []{return I::sraw(0xFFFFFFFF40000000LL, 30);},
            "sraw, truncate");

    return 0;
}
