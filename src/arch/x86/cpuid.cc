/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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

#include "arch/x86/cpuid.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"

namespace X86ISA {
    enum StandardCpuidFunction {
        VendorAndLargestStdFunc,
        FamilyModelStepping,
        NumStandardCpuidFuncs
    };

    enum ExtendedCpuidFunctions {
        VendorAndLargestExtFunc,
        FamilyModelSteppingBrandFeatures,
        NameString1,
        NameString2,
        NameString3,
        L1CacheAndTLB,
        L2L3CacheAndL2TLB,
        APMInfo,
        LongModeAddressSize,
 
        /*
         * The following are defined by the spec but not yet implemented
         */
/*      // Function 9 is reserved
        SVMInfo = 10,
        // Functions 11-24 are reserved
        TLB1GBPageInfo = 25,
        PerformanceInfo,*/

        NumExtendedCpuidFuncs
    };

    static const int vendorStringSize = 13;
    static const char vendorString[vendorStringSize] = "M5 Simulator";
    static const int nameStringSize = 48;
    static const char nameString[nameStringSize] = "Fake M5 x86_64 CPU";

    uint64_t
    stringToRegister(const char *str)
    {
        uint64_t reg = 0;
        for (int pos = 3; pos >=0; pos--) {
            reg <<= 8;
            reg |= str[pos];
        }
        return reg;
    }

    bool
    doCpuid(ThreadContext * tc, uint32_t function,
            uint32_t index, CpuidResult &result)
    {
        uint16_t family = bits(function, 31, 16);
        uint16_t funcNum = bits(function, 15, 0);
        if (family == 0x8000) {
            // The extended functions
            switch (funcNum) {
              case VendorAndLargestExtFunc:
                assert(vendorStringSize >= 12);
                result = CpuidResult(
                        0x80000000 + NumExtendedCpuidFuncs - 1,
                        stringToRegister(vendorString),
                        stringToRegister(vendorString + 4),
                        stringToRegister(vendorString + 8));
                break;
              case FamilyModelSteppingBrandFeatures:
                result = CpuidResult(0x00020f51, 0x00000405,
                                     0xe3d3fbff, 0x00000001);
                break;
              case NameString1:
              case NameString2:
              case NameString3:
                {
                    // Zero fill anything beyond the end of the string. This
                    // should go away once the string is a vetted parameter.
                    char cleanName[nameStringSize];
                    memset(cleanName, '\0', nameStringSize);
                    strncpy(cleanName, nameString, nameStringSize);

                    int offset = (funcNum - NameString1) * 16;
                    assert(nameStringSize >= offset + 16);
                    result = CpuidResult(
                            stringToRegister(cleanName + offset + 0),
                            stringToRegister(cleanName + offset + 4),
                            stringToRegister(cleanName + offset + 12),
                            stringToRegister(cleanName + offset + 8));
                }
                break;
              case L1CacheAndTLB:
                result = CpuidResult(0xff08ff08, 0xff20ff20,
                                     0x40020140, 0x40020140);
                break;
              case L2L3CacheAndL2TLB:
                result = CpuidResult(0x00000000, 0x42004200,
                                     0x00000000, 0x04008140);
                break;
              case APMInfo:
                result = CpuidResult(0x80000018, 0x68747541,
                                     0x69746e65, 0x444d4163);
                break;
              case LongModeAddressSize:
                result = CpuidResult(0x0000ffff, 0x00000000,
                                     0x00000000, 0x00000000);
                break;
/*            case SVMInfo:
              case TLB1GBPageInfo:
              case PerformanceInfo:*/
              default:
                warn("x86 cpuid: unimplemented function %u", funcNum);
                return false;
            }
        } else if(family == 0x0000) {
            // The standard functions
            switch (funcNum) {
              case VendorAndLargestStdFunc:
                assert(vendorStringSize >= 12);
                result = CpuidResult(
                        NumStandardCpuidFuncs - 1,
                        stringToRegister(vendorString),
                        stringToRegister(vendorString + 4),
                        stringToRegister(vendorString + 8));
                break;
              case FamilyModelStepping:
                result = CpuidResult(0x00020f51, 0x00000805,
                                     0xe7dbfbff, 0x00000001);
                break;
              default:
                warn("x86 cpuid: unimplemented function %u", funcNum);
                return false;
            }
        } else {
            warn("x86 cpuid: unknown family %#x", family);
            return false;
        }

        return true;
    }
} // namespace X86ISA
