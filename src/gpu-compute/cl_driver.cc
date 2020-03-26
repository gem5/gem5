/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Anthony Gutierrez
 */

#include "gpu-compute/cl_driver.hh"

#include <memory>

#include "base/intmath.hh"
#include "cpu/thread_context.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/hsa_code.hh"
#include "gpu-compute/hsa_kernel_info.hh"
#include "gpu-compute/hsa_object.hh"
#include "params/ClDriver.hh"
#include "sim/process.hh"
#include "sim/syscall_emul_buf.hh"

ClDriver::ClDriver(ClDriverParams *p)
    : EmulatedDriver(p), hsaCode(0)
{
    for (const auto &codeFile : p->codefile)
        codeFiles.push_back(&codeFile);

    maxFuncArgsSize = 0;

    for (int i = 0; i < codeFiles.size(); ++i) {
        HsaObject *obj = HsaObject::createHsaObject(*codeFiles[i]);

        for (int k = 0; k < obj->numKernels(); ++k) {
            assert(obj->getKernel(k));
            kernels.push_back(obj->getKernel(k));
            kernels.back()->setReadonlyData((uint8_t*)obj->readonlyData);
            int kern_funcargs_size = kernels.back()->funcarg_size;
            maxFuncArgsSize = maxFuncArgsSize < kern_funcargs_size ?
                kern_funcargs_size : maxFuncArgsSize;
        }
    }

    int name_offs = 0;
    int code_offs = 0;

    for (int i = 0; i < kernels.size(); ++i) {
        kernelInfo.push_back(HsaKernelInfo());
        HsaCode *k = kernels[i];

        k->generateHsaKernelInfo(&kernelInfo[i]);

        kernelInfo[i].name_offs = name_offs;
        kernelInfo[i].code_offs = code_offs;

        name_offs += k->name().size() + 1;
        code_offs += k->numInsts() * sizeof(TheGpuISA::RawMachInst);
    }
}

void
ClDriver::handshake(GpuDispatcher *_dispatcher)
{
    dispatcher = _dispatcher;
    dispatcher->setFuncargsSize(maxFuncArgsSize);
}

int
ClDriver::open(ThreadContext *tc, int mode, int flags)
{
    auto p = tc->getProcessPtr();
    std::shared_ptr<DeviceFDEntry> fdp;
    fdp = std::make_shared<DeviceFDEntry>(this, filename);
    int tgt_fd = p->fds->allocFD(fdp);
    return tgt_fd;
}

int
ClDriver::ioctl(ThreadContext *tc, unsigned req, Addr buf_addr)
{
    switch (req) {
      case HSA_GET_SIZES:
        {
            TypedBufferArg<HsaDriverSizes> sizes(buf_addr);
            sizes->num_kernels = kernels.size();
            sizes->string_table_size = 0;
            sizes->code_size = 0;
            sizes->readonly_size = 0;

            if (kernels.size() > 0) {
                // all kernels will share the same read-only memory
                sizes->readonly_size =
                    kernels[0]->getSize(HsaCode::MemorySegment::READONLY);
                // check our assumption
                for (int i = 1; i<kernels.size(); ++i) {
                    assert(sizes->readonly_size ==
                    kernels[i]->getSize(HsaCode::MemorySegment::READONLY));
                }
            }

            for (int i = 0; i < kernels.size(); ++i) {
                HsaCode *k = kernels[i];
                // add one for terminating '\0'
                sizes->string_table_size += k->name().size() + 1;
                sizes->code_size +=
                    k->numInsts() * sizeof(TheGpuISA::RawMachInst);
            }

            sizes.copyOut(tc->getVirtProxy());
        }
        break;

      case HSA_GET_KINFO:
        {
            TypedBufferArg<HsaKernelInfo>
                kinfo(buf_addr, sizeof(HsaKernelInfo) * kernels.size());

            for (int i = 0; i < kernels.size(); ++i) {
                HsaKernelInfo *ki = &kinfo[i];
                ki->name_offs = kernelInfo[i].name_offs;
                ki->code_offs = kernelInfo[i].code_offs;
                ki->sRegCount = kernelInfo[i].sRegCount;
                ki->dRegCount = kernelInfo[i].dRegCount;
                ki->cRegCount = kernelInfo[i].cRegCount;
                ki->static_lds_size  = kernelInfo[i].static_lds_size;
                ki->private_mem_size = kernelInfo[i].private_mem_size;
                ki->spill_mem_size   = kernelInfo[i].spill_mem_size;
            }

            kinfo.copyOut(tc->getVirtProxy());
        }
        break;

      case HSA_GET_STRINGS:
        {
            int string_table_size = 0;
            for (int i = 0; i < kernels.size(); ++i) {
                HsaCode *k = kernels[i];
                string_table_size += k->name().size() + 1;
            }

            BufferArg buf(buf_addr, string_table_size);
            char *bufp = (char*)buf.bufferPtr();

            for (int i = 0; i < kernels.size(); ++i) {
                HsaCode *k = kernels[i];
                const char *n = k->name().c_str();

                // idiomatic string copy
                while ((*bufp++ = *n++));
            }

            assert(bufp - (char *)buf.bufferPtr() == string_table_size);

            buf.copyOut(tc->getVirtProxy());
        }
        break;

      case HSA_GET_READONLY_DATA:
        {
            // we can pick any kernel --- they share the same
            // readonly segment (this assumption is checked in GET_SIZES)
            uint64_t size =
                kernels.back()->getSize(HsaCode::MemorySegment::READONLY);
            BufferArg data(buf_addr, size);
            char *datap = (char *)data.bufferPtr();
            memcpy(datap,
                   kernels.back()->readonly_data,
                   size);
            data.copyOut(tc->getVirtProxy());
        }
        break;

      case HSA_GET_CODE:
        {
            // set hsaCode pointer
            hsaCode = buf_addr;
            int code_size = 0;

            for (int i = 0; i < kernels.size(); ++i) {
                HsaCode *k = kernels[i];
                code_size += k->numInsts() * sizeof(TheGpuISA::RawMachInst);
            }

            TypedBufferArg<TheGpuISA::RawMachInst> buf(buf_addr, code_size);
            TheGpuISA::RawMachInst *bufp = buf;

            int buf_idx = 0;

            for (int i = 0; i < kernels.size(); ++i) {
                HsaCode *k = kernels[i];

                for (int j = 0; j < k->numInsts(); ++j) {
                    bufp[buf_idx] = k->insts()->at(j);
                    ++buf_idx;
                }
            }

            buf.copyOut(tc->getVirtProxy());
        }
        break;

      case HSA_GET_CU_CNT:
        {
            BufferArg buf(buf_addr, sizeof(uint32_t));
            *((uint32_t*)buf.bufferPtr()) = dispatcher->getNumCUs();
            buf.copyOut(tc->getVirtProxy());
        }
        break;

      case HSA_GET_VSZ:
        {
            BufferArg buf(buf_addr, sizeof(uint32_t));
            *((uint32_t*)buf.bufferPtr()) = dispatcher->wfSize();
            buf.copyOut(tc->getVirtProxy());
        }
        break;
      case HSA_GET_HW_STATIC_CONTEXT_SIZE:
        {
            BufferArg buf(buf_addr, sizeof(uint32_t));
            *((uint32_t*)buf.bufferPtr()) = dispatcher->getStaticContextSize();
            buf.copyOut(tc->getVirtProxy());
        }
        break;

      default:
        fatal("ClDriver: bad ioctl %d\n", req);
    }

    return 0;
}

const char*
ClDriver::codeOffToKernelName(uint64_t code_ptr)
{
    assert(hsaCode);
    uint32_t code_offs = code_ptr - hsaCode;

    for (int i = 0; i < kernels.size(); ++i) {
        if (code_offs == kernelInfo[i].code_offs) {
            return kernels[i]->name().c_str();
        }
    }

    return nullptr;
}

ClDriver*
ClDriverParams::create()
{
    return new ClDriver(this);
}
