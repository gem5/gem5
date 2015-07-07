/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Andreas Sandberg
 */

#include "libnomali/nomali.h"

#include <cstring>

#include "mali_t6xx.hh"
#include "mali_t7xx.hh"

#define EXPORT __attribute__ ((visibility ("default")))

static const char *errstrs[] = {
    "No error",
    "Unknown error",
    "Memory allocation failed",
    "Invalid model handle",
    "Invalid parameter",
};

static_assert(sizeof(errstrs) / sizeof(*errstrs) == NOMALI_E_NUM_ERRORS,
              "NoMali API error descriptions out of sync!");

class NoMaliApi
{
  public:
    NoMaliApi();
    ~NoMaliApi();

    void setGpu(NoMali::GPU *gpu) { _gpu = gpu; }

  public:
    nomali_error_t setCallback(const nomali_callback_t *callback);

    nomali_error_t getInfo(nomali_info_t *info);

    nomali_error_t reset();
    nomali_error_t regRead(uint32_t *value, nomali_addr_t addr);
    nomali_error_t regWrite(nomali_addr_t addr, uint32_t value);
    nomali_error_t regReadRaw(uint32_t *value, nomali_addr_t addr);
    nomali_error_t regWriteRaw(nomali_addr_t addr, uint32_t value);
    nomali_error_t intState(int *state, nomali_int_t intno) const;

  public:
    void callbackInt(nomali_int_t intno, int set);

  private:
    nomali_callback_t callbacks[NOMALI_CALLBACK_NUM_CALLBACKS];

    NoMali::GPU *_gpu;
};

template<class BaseGpu>
class NoMaliApiGpu
    : public BaseGpu
{
  public:
    template<typename... Args>
    NoMaliApiGpu(NoMaliApi &_api, Args &&... args)
        : BaseGpu(std::forward<Args>(args)...),
          api(_api)
    {
        BaseGpu::reset();
    }

  public:
    void intJob(int set) override { api.callbackInt(NOMALI_INT_JOB, set); }
    void intMMU(int set) override { api.callbackInt(NOMALI_INT_MMU, set); }
    void intGPU(int set) override { api.callbackInt(NOMALI_INT_GPU, set); }

  private:
    NoMaliApi &api;
};


NoMaliApi::NoMaliApi()
    : _gpu(nullptr)
{
    memset(callbacks, 0, sizeof(callbacks));
}


NoMaliApi::~NoMaliApi()
{
}

nomali_error_t
NoMaliApi::setCallback(const nomali_callback_t *callback)
{
    if (!callback ||
        callback->type >= NOMALI_CALLBACK_NUM_CALLBACKS)
        return NOMALI_E_INVALID;

    callbacks[callback->type] = *callback;

    return NOMALI_E_OK;
}

nomali_error_t
NoMaliApi::getInfo(nomali_info_t *info)
{
    if (!info)
        return NOMALI_E_INVALID;

    info->reg_size = 0x4000;

    return NOMALI_E_OK;
}

nomali_error_t
NoMaliApi::reset()
{
    _gpu->reset();
    return NOMALI_E_OK;
}

nomali_error_t
NoMaliApi::regRead(uint32_t *value, nomali_addr_t addr)
{
    if (!value)
        return NOMALI_E_INVALID;

    *value = _gpu->readReg(NoMali::RegAddr(addr));

    return NOMALI_E_OK;
}

nomali_error_t
NoMaliApi::regWrite(nomali_addr_t addr, uint32_t value)
{
    _gpu->writeReg(NoMali::RegAddr(addr), value);

    return NOMALI_E_OK;
}


nomali_error_t
NoMaliApi::regReadRaw(uint32_t *value, nomali_addr_t addr)
{
    if (!value)
        return NOMALI_E_INVALID;

    *value = _gpu->readRegRaw(NoMali::RegAddr(addr));

    return NOMALI_E_OK;
}

nomali_error_t
NoMaliApi::regWriteRaw(nomali_addr_t addr, uint32_t value)
{
    _gpu->writeRegRaw(NoMali::RegAddr(addr), value);

    return NOMALI_E_OK;
}

nomali_error_t
NoMaliApi::intState(int *state, nomali_int_t intno) const
{
    if (!state)
        return NOMALI_E_INVALID;

    switch (intno) {
      case NOMALI_INT_GPU:
        *state = _gpu->intGPUAsserted();
        break;

      case NOMALI_INT_JOB:
        *state = _gpu->intJobAsserted();
        break;

      case NOMALI_INT_MMU:
        *state = _gpu->intMMUAsserted();
        break;

      default:
        return NOMALI_E_INVALID;
    }

    return NOMALI_E_OK;
}


void
NoMaliApi::callbackInt(nomali_int_t intno, int set)
{
    const nomali_callback_t &c(callbacks[NOMALI_CALLBACK_INT]);

    if (c.func.interrupt)
        c.func.interrupt(static_cast<nomali_handle_t>(this), c.usr, intno, set);
}



static NoMaliApi *
get_gpu(nomali_handle_t h)
{
    return h ? static_cast<NoMaliApi *>(h) : nullptr;
}


extern "C" EXPORT nomali_api_version_t
nomali_api_version()
{
    return NOMALI_API_VERSION;
}

extern "C" EXPORT nomali_error_t
nomali_create(nomali_handle_t *h, const nomali_config_t *cfg)
{
    if (h && cfg) {
        NoMaliApi *api(new NoMaliApi());
        *h = api;
        if (!h)
            return NOMALI_E_MEMORY;

        NoMali::GPU *gpu;
        switch (cfg->type) {
          case NOMALI_GPU_T60X:
            gpu = new NoMaliApiGpu<NoMali::MaliT60x>(
                *api,
                cfg->ver_maj, cfg->ver_min, cfg->ver_status);
            break;

          case NOMALI_GPU_T62X:
            gpu = new NoMaliApiGpu<NoMali::MaliT62x>(
                *api,
                cfg->ver_maj, cfg->ver_min, cfg->ver_status);
            break;


          case NOMALI_GPU_T76X:
            gpu = new NoMaliApiGpu<NoMali::MaliT76x>(
                *api,
                cfg->ver_maj, cfg->ver_min, cfg->ver_status);
            break;

          default:
            delete api;
            return NOMALI_E_INVALID;
        };

        if (!gpu) {
            delete api;
            return NOMALI_E_MEMORY;
        }

        api->setGpu(gpu);

        return NOMALI_E_OK;
    } else {
        return NOMALI_E_INVALID;
    }
}

extern "C" EXPORT nomali_error_t
nomali_destroy(nomali_handle_t h)
{
    NoMaliApi *gpu(get_gpu(h));

    if (gpu) {
        delete gpu;
        return NOMALI_E_OK;
    } else {
        return NOMALI_E_HANDLE;
    }
}

extern "C" EXPORT const char *
nomali_errstr(nomali_error_t error)
{
    if (error < NOMALI_E_NUM_ERRORS)
        return errstrs[error];
    else
        return "Invalid error number";
}

extern "C" EXPORT nomali_error_t
nomali_set_callback(nomali_handle_t h,
                    const nomali_callback_t *callback)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->setCallback(callback) : NOMALI_E_HANDLE;
}

extern "C" EXPORT nomali_error_t
nomali_get_info(nomali_handle_t h, nomali_info_t *info)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->getInfo(info) : NOMALI_E_HANDLE;
}

extern "C" EXPORT nomali_error_t
nomali_reset(nomali_handle_t h)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->reset() : NOMALI_E_HANDLE;
}

extern "C" EXPORT nomali_error_t
nomali_reg_read(nomali_handle_t h, uint32_t *value,
                nomali_addr_t addr)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->regRead(value, addr) : NOMALI_E_HANDLE;
}

extern "C" EXPORT nomali_error_t
nomali_reg_write(nomali_handle_t h,
                 nomali_addr_t addr, uint32_t value)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->regWrite(addr, value) : NOMALI_E_HANDLE;
}


extern "C" EXPORT nomali_error_t
nomali_reg_read_raw(nomali_handle_t h, uint32_t *value,
                    nomali_addr_t addr)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->regReadRaw(value, addr) : NOMALI_E_HANDLE;
}

extern "C" EXPORT nomali_error_t
nomali_reg_write_raw(nomali_handle_t h,
                     nomali_addr_t addr, uint32_t value)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->regWriteRaw(addr, value) : NOMALI_E_HANDLE;
}

extern "C" EXPORT nomali_error_t
nomali_int_state(nomali_handle_t h, int *state,
                 nomali_int_t intno)
{
    NoMaliApi *gpu(get_gpu(h));
    return gpu ? gpu->intState(state, intno) : NOMALI_E_HANDLE;
}

