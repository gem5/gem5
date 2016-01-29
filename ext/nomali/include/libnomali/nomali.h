/*
 * Copyright (c) 2014-2016 ARM Limited
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

#ifndef _LIBNOMALI_NOMALI_HH
#define _LIBNOMALI_NOMALI_HH

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file libnomali/nomali.h
 * @short This header defines the NoMali stub GPU model interface.
 *
 */

/** Opaque NoMali model handle. */
typedef void* nomali_handle_t;

/**
 * NoMali error codes.
 */
enum {
    /** No error */
    NOMALI_E_OK = 0,
    /** Unknown error */
    NOMALI_E_UNKNOWN,
    /** Memory allocation failed */
    NOMALI_E_MEMORY,
    /** Invalid model handle */
    NOMALI_E_HANDLE,
    /** Invalid parameter */
    NOMALI_E_INVALID,

    /**
     * Number of errors defined
     *
     * @note This error, and higher error numbers, can be issued if
     * the library is newer than the header file. Software should
     * tread this condition as an unknown error.
     */
    NOMALI_E_NUM_ERRORS
};
typedef int nomali_error_t;

enum {
    NOMALI_GPU_T60X = 0,
    NOMALI_GPU_T62X,
    NOMALI_GPU_T76X,

    NOMALI_GPU_T760 = NOMALI_GPU_T76X,
};
typedef int nomali_gpu_type_t;

typedef struct {
    nomali_gpu_type_t type;

    unsigned ver_maj;
    unsigned ver_min;
    unsigned ver_status;
} nomali_config_t;

enum {
    /** Model is signalling an interrupt */
    NOMALI_CALLBACK_INT = 0,
    /** Model read physical memory callback */
    NOMALI_CALLBACK_MEMREAD,
    /** Model write physical memory callback */
    NOMALI_CALLBACK_MEMWRITE,
    /** Model reset callback */
    NOMALI_CALLBACK_RESET,

    /** Number of defined callbacks */
    NOMALI_CALLBACK_NUM_CALLBACKS
};
typedef int nomali_callback_type_t;

enum {
    NOMALI_INT_GPU = 0,
    NOMALI_INT_JOB,
    NOMALI_INT_MMU,
};
typedef int nomali_int_t;

typedef uint64_t nomali_addr_t;
typedef uint64_t nomali_size_t;

/**
 * Callback information structure.
 */
typedef struct {
    /** Callback type */
    nomali_callback_type_t type;
    /** Pointer to user-defined data associated with callback */
    void *usr;
    /** Pointer to callback function */
    union {
        /**
         * Interrupt state change
         *
         * @param h Model instance handle.
         * @param usr User-defined data associated with callback.
         * @param intno Interrupt number.
         * @param set Non-zero if raising an interrupt, zero if clearing.
         */
        void (*interrupt)(nomali_handle_t h, void *usr,
                          nomali_int_t intno, int set);
        void (*memwrite)(nomali_handle_t h, void *usr,
                         nomali_addr_t addr, uint32_t value);
        uint32_t (*memread)(nomali_handle_t h, void *usr,
                            nomali_addr_t addr);
        void (*reset)(nomali_handle_t h, void *usr);
    } func;
} nomali_callback_t;

/**
 * GPU information struct. See nomali_get_info().
 */
typedef struct {
    /** Size (in bytes) of the register window used by the GPU */
    nomali_size_t reg_size;
} nomali_info_t;

typedef uint32_t nomali_api_version_t;

/**
 * Current version of the NoMali API
 *
 * This version number will increase whenever the API changes.
 *
 * @see nomali_api_version()
 */
#define NOMALI_API_VERSION 0

/**
 * Get the version of the API implemented by the library.
 *
 * Before instantiating a NoMali model, the driving application need
 * to ensure that the library implements a compatible version of the
 * NoMali API. This is done by calling this function and matching the
 * return value with the NOMALI_API_VERSION define. The result of any
 * call to the NoMali library is undefined if there is a miss-match
 * between the two.
 */
nomali_api_version_t nomali_api_version();

/**
 * Create an instance of the NoMali model.
 *
 * @param[out] h Handle of the new NoMali model instance, undefined on
 *               error.
 *
 * @param[in] cfg NoMali GPU configuration.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_MEMORY if a memory allocation failed.
 * @error NOMALI_E_INVALID if a pointer to an output parameter is
 *                         invalid.
 */
nomali_error_t nomali_create(nomali_handle_t *h, const nomali_config_t *cfg);
/**
 * Destroy and free resources used by an existing NoMali instance.
 *
 * @param[in] h Model instance handle.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 */
nomali_error_t nomali_destroy(nomali_handle_t h);


/**
 * Get a textual description of an error number.
 *
 * @param[in] error Error number to resolve.
 *
 * @return Pointer to a constant, null-terminated, string describing
 * an error number.
 */
const char *nomali_errstr(nomali_error_t error);

/**
 * Setup callbacks from the model.
 *
 * @param[in] h        Model instance handle.
 * @param[in] callback Structure describing the new callback to be
 *                     installed.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 * @error NOMALI_E_INVALID if the callback type was invalid.
 *
 * @see nomali_callback_t
 */
nomali_error_t nomali_set_callback(nomali_handle_t h,
                                   const nomali_callback_t *callback);

/**
 * Get information about the hardware simulated by the model.
 *
 * @param[in]  h       Model instance handle.
 * @param[out] info    Structure describing the model.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 * @error NOMALI_E_INVALID if info is not pointing to a valid
 *                         location.
 *
 * @see nomali_info_t
 */
nomali_error_t nomali_get_info(nomali_handle_t h,
                               nomali_info_t *info);

/**
 * Perform a reset of the device.
 *
 * @param[in]  h     Model instance handle.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 */
nomali_error_t nomali_reset(nomali_handle_t h);

/**
 * Read a register within the device.
 *
 * @param[in]  h     Model instance handle.
 * @param[out] value Pointer to output.
 * @param[in]  addr  Address to read.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 * @error NOMALI_E_INVALID if an invalid register was specified or if the
 *                         pointer to the output location was invalid.
 */
nomali_error_t nomali_reg_read(nomali_handle_t h, uint32_t *value,
                               nomali_addr_t addr);

/**
 * Write to a register within the device.
 *
 * @param[in] h     Model instance handle.
 * @param[in] addr  Address to read.
 * @param[in] value Value to write to the register.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 * @error NOMALI_E_INVALID if an invalid register was specified.
 */
nomali_error_t nomali_reg_write(nomali_handle_t h,
                                nomali_addr_t addr, uint32_t value);

/**
 * Read a register without side effects.
 *
 * @param[in]  h     Model instance handle.
 * @param[out] value Pointer to output.
 * @param[in]  addr  Address to read.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 * @error NOMALI_E_INVALID if an invalid register was specified or if the
 *                         pointer to the output location was invalid.
 */
nomali_error_t nomali_reg_read_raw(nomali_handle_t h, uint32_t *value,
                                   nomali_addr_t addr);

/**
 * Write to a register without side effects.
 *
 * @param[in] h     Model instance handle.
 * @param[in] addr  Address to read.
 * @param[in] value Value to write to the register.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 * @error NOMALI_E_INVALID if an invalid register was specified.
 */
nomali_error_t nomali_reg_write_raw(nomali_handle_t h,
                                    nomali_addr_t addr, uint32_t value);

/**
 * Get the state of an interrupt line
 *
 * This function queries the state of one of the GPU's interrupt
 * lines. The state of the interrupt line is returned in 'state',
 * which is 1 if the interrupt is being asserted and 0 otherwise. The
 * value of the state variable is undefined if the function call
 * fails.
 *
 * @param[in]  h     Model instance handle.
 * @param[out] state Pointer to output, 1 if the interrupt is
 *                   asserted, 0 otherwise.
 * @param[in]  intno Interrupt to query.
 *
 * @errors
 * @error NOMALI_E_OK on success.
 * @error NOMALI_E_HANDLE if the handle was invalid.
 * @error NOMALI_E_INVALID if an invalid interrupt was specified or if
 *                         pointer to the output location was invalid.
 */
nomali_error_t nomali_int_state(nomali_handle_t h, int *state,
                                nomali_int_t intno);

#ifdef __cplusplus
};
#endif

#endif /* _LIBNOMALI_NOMALI_HH */
