/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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
 * Author: Marc Orr, Brad Beckmann
 */

#include <CL/cl.h>
#include <malloc.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#define SUCCESS 0
#define FAILURE 1

// OpenCL datastructures
cl_context       context;
cl_device_id     *devices;
cl_command_queue commandQueue;
cl_program       program;
cl_kernel        readKernel;

// Application datastructures
const int CACHE_LINE_SIZE = 64;
size_t grid_size = 512;
size_t work_group_size = 256;

// arguments
const int code_size = 5;
const char *code = "hello";
int *keys;
char *msg;
int chars_decoded = 0;

/*
    Setup data structures for application/algorithm
*/
int
setupDataStructs()
{
    msg = (char *)memalign(CACHE_LINE_SIZE, (grid_size + 1) * sizeof(char));
    if (msg == NULL) {
        printf("%s:%d: error: %s\n", __FILE__, __LINE__,
               "could not allocate host buffers\n");
       exit(-1);
    }
    msg[grid_size] = '\0';

    keys = (int *)memalign(CACHE_LINE_SIZE, code_size * sizeof(int));
    keys[0] = 23;
    keys[1] = 0;
    keys[2] = 0;
    keys[3] = 0;
    keys[4] = 0;

    return SUCCESS;
}

/* Setup OpenCL data structures */
int
setupOpenCL()
{
    cl_int status = 0;
    size_t deviceListSize;

    // 1. Get platform
    cl_uint numPlatforms;
    cl_platform_id platform = NULL;
    status = clGetPlatformIDs(0, NULL, &numPlatforms);
    if (status != CL_SUCCESS) {
        printf("Error: Getting Platforms. (clGetPlatformsIDs)\n");
        return FAILURE;
    }

    if (numPlatforms > 0) {
        cl_platform_id *platforms = new cl_platform_id[numPlatforms];
        status = clGetPlatformIDs(numPlatforms, platforms, NULL);
        if (status != CL_SUCCESS) {
            printf("Error: Getting Platform Ids. (clGetPlatformsIDs)\n");
            return FAILURE;
        }
        for (int i = 0; i < numPlatforms; ++i) {
            char pbuff[100];
            status = clGetPlatformInfo(platforms[i], CL_PLATFORM_VENDOR,
                                       sizeof(pbuff), pbuff, NULL);
            if (status != CL_SUCCESS) {
                printf("Error: Getting Platform Info.(clGetPlatformInfo)\n");
                return FAILURE;
            }
            platform = platforms[i];
            if (!strcmp(pbuff, "Advanced Micro Devices, Inc.")) {
                break;
            }
        }
        delete platforms;
    }

    if (NULL == platform) {
        printf("NULL platform found so Exiting Application.\n");
        return FAILURE;
    }

    // 2. create context from platform
    cl_context_properties cps[3] =
        {CL_CONTEXT_PLATFORM, (cl_context_properties)platform, 0};
    context = clCreateContextFromType(cps, CL_DEVICE_TYPE_GPU, NULL, NULL,
                                      &status);
    if (status != CL_SUCCESS) {
        printf("Error: Creating Context. (clCreateContextFromType)\n");
        return FAILURE;
    }

    // 3. Get device info
    // 3a. Get # of devices
    status = clGetContextInfo(context, CL_CONTEXT_DEVICES, 0, NULL,
                              &deviceListSize);
    if (status != CL_SUCCESS) {
        printf("Error: Getting Context Info (1st clGetContextInfo)\n");
        return FAILURE;
    }

    // 3b. Get the device list data
    devices = (cl_device_id *)malloc(deviceListSize);
    if (devices == 0) {
        printf("Error: No devices found.\n");
        return FAILURE;
    }
    status = clGetContextInfo(context, CL_CONTEXT_DEVICES, deviceListSize,
                              devices, NULL);
    if (status != CL_SUCCESS) {
        printf("Error: Getting Context Info (2nd clGetContextInfo)\n");
        return FAILURE;
    }

    // 4. Create command queue for device
    commandQueue = clCreateCommandQueue(context, devices[0], 0, &status);
    if (status != CL_SUCCESS) {
        printf("Creating Command Queue. (clCreateCommandQueue)\n");
        return FAILURE;
    }

    const char *source = "dummy text";

    size_t sourceSize[] = {strlen(source)};

    // 5b. Register the kernel with the runtime
    program = clCreateProgramWithSource(context, 1, &source, sourceSize,
                                        &status);
    if (status != CL_SUCCESS) {
      printf("Error: Loading kernel (clCreateProgramWithSource)\n");
      return FAILURE;
    }

    status = clBuildProgram(program, 1, devices, NULL, NULL, NULL);
    if (status != CL_SUCCESS) {
        printf("Error: Building kernel (clBuildProgram)\n");
        return FAILURE;
    }

    readKernel = clCreateKernel(program, "read_kernel", &status);
    if (status != CL_SUCCESS) {
        printf("Error: Creating readKernel from program. (clCreateKernel)\n");
        return FAILURE;
    }

    return SUCCESS;
}


/* Run kernels */
int
runCLKernel(cl_kernel kernel)
{
    cl_int   status;
    cl_event event;
    size_t globalThreads[1] = {grid_size};
    size_t localThreads[1] = {work_group_size};

    // 1. Set arguments
    // 1a. code size
    size_t code_size = strlen(code);
    status = clSetKernelArg(kernel, 0, sizeof(size_t), &code_size);
    if (status != CL_SUCCESS) {
        printf("Error: Setting kernel argument. (code_size)\n");
        return FAILURE;
    }

    // 1b. code
    status = clSetKernelArg(kernel, 1, sizeof(char *), (void *)&code);
    if (status != CL_SUCCESS) {
        printf("Error: Setting kernel argument. (code_in)\n");
        return FAILURE;
    }

    // 1c. keys
    printf("keys = %p, &keys = %p, keys[0] = %d\n", keys, &keys, keys[0]);
    status = clSetKernelArg(kernel, 2, sizeof(int *), (void *)&keys);
    if (status != CL_SUCCESS) {
        printf("Error: Setting kernel argument. (key_arr)\n");
        return FAILURE;
    }

    // 1d. msg
    status = clSetKernelArg(kernel, 3, sizeof(char *), (void *)&msg);
    if (status != CL_SUCCESS) {
        printf("Error: Setting kernel argument. (memOut)\n");
        return FAILURE;
    }

    // 1e. chars_decoded
    int *chars_decoded_ptr = &chars_decoded;
    status = clSetKernelArg(kernel, 4, sizeof(int *),
                            (void *)&chars_decoded_ptr);
    if (status != CL_SUCCESS) {
        printf("Error: Setting kernel argument. (memOut)\n");
        return FAILURE;
    }

    // 2. Launch kernel
    status = clEnqueueNDRangeKernel(commandQueue, kernel, 1, NULL,
                                    globalThreads, localThreads, 0, NULL,
                                    &event);
    if (status != CL_SUCCESS) {
        printf("Error: Enqueue failed. (clEnqueueNDRangeKernel)\n");
        return FAILURE;
    }

    // 3. Wait for the kernel
    status = clWaitForEvents(1, &event);
    if (status != CL_SUCCESS) {
        printf("Error: Waiting for kernel run to finish. (clWaitForEvents)\n");
        return FAILURE;
    }

    // 4. Cleanup
    status = clReleaseEvent(event);
    if (status != CL_SUCCESS) {
        printf("Error: Release event object. (clReleaseEvent)\n");
        return FAILURE;
    }

    return SUCCESS;
}


/* Release OpenCL resources (Context, Memory etc.) */
int
cleanupCL()
{
    cl_int status;
    status = clReleaseKernel(readKernel);
    if (status != CL_SUCCESS) {
        printf("Error: In clReleaseKernel \n");
        return FAILURE;
    }
    status = clReleaseProgram(program);
    if (status != CL_SUCCESS) {
        printf("Error: In clReleaseProgram\n");
        return FAILURE;
    }
    status = clReleaseCommandQueue(commandQueue);
    if (status != CL_SUCCESS) {
        printf("Error: In clReleaseCommandQueue\n");
        return FAILURE;
    }
    status = clReleaseContext(context);
    if (status != CL_SUCCESS) {
        printf("Error: In clReleaseContext\n");
        return FAILURE;
    }

    return SUCCESS;
}

int
main(int argc, char * argv[])
{
    // Initialize Host application
    if (setupDataStructs() != SUCCESS) {
        return FAILURE;
    }

    // Initialize OpenCL resources
    if (setupOpenCL() != SUCCESS) {
        return FAILURE;
    }

    // Run the CL program
    if (runCLKernel(readKernel) != SUCCESS) {
        return FAILURE;
    }
    printf("the gpu says:\n");
    printf("%s\n", msg);

    // Releases OpenCL resources
    if (cleanupCL()!= SUCCESS) {
        return FAILURE;
    }

    return SUCCESS;
}
