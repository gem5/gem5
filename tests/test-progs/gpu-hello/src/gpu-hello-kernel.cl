/*
 * Copyright (c) 2014-2015 Advanced Micro Devices, Inc.
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
 * Author: Marc Orr
 */


__kernel void read_kernel(size_t code_size,
                          __global char *code_in,
                          __global int *key_arr,
                          __global char *msg_out,
                          __global int *chars_decoded)
{
    size_t gid = get_global_id(0);
    size_t my_idx = gid % code_size;
    bool decode = 0;
    __local atomic_int lcount;

    if (get_local_id(0) == 0) {
        lcount=0;
    }
    barrier(CLK_LOCAL_MEM_FENCE);

    // read code
    char mycode = code_in[my_idx];

    // decode
    int my_key = key_arr[my_idx];
    if (my_key) {
        decode = 1;
        for (int n = 0; n < my_key; n++) {
            mycode++;
        }
    }

    // write out msg
    msg_out[gid] = mycode;

    if (decode) {
        atomic_fetch_add((atomic_int *)(&lcount), 1);
    }
    barrier(CLK_LOCAL_MEM_FENCE);


    if (get_local_id(0) == 0) {
        int _lcount = atomic_load(&lcount);
        atomic_fetch_add((atomic_int *)chars_decoded, _lcount);
    }
}
