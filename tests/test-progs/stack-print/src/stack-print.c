/*
 * Copyright (c) 2017 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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
 * Author: Brandon Potter
 */

#include <elf.h>
#include <stdio.h>

int main(int argc, char **argv, char **envp)
{
    int i;

    printf("%p: argc: [%d]\n", &argc, argc);
    printf("\n");

    for (i = 0; i < argc; i++)
        printf("%p: argv[%d]: [%s]\n", &argv[i], i, argv[i]);
    printf("\n");

    i = 0;
    while (envp[i] != NULL) {
        printf("%p: envp[%d]: [%s]\n", &envp[i], i, envp[i]);
        i++;
    }
    printf("\n");

    Elf64_auxv_t *auxv = (Elf64_auxv_t*)&envp[--i];
    while (auxv++) {
        char *type;
        switch(auxv->a_type) {
            case AT_IGNORE:
                type = "AT_IGNORE";
                break;
            case AT_EXECFD:
                type = "AT_EXECFD";
                break;
            case AT_PHDR:
                type = "AT_PHDR";
                break;
            case AT_PHENT:
                type = "AT_PHENT";
                break;
            case AT_PHNUM:
                type = "AT_PHNUM";
                break;
            case AT_PAGESZ:
                type = "AT_PAGESZ";
                break;
            case AT_BASE:
                type = "AT_BASE";
                break;
            case AT_FLAGS:
                type = "AT_FLAGS";
                break;
            case AT_ENTRY:
                type = "AT_ENTRY";
                break;
            case AT_NOTELF:
                type = "AT_NOTELF";
                break;
            case AT_UID:
                type = "AT_UID";
                break;
            case AT_EUID:
                type = "AT_EUID";
                break;
            case AT_GID:
                type = "AT_GID";
                break;
            case AT_EGID:
                type = "AT_EGID";
                break;
            case AT_CLKTCK:
                type = "AT_CLKTCK";
                break;
            case AT_PLATFORM:
                type = "AT_PLATFORM";
                break;
            case AT_HWCAP:
                type = "AT_HWCAP";
                break;
            case AT_FPUCW:
                type = "AT_FPUCW";
                break;
            case AT_DCACHEBSIZE:
                type = "AT_DCACHEBSIZE";
                break;
            case AT_ICACHEBSIZE:
                type = "AT_ICACHEBSIZE";
                break;
            case AT_UCACHEBSIZE:
                type = "AT_UCACHEBSIZE";
                break;
            case AT_IGNOREPPC:
                type = "AT_IGNOREPPC";
                break;
            case AT_SECURE:
                type = "AT_SECURE";
                break;
            case AT_BASE_PLATFORM:
                type = "AT_BASE_PLATFORM";
                break;
            case AT_RANDOM:
                type = "AT_RANDOM";
                break;
            case AT_EXECFN:
                type = "AT_EXECFN";
                break;
            case AT_SYSINFO:
                type = "AT_SYSINFO";
                break;
            case AT_SYSINFO_EHDR:
                type = "AT_SYSINFO_EHDR";
                break;
            case AT_L1I_CACHESHAPE:
                type = "AT_L1I_CACHESHAPE";
                break;
            case AT_L1D_CACHESHAPE:
                type = "AT_L1D_CACHESHAPE";
                break;
            case AT_L2_CACHESHAPE:
                type = "AT_L2_CACHESHAPE";
                break;
            case AT_L3_CACHESHAPE:
                type = "AT_L3_CACHESHAPE";
                break;
            case AT_NULL:
            default:
                printf("\n");
                return 0;
        }
        printf("%p: %s: [%lx]\n", auxv, type, auxv->a_un.a_val);
    }
}

