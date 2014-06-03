/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
 *                          All Rights Reserved
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
 * Author: Yasuko Eckert
 *
 ***************************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__

#include <string>

#include "xmlParser.h"

// Macro definitions to do string comparson to specific parameter/stat.
// Note: These macros assume node_name and value variables of type XMLCSTR
//       to exist already.
#define STRCMP(var, str) else if (strcmp(var, str) == 0)

#define ASSIGN_INT_IF(str, lhs) STRCMP(node_name, str) \
lhs = atoi(value)

#define ASSIGN_FP_IF(str, lhs) STRCMP(node_name, str) \
lhs = atof(value)

#define ASSIGN_STR_IF(str, lhs) STRCMP(node_name, str) \
lhs = string(value)

#define ASSIGN_ENUM_IF(str, lhs, etype) STRCMP(node_name, str) \
lhs = (etype)atoi(value)


// Constants shared across many system components
#define BITS_PER_BYTE 8.0
#define MIN_BUFFER_SIZE 64
// CAM structures do not have any associativity
#define CAM_ASSOC 0

#endif // __COMMON_H__
