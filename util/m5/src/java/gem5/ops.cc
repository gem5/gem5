/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 */

#include <stdint.h>
#include <iostream>
#include <string>

#include "call_type/addr_dt.hh"
#include "call_type/inst_dt.hh"
#include "call_type/semi_dt.hh"
#include "gem5/m5ops.h"
#include "java/gem5_Ops.h"
#include "m5_mmap.h"

#define _stringify(x) #x
#define stringify(x) _stringify(x)

struct JavaCallType
{
    const std::string name;
    DispatchTable *dt;
};

JavaCallType java_call_types[] = {
#if CALL_TYPE_addr_ENABLED
    { "addr", &addr_dispatch },
#endif
#if CALL_TYPE_inst_ENABLED
    { "inst", &inst_dispatch },
#endif
#if CALL_TYPE_semi_ENABLED
    { "semi", &semi_dispatch },
#endif
};

/*
 * C library interface for gem5Op JNI
 */

JNIEXPORT void JNICALL
Java_gem5_Ops_setupCallTypes(JNIEnv *env, jclass clazz)
{
    jclass map_class = env->FindClass("java/util/HashMap");
    jmethodID map_constr_id = env->GetMethodID(map_class, "<init>", "()V");
    jobject map = env->NewObject(map_class, map_constr_id);

    jfieldID map_field_id = env->GetStaticFieldID(
            clazz, "_callTypes", "Ljava/util/Map;");
    env->SetStaticObjectField(clazz, map_field_id, map);

    jmethodID map_put_id = env->GetMethodID(map_class, "put",
            "(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;");

    jmethodID ops_constr_id = env->GetMethodID(clazz, "<init>", "()V");
    jfieldID ptr_field_id = env->GetFieldID(clazz, "dispatchTablePtr", "J");

    for (const auto &ct: java_call_types) {
        jobject ops = env->NewObject(clazz, ops_constr_id);
        env->SetLongField(ops, ptr_field_id, (int64_t)(intptr_t)ct.dt);

        jstring name = env->NewStringUTF(ct.name.c_str());

        env->CallObjectMethod(map, map_put_id, name, ops);

        if (ct.name == stringify(CALL_TYPE_DEFAULT)) {
            jstring default_name = env->NewStringUTF("default");
            env->CallObjectMethod(map, map_put_id, default_name, ops);
            env->DeleteLocalRef(default_name);
        }

        env->DeleteLocalRef(name);
        env->DeleteLocalRef(ops);
    }

    env->DeleteLocalRef(map);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_setAddr(JNIEnv *env, jclass clazz, jlong addr)
{
    m5op_addr = addr;
}

JNIEXPORT void JNICALL
Java_gem5_Ops_mapMem(JNIEnv *env, jclass clazz)
{
    map_m5_mem();
}

JNIEXPORT void JNICALL
Java_gem5_Ops_unmapMem(JNIEnv *env, jclass clazz)
{
    unmap_m5_mem();
}

static DispatchTable *
getDispatchTable(JNIEnv *env, jobject obj)
{
    jclass clazz = env->GetObjectClass(obj);
    jfieldID ptr_field_id = env->GetFieldID(clazz, "dispatchTablePtr", "J");
    return (DispatchTable *)(intptr_t)(env->GetLongField(obj, ptr_field_id));
}

JNIEXPORT void JNICALL
Java_gem5_Ops_arm(JNIEnv *env, jobject obj, jlong j_address)
{
    getDispatchTable(env, obj)->m5_arm(j_address);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_quiesce(JNIEnv *env, jobject obj)
{
    getDispatchTable(env, obj)->m5_quiesce();
}

JNIEXPORT void JNICALL
Java_gem5_Ops_quiesce_1ns(JNIEnv *env, jobject obj, jlong j_ns)
{
    getDispatchTable(env, obj)->m5_quiesce_ns(j_ns);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_quiesce_1cycle(JNIEnv *env, jobject obj, jlong j_cycles)
{
    getDispatchTable(env, obj)->m5_quiesce_cycle(j_cycles);
}

JNIEXPORT jlong JNICALL
Java_gem5_Ops_quiesce_1time(JNIEnv *env, jobject obj)
{
    uint64_t time = getDispatchTable(env, obj)->m5_quiesce_time();
    if (time & 0x8000000000000000ULL)
        printf("Truncated return value from quiesceTime() to 63 bits\n");
    return (time & 0x7FFFFFFFFFFFFFFFULL);
}

JNIEXPORT jlong JNICALL
Java_gem5_Ops_rpns(JNIEnv *env, jobject obj)
{
    uint64_t time = getDispatchTable(env, obj)->m5_rpns();
    if (time & 0x8000000000000000ULL)
        printf("Truncated return value from rpns() to 63 bits\n");
    return (time & 0x7FFFFFFFFFFFFFFFULL);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_wake_1cpu(JNIEnv *env, jobject obj, jlong j_cpuid)
{
    getDispatchTable(env, obj)->m5_wake_cpu(j_cpuid);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_exit(JNIEnv *env, jobject obj, jlong j_ns_delay)
{
    getDispatchTable(env, obj)->m5_exit(j_ns_delay);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_fail(JNIEnv *env, jobject obj, jlong j_ns_delay, jlong j_code)
{
    getDispatchTable(env, obj)->m5_fail(j_ns_delay, j_code);
}

JNIEXPORT jlong JNICALL
Java_gem5_Ops_sum(JNIEnv *env, jobject obj, jlong a, jlong b, jlong c,
                    jlong d, jlong e, jlong f)
{
    uint64_t result = getDispatchTable(env, obj)->m5_sum(a, b, c, d, e, f);
    if (result & 0x8000000000000000ULL)
        printf("Truncated return value from sum() to 63 bits\n");
    return (result & 0x7FFFFFFFFFFFFFFFULL);
}

JNIEXPORT jlong JNICALL
Java_gem5_Ops_init_1param(JNIEnv *env, jobject obj, jlong j_key_str1,
                           jlong j_key_str2)
{
    uint64_t param = getDispatchTable(env, obj)->m5_init_param(
            j_key_str1, j_key_str2);
    if (param & 0x8000000000000000ULL)
        printf("Truncated return value from m_initparam() to 63 bits\n");
    return (param & 0x7FFFFFFFFFFFFFFFULL);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_checkpoint(JNIEnv *env, jobject obj,
                           jlong j_ns_delay, jlong j_ns_period)
{
    getDispatchTable(env, obj)->m5_checkpoint(j_ns_delay, j_ns_period);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_reset_1stats(JNIEnv *env, jobject obj,
                             jlong j_ns_delay, jlong j_ns_period)
{
    getDispatchTable(env, obj)->m5_reset_stats(j_ns_delay, j_ns_period);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_dump_1stats(JNIEnv *env, jobject obj,
                            jlong j_ns_delay, jlong j_ns_period)
{
    getDispatchTable(env, obj)->m5_dump_stats(j_ns_delay, j_ns_period);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_dump_1reset_1stats(JNIEnv *env, jobject obj,
                                  jlong j_ns_delay, jlong j_ns_period)
{
    getDispatchTable(env, obj)->m5_dump_reset_stats(j_ns_delay, j_ns_period);
}

JNIEXPORT jlong JNICALL
Java_gem5_Ops_read_1file(JNIEnv *env, jobject obj,
                           jbyteArray j_buffer, jlong j_len, jlong j_offset)
{
    jbyte *buffer = env->GetByteArrayElements(j_buffer, 0);

    uint64_t result = getDispatchTable(env, obj)->m5_read_file(
            buffer, j_len, j_offset);

    env->ReleaseByteArrayElements(j_buffer, buffer, JNI_ABORT);
    return (result & 0x7FFFFFFFFFFFFFFFULL);
}

JNIEXPORT jlong JNICALL
Java_gem5_Ops_write_1file(JNIEnv *env, jobject obj,
                            jbyteArray j_buffer, jlong j_len, jlong j_offset,
                            jstring j_filename)
{
    jbyte *buffer = env->GetByteArrayElements(j_buffer, 0);
    const char *filename = env->GetStringUTFChars(j_filename, NULL);

    uint64_t result = getDispatchTable(env, obj)->m5_write_file(
            buffer, j_len, j_offset, filename);

    env->ReleaseStringUTFChars(j_filename, filename);
    env->ReleaseByteArrayElements(j_buffer, buffer, JNI_ABORT);
    return (result & 0x7FFFFFFFFFFFFFFFULL);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_debug_1break(JNIEnv *env, jobject obj)
{
    getDispatchTable(env, obj)->m5_debug_break();
}

JNIEXPORT void JNICALL
Java_gem5_Ops_switch_1cpu (JNIEnv *env, jobject obj)
{
    getDispatchTable(env, obj)->m5_switch_cpu();
}

JNIEXPORT void JNICALL
Java_gem5_Ops_dist_1toggle_1sync(JNIEnv *env, jobject obj)
{
    getDispatchTable(env, obj)->m5_dist_toggle_sync();
}

JNIEXPORT void JNICALL
Java_gem5_Ops_add_symbol(JNIEnv *env, jobject obj,
        jlong j_addr, jstring j_symbol)
{
    const char *symbol = env->GetStringUTFChars(j_symbol, NULL);

    getDispatchTable(env, obj)->m5_add_symbol(j_addr, symbol);

    env->ReleaseStringUTFChars(j_symbol, symbol);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_load_1symbol(JNIEnv *env, jobject obj)
{
    getDispatchTable(env, obj)->m5_load_symbol();
}

JNIEXPORT void JNICALL
Java_gem5_Ops_panic(JNIEnv *env, jobject obj)
{
    getDispatchTable(env, obj)->m5_panic();
}

JNIEXPORT void JNICALL
Java_gem5_Ops_work_1begin(JNIEnv *env, jobject obj,
                            jlong j_workid, jlong j_threadid)
{
    getDispatchTable(env, obj)->m5_work_begin(j_workid, j_threadid);
}

JNIEXPORT void JNICALL
Java_gem5_Ops_work_1end(JNIEnv *env, jobject obj,
                          jlong j_workid, jlong j_threadid)
{
    getDispatchTable(env, obj)->m5_work_end(j_workid, j_threadid);
}
