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

package gem5;

import java.io.*;
import java.util.*;

/**
 * Java class to implement JNI for m5Ops
 */

public class Ops {
    private Ops() {}

    private static native void setupCallTypes();
    private long dispatchTablePtr;

    private static Map<String, Ops> _callTypes;
    public static final Map<String, Ops> callTypes;

    public static native void setAddr(long addr);
    public static native void mapMem();
    public static native void unmapMem();

    static {
        try {
            File temp_lib = File.createTempFile("gem5Ops", ".so");
            temp_lib.deleteOnExit();

            InputStream in = Ops.class.getResourceAsStream("/libgem5Ops.so");
            byte[] buffer = new byte[in.available()];
            in.read(buffer);
            OutputStream out = new FileOutputStream(temp_lib);
            out.write(buffer);

            System.load(temp_lib.getAbsolutePath());
        } catch (Exception e) {
            throw new RuntimeException(e.getMessage());
        }

        setupCallTypes();
        callTypes = Collections.unmodifiableMap(_callTypes);
    }

    public native void arm(long address);
    public native void quiesce();
    public native void quiesce_ns(long ns);
    public native void quiesce_cycle(long cycles);
    public native long quiesce_time();
    public native long rpns();
    public native void wake_cpu(long cpuid);

    public native void exit(long ns_delay);
    public native void fail(long ns_delay, long code);
    public native long sum(long a, long b, long c, long d, long e, long f);
    public native long init_param(long key_str1, long key_str2);
    public native void checkpoint(long ns_delay, long ns_period);
    public native void reset_stats(long ns_delay, long ns_period);
    public native void dump_stats(long ns_delay, long ns_period);
    public native void dump_reset_stats(long ns_delay, long ns_period);
    public native long read_file(byte[] buffer, long len, long offset);
    public native long write_file(byte[] buffer, long len, long offset,
                                  String filename);
    public native void debug_break();
    public native void switch_cpu();
    public native void dist_toggle_sync();
    public native void add_symbol(long addr, String symbol);
    public native void load_symbol();
    public native void panic();
    public native void work_begin(long workid, long threadid);
    public native void work_end(long workid, long threadid);
}
