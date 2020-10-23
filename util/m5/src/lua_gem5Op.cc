/* Copyright (c) 2017 Hanhwi Jang
 * All rights reserved.

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
 */


#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#include <cassert>
#include <cstdlib>

#include <gem5/m5ops.h>

#include "m5_mmap.h"

static int
do_arm(lua_State *L)
{
    uint64_t address = lua_tointeger(L, 1);
    m5_arm(address);
    return 0;
}

static int
do_quiesce(lua_State *L)
{
    m5_quiesce();
    return 0;
}

static int
do_quiesce_ns(lua_State *L)
{
    uint64_t ns = lua_tointeger(L, 1);
    m5_quiesce_ns(ns);
    return 0;
}

static int
do_quiesce_cycle(lua_State *L)
{
    uint64_t cycles = lua_tointeger(L, 1);
    m5_quiesce_cycle(cycles);
    return 0;
}

static int
do_quiesce_time(lua_State *L)
{
    uint64_t ns = m5_quiesce_time();
    lua_pushinteger(L, ns);
    return 1;
}

static int
do_rpns(lua_State *L)
{
    uint64_t ns = m5_rpns();
    lua_pushinteger(L, ns);
    return 1;
}

static int
do_wake_cpu(lua_State *L)
{
    uint64_t cpuid = lua_tointeger(L, 1);
    m5_wake_cpu(cpuid);
    return 0;
}

static int
do_exit(lua_State *L)
{
    uint64_t ns_delay = lua_tointeger(L, 1);
    m5_exit(ns_delay);
    return 0;
}

static int
do_sum(lua_State *L)
{
    uint64_t a = lua_tointeger(L, 1);
    uint64_t b = lua_tointeger(L, 2);
    uint64_t c = lua_tointeger(L, 3);
    uint64_t d = lua_tointeger(L, 4);
    uint64_t e = lua_tointeger(L, 5);
    uint64_t f = lua_tointeger(L, 6);
    uint64_t sum = m5_sum(a, b, c, d, e, f);
    lua_pushinteger(L, sum);
    return 1;
}

static int
do_fail(lua_State *L)
{
    uint64_t ns_delay = lua_tointeger(L, 1);
    uint64_t code = lua_tointeger(L, 2);
    m5_fail(ns_delay, code);
    return 0;
}

static int
do_init_param(lua_State *L)
{
    uint64_t key_str1 = lua_tointeger(L, 1);
    uint64_t key_str2 = lua_tointeger(L, 2);
    lua_pushinteger(L, m5_init_param(key_str1, key_str2));
    return 1;
}

static int
do_checkpoint(lua_State *L)
{
    uint64_t delay = lua_tointeger(L, 1);
    uint64_t period = lua_tointeger(L, 2);
    m5_checkpoint(delay, period);
    return 0;
}

static int
do_reset_stats(lua_State *L)
{
    uint64_t ns_delay = lua_tointeger(L, 1);
    uint64_t ns_period = lua_tointeger(L, 2);
    m5_reset_stats(ns_delay, ns_period);
    return 0;
}

static int
do_dump_stats(lua_State *L)
{
    uint64_t delay = lua_tointeger(L, 1);
    uint64_t period = lua_tointeger(L, 2);
    m5_dump_stats(delay, period);
    return 0;
}

static int
do_dump_reset_stats(lua_State *L)
{
    uint64_t delay = lua_tointeger(L, 1);
    uint64_t period = lua_tointeger(L, 2);
    m5_dump_reset_stats(delay, period);
    return 0;
}

static int
do_read_file(lua_State *L)
{
    uint64_t len = lua_tointeger(L, 1);
    uint64_t offset = lua_tointeger(L, 2);
    char *buf = (char *)malloc(len);
    uint64_t readlen = m5_read_file(buf, len, offset);
    lua_pushlstring(L, buf, readlen);
    return 1;
}

static int
do_write_file(lua_State *L)
{
    const char* buf = lua_tostring(L, 1);
    uint64_t len = lua_tointeger(L, 2);
    assert(len <= lua_strlen(L, 1));
    uint64_t offset = lua_tointeger(L, 3);
    const char *filename = lua_tostring(L, 4);
    uint64_t w_len = m5_write_file((void *)buf, len, offset, filename);
    lua_pushinteger(L, w_len);
    return 1;
}

static int
do_debug_break(lua_State *L)
{
    m5_debug_break();
    return 0;
}

static int
do_switch_cpu(lua_State *L)
{
    m5_switch_cpu();
    return 0;
}

static int
do_dist_toggle_sync(lua_State *L)
{
    m5_dist_toggle_sync();
    return 0;
}

static int
do_add_symbol(lua_State *L)
{
    uint64_t addr = lua_tointeger(L, 1);
    char *string = (char*) lua_tostring(L, 2);
    m5_add_symbol(addr, string);
    return 0;
}

static int
do_loadsymbol(lua_State *L)
{
    m5_load_symbol();
    return 0;
}

static int
do_panic(lua_State *L)
{
    m5_panic();
    return 0;
}

static int
do_work_begin(lua_State *L)
{
    uint64_t workid = lua_tointeger(L, 1);
    uint64_t threadid = lua_tointeger(L, 2);
    m5_work_begin(workid, threadid);
    return 0;
}

static int
do_work_end(lua_State *L)
{
    uint64_t workid = lua_tointeger(L, 1);
    uint64_t threadid = lua_tointeger(L, 2);
    m5_work_end(workid, threadid);
    return 0;
}

extern "C"
{

int luaopen_gem5OpLua(lua_State *);

}

int
luaopen_gem5OpLua(lua_State *L)
{
    map_m5_mem();
#define ADD_FUNC(fname) do{                         \
        lua_pushcfunction(L, fname);                \
        lua_setfield(L, -2, #fname);                \
    }while (0)

    lua_newtable(L);
    ADD_FUNC(do_arm);
    ADD_FUNC(do_quiesce);
    ADD_FUNC(do_quiesce_ns);
    ADD_FUNC(do_quiesce_cycle);
    ADD_FUNC(do_quiesce_time);
    ADD_FUNC(do_rpns);
    ADD_FUNC(do_wake_cpu);
    ADD_FUNC(do_exit);
    ADD_FUNC(do_fail);
    ADD_FUNC(do_init_param);
    ADD_FUNC(do_checkpoint);
    ADD_FUNC(do_reset_stats);
    ADD_FUNC(do_dump_stats);
    ADD_FUNC(do_dump_reset_stats);
    ADD_FUNC(do_read_file);
    ADD_FUNC(do_write_file);
    ADD_FUNC(do_debug_break);
    ADD_FUNC(do_switch_cpu);
    ADD_FUNC(do_dist_toggle_sync);
    ADD_FUNC(do_add_symbol);
    ADD_FUNC(do_loadsymbol);
    ADD_FUNC(do_panic);
    ADD_FUNC(do_work_begin);
    ADD_FUNC(do_work_end);
#undef ADD_FUNC
    return 1;
}
