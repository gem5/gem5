/*
 * Copyright (c) 2004 The Regents of The University of Michigan
 * All rights reserved.
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

#include <vector>

#ifdef USE_MYSQL
#include "base/cprintf.hh"
#include "base/misc.hh"
#include "base/mysql.hh"
#include "base/stats/events.hh"
#include "base/stats/mysql.hh"
#include "base/stats/mysql_run.hh"
#include "base/str.hh"
#endif

#include "base/match.hh"
#include "sim/host.hh"
#include "sim/sim_object.hh"
#include "sim/universe.hh"

using namespace std;

namespace Stats {

Tick EventStart = ULL(0x7fffffffffffffff);

ObjectMatch event_ignore;

#ifdef USE_MYSQL
class InsertEvent
{
  private:
    char *query;
    int size;
    bool first;
    static const int maxsize = 1024*1024;

    typedef map<string, uint32_t> event_map_t;
    event_map_t events;

    MySQL::Connection &mysql;
    uint16_t run;

  public:
    InsertEvent()
        : mysql(MySqlDB.conn()), run(MySqlDB.run())
    {
        query = new char[maxsize + 1];
        size = 0;
        first = true;
        flush();
    }
    ~InsertEvent()
    {
        flush();
    }

    void flush();
    void insert(const string &stat);
};

void
InsertEvent::insert(const string &stat)
{
    assert(mysql.connected());

    event_map_t::iterator i = events.find(stat);
    uint32_t event;
    if (i == events.end()) {
        mysql.query(
            csprintf("SELECT en_id "
                     "from event_names "
                     "where en_name=\"%s\"",
                     stat));

        MySQL::Result result = mysql.store_result();
        if (!result)
            panic("could not get a run\n%s\n", mysql.error);

        assert(result.num_fields() == 1);
        MySQL::Row row = result.fetch_row();
        if (row) {
            if (!to_number(row[0], event))
                panic("invalid event id: %s\n", row[0]);
        } else {
            mysql.query(
                csprintf("INSERT INTO "
                         "event_names(en_name)"
                         "values(\"%s\")",
                         stat));

            if (mysql.error)
                panic("could not get a run\n%s\n", mysql.error);

            event = mysql.insert_id();
        }
    } else {
        event = (*i).second;
    }

    if (size + 1024 > maxsize)
        flush();

    if (!first) {
        query[size++] = ',';
        query[size] = '\0';
    }

    first = false;

    size += sprintf(query + size, "(%u,%u,%llu)",
                    event, run, (unsigned long long)curTick);
}

void
InsertEvent::flush()
{
    if (size) {
        MySQL::Connection &mysql = MySqlDB.conn();
        assert(mysql.connected());
        mysql.query(query);
    }

    query[0] = '\0';
    size = 0;
    first = true;
    strcpy(query, "INSERT INTO "
           "events(ev_event, ev_run, ev_tick)"
           "values");
    size = strlen(query);
}

void
__event(const string &stat)
{
    static InsertEvent event;
    event.insert(stat);
}

#endif

/* namespace Stats */ }
