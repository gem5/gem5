/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#include <cassert>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/mysql.hh"
#include "base/statistics.hh"
#include "base/stats/flags.hh"
#include "base/stats/mysql.hh"
#include "base/stats/mysql_run.hh"
#include "base/stats/statdb.hh"
#include "base/stats/types.hh"
#include "base/str.hh"
#include "sim/host.hh"

using namespace std;

namespace Stats {

MySqlRun MySqlDB;

bool
MySqlConnected()
{
    return MySqlDB.connected();
}

void
MySqlRun::connect(const string &host, const string &user, const string &passwd,
                  const string &db, const string &name, const string &project)
{
    if (connected())
        panic("can only get one database connection at this time!");

    mysql.connect(host, user, passwd, db);
    if (mysql.error)
        panic("could not connect to database server\n%s\n", mysql.error);

    remove(name);
    cleanup();
    setup(name, user, project);
}

void
MySqlRun::setup(const string &name, const string &user, const string &project)
{
    assert(mysql.connected());

    stringstream insert;
    ccprintf(insert,
             "INSERT INTO "
             "runs(rn_name, rn_user, rn_project, rn_date, rn_expire)"
             "values(\"%s\", \"%s\", \"%s\", NOW(),"
             "DATE_ADD(CURDATE(), INTERVAL 31 DAY))",
             name, user, project);

    mysql.query(insert);
    if (mysql.error)
        panic("could not get a run\n%s\n", mysql.error);

    run_id = mysql.insert_id();
}

void
MySqlRun::remove(const string &name)
{
    assert(mysql.connected());
    stringstream sql;
    ccprintf(sql, "DELETE FROM runs WHERE rn_name=\"%s\"", name);
    mysql.query(sql);
}

void
MySqlRun::cleanup()
{
    assert(mysql.connected());

    mysql.query("DELETE data "
                "FROM data "
                "LEFT JOIN runs ON dt_run=rn_id "
                "WHERE rn_id IS NULL");

    mysql.query("DELETE formula_ref "
                "FROM formula_ref "
                "LEFT JOIN runs ON fr_run=rn_id "
                "WHERE rn_id IS NULL");

    mysql.query("DELETE formulas "
                "FROM formulas "
                "LEFT JOIN formula_ref ON fm_stat=fr_stat "
                "WHERE fr_stat IS NULL");

    mysql.query("DELETE stats "
                "FROM stats "
                "LEFT JOIN data ON st_id=dt_stat "
                "WHERE dt_stat IS NULL");

    mysql.query("DELETE subdata "
                "FROM subdata "
                "LEFT JOIN data ON sd_stat=dt_stat "
                "WHERE dt_stat IS NULL");

    mysql.query("DELETE bins "
                "FROM bins "
                "LEFT JOIN data ON bn_id=dt_bin "
                "WHERE dt_bin IS NULL");
}

void
SetupStat::init()
{
    name = "";
    descr = "";
    type = "";
    print = false;
    prereq = 0;
    prec = -1;
    nozero = false;
    nonan = false;
    total = false;
    pdf = false;
    cdf = false;
    min = 0;
    max = 0;
    bktsize = 0;
    size = 0;
}

unsigned
SetupStat::setup()
{
    MySQL::Connection &mysql = MySqlDB.conn();

    stringstream insert;
    ccprintf(insert,
             "INSERT INTO "
             "stats(st_name, st_descr, st_type, st_print, st_prereq, "
             "st_prec, st_nozero, st_nonan, st_total, st_pdf, st_cdf, "
             "st_min, st_max, st_bktsize, st_size)"
             "values(\"%s\",\"%s\",\"%s\","
             "        %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d)",
             name, descr, type, print, prereq, (int)prec, nozero, nonan,
             total, pdf, cdf,
             min, max, bktsize, size);

    mysql.query(insert);
    if (!mysql.error)
        return mysql.insert_id();

    stringstream select;
    ccprintf(select, "SELECT * FROM stats WHERE st_name=\"%s\"", name);

    mysql.query(select);
    MySQL::Result result = mysql.store_result();
    if (!result)
        panic("could not get a run\n%s\n", mysql.error);


    assert(result.num_fields() == 16);
    MySQL::Row row = result.fetch_row();
    if (!row)
        panic("could not get a run\n%s\n", mysql.error);

    bool tb;
    int8_t ti8;
    uint16_t tu16;
    int64_t ti64;
    uint64_t tu64;

    if (name != (char *)row[1])
        panic("failed stat check on %s:name. %s != %s\n",
              name, name, row[1]);

    if (descr != (char *)row[2])
        panic("failed stat check on %s:descr. %s != %s\n",
              name, descr, row[2]);

    if (type != (char *)row[3])
        panic("failed stat check on %s:type. %s != %s\n",
              name, type, row[3]);

    if (!to_number(row[4], tb) || print != tb)
        panic("failed stat check on %s:print. %d != %d\n",
              name, print, tb);

    if (!to_number(row[6], ti8) || prec != ti8)
        panic("failed stat check on %s:prec. %d != %d\n",
              name, prec, ti8);

    if (!to_number(row[7], tb) || nozero != tb)
        panic("failed stat check on %s:nozero. %d != %d\n",
              name, nozero, tb);

    if (!to_number(row[8], tb) || nonan != tb)
        panic("failed stat check on %s:nonan. %d != %d\n",
              name, nonan, tb);

    if (!to_number(row[9], tb) || total != tb)
        panic("failed stat check on %s:total. %d != %d\n",
              name, total, tb);

    if (!to_number(row[10], tb) || pdf != tb)
        panic("failed stat check on %s:pdf. %d != %d\n",
              name, pdf, tb);

    if (!to_number(row[11], tb) || cdf != tb)
        panic("failed stat check on %s:cdf. %d != %d\n",
              name, cdf, tb);

    if (!to_number(row[12], ti64) || min != ti64)
        panic("failed stat check on %s:min. %d != %d\n",
              name, min, ti64);

    if (!to_number(row[13], ti64) || max != ti64)
        panic("failed stat check on %s:max. %d != %d\n",
              name, max, ti64);

    if (!to_number(row[14], tu64) || bktsize != tu64)
        panic("failed stat check on %s:bktsize. %d != %d\n",
              name, bktsize, tu64);

    if (!to_number(row[15], tu16) || size != tu16)
        panic("failed stat check on %s:size. %d != %d\n",
              name, size, tu16);

    to_number(row[5], prereq);
    uint16_t statid;
    to_number(row[0], statid);
    return statid;
}

unsigned
SetupBin(const string &bin)
{
    MySQL::Connection &mysql = MySqlDB.conn();
    assert(mysql.connected());

    using namespace MySQL;
    stringstream select;
    ccprintf(select, "SELECT bn_id FROM bins WHERE bn_name=\"%s\"", bin);

    mysql.query(select);
    MySQL::Result result = mysql.store_result();
    if (result) {
        assert(result.num_fields() == 1);
        Row row = result.fetch_row();
        if (row) {
            uint16_t bin_id;
            to_number(row[0], bin_id);
            return bin_id;
        }
    }

    stringstream insert;
    ccprintf(insert, "INSERT INTO bins(bn_name) values(\"%s\")", bin);

    mysql.query(insert);
    if (mysql.error)
        panic("could not get a run\n%s\n", mysql.error);

    return mysql.insert_id();
}

InsertData::InsertData()
{
    query = new char[maxsize + 1];
    size = 0;
    flush();
}

InsertData::~InsertData()
{
    delete [] query;
}

void
InsertData::flush()
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
           "data(dt_stat,dt_x,dt_y,dt_run,dt_sample,dt_bin,dt_data) "
           "values");
    size = strlen(query);
}

void
InsertData::insert()
{
    if (size + 1024 > maxsize)
        flush();

    if (!first) {
        query[size++] = ',';
        query[size] = '\0';
    }

    first = false;

    size += sprintf(query + size, "(%u,%d,%d,%u,%llu,%u,\"%f\")",
                    stat, x, y, MySqlDB.run(), (unsigned long long)sample,
                    bin, data);
}

struct InsertSubData
{
    uint16_t stat;
    int16_t x;
    int16_t y;
    string name;
    string descr;

    void setup();
};

void
InsertSubData::setup()
{
    MySQL::Connection &mysql = MySqlDB.conn();
    assert(mysql.connected());
    stringstream insert;
    ccprintf(insert,
             "INSERT INTO subdata(sd_stat,sd_x,sd_y,sd_name,sd_descr) "
             "values(%d,%d,%d,\"%s\",\"%s\")",
             stat, x, y, name, descr);

    mysql.query(insert);
}

void
InsertFormula(uint16_t stat, const string &formula)
{
    MySQL::Connection &mysql = MySqlDB.conn();
    assert(mysql.connected());
    stringstream insert_formula;
    ccprintf(insert_formula,
             "INSERT INTO formulas(fm_stat,fm_formula) values(%d, \"%s\")",
             stat, formula);

    mysql.query(insert_formula);

    stringstream insert_ref;
    ccprintf(insert_ref,
             "INSERT INTO formula_ref(fr_stat,fr_run) values(%d, %d)",
             stat, MySqlDB.run());

        mysql.query(insert_ref);
}

void
UpdatePrereq(uint16_t stat, uint16_t prereq)
{
    MySQL::Connection &mysql = MySqlDB.conn();
    assert(mysql.connected());
    stringstream update;
    ccprintf(update, "UPDATE stats SET st_prereq=%d WHERE st_id=%d",
             prereq, stat);
    mysql.query(update);
}

void
MySql::configure()
{
    /*
     * set up all stats!
     */
    using namespace Database;
    stat_list_t::const_iterator i, end = stats().end();
    for (i = stats().begin(); i != end; ++i)
        (*i)->visit(*this);

    for (i = stats().begin(); i != end; ++i) {
        StatData *data = *i;
        if (data->prereq) {
            uint16_t stat_id = find(data->id);
            uint16_t prereq_id = find(data->prereq->id);
            assert(stat_id && prereq_id);

            UpdatePrereq(stat_id, prereq_id);
        }
    }

    configured = true;
}


void
MySql::configure(const StatData &data, string type)
{
    stat.init();
    stat.name = data.name;
    stat.descr = data.desc;
    stat.type = type;
    stat.print = data.flags & print;
    stat.prec = data.precision;
    stat.nozero = data.flags & nozero;
    stat.nonan = data.flags & nonan;
    stat.total = data.flags & total;
    stat.pdf = data.flags & pdf;
    stat.cdf = data.flags & cdf;
}

void
MySql::configure(const ScalarData &data)
{
    configure(data, "SCALAR");
    insert(data.id, stat.setup());
}

void
MySql::configure(const VectorData &data)
{
    configure(data, "VECTOR");
    uint16_t statid = stat.setup();

    if (!data.subnames.empty()) {
        InsertSubData subdata;
        subdata.stat = statid;
        subdata.y = 0;
        for (int i = 0; i < data.subnames.size(); ++i) {
            subdata.x = i;
            subdata.name = data.subnames[i];
            subdata.descr = data.subdescs.empty() ? "" : data.subdescs[i];

            if (!subdata.name.empty() || !subdata.descr.empty())
                subdata.setup();
        }
    }

    insert(data.id, statid);
}

void
MySql::configure(const DistData &data)
{
    configure(data, "DIST");
    if (!data.data.fancy) {
        stat.size = data.data.size;
        stat.min = data.data.min;
        stat.max = data.data.max;
        stat.bktsize = data.data.bucket_size;
    }
    insert(data.id, stat.setup());
}

void
MySql::configure(const VectorDistData &data)
{
    configure(data, "VECTORDIST");

    if (!data.data[0].fancy) {
        stat.size = data.data[0].size;
        stat.min = data.data[0].min;
        stat.max = data.data[0].max;
        stat.bktsize = data.data[0].bucket_size;
    }

    uint16_t statid = stat.setup();

    if (!data.subnames.empty()) {
        InsertSubData subdata;
        subdata.stat = statid;
        subdata.y = 0;
        for (int i = 0; i < data.subnames.size(); ++i) {
            subdata.x = i;
            subdata.name = data.subnames[i];
            subdata.descr = data.subdescs.empty() ? "" : data.subdescs[i];
            if (!subdata.name.empty() || !subdata.descr.empty())
                subdata.setup();
        }
    }

    insert(data.id, statid);
}

void
MySql::configure(const Vector2dData &data)
{
    configure(data, "VECTOR2D");
    uint16_t statid = stat.setup();

    if (!data.subnames.empty()) {
        InsertSubData subdata;
        subdata.stat = statid;
        subdata.y = 0;
        for (int i = 0; i < data.subnames.size(); ++i) {
            subdata.x = i;
            subdata.name = data.subnames[i];
            subdata.descr = data.subdescs.empty() ? "" : data.subdescs[i];
            if (!subdata.name.empty() || !subdata.descr.empty())
                subdata.setup();
        }
    }

    if (!data.y_subnames.empty()) {
        InsertSubData subdata;
        subdata.stat = statid;
        subdata.x = 0;
        subdata.descr = "";
        for (int i = 0; i < data.y_subnames.size(); ++i) {
            subdata.y = i;
            subdata.name = data.y_subnames[i];
            if (!subdata.name.empty())
                subdata.setup();
        }
    }

    insert(data.id, statid);
}

void
MySql::configure(const FormulaData &data)
{
    configure(data, "FORMULA");
    insert(data.id, stat.setup());
}

void
MySql::output(const string &bin)
{
    // set up new bin in database if there is a bin name
    newdata.bin = bin.empty() ? 0 : SetupBin(bin);

    Database::stat_list_t::const_iterator i, end = Database::stats().end();
    for (i = Database::stats().begin(); i != end; ++i)
        (*i)->visit(*this);
}

bool
MySql::valid() const
{
    return MySqlDB.connected();
}

void
MySql::output()
{
    using namespace Database;
    assert(valid());

    if (!configured)
        configure();

    // store sample #
    newdata.sample = curTick;

    if (bins().empty()) {
        output(string(""));
    } else {
        bin_list_t::iterator i, end = bins().end();
        for (i = bins().begin(); i != end; ++i) {
            MainBin *bin = *i;
            bin->activate();
            output(bin->name());
        }
    }

    newdata.flush();
}

void
MySql::output(const ScalarData &data)
{
    newdata.stat = find(data.id);
    newdata.x = 0;
    newdata.y = 0;
    newdata.data = data.value();

    newdata.insert();
}

void
MySql::output(const VectorData &data)
{
    newdata.stat = find(data.id);
    newdata.y = 0;

    const VCounter &cvec = data.value();
    int size = data.size();
    for (int x = 0; x < size; x++) {
        newdata.x = x;
        newdata.data = cvec[x];
        newdata.insert();
    }
}

void
MySql::output(const DistDataData &data)
{
    const int db_sum = -1;
    const int db_squares = -2;
    const int db_samples = -3;
    const int db_min_val = -4;
    const int db_max_val = -5;
    const int db_underflow = -6;
    const int db_overflow = -7;

    newdata.x = db_sum;
    newdata.data = data.sum;
    newdata.insert();

    newdata.x = db_squares;
    newdata.data = data.squares;
    newdata.insert();

    newdata.x = db_samples;
    newdata.data = data.samples;
    newdata.insert();

    if (data.samples && !data.fancy) {
        newdata.x = db_min_val;
        newdata.data = data.min_val;
        newdata.insert();

        newdata.x = db_max_val;
        newdata.data = data.max_val;
        newdata.insert();

        newdata.x = db_underflow;
        newdata.data = data.underflow;
        newdata.insert();

        newdata.x = db_overflow;
        newdata.data = data.overflow;
        newdata.insert();

        int size = data.cvec.size();
        for (int x = 0; x < size; x++) {
            newdata.x = x;
            newdata.data = data.cvec[x];
            newdata.insert();
        }
    }
}


void
MySql::output(const DistData &data)
{
    newdata.stat = find(data.id);
    newdata.y = 0;
    output(data.data);
}

void
MySql::output(const VectorDistData &data)
{
    newdata.stat = find(data.id);

    int size = data.data.size();
    for (int y = 0; y < size; ++y) {
        newdata.y = y;
        output(data.data[y]);
    }
}

void
MySql::output(const Vector2dData &data)
{
    newdata.stat = find(data.id);

    int index = 0;
    for (int x = 0; x < data.x; x++) {
        newdata.x = x;
        for (int y = 0; y < data.y; y++) {
            newdata.y = y;
            newdata.data = data.cvec[index++];
            newdata.insert();
        }
    }
}

void
MySql::output(const FormulaData &data)
{
    InsertFormula(find(data.id), data.str());
}

/*
 * Implement the visitor
 */
void
MySql::visit(const ScalarData &data)
{
    if (!configured)
        configure(data);
    else
        output(data);
}

void
MySql::visit(const VectorData &data)
{
    if (!configured)
        configure(data);
    else
        output(data);
}

void
MySql::visit(const DistData &data)
{
    if (!configured)
        configure(data);
    else
        output(data);
}

void
MySql::visit(const VectorDistData &data)
{
    if (!configured)
        configure(data);
    else
        output(data);
}

void
MySql::visit(const Vector2dData &data)
{
    if (!configured)
        configure(data);
    else
        output(data);
}

void
MySql::visit(const FormulaData &data)
{
    if (!configured)
        configure(data);
    else
        output(data);
}

/* namespace Stats */ }
