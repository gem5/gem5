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

#ifndef __BASE_MYQSL_HH__
#define __BASE_MYQSL_HH__

#define TO_BE_INCLUDED_LATER 0

#include <cassert>
#include <iosfwd>
#include <mysql.h>
#include <string>
#include <sstream>

namespace MySQL {

class Error
{
  protected:
    const char *error;

  public:
    Error() : error(NULL) {}

    Error &clear() { error = NULL; return *this; }
    Error &set(const char *err) { error = err; return *this; }

    const char *string() const { return error; }

    operator bool() const { return error != NULL; }
    bool operator!() const { return error == NULL; }
};

std::ostream &operator<<(std::ostream &stream, const Error &error);

class Result
{
  private:
    MYSQL_RES *result;
    int *refcount;

    void
    decref()
    {
        if (!refcount)
            return;

        *refcount -= 1;
        if (*refcount == 0) {
            mysql_free_result(result);
            delete refcount;
        }

        refcount = NULL;
    }

  public:
    Result()
        : result(0), refcount(NULL)
    { }

    Result(MYSQL_RES *res)
        : result(res)
    {
        if (result)
            refcount = new int(1);
    }

    Result(const Result &result)
        : result(result.result), refcount(result.refcount)
    {
        if (result)
            *refcount += 1;
    }

    ~Result()
    {
        decref();
    }

    const Result &
    operator=(MYSQL_RES *res)
    {
        decref();
        result = res;
        if (result)
            refcount = new int(1);

        return *this;
    }

    const Result &
    operator=(const Result &res)
    {
        decref();
        result = res.result;
        refcount = res.refcount;
        if (result)
            *refcount += 1;

        return *this;
    }

    operator bool() const { return result != NULL; }
    bool operator!() const { return result == NULL; }

    unsigned
    num_fields()
    {
        assert(result);
        return mysql_num_fields(result);
    }

    MYSQL_ROW
    fetch_row()
    {
        return mysql_fetch_row(result);
    }

    unsigned long *
    fetch_lengths()
    {
        return mysql_fetch_lengths(result);
    }
};

typedef MYSQL_ROW Row;

class Connection
{
  protected:
    MYSQL mysql;
    bool valid;

  protected:
    std::string _host;
    std::string _user;
    std::string _passwd;
    std::string _database;

  public:
    Connection();
    virtual ~Connection();

    bool connected() const { return valid; }
    bool connect(const std::string &host, const std::string &user,
                 const std::string &passwd, const std::string &database);
    void close();

  public:
    Error error;
    operator MYSQL *() { return &mysql; }

  public:
    bool
    query(const std::string &sql)
    {
        DPRINTF(SQL, "Sending SQL query to server:\n%s", sql);
        error.clear();
        if (mysql_real_query(&mysql, sql.c_str(), sql.size()))
            error.set(mysql_error(&mysql));

        return error;
    }

    bool
    query(const std::stringstream &sql)
    {
        return query(sql.str());
    }

    unsigned
    field_count()
    {
        return mysql_field_count(&mysql);
    }

    unsigned
    affected_rows()
    {
        return mysql_affected_rows(&mysql);
    }

    unsigned
    insert_id()
    {
        return mysql_insert_id(&mysql);
    }


    Result
    store_result()
    {
        error.clear();
        Result result = mysql_store_result(&mysql);
        if (!result)
            error.set(mysql_error(&mysql));

        return result;
    }
};

#if 0
class BindProxy
{
    MYSQL_BIND *bind;
    BindProxy(MYSQL_BIND *b) : bind(b) {}

    void operator=(bool &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_TINY;
        bind->buffer = (char *)&buffer;
    }

    void operator=(int8_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_TINY;
        bind->buffer = (char *)&buffer;
    }

    void operator=(int16_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_SHORT;
        bind->buffer = (char *)&buffer;
    }

    void operator=(int32_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_LONG;
        bind->buffer = (char *)&buffer;
    }

    void operator=(int64_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_LONGLONG;
        bind->buffer = (char *)&buffer;
    }

    void operator=(uint8_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_TINY;
        bind->buffer = (char *)&buffer;
    }

    void operator=(uint16_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_SHORT;
        bind->buffer = (char *)&buffer;
    }

    void operator=(uint32_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_LONG;
        bind->buffer = (char *)&buffer;
    }

    void operator=(uint64_t &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_LONGLONG;
        bind->buffer = (char *)&buffer;
    }

    void operator=(float &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_FLOAT;
        bind->buffer = (char *)&buffer;
    }

    void operator=(double &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_DOUBLE;
        bind->buffer = (char *)&buffer;
    }

    void operator=(Time &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_DATE;
        bind->buffer = (char *)&buffer;
    }

    void operator=(const char *buffer)
    {
        bind->buffer_type = MYSQL_TYPE_VAR_STRING;
        bind->buffer = buffer;
    }

    void operator=(const std::string &buffer)
    {
        bind->buffer_type = MYSQL_TYPE_VAR_STRING;
        bind->buffer = (char *)&buffer;
        bind->length = buffer.length;
    }

    bool
    set_null(bool null)
    {
        bind->is_null = null;
    }
};

class Statement
{
  protected:
    Error &error;
    MYSQL_STMT *stmt;
    MYSQL_BIND *bind;
    int size;

  public:
    Statement(Connection &mysql)
        : error(mysql.error), bind(NULL), size(0)
    {
        stmt = mysql_stmt_init(mysql);
        assert(valid() && "mysql_stmt_init(), out of memory\n");
    }

    ~Statement()
    {
        assert(valid());
        error.clear();
        if (mysql_stmt_close(stmt))
            error.set(mysql_stmt_error(stmt));

        if (bind)
            delete [] bind;
    }

    bool valid()
    {
        return stmt != NULL;
    }

    void prepare(const std::string &query)
    {
        assert(valid());
        mysql.error.clear();
        if (mysql_stmt_prepare(mysql, query, strlen(query)))
            mysql.error.set(mysql_stmt_error(stmt));

        int size = count();
        bind = new MYSQL_BIND[size];
    }

    unsigned count()
    {
        assert(valid());
        return mysql_stmt_param_count(stmt);
    }

    unsigned affected()
    {
        assert(valid());
        return mysql_stmt_affected_rows(stmt);
    }

    void bind(MYSQL_BIND *bind)
    {
        mysql.error.clear();
        if (mysql_stmt_bind_param(stmt, bind))
            mysql.error.set(mysql_stmt_error(stmt));
    }

    BindProxy operator[](int index)
    {
        assert(index > 0 && index < N);
        return &bind[N];
    }

    operator MYSQL_BIND *()
    {
        return bind;
    }

    void operator()()
    {
        assert(valid());
        error.clear();
        if (mysql_stmt_execute(stmt))
            error.set(mysql_stmt_error(stmt));
    }
}
#endif

/* namespace MySQL */ }

#endif // __BASE_MYQSL_HH__
