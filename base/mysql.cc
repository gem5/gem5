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

#include <iostream>

#include "base/mysql.hh"
#include "base/trace.hh"

using namespace std;

namespace MySQL {

inline const char *
charstar(const string &string)
{
    return string.empty() ? NULL : string.c_str();
}

ostream &
operator<<(ostream &stream, const Error &error)
{
    stream << error.string();
    return stream;
}

/*
 * The connection class
 */
Connection::Connection()
    : valid(false)
{
}

Connection::~Connection()
{
    if (valid)
        close();
}


bool
Connection::connect(const string &xhost, const string &xuser,
                    const string &xpasswd, const string &xdatabase)
{
    if (connected())
        return error.set("Already Connected");

    _host = xhost;
    _user = xuser;
    _passwd = xpasswd;
    _database = xdatabase;

    error.clear();

    mysql_init(&mysql);
    mysql_options(&mysql, MYSQL_OPT_COMPRESS, 0); // might want to be 1
    mysql_options(&mysql, MYSQL_READ_DEFAULT_GROUP, "odbc");
    if (!mysql_real_connect(&mysql, charstar(_host), charstar(_user),
                            charstar(_passwd), charstar(_database),
                            0, NULL, 0))
        return error.set(mysql_error(&mysql));

    valid = true;
    return false;
}

void
Connection::close()
{
    mysql_close(&mysql);
}

bool
Connection::query(const string &sql)
{
    DPRINTF(SQL, "Sending SQL query to server:\n%s", sql);
    error.clear();
    if (mysql_real_query(&mysql, sql.c_str(), sql.size()))
        error.set(mysql_error(&mysql));

    return error;
}


/* namespace MySQL */ }
