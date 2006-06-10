# Copyright (c) 2003-2004 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import MySQLdb, re, string

def statcmp(a, b):
    v1 = a.split('.')
    v2 = b.split('.')

    last = min(len(v1), len(v2)) - 1
    for i,j in zip(v1[0:last], v2[0:last]):
        if i != j:
            return cmp(i, j)

    # Special compare for last element.
    if len(v1) == len(v2):
        return cmp(v1[last], v2[last])
    else:
        return cmp(len(v1), len(v2))

class RunData:
    def __init__(self, row):
        self.run = int(row[0])
        self.name = row[1]
        self.user = row[2]
        self.project = row[3]

class SubData:
    def __init__(self, row):
        self.stat = int(row[0])
        self.x = int(row[1])
        self.y = int(row[2])
        self.name = row[3]
        self.descr = row[4]

class Data:
    def __init__(self, row):
        if len(row) != 5:
            raise 'stat db error'
        self.stat = int(row[0])
        self.run = int(row[1])
        self.x = int(row[2])
        self.y = int(row[3])
        self.data = float(row[4])

    def __repr__(self):
        return '''Data(['%d', '%d', '%d', '%d', '%f'])''' % ( self.stat,
            self.run, self.x, self.y, self.data)

class StatData(object):
    def __init__(self, row):
        self.stat = int(row[0])
        self.name = row[1]
        self.desc = row[2]
        self.type = row[3]
        self.prereq = int(row[5])
        self.precision = int(row[6])

        import flags
        self.flags = 0
        if int(row[4]): self.flags |= flags.printable
        if int(row[7]): self.flags |= flags.nozero
        if int(row[8]): self.flags |= flags.nonan
        if int(row[9]): self.flags |= flags.total
        if int(row[10]): self.flags |= flags.pdf
        if int(row[11]): self.flags |= flags.cdf

        if self.type == 'DIST' or self.type == 'VECTORDIST':
            self.min = float(row[12])
            self.max = float(row[13])
            self.bktsize = float(row[14])
            self.size = int(row[15])

        if self.type == 'FORMULA':
            self.formula = self.db.allFormulas[self.stat]

class Node(object):
    def __init__(self, name):
        self.name = name
    def __str__(self):
        return self.name

class Result(object):
    def __init__(self, x, y):
        self.data = {}
        self.x = x
        self.y = y

    def __contains__(self, run):
        return run in self.data

    def __getitem__(self, run):
        if run not in self.data:
            self.data[run] = [ [ 0.0 ] * self.y for i in xrange(self.x) ]
        return self.data[run]

class Database(object):
    def __init__(self):
        self.host = 'zizzer.pool'
        self.user = ''
        self.passwd = ''
        self.db = 'm5stats'
        self.cursor = None

        self.allStats = []
        self.allStatIds = {}
        self.allStatNames = {}

        self.allSubData = {}

        self.allRuns = []
        self.allRunIds = {}
        self.allRunNames = {}

        self.allFormulas = {}

        self.stattop = {}
        self.statdict = {}
        self.statlist = []

        self.mode = 'sum';
        self.runs = None
        self.ticks = None
        self.method = 'sum'
        self._method = type(self).sum

    def get(self, job, stat, system=None):
        run = self.allRunNames.get(str(job), None)
        if run is None:
            return None

        from info import ProxyError, scalar, vector, value, values, total, len
        if system is None and hasattr(job, 'system'):
            system = job.system

        if system is not None:
            stat.system = self[system]
        try:
            if scalar(stat):
                return value(stat, run.run)
            if vector(stat):
                return values(stat, run.run)
        except ProxyError:
            return None

        return None

    def query(self, sql):
        self.cursor.execute(sql)

    def update_dict(self, dict):
        dict.update(self.stattop)

    def append(self, stat):
        statname = re.sub(':', '__', stat.name)
        path = string.split(statname, '.')
        pathtop = path[0]
        fullname = ''

        x = self
        while len(path) > 1:
            name = path.pop(0)
            if not x.__dict__.has_key(name):
                x.__dict__[name] = Node(fullname + name)
            x = x.__dict__[name]
            fullname = '%s%s.' % (fullname, name)

        name = path.pop(0)
        x.__dict__[name] = stat

        self.stattop[pathtop] = self.__dict__[pathtop]
        self.statdict[statname] = stat
        self.statlist.append(statname)

    def connect(self):
        # connect
        self.thedb = MySQLdb.connect(db=self.db,
                                     host=self.host,
                                     user=self.user,
                                     passwd=self.passwd)

        # create a cursor
        self.cursor = self.thedb.cursor()

        self.query('''select rn_id,rn_name,rn_sample,rn_user,rn_project
                   from runs''')
        for result in self.cursor.fetchall():
            run = RunData(result);
            self.allRuns.append(run)
            self.allRunIds[run.run] = run
            self.allRunNames[run.name] = run

        self.query('select sd_stat,sd_x,sd_y,sd_name,sd_descr from subdata')
        for result in self.cursor.fetchall():
            subdata = SubData(result)
            if self.allSubData.has_key(subdata.stat):
                self.allSubData[subdata.stat].append(subdata)
            else:
                self.allSubData[subdata.stat] = [ subdata ]

        self.query('select * from formulas')
        for id,formula in self.cursor.fetchall():
            self.allFormulas[int(id)] = formula.tostring()

        StatData.db = self
        self.query('select * from stats')
        import info
        for result in self.cursor.fetchall():
            stat = info.NewStat(self, StatData(result))
            self.append(stat)
            self.allStats.append(stat)
            self.allStatIds[stat.stat] = stat
            self.allStatNames[stat.name] = stat

    # Name: listruns
    # Desc: Prints all runs matching a given user, if no argument
    #       is given all runs are returned
    def listRuns(self, user=None):
        print '%-40s %-10s %-5s' % ('run name', 'user', 'id')
        print '-' * 62
        for run in self.allRuns:
            if user == None or user == run.user:
                print '%-40s %-10s %-10d' % (run.name, run.user, run.run)

    # Name: listTicks
    # Desc: Prints all samples for a given run
    def listTicks(self, runs=None):
        print "tick"
        print "----------------------------------------"
        sql = 'select distinct dt_tick from data where dt_stat=1180 and ('
        if runs != None:
            first = True
            for run in runs:
               if first:
            #       sql += ' where'
                   first = False
               else:
                   sql += ' or'
               sql += ' dt_run=%s' % run.run
            sql += ')'
        self.query(sql)
        for r in self.cursor.fetchall():
            print r[0]

    # Name: retTicks
    # Desc: Prints all samples for a given run
    def retTicks(self, runs=None):
        sql = 'select distinct dt_tick from data where dt_stat=1180 and ('
        if runs != None:
            first = True
            for run in runs:
               if first:
                   first = False
               else:
                   sql += ' or'
               sql += ' dt_run=%s' % run.run
            sql += ')'
        self.query(sql)
        ret = []
        for r in self.cursor.fetchall():
            ret.append(r[0])
        return ret

    # Name: liststats
    # Desc: Prints all statistics that appear in the database,
    #         the optional argument is a regular expression that can
    #         be used to prune the result set
    def listStats(self, regex=None):
        print '%-60s %-8s %-10s' % ('stat name', 'id', 'type')
        print '-' * 80

        rx = None
        if regex != None:
            rx = re.compile(regex)

        stats = [ stat.name for stat in self.allStats ]
        stats.sort(statcmp)
        for stat in stats:
            stat = self.allStatNames[stat]
            if rx == None or rx.match(stat.name):
                print '%-60s %-8s %-10s' % (stat.name, stat.stat, stat.type)

    # Name: liststats
    # Desc: Prints all statistics that appear in the database,
    #         the optional argument is a regular expression that can
    #         be used to prune the result set
    def listFormulas(self, regex=None):
        print '%-60s %s' % ('formula name', 'formula')
        print '-' * 80

        rx = None
        if regex != None:
            rx = re.compile(regex)

        stats = [ stat.name for stat in self.allStats ]
        stats.sort(statcmp)
        for stat in stats:
            stat = self.allStatNames[stat]
            if stat.type == 'FORMULA' and (rx == None or rx.match(stat.name)):
                print '%-60s %s' % (stat.name, self.allFormulas[stat.stat])

    def getStat(self, stats):
        if type(stats) is not list:
            stats = [ stats ]

        ret = []
        for stat in stats:
            if type(stat) is int:
                ret.append(self.allStatIds[stat])

            if type(stat) is str:
                rx = re.compile(stat)
                for stat in self.allStats:
                    if rx.match(stat.name):
                        ret.append(stat)
        return ret

    #########################################
    # get the data
    #
    def query(self, op, stat, ticks, group=False):
        sql = 'select '
        sql += 'dt_stat as stat, '
        sql += 'dt_run as run, '
        sql += 'dt_x as x, '
        sql += 'dt_y as y, '
        if group:
            sql += 'dt_tick as tick, '
        sql += '%s(dt_data) as data ' % op
        sql += 'from data '
        sql += 'where '

        if isinstance(stat, list):
            val = ' or '.join([ 'dt_stat=%d' % s.stat for s in stat ])
            sql += ' (%s)' % val
        else:
            sql += ' dt_stat=%d' % stat.stat

        if self.runs != None and len(self.runs):
            val = ' or '.join([ 'dt_run=%d' % r for r in self.runs ])
            sql += ' and (%s)' % val

        if ticks != None and len(ticks):
            val = ' or '.join([ 'dt_tick=%d' % s for s in ticks ])
            sql += ' and (%s)' % val

        sql += ' group by dt_stat,dt_run,dt_x,dt_y'
        if group:
            sql += ',dt_tick'
        return sql

    # Name: sum
    # Desc: given a run, a stat and an array of samples, total the samples
    def sum(self, *args, **kwargs):
        return self.query('sum', *args, **kwargs)

    # Name: avg
    # Desc: given a run, a stat and an array of samples, average the samples
    def avg(self, stat, ticks):
        return self.query('avg', *args, **kwargs)

    # Name: stdev
    # Desc: given a run, a stat and an array of samples, get the standard
    #       deviation
    def stdev(self, stat, ticks):
        return self.query('stddev', *args, **kwargs)

    def __setattr__(self, attr, value):
        super(Database, self).__setattr__(attr, value)
        if attr != 'method':
            return

        if value == 'sum':
            self._method = self.sum
        elif value == 'avg':
            self._method = self.avg
        elif value == 'stdev':
            self._method = self.stdev
        else:
            raise AttributeError, "can only set get to: sum | avg | stdev"

    def data(self, stat, ticks=None):
        if ticks is None:
            ticks = self.ticks
        sql = self._method(self, stat, ticks)
        self.query(sql)

        runs = {}
        xmax = 0
        ymax = 0
        for x in self.cursor.fetchall():
            data = Data(x)
            if not runs.has_key(data.run):
                runs[data.run] = {}
            if not runs[data.run].has_key(data.x):
                runs[data.run][data.x] = {}

            xmax = max(xmax, data.x)
            ymax = max(ymax, data.y)
            runs[data.run][data.x][data.y] = data.data

        results = Result(xmax + 1, ymax + 1)
        for run,data in runs.iteritems():
            result = results[run]
            for x,ydata in data.iteritems():
                for y,data in ydata.iteritems():
                    result[x][y] = data
        return results

    def __getitem__(self, key):
        return self.stattop[key]
