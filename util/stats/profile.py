# Copyright (c) 2005 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert

from orderdict import orderdict
import output

class FileData(dict):
    def __init__(self, filename):
        self.filename = filename
        fd = file(filename)
        current = []
        for line in fd:
            line = line.strip()
            if line.startswith('>>>'):
                current = []
                self[line[3:]] = current
            else:
                current.append(line)
        fd.close()

class RunData(dict):
    def __init__(self, filename):
        self.filename = filename

    def __getattribute__(self, attr):
        if attr == 'total':
            total = 0.0
            for value in self.itervalues():
                total += value
            return total

        if attr == 'filedata':
            return FileData(self.filename)

        if attr == 'maxsymlen':
            return max([ len(sym) for sym in self.iterkeys() ])

        return super(RunData, self).__getattribute__(attr)

    def display(self, output=None, limit=None, maxsymlen=None):
        if not output:
            import sys
            output = sys.stdout
        elif isinstance(output, str):
            output = file(output, 'w')

        total = float(self.total)

        # swap (string,count) order so we can sort on count
        symbols = [ (count,name) for name,count in self.iteritems() ]
        symbols.sort(reverse=True)
        if limit is not None:
            symbols = symbols[:limit]

        if not maxsymlen:
            maxsymlen = self.maxsymlen

        symbolf = "%-" + str(maxsymlen + 1) + "s %.2f%%"
        for number,name in symbols:
            print >>output, symbolf % (name, 100.0 * (float(number) / total))

class PCData(RunData):
    def __init__(self, filename=None, categorize=None, showidle=True):
        super(PCData, self).__init__(self, filename)

        filedata = self.filedata['PC data']
        for line in filedata:
            (symbol, count) = line.split()
            if symbol == "0x0":
                continue
            count = int(count)

            if categorize is not None:
                category = categorize(symbol)
                if category is None:
                    category = 'other'
                elif category == 'idle' and not showidle:
                    continue

                self[category] = count

class FuncNode(object):
    def __new__(cls, filedata=None):
        if filedata is None:
            return super(FuncNode, cls).__new__(cls)

        nodes = {}
        for line in filedata['function data']:
            data = line.split(' ')
            node_id = long(data[0], 16)
            node = FuncNode()
            node.symbol = data[1]
            if node.symbol == '':
                node.symbol = 'unknown'
            node.count = long(data[2])
            node.children = [ long(child, 16) for child in data[3:] ]
            nodes[node_id] = node

        for node in nodes.itervalues():
            children = []
            for cid in node.children:
                child = nodes[cid]
                children.append(child)
                child.parent = node
            node.children = tuple(children)
        if not nodes:
            print filedata.filename
            print nodes
        return nodes[0]

    def total(self):
        total = self.count
        for child in self.children:
            total += child.total()

        return total

    def aggregate(self, dict, categorize, incategory):
        category = None
        if categorize:
            category = categorize(self.symbol)

        total = self.count
        for child in self.children:
            total += child.aggregate(dict, categorize, category or incategory)

        if category:
            dict[category] = dict.get(category, 0) + total
            return 0
        elif not incategory:
            dict[self.symbol] = dict.get(self.symbol, 0) + total

        return total

    def dump(self):
        kids = [ child.symbol for child in self.children]
        print '%s %d <%s>' % (self.symbol, self.count, ', '.join(kids))
        for child in self.children:
            child.dump()

    def _dot(self, dot, threshold, categorize, total):
        from pydot import Dot, Edge, Node
        self.dot_node = None

        value = self.total() * 100.0 / total
        if value < threshold:
            return
        if categorize:
            category = categorize(self.symbol)
            if category and category != 'other':
                return
        label = '%s %.2f%%' % (self.symbol, value)
        self.dot_node = Node(self, label=label)
        dot.add_node(self.dot_node)

        for child in self.children:
            child._dot(dot, threshold, categorize, total)
            if child.dot_node is not None:
                dot.add_edge(Edge(self, child))

    def _cleandot(self):
        for child in self.children:
            child._cleandot()
            self.dot_node = None
            del self.__dict__['dot_node']

    def dot(self, dot, threshold=0.1, categorize=None):
        self._dot(dot, threshold, categorize, self.total())
        self._cleandot()

class FuncData(RunData):
    def __init__(self, filename, categorize=None):
        super(FuncData, self).__init__(filename)
        tree = self.tree
        tree.aggregate(self, categorize, incategory=False)
        self.total = tree.total()

    def __getattribute__(self, attr):
        if attr == 'tree':
            return FuncNode(self.filedata)
        return super(FuncData, self).__getattribute__(attr)

    def displayx(self, output=None, maxcount=None):
        if output is None:
            import sys
            output = sys.stdout

        items = [ (val,key) for key,val in self.iteritems() ]
        items.sort(reverse=True)
        for val,key in items:
            if maxcount is not None:
                if maxcount == 0:
                    return
                maxcount -= 1

            percent = val * 100.0 / self.total
            print >>output, '%-30s %8s' % (key, '%3.2f%%' % percent)

class Profile(object):
    # This list controls the order of values in stacked bar data output
    default_categories = [ 'interrupt',
                           'driver',
                           'stack',
                           'buffer',
                           'copy',
                           'syscall',
                           'user',
                           'other',
                           'idle']

    def __init__(self, datatype, categorize=None):
        categories = Profile.default_categories

        self.datatype = datatype
        self.categorize = categorize
        self.data = {}
        self.categories = categories[:]
        self.rcategories = categories[:]
        self.rcategories.reverse()
        self.cpu = 0

    # Read in files
    def inputdir(self, directory):
        import os, os.path, re
        from os.path import expanduser, join as joinpath

        directory = expanduser(directory)
        label_ex = re.compile(r'profile\.(.*).dat')
        for root,dirs,files in os.walk(directory):
            for name in files:
                match = label_ex.match(name)
                if not match:
                    continue

                filename = joinpath(root, name)
                prefix = os.path.commonprefix([root, directory])
                dirname = root[len(prefix)+1:]
                data = self.datatype(filename, self.categorize)
                self.setdata(dirname, match.group(1), data)

    def setdata(self, run, cpu, data):
        if run not in self.data:
            self.data[run] = {}

        if cpu in self.data[run]:
            raise AttributeError, \
                  'data already stored for run %s and cpu %s' % (run, cpu)

        self.data[run][cpu] = data

    def getdata(self, run, cpu):
        try:
            return self.data[run][cpu]
        except KeyError:
            print run, cpu
            return None

    def alldata(self):
        for run,cpus in self.data.iteritems():
            for cpu,data in cpus.iteritems():
                yield run,cpu,data

    def get(self, job, stat, system=None):
        if system is None and hasattr('system', job):
            system = job.system

        if system is None:
            raise AttributeError, 'The job must have a system set'

        cpu = '%s.run%d' % (system, self.cpu)

        data = self.getdata(str(job), cpu)
        if not data:
            return None

        values = []
        for category in self.categories:
            val = float(data.get(category, 0.0))
            if val < 0.0:
                raise ValueError, 'value is %f' % val
            values.append(val)
        total = sum(values)
        return [ v / total * 100.0 for v in values ]

    def dump(self):
        for run,cpu,data in self.alldata():
            print 'run %s, cpu %s' % (run, cpu)
            data.dump()
            print

    def write_dot(self, threshold, jobfile=None, jobs=None):
        import pydot

        if jobs is None:
            jobs = [ job for job in jobfile.jobs() ]

        for job in jobs:
            cpu =  '%s.run%d' % (job.system, self.cpu)
            symbols = self.getdata(job.name, cpu)
            if not symbols:
                continue

            dot = pydot.Dot()
            symbols.tree.dot(dot, threshold=threshold)
            dot.write(symbols.filename[:-3] + 'dot')

    def write_txt(self, jobfile=None, jobs=None, limit=None):
        if jobs is None:
            jobs = [ job for job in jobfile.jobs() ]

        for job in jobs:
            cpu =  '%s.run%d' % (job.system, self.cpu)
            symbols = self.getdata(job.name, cpu)
            if not symbols:
                continue

            output = file(symbols.filename[:-3] + 'txt', 'w')
            symbols.display(output, limit)

    def display(self, jobfile=None, jobs=None, limit=None):
        if jobs is None:
            jobs = [ job for job in jobfile.jobs() ]

        maxsymlen = 0

        thejobs = []
        for job in jobs:
            cpu =  '%s.run%d' % (job.system, self.cpu)
            symbols = self.getdata(job.name, cpu)
            if symbols:
                thejobs.append(job)
                maxsymlen = max(maxsymlen, symbols.maxsymlen)

        for job in thejobs:
            cpu =  '%s.run%d' % (job.system, self.cpu)
            symbols = self.getdata(job.name, cpu)
            print job.name
            symbols.display(limit=limit, maxsymlen=maxsymlen)
            print


from categories import func_categorize, pc_categorize
class PCProfile(Profile):
    def __init__(self, categorize=pc_categorize):
        super(PCProfile, self).__init__(PCData, categorize)


class FuncProfile(Profile):
    def __init__(self, categorize=func_categorize):
        super(FuncProfile, self).__init__(FuncData, categorize)

def usage(exitcode = None):
    print '''\
Usage: %s [-bc] [-g <dir>] [-j <jobfile>] [-n <num>]

    -c           groups symbols into categories
    -b           dumps data for bar charts
    -d           generate dot output
    -g <d>       draw graphs and send output to <d>
    -j <jobfile> specify a different jobfile (default is Test.py)
    -n <n>       selects number of top symbols to print (default 5)
''' % sys.argv[0]

    if exitcode is not None:
        sys.exit(exitcode)

if __name__ == '__main__':
    import getopt, re, sys
    from os.path import expanduser
    from output import StatOutput

    # default option values
    numsyms = 10
    graph = None
    cpus = [ 0 ]
    categorize = False
    showidle = True
    funcdata = True
    jobfilename = 'Test.py'
    dodot = False
    dotfile = None
    textout = False
    threshold = 0.01
    inputfile = None

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'C:cdD:f:g:ij:n:pT:t')
    except getopt.GetoptError:
        usage(2)

    for o,a in opts:
        if o == '-C':
            cpus = [ int(x) for x in a.split(',') ]
        elif o == '-c':
            categorize = True
        elif o == '-D':
            dotfile = a
        elif o == '-d':
            dodot = True
        elif o == '-f':
            inputfile = expanduser(a)
        elif o == '-g':
            graph = a
        elif o == '-i':
            showidle = False
        elif o == '-j':
            jobfilename = a
        elif o == '-n':
            numsyms = int(a)
        elif o == '-p':
            funcdata = False
        elif o == '-T':
            threshold = float(a)
        elif o == '-t':
            textout = True

    if args:
        print "'%s'" % args, len(args)
        usage(1)

    if inputfile:
        catfunc = None
        if categorize:
            catfunc = func_categorize
        data = FuncData(inputfile, categorize=catfunc)

        if dodot:
            import pydot
            dot = pydot.Dot()
            data.tree.dot(dot, threshold=threshold)
            #dot.orientation = 'landscape'
            #dot.ranksep='equally'
            #dot.rank='samerank'
            dot.write(dotfile, format='png')
        else:
            data.display(limit=numsyms)

    else:
        from jobfile import JobFile
        jobfile = JobFile(jobfilename)

        if funcdata:
            profile = FuncProfile()
        else:
            profile = PCProfile()

        if not categorize:
            profile.categorize = None
        profile.inputdir(jobfile.rootdir)

        if graph:
            for cpu in cpus:
                profile.cpu = cpu
                if funcdata:
                    name = 'funcstacks%d' % cpu
                else:
                    name = 'stacks%d' % cpu
                output = StatOutput(jobfile, info=profile)
                output.xlabel = 'System Configuration'
                output.ylabel = '% CPU utilization'
                output.stat = name
                output.graph(name, graph)

        if dodot:
            for cpu in cpus:
                profile.cpu = cpu
                profile.write_dot(jobfile=jobfile, threshold=threshold)

        if textout:
            for cpu in cpus:
                profile.cpu = cpu
                profile.write_txt(jobfile=jobfile)

        if not graph and not textout and not dodot:
            for cpu in cpus:
                if not categorize:
                    profile.categorize = None
                profile.cpu = cpu
                profile.display(jobfile=jobfile, limit=numsyms)
