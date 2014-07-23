# Copyright (c) 2013 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
# Authors: Andrew Bardsley

import parse
import colours
from colours import unknownColour
from point import Point
import re
import blobs
from time import time as wall_time
import os

id_parts = "TSPLFE"

all_ids = set(id_parts)
no_ids = set([])

class BlobDataSelect(object):
    """Represents which data is displayed for Ided object"""
    def __init__(self):
        # Copy all_ids
        self.ids = set(all_ids)

    def __and__(self, rhs):
        """And for filtering"""
        ret = BlobDataSelect()
        ret.ids = self.ids.intersection(rhs.ids)
        return ret

class BlobVisualData(object):
    """Super class for block data colouring"""
    def to_striped_block(self, select):
        """Return an array of colours to use for a striped block"""
        return unknownColour

    def get_inst(self):
        """Get an instruction Id (if any) from this data"""
        return None

    def get_line(self):
        """Get a line Id (if any) from this data"""
        return None

    def __repr__(self):
        return self.__class__.__name__ + '().from_string(' + \
            self.__str__() + ')'

    def __str__(self):
        return ''

class Id(BlobVisualData):
    """A line or instruction id"""
    def __init__(self):
        self.isFault = False
        self.threadId = 0
        self.streamSeqNum = 0
        self.predictionSeqNum = 0
        self.lineSeqNum = 0
        self.fetchSeqNum = 0
        self.execSeqNum = 0

    def as_list(self):
        return [self.threadId, self.streamSeqNum, self.predictionSeqNum,
            self.lineSeqNum, self.fetchSeqNum, self.execSeqNum]

    def __cmp__(self, right):
        return cmp(self.as_list(), right.as_list())

    def from_string(self, string):
        m = re.match('^(F;)?(\d+)/(\d+)\.(\d+)/(\d+)(/(\d+)(\.(\d+))?)?',
            string)

        def seqnum_from_string(string):
            if string is None:
                return 0
            else:
                return int(string)

        if m is None:
            print 'Invalid Id string', string
        else:
            elems = m.groups()

            if elems[0] is not None:
                self.isFault = True
            else:
                self.isFault = False

            self.threadId = seqnum_from_string(elems[1])
            self.streamSeqNum = seqnum_from_string(elems[2])
            self.predictionSeqNum = seqnum_from_string(elems[3])
            self.lineSeqNum = seqnum_from_string(elems[4])
            self.fetchSeqNum = seqnum_from_string(elems[6])
            self.execSeqNum = seqnum_from_string(elems[8])
        return self

    def get_inst(self):
        if self.fetchSeqNum != 0:
            return self
        else:
            return None

    def get_line(self):
        return self

    def __str__(self):
        """Returns the usual id T/S.P/L/F.E string"""
        return (
            str(self.threadId) + '/' +
            str(self.streamSeqNum) + '.' +
            str(self.predictionSeqNum) + '/' +
            str(self.lineSeqNum) + '/' +
            str(self.fetchSeqNum) + '.' +
            str(self.execSeqNum))

    def to_striped_block(self, select):
        ret = []

        if self.isFault:
            ret.append(colours.faultColour)

        if 'T' in select.ids:
            ret.append(colours.number_to_colour(self.threadId))
        if 'S' in select.ids:
            ret.append(colours.number_to_colour(self.streamSeqNum))
        if 'P' in select.ids:
            ret.append(colours.number_to_colour(self.predictionSeqNum))
        if 'L' in select.ids:
            ret.append(colours.number_to_colour(self.lineSeqNum))
        if self.fetchSeqNum != 0 and 'F' in select.ids:
            ret.append(colours.number_to_colour(self.fetchSeqNum))
        if self.execSeqNum != 0 and 'E' in select.ids:
            ret.append(colours.number_to_colour(self.execSeqNum))

        if len(ret) == 0:
            ret = [colours.unknownColour]

        if self.isFault:
            ret.append(colours.faultColour)

        return ret

class Branch(BlobVisualData):
    """Branch data new stream and prediction sequence numbers, a branch
    reason and a new PC"""
    def __init__(self):
        self.newStreamSeqNum = 0
        self.newPredictionSeqNum = 0
        self.newPC = 0
        self.reason = "NoBranch"
        self.id = Id()

    def from_string(self, string):
        m = re.match('^(\w+);(\d+)\.(\d+);([0-9a-fA-Fx]+);(.*)$', string)

        if m is not None:
            self.reason, newStreamSeqNum, newPredictionSeqNum, \
                newPC, id = m.groups()

            self.newStreamSeqNum = int(newStreamSeqNum)
            self.newPredictionSeqNum = int(newPredictionSeqNum)
            self.newPC = int(newPC, 0)
            self.id = special_view_decoder(Id)(id)
            # self.branch = special_view_decoder(Branch)(branch)
        else:
            print "Bad Branch data:", string
        return self

    def to_striped_block(self, select):
        return [colours.number_to_colour(self.newStreamSeqNum),
            colours.number_to_colour(self.newPredictionSeqNum),
            colours.number_to_colour(self.newPC)]

class Counts(BlobVisualData):
    """Treat the input data as just a /-separated list of count values (or
    just a single value)"""
    def __init__(self):
        self.counts = []

    def from_string(self, string):
        self.counts = map(int, re.split('/', string))
        return self

    def to_striped_block(self, select):
        return map(colours.number_to_colour, self.counts)

class Colour(BlobVisualData):
    """A fixed colour block, used for special colour decoding"""
    def __init__(self, colour):
        self.colour = colour

    def to_striped_block(self, select):
        return [self.colour]

class DcacheAccess(BlobVisualData):
    """Data cache accesses [RW];id"""
    def __init__(self):
        self.direc = 'R'
        self.id = Id()

    def from_string(self, string):
        self.direc, id = re.match('^([RW]);([^;]*);.*$', string).groups()
        self.id.from_string(id)
        return self

    def get_inst(self):
        return self.id

    def to_striped_block(self, select):
        if self.direc == 'R':
            direc_colour = colours.readColour
        elif self.direc == 'R':
            direc_colour = colours.writeColour
        else:
            direc_colour = colours.errorColour
        return [direc_colour] + self.id.to_striped_block(select)

class ColourPattern(object):
    """Super class for decoders that make 2D grids rather than just single
    striped blocks"""
    def elems(self):
        return []

    def to_striped_block(self, select):
        return [[[colours.errorColour]]]

def special_view_decoder(class_):
    """Generate a decode function that checks for special character
    arguments first (and generates a fixed colour) before building a
    BlobVisualData of the given class"""
    def decode(symbol):
        if symbol in special_state_colours:
            return Colour(special_state_colours[symbol])
        else:
            return class_().from_string(symbol)
    return decode

class TwoDColours(ColourPattern):
    """A 2D grid pattern decoder"""
    def __init__(self, blockss):
        self.blockss = blockss

    @classmethod
    def decoder(class_, elemClass, dataName):
        """Factory for making decoders for particular block types"""
        def decode(pairs):
            if dataName not in pairs:
                print 'TwoDColours: no event data called:', \
                    dataName, 'in:', pairs
                return class_([[Colour(colours.errorColour)]])
            else:
                parsed = parse.list_parser(pairs[dataName])
                return class_(parse.map2(special_view_decoder(elemClass), \
                    parsed))
        return decode

    @classmethod
    def indexed_decoder(class_, elemClass, dataName, picPairs):
        """Factory for making decoders for particular block types but
        where the list elements are pairs of (index, data) and
        strip and stripelems counts are picked up from the pair
        data on the decoder's picture file.  This gives a 2D layout
        of the values with index 0 at strip=0, elem=0 and index 1
        at strip=0, elem=1"""
        def decode(pairs):
            if dataName not in pairs:
                print 'TwoDColours: no event data called:', \
                    dataName, 'in:', pairs
                return class_([[Colour(colours.errorColour)]])
            else:
                strips = int(picPairs['strips'])
                strip_elems = int(picPairs['stripelems'])

                raw_iv_pairs = pairs[dataName]

                parsed = parse.parse_indexed_list(raw_iv_pairs)

                array = [[Colour(colours.emptySlotColour)
                    for i in xrange(0, strip_elems)]
                    for j in xrange(0, strips)]

                for index, value in parsed:
                    try:
                        array[index % strips][index / strips] = \
                            special_view_decoder(elemClass)(value)
                    except:
                        print "Element out of range strips: %d," \
                            " stripelems %d, index: %d" % (strips,
                            strip_elems, index)

                # return class_(array)
                return class_(array)
        return decode

    def elems(self):
        """Get a flat list of all elements"""
        ret = []
        for blocks in self.blockss:
            ret += blocks
        return ret

    def to_striped_block(self, select):
        return parse.map2(lambda d: d.to_striped_block(select), self.blockss)

class FrameColours(ColourPattern):
    """Decode to a 2D grid which has a single occupied row from the event
    data and some blank rows forming a frame with the occupied row as a
    'title' coloured stripe"""
    def __init__(self, block, numBlankSlots):
        self.numBlankSlots = numBlankSlots
        self.block = block

    @classmethod
    def decoder(class_, elemClass, numBlankSlots, dataName):
        """Factory for element type"""
        def decode(pairs):
            if dataName not in pairs:
                print 'FrameColours: no event data called:', dataName, \
                    'in:', pairs
                return class_([Colour(colours.errorColour)])
            else:
                parsed = parse.list_parser(pairs[dataName])
                return class_(special_view_decoder(elemClass)
                    (parsed[0][0]), numBlankSlots)
        return decode

    def elems(self):
        return [self.block]

    def to_striped_block(self, select):
        return ([[self.block.to_striped_block(select)]] +
            (self.numBlankSlots * [[[colours.backgroundColour]]]))

special_state_colours = {
    'U': colours.unknownColour,
    'B': colours.blockedColour,
    '-': colours.bubbleColour,
    '': colours.emptySlotColour,
    'E': colours.emptySlotColour,
    'R': colours.reservedSlotColour,
    'X': colours.errorColour,
    'F': colours.faultColour,
    'r': colours.readColour,
    'w': colours.writeColour
    }

special_state_names = {
    'U': '(U)nknown',
    'B': '(B)locked',
    '-': '(-)Bubble',
    '': '()Empty',
    'E': '(E)mpty',
    'R': '(R)eserved',
    'X': '(X)Error',
    'F': '(F)ault',
    'r': '(r)ead',
    'w': '(w)rite'
    }

special_state_chars = special_state_colours.keys()

# The complete set of available block data types
decoder_element_classes = {
    'insts': Id,
    'lines': Id,
    'branch': Branch,
    'dcache': DcacheAccess,
    'counts': Counts
    }

indexed_decoder_element_classes = {
    'indexedCounts' : Counts
    }

def find_colour_decoder(stripSpace, decoderName, dataName, picPairs):
    """Make a colour decoder from some picture file blob attributes"""
    if decoderName == 'frame':
        return FrameColours.decoder(Counts, stripSpace, dataName)
    elif decoderName in decoder_element_classes:
        return TwoDColours.decoder(decoder_element_classes[decoderName],
            dataName)
    elif decoderName in indexed_decoder_element_classes:
        return TwoDColours.indexed_decoder(
            indexed_decoder_element_classes[decoderName], dataName, picPairs)
    else:
        return None

class IdedObj(object):
    """An object identified by an Id carrying paired data.
    The super class for Inst and Line"""

    def __init__(self, id, pairs={}):
        self.id = id
        self.pairs = pairs

    def __cmp__(self, right):
        return cmp(self.id, right.id)

    def table_line(self):
        """Represent the object as a list of table row data"""
        return []

    # FIXME, add a table column titles?

    def __repr__(self):
        return ' '.join(self.table_line())

class Inst(IdedObj):
    """A non-fault instruction"""
    def __init__(self, id, disassembly, addr, pairs={}):
        super(Inst,self).__init__(id, pairs)
        if 'nextAddr' in pairs:
            self.nextAddr = int(pairs['nextAddr'], 0)
            del pairs['nextAddr']
        else:
            self.nextAddr = None
        self.disassembly = disassembly
        self.addr = addr

    def table_line(self):
        if self.nextAddr is not None:
            addrStr = '0x%x->0x%x' % (self.addr, self.nextAddr)
        else:
            addrStr = '0x%x' % self.addr
        ret = [addrStr, self.disassembly]
        for name, value in self.pairs.iteritems():
            ret.append("%s=%s" % (name, str(value)))
        return ret

class InstFault(IdedObj):
    """A fault instruction"""
    def __init__(self, id, fault, addr, pairs={}):
        super(InstFault,self).__init__(id, pairs)
        self.fault = fault
        self.addr = addr

    def table_line(self):
        ret = ["0x%x" % self.addr, self.fault]
        for name, value in self.pairs:
            ret.append("%s=%s", name, str(value))
        return ret

class Line(IdedObj):
    """A fetched line"""
    def __init__(self, id, vaddr, paddr, size, pairs={}):
        super(Line,self).__init__(id, pairs)
        self.vaddr = vaddr
        self.paddr = paddr
        self.size = size

    def table_line(self):
        ret = ["0x%x/0x%x" % (self.vaddr, self.paddr), "%d" % self.size]
        for name, value in self.pairs:
            ret.append("%s=%s", name, str(value))
        return ret

class LineFault(IdedObj):
    """A faulting line"""
    def __init__(self, id, fault, vaddr, pairs={}):
        super(LineFault,self).__init__(id, pairs)
        self.vaddr = vaddr
        self.fault = fault

    def table_line(self):
        ret = ["0x%x" % self.vaddr, self.fault]
        for name, value in self.pairs:
            ret.append("%s=%s", name, str(value))
        return ret

class BlobEvent(object):
    """Time event for a single blob"""
    def __init__(self, unit, time, pairs = {}):
        # blob's unit name
        self.unit = unit
        self.time = time
        # dict of picChar (blob name) to visual data
        self.visuals = {}
        # Miscellaneous unparsed MinorTrace line data
        self.pairs = pairs
        # Non-MinorTrace debug printout for this unit at this time
        self.comments = []

    def find_ided_objects(self, model, picChar, includeInstLines):
        """Find instructions/lines mentioned in the blob's event
        data"""
        ret = []
        if picChar in self.visuals:
            blocks = self.visuals[picChar].elems()
            def find_inst(data):
                instId = data.get_inst()
                lineId = data.get_line()
                if instId is not None:
                    inst = model.find_inst(instId)
                    line = model.find_line(instId)
                    if inst is not None:
                        ret.append(inst)
                    if includeInstLines and line is not None:
                        ret.append(line)
                elif lineId is not None:
                    line = model.find_line(lineId)
                    if line is not None:
                        ret.append(line)
            map(find_inst, blocks)
        return sorted(ret)

class BlobModel(object):
    """Model bringing together blob definitions and parsed events"""
    def __init__(self, unitNamePrefix=''):
        self.blobs = []
        self.unitNameToBlobs = {}
        self.unitEvents = {}
        self.clear_events()
        self.picSize = Point(20,10)
        self.lastTime = 0
        self.unitNamePrefix = unitNamePrefix

    def clear_events(self):
        """Drop all events and times"""
        self.lastTime = 0
        self.times = []
        self.insts = {}
        self.lines = {}
        self.numEvents = 0

        for unit, events in self.unitEvents.iteritems():
            self.unitEvents[unit] = []

    def add_blob(self, blob):
        """Add a parsed blob to the model"""
        self.blobs.append(blob)
        if blob.unit not in self.unitNameToBlobs:
            self.unitNameToBlobs[blob.unit] = []

        self.unitNameToBlobs[blob.unit].append(blob)

    def add_inst(self, inst):
        """Add a MinorInst instruction definition to the model"""
        # Is this a non micro-op instruction.  Microops (usually) get their
        #   fetchSeqNum == 0 varient stored first
        macroop_key = (inst.id.fetchSeqNum, 0)
        full_key = (inst.id.fetchSeqNum, inst.id.execSeqNum)

        if inst.id.execSeqNum != 0 and macroop_key not in self.insts:
            self.insts[macroop_key] = inst

        self.insts[full_key] = inst

    def find_inst(self, id):
        """Find an instruction either as a microop or macroop"""
        macroop_key = (id.fetchSeqNum, 0)
        full_key = (id.fetchSeqNum, id.execSeqNum)

        if full_key in self.insts:
            return self.insts[full_key]
        elif macroop_key in self.insts:
            return self.insts[macroop_key]
        else:
            return None

    def add_line(self, line):
        """Add a MinorLine line to the model"""
        self.lines[line.id.lineSeqNum] = line

    def add_unit_event(self, event):
        """Add a single event to the model.  This must be an event at a
        time >= the current maximum time"""
        if event.unit in self.unitEvents:
            events = self.unitEvents[event.unit]
            if len(events) > 0 and events[len(events)-1].time > event.time:
                print "Bad event ordering"
            events.append(event)
        self.numEvents += 1
        self.lastTime = max(self.lastTime, event.time)

    def extract_times(self):
        """Extract a list of all the times from the seen events.  Call after
        reading events to give a safe index list to use for time indices"""
        times = {}
        for unitEvents in self.unitEvents.itervalues():
            for event in unitEvents:
                times[event.time] = 1
        self.times = times.keys()
        self.times.sort()

    def find_line(self, id):
        """Find a line by id"""
        key = id.lineSeqNum
        return self.lines.get(key, None)

    def find_event_bisection(self, unit, time, events,
        lower_index, upper_index):
        """Find an event by binary search on time indices"""
        while lower_index <= upper_index:
            pivot = (upper_index + lower_index) / 2
            pivotEvent = events[pivot]
            event_equal = (pivotEvent.time == time or
                (pivotEvent.time < time and
                    (pivot == len(events) - 1 or
                        events[pivot + 1].time > time)))

            if event_equal:
                return pivotEvent
            elif time > pivotEvent.time:
                if pivot == upper_index:
                    return None
                else:
                    lower_index = pivot + 1
            elif time < pivotEvent.time:
                if pivot == lower_index:
                    return None
                else:
                    upper_index = pivot - 1
            else:
                return None
        return None

    def find_unit_event_by_time(self, unit, time):
        """Find the last event for the given unit at time <= time"""
        if unit in self.unitEvents:
            events = self.unitEvents[unit]
            ret = self.find_event_bisection(unit, time, events,
                0, len(events)-1)

            return ret
        else:
            return None

    def find_time_index(self, time):
        """Find a time index close to the given time (where
        times[return] <= time and times[return+1] > time"""
        ret = 0
        lastIndex = len(self.times) - 1
        while ret < lastIndex and self.times[ret + 1] <= time:
            ret += 1
        return ret

    def add_minor_inst(self, rest):
        """Parse and add a MinorInst line to the model"""
        pairs = parse.parse_pairs(rest)
        other_pairs = dict(pairs)

        id = Id().from_string(pairs['id'])
        del other_pairs['id']

        addr = int(pairs['addr'], 0)
        del other_pairs['addr']

        if 'inst' in other_pairs:
            del other_pairs['inst']

            # Collapse unnecessary spaces in disassembly
            disassembly = re.sub('  *', ' ',
                re.sub('^ *', '', pairs['inst']))

            inst = Inst(id, disassembly, addr, other_pairs)
            self.add_inst(inst)
        elif 'fault' in other_pairs:
            del other_pairs['fault']

            inst = InstFault(id, pairs['fault'], addr, other_pairs)

            self.add_inst(inst)

    def add_minor_line(self, rest):
        """Parse and add a MinorLine line to the model"""
        pairs = parse.parse_pairs(rest)
        other_pairs = dict(pairs)

        id = Id().from_string(pairs['id'])
        del other_pairs['id']

        vaddr = int(pairs['vaddr'], 0)
        del other_pairs['vaddr']

        if 'paddr' in other_pairs:
            del other_pairs['paddr']
            del other_pairs['size']
            paddr = int(pairs['paddr'], 0)
            size = int(pairs['size'], 0)

            self.add_line(Line(id,
                vaddr, paddr, size, other_pairs))
        elif 'fault' in other_pairs:
            del other_pairs['fault']

            self.add_line(LineFault(id, pairs['fault'], vaddr, other_pairs))

    def load_events(self, file, startTime=0, endTime=None):
        """Load an event file and add everything to this model"""
        def update_comments(comments, time):
            # Add a list of comments to an existing event, if there is one at
            #   the given time, or create a new, correctly-timed, event from
            #   the last event and attach the comments to that
            for commentUnit, commentRest in comments:
                event = self.find_unit_event_by_time(commentUnit, time)
                # Find an event to which this comment can be attached
                if event is None:
                    # No older event, make a new empty one
                    event = BlobEvent(commentUnit, time, {})
                    self.add_unit_event(event)
                elif event.time != time:
                    # Copy the old event and make a new one with the right
                    #   time and comment
                    newEvent = BlobEvent(commentUnit, time, event.pairs)
                    newEvent.visuals = dict(event.visuals)
                    event = newEvent
                    self.add_unit_event(event)
                event.comments.append(commentRest)

        self.clear_events()

        # A negative time will *always* be different from an event time
        time = -1
        time_events = {}
        last_time_lines = {}
        minor_trace_line_count = 0
        comments = []

        default_colour = [[colours.unknownColour]]
        next_progress_print_event_count = 1000

        if not os.access(file, os.R_OK):
            print 'Can\'t open file', file
            exit(1)
        else:
            print 'Opening file', file

        f = open(file)

        start_wall_time = wall_time()

        # Skip leading events
        still_skipping = True
        l = f.readline()
        while l and still_skipping:
            match = re.match('^\s*(\d+):', l)
            if match is not None:
                event_time = match.groups()
                if int(event_time[0]) >= startTime:
                    still_skipping = False
                else:
                    l = f.readline()
            else:
                l = f.readline()

        match_line_re = re.compile(
            '^\s*(\d+):\s*([\w\.]+):\s*(Minor\w+:)?\s*(.*)$')

        # Parse each line of the events file, accumulating comments to be
        #   attached to MinorTrace events when the time changes
        reached_end_time = False
        while not reached_end_time and l:
            match = match_line_re.match(l)
            if match is not None:
                event_time, unit, line_type, rest = match.groups()
                event_time = int(event_time)

                unit = re.sub('^' + self.unitNamePrefix + '\.?(.*)$',
                    '\\1', unit)

                # When the time changes, resolve comments
                if event_time != time:
                    if self.numEvents > next_progress_print_event_count:
                        print ('Parsed to time: %d' % event_time)
                        next_progress_print_event_count = (
                            self.numEvents + 1000)
                    update_comments(comments, time)
                    comments = []
                    time = event_time

                if line_type is None:
                    # Treat this line as just a 'comment'
                    comments.append((unit, rest))
                elif line_type == 'MinorTrace:':
                    minor_trace_line_count += 1

                    # Only insert this event if it's not the same as
                    #   the last event we saw for this unit
                    if last_time_lines.get(unit, None) != rest:
                        event = BlobEvent(unit, event_time, {})
                        pairs = parse.parse_pairs(rest)
                        event.pairs = pairs

                        # Try to decode the colour data for this event
                        blobs = self.unitNameToBlobs.get(unit, [])
                        for blob in blobs:
                            if blob.visualDecoder is not None:
                                event.visuals[blob.picChar] = (
                                    blob.visualDecoder(pairs))

                        self.add_unit_event(event)
                        last_time_lines[unit] = rest
                elif line_type == 'MinorInst:':
                    self.add_minor_inst(rest)
                elif line_type == 'MinorLine:':
                    self.add_minor_line(rest)

            if endTime is not None and time > endTime:
                reached_end_time = True

            l = f.readline()

        update_comments(comments, time)
        self.extract_times()
        f.close()

        end_wall_time = wall_time()

        print 'Total events:', minor_trace_line_count, 'unique events:', \
            self.numEvents
        print 'Time to parse:', end_wall_time - start_wall_time

    def add_blob_picture(self, offset, pic, nameDict):
        """Add a parsed ASCII-art pipeline markup to the model"""
        pic_width = 0
        for line in pic:
            pic_width = max(pic_width, len(line))
        pic_height = len(pic)

        # Number of horizontal characters per 'pixel'.  Should be 2
        charsPerPixel = 2

        # Clean up pic_width to a multiple of charsPerPixel
        pic_width = (pic_width + charsPerPixel - 1) // 2

        self.picSize = Point(pic_width, pic_height)

        def pic_at(point):
            """Return the char pair at the given point.
            Returns None for characters off the picture"""
            x, y = point.to_pair()
            x *= 2
            if y >= len(pic) or x >= len(pic[y]):
                return None
            else:
                return pic[y][x:x + charsPerPixel]

        def clear_pic_at(point):
            """Clear the chars at point so we don't trip over them again"""
            line = pic[point.y]
            x = point.x * charsPerPixel
            pic[point.y] = line[0:x] + (' ' * charsPerPixel) + \
                line[x + charsPerPixel:]

        def skip_same_char(start, increment):
            """Skip characters which match pic_at(start)"""
            char = pic_at(start)
            hunt = start
            while pic_at(hunt) == char:
                hunt += increment
            return hunt

        def find_size(start):
            """Find the size of a rectangle with top left hand corner at
            start consisting of (at least) a -. shaped corner describing
            the top right corner of a rectangle of the same char"""
            char = pic_at(start)
            hunt_x = skip_same_char(start, Point(1,0))
            hunt_y = skip_same_char(start, Point(0,1))
            off_bottom_right = (hunt_x * Point(1,0)) + (hunt_y * Point(0,1))
            return off_bottom_right - start

        def point_return(point):
            """Carriage return, line feed"""
            return Point(0, point.y + 1)

        def find_arrow(start):
            """Find a simple 1-char wide arrow"""

            def body(endChar, contChar, direc):
                arrow_point = start
                arrow_point += Point(0, 1)
                clear_pic_at(start)
                while pic_at(arrow_point) == contChar:
                    clear_pic_at(arrow_point)
                    arrow_point += Point(0, 1)

                if pic_at(arrow_point) == endChar:
                    clear_pic_at(arrow_point)
                    self.add_blob(blobs.Arrow('_', start + offset,
                        direc = direc,
                        size = (Point(1, 1) + arrow_point - start)))
                else:
                    print 'Bad arrow', start

            char = pic_at(start)
            if char == '-\\':
                body('-/', ' :', 'right')
            elif char == '/-':
                body('\\-', ': ', 'left')

        blank_chars = ['  ', ' :', ': ']

        # Traverse the picture left to right, top to bottom to find blobs
        seen_dict = {}
        point = Point(0,0)
        while pic_at(point) is not None:
            while pic_at(point) is not None:
                char = pic_at(point)
                if char == '->':
                    self.add_blob(blobs.Arrow('_', point + offset,
                        direc = 'right'))
                elif char == '<-':
                    self.add_blob(blobs.Arrow('_', point + offset,
                        direc = 'left'))
                elif char == '-\\' or char == '/-':
                    find_arrow(point)
                elif char in blank_chars:
                    pass
                else:
                    if char not in seen_dict:
                        size = find_size(point)
                        topLeft = point + offset
                        if char not in nameDict:
                            # Unnamed blobs
                            self.add_blob(blobs.Block(char,
                                nameDict.get(char, '_'),
                                topLeft, size = size))
                        else:
                            # Named blobs, set visual info.
                            blob = nameDict[char]
                            blob.size = size
                            blob.topLeft = topLeft
                            self.add_blob(blob)
                    seen_dict[char] = True
                point = skip_same_char(point, Point(1,0))
            point = point_return(point)

    def load_picture(self, filename):
        """Load a picture file into the model"""
        def parse_blob_description(char, unit, macros, pairsList):
            # Parse the name value pairs in a blob-describing line
            def expand_macros(pairs, newPairs):
                # Recursively expand macros
                for name, value in newPairs:
                    if name in macros:
                        expand_macros(pairs, macros[name])
                    else:
                        pairs[name] = value
                return pairs

            pairs = expand_macros({}, pairsList)

            ret = None

            typ = pairs.get('type', 'block')
            colour = colours.name_to_colour(pairs.get('colour', 'black'))

            if typ == 'key':
                ret = blobs.Key(char, unit, Point(0,0), colour)
            elif typ == 'block':
                ret = blobs.Block(char, unit, Point(0,0), colour)
            else:
                print "Bad picture blog type:", typ

            if 'hideId' in pairs:
                hide = pairs['hideId']
                ret.dataSelect.ids -= set(hide)

            if typ == 'block':
                ret.displayName = pairs.get('name', unit)
                ret.nameLoc = pairs.get('nameLoc', 'top')
                ret.shape = pairs.get('shape', 'box')
                ret.stripDir = pairs.get('stripDir', 'horiz')
                ret.stripOrd = pairs.get('stripOrd', 'LR')
                ret.blankStrips = int(pairs.get('blankStrips', '0'))
                ret.shorten = int(pairs.get('shorten', '0'))

                if 'decoder' in pairs:
                    decoderName = pairs['decoder']
                    dataElement = pairs.get('dataElement', decoderName)

                    decoder = find_colour_decoder(ret.blankStrips,
                        decoderName, dataElement, pairs)
                    if decoder is not None:
                        ret.visualDecoder = decoder
                    else:
                        print 'Bad visualDecoder requested:', decoderName

                if 'border' in pairs:
                    border = pairs['border']
                    if border == 'thin':
                        ret.border = 0.2
                    elif border == 'mid':
                        ret.border = 0.5
                    else:
                        ret.border = 1.0
            elif typ == 'key':
                ret.colours = pairs.get('colours', ret.colours)

            return ret

        def line_is_comment(line):
            """Returns true if a line starts with #, returns False
            for lines which are None"""
            return line is not None \
                and re.match('^\s*#', line) is not None

        def get_line(f):
            """Get a line from file f extending that line if it ends in
            '\' and dropping lines that start with '#'s"""
            ret = f.readline()

            # Discard comment lines
            while line_is_comment(ret):
                ret = f.readline()

            if ret is not None:
                extend_match = re.match('^(.*)\\\\$', ret)

                while extend_match is not None:
                    new_line = f.readline()

                    if new_line is not None and not line_is_comment(new_line):
                        line_wo_backslash, = extend_match.groups()
                        ret = line_wo_backslash + new_line
                        extend_match = re.match('^(.*)\\\\$', ret)
                    else:
                        extend_match = None

            return ret

        # Macros are recursively expanded into name=value pairs
        macros = {}

        if not os.access(filename, os.R_OK):
            print 'Can\'t open file', filename
            exit(1)
        else:
            print 'Opening file', filename

        f = open(filename)
        l = get_line(f)
        picture = []
        blob_char_dict = {}

        self.unitEvents = {}
        self.clear_events()

        # Actually parse the file
        in_picture = False
        while l:
            l = parse.remove_trailing_ws(l)
            l = re.sub('#.*', '', l)

            if re.match("^\s*$", l) is not None:
                pass
            elif l == '<<<':
                in_picture = True
            elif l == '>>>':
                in_picture = False
            elif in_picture:
                picture.append(re.sub('\s*$', '', l))
            else:
                line_match = re.match(
                    '^([a-zA-Z0-9][a-zA-Z0-9]):\s+([\w.]+)\s*(.*)', l)
                macro_match = re.match('macro\s+(\w+):(.*)', l)

                if macro_match is not None:
                    name, defn = macro_match.groups()
                    macros[name] = parse.parse_pairs_list(defn)
                elif line_match is not None:
                    char, unit, pairs = line_match.groups()
                    blob = parse_blob_description(char, unit, macros,
                        parse.parse_pairs_list(pairs))
                    blob_char_dict[char] = blob
                    # Setup the events structure
                    self.unitEvents[unit] = []
                else:
                    print 'Problem with Blob line:', l

            l = get_line(f)

        self.blobs = []
        self.add_blob_picture(Point(0,1), picture, blob_char_dict)
