# Copyright (c) 2011 Advanced Micro Devices, Inc.
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
# Author: Steve Reinhardt

import sys
import fcntl
import termios
import struct

# Intended usage example:
#
# if force_colors:
#    from m5.util.terminal import termcap
# elif no_colors:
#    from m5.util.terminal import no_termcap as termcap
# else:
#    from m5.util.terminal import tty_termcap as termcap
# print termcap.Blue + "This could be blue!" + termcap.Normal

# ANSI color names in index order
color_names = "Black Red Green Yellow Blue Magenta Cyan White".split()
default_separator = '='

# Character attribute capabilities.  Note that not all terminals
# support all of these capabilities, or support them
# differently/meaningfully.  For example:
#
# - In PuTTY (with the default settings), Dim has no effect, Standout
#   is the same as Reverse, and Blink does not blink but switches to a
#   gray background.
#
# Please feel free to add information about other terminals here.
#
capability_map = {
         'Bold': 'bold',
          'Dim': 'dim',
        'Blink': 'blink',
    'Underline': 'smul',
      'Reverse': 'rev',
     'Standout': 'smso',
       'Normal': 'sgr0'
}

capability_names = capability_map.keys()

def null_cap_string(s, *args):
    return ''

try:
    import curses
    curses.setupterm()
    def cap_string(s, *args):
        cap = curses.tigetstr(s)
        if cap:
            return curses.tparm(cap, *args).decode("utf-8")
        else:
            return ''
except:
    cap_string = null_cap_string

class ColorStrings(object):
    def __init__(self, cap_string):
        for i, c in enumerate(color_names):
            setattr(self, c, cap_string('setaf', i))
        for name, cap in capability_map.items():
            setattr(self, name, cap_string(cap))

termcap = ColorStrings(cap_string)
no_termcap = ColorStrings(null_cap_string)

if sys.stdout.isatty():
    tty_termcap = termcap
else:
    tty_termcap = no_termcap

def get_termcap(use_colors = None):
    if use_colors:
        return termcap
    elif use_colors is None:
        # option unspecified; default behavior is to use colors iff isatty
        return tty_termcap
    else:
        return no_termcap

def terminal_size():
    '''Return the (width, heigth) of the terminal screen.'''
    try:
        h, w, hp, wp = struct.unpack('HHHH',
            fcntl.ioctl(0, termios.TIOCGWINSZ,
            struct.pack('HHHH', 0, 0, 0, 0)))
        return w, h
    except IOError:
        # It's possible that in sandboxed environments the above ioctl is not
        # allowed (e.g., some jenkins setups)
        return 80, 24


def separator(char=default_separator, color=None):
    '''
    Return a separator of the given character that is the length of the full
    width of the terminal screen.
    '''
    (w, h) = terminal_size()
    if color:
        return color + char*w + termcap.Normal
    else:
        return char*w

def insert_separator(inside, char=default_separator,
                     min_barrier=3, color=None):
    '''
    Place the given string inside of the separator. If it does not fit inside,
    expand the separator to fit it with at least min_barrier.

    .. seealso:: :func:`separator`
    '''
    # Use a bytearray so it's efficient to manipulate
    string = bytearray(separator(char, color=color), 'utf-8')

    # Check if we can fit inside with at least min_barrier.
    gap = (len(string) - len(inside)) - min_barrier * 2
    if gap > 0:
        # We'll need to expand the string to fit us.
        string.extend([ char for _ in range(-gap)])
    # Emplace inside
    middle = (len(string)-1)//2
    start_idx = middle - len(inside)//2
    string[start_idx:len(inside)+start_idx] = str.encode(inside)
    return str(string.decode("utf-8"))


if __name__ == '__main__':
    def test_termcap(obj):
        for c_name in color_names:
            c_str = getattr(obj, c_name)
            print(c_str + c_name + obj.Normal)
            for attr_name in capability_names:
                if attr_name == 'Normal':
                    continue
                attr_str = getattr(obj, attr_name)
                print(attr_str + c_str + attr_name + " " + c_name + obj.Normal)
            print(obj.Bold + obj.Underline + \
                  c_name + "Bold Underline " + c_str + obj.Normal)

    print("=== termcap enabled ===")
    test_termcap(termcap)
    print(termcap.Normal)
    print("=== termcap disabled ===")
    test_termcap(no_termcap)
