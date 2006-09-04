# Copyright (c) 2004-2006 The Regents of The University of Michigan
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
# Authors: Steve Reinhardt
#          Nathan Binkert

#############################
#
# Utility classes & methods
#
#############################

class Singleton(type):
    def __call__(cls, *args, **kwargs):
        if hasattr(cls, '_instance'):
            return cls._instance

        cls._instance = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instance

# Apply method to object.
# applyMethod(obj, 'meth', <args>) is equivalent to obj.meth(<args>)
def applyMethod(obj, meth, *args, **kwargs):
    return getattr(obj, meth)(*args, **kwargs)

# If the first argument is an (non-sequence) object, apply the named
# method with the given arguments.  If the first argument is a
# sequence, apply the method to each element of the sequence (a la
# 'map').
def applyOrMap(objOrSeq, meth, *args, **kwargs):
    if not isinstance(objOrSeq, (list, tuple)):
        return applyMethod(objOrSeq, meth, *args, **kwargs)
    else:
        return [applyMethod(o, meth, *args, **kwargs) for o in objOrSeq]


