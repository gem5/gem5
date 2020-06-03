# Copyright (c) 2020 Jason Lowe-Power
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


import os
import logging
import sys

from gui_views.state import get_path

get_path()
sys.path.append(os.getenv('gem5_path'))

from m5_calls import *

class M5CallTester():
    def __init__(self):
        self.catalog = None

    def catalogTest(self):
        """ Make sure the catalog is received """
        new_catalog = getObjLists()
        if new_catalog != None:
            self.catalog = new_catalog
            return True
        else:
            return False

    def objectTest(self, object):
        """ Test if an object is a SimObject """
        return isSimObject(object)

    def portTest(self, object):
        """ Test for getting ports for an object from gem5 """
        port_info = getPortInfo(object)
        if port_info is None:
            return False
        # For some reason getting the length of the dictionaries results in
        #   different numbers for some objects even though they have the same
        #   number of elements. To get around use list generator for the keys
        len1 = len([key for key in port_info.keys()])
        len2 = len([key for key in object._ports.keys()])
        return (len1 == len2)

    def paramTest(self, object):
        """ Test for getting params for an object from gem5 """
        param_info = getParamInfo(object)
        if param_info is None:
            return False
        # For some reason getting the length of the dictionaries results in
        #   different numbers for some objects even though they have the same
        #   number of elements. To get around use list generator for the keys
        len1 = len([key for key in param_info.keys()])
        len2 = len([key for key in object._params.keys()])
        return (len1 == len2)

    def setParamValueTest(self, object):
        """ Test for setting the param of the object """
        param_info = getParamInfo(object)
        list1 = sorted(list(param_info.keys()))
        try:
            for key in param_info.keys():
                if param_info[key]["Default"] is not None:
                    setattr(object, key, param_info[key]["Default"])
            return True
        except:
             return False


    def runSuite(self):
        """ The unit test suite """
        if self.catalogTest():
            for key, value in self.catalog[1].items():
                instance = value()
                if not self.objectTest(value):
                    logging.error("Object Test Failed for " + key)

                if not self.portTest(value):
                    logging.error("Port Test Failed for " + key)

                if self.paramTest(value):
                    if not self.setParamValueTest(instance):
                        logging.error("Set Param Value Test Failed for " + key)
                else:
                    logging.error("Param Test Failed for " + key)
        else:
            logging.error("Catalog Test Failed")

if __name__ == "__m5_main__":
    tester = M5CallTester()
    tester.runSuite()
