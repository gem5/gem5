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

import sys, random
import copy
import json
import logging

from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *
from graphic_scene import *
from m5_calls import *

from gui_views import state

class DebugWidget(QWidget):
    """Displays options for development and allows enabling of these options"""
    def __init__(self, state):
        super(DebugWidget, self).__init__()
        self.state = state
        self.flags = getDebugFlags()
        self.debugLayout = QVBoxLayout()
        self.debugStatements = True

        # Add check box to redirect debug statements to log file
        self.fileBox = QCheckBox("Log to File")
        self.fileBox.setChecked(True)
        self.fileBox.stateChanged.connect(lambda:self.btnState(self.fileBox))

        # Add input to edit name of log file
        self.logfileEdit = QLineEdit()
        self.logfileEdit.setPlaceholderText("Enter filename")
        self.logfileEdit.setText("debug.log")
        self.logfileEdit.setFixedWidth(150)
        self.logfileEdit.editingFinished.connect(self.switchDebugOutput)

        # Add check box to redirect debug statements to stdout
        self.stdoutBox = QCheckBox("Log to Stdout")
        self.stdoutBox.toggled.connect(lambda:self.btnState(self.stdoutBox))

        # search bar for the debug flags
        self.flagSearch = QLineEdit()
        self.flagSearch.setPlaceholderText("Search for a debug flag here!")
        self.flagSearch.setFixedWidth(250)
        self.flagSearch.textChanged.connect(self.searchFlag)

        self.flagList = self.createFlagList()
        self.flagList.resize(250, 250)

        # Add widgets to layout
        self.debugLayout.addWidget(self.fileBox)
        self.debugLayout.addWidget(self.logfileEdit)
        self.debugLayout.addWidget(self.stdoutBox)
        self.debugLayout.addWidget(self.flagSearch)
        self.debugLayout.addWidget(self.flagList, 50)
        self.debugLayout.addStretch(5)
        self.debugLayout.setSpacing(10)

        # Add debug layout to the main layout
        self.setLayout(self.debugLayout)
        self.switchDebugOutput()


    def searchFlag(self, text):
        """ Search the list widget for flags that match the text"""
        for row in range(self.flagList.count()):
            flag = self.flagList.item(row)
            if text:
                flag.setHidden(not text in flag.text())
            else:
                flag.setHidden(False)


    def switchDebugOutput(self):
        """ This handler switches between stdout and a file for debug msgs"""
        # Get rid of current stream
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)

        if self.debugStatements:
            # redirect debug msgs to file
            logfile = self.logfileEdit.text()
            if '.' not in logfile:
                logfile += '.log'

            logging.basicConfig(filename=logfile, filemode='w', level= \
                logging.DEBUG, format='%(name)s - %(levelname)s - %(message)s')
        else:
            # redirect debug msgs to terminal
            logging.basicConfig(level=logging.DEBUG)


    def btnState(self, box):
        """ If file_box or stdout_box are checked the other shld be \
            unchecked"""
        if box.text() == "Log to File":
            if box.isChecked():
                self.stdoutBox.setChecked(False)
                # should not edit filename
                self.logfileEdit.setReadOnly(False)
                self.debugStatements = True
                self.switchDebugOutput()

        if box.text() == "Log to Stdout":
            if box.isChecked():
                self.fileBox.setChecked(False)
                self.logfileEdit.setReadOnly(True)
                self.debugStatements = False
                self.switchDebugOutput()

    def createFlagList(self):
        """ Create list widget with all the debug flags"""
        flag_list = QListWidget()
        for key in self.flags.keys():
            flag_item = QListWidgetItem()
            flag_item.setText(key)
            flag_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            flag_item.setCheckState(Qt.Unchecked)
            flag_list.addItem(flag_item)

        flag_list.itemClicked.connect(self.flagEnable)
        return flag_list

    def flagEnable(self, item):
        """ Event handler to enable or disable debug flags """
        if item.checkState() == Qt.Checked:
            logging.debug('"%s" Checked' % item.text())
            self.flags[item.text()].enable()
        elif item.checkState() == Qt.Unchecked:
            logging.debug('"%s" Unchecked' % item.text())
            self.flags[item.text()].disable()
        else:
            logging.debug('"%s" Clicked' % item.text())
