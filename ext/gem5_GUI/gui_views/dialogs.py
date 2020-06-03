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

import sys

from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *

from graphic_scene import *

class instantiateDialog(QDialog):
    def __init__(self, state):
        # Add dialog in the context of the main window
        super(instantiateDialog, self).__init__(state.mainWindow.main)

        # Configure the dialog with text and options
        self.setWindowTitle("Entering Instantiate Mode")
        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.text = QLabel(self)
        self.text.setText("""Warning: Once you instantiate, you cannot modify
                            any values. As such, we will save before
                            continuing.""")

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        # Create new layout for the dialog box and add text/button widgets
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)


class saveChangesDialog(QDialog):
    def __init__(self, reason, state):
        # Add dialog in the context of the main window
        super(saveChangesDialog, self).__init__(state.mainWindow.main)

        # Configure the dialog with text and options
        self.setWindowTitle("Save Changes")
        QBtn = QDialogButtonBox.Yes | QDialogButtonBox.No

        self.text = QLabel(self)
        self.text.setText("Would you like to save before " + reason + "?")

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        # Create new layout for the dialog box and add text/button widgets
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)


class deleteWireDialog(QDialog):
    def __init__(self, dialogText):
        super(deleteWireDialog, self).__init__()

        # Configure the dialog with text and options
        self.setWindowTitle("Deleting wire")
        QBtn = QDialogButtonBox.Yes | QDialogButtonBox.No

        self.text = QLabel(self)
        self.text.setText(dialogText)

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        # Create new layout for the dialog box and add text/button widgets
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)


class errorDialog(QDialog):
    def __init__(self, state, msg):
        # Add dialog in the context of the main window
        super(errorDialog, self).__init__(state.mainWindow.main)

        # Configure the dialog with text and options
        self.setWindowTitle("FATAL ERROR")
        QBtn = QDialogButtonBox.Ok

        self.text = QLabel(self)
        self.text.setText(msg)

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)

        # Create new layout for the dialog box and add text/button widgets
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)


class addChildDialog(QDialog):
    def __init__(self, dialogText):
        super(addChildDialog, self).__init__()

        # Configure the dialog with text and options
        self.setWindowTitle("Add new child")
        QBtn = QDialogButtonBox.Yes | QDialogButtonBox.No

        self.text = QLabel(self)
        self.text.setText(dialogText)

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        # Create new layout for the dialog box and add text/button widgets
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)
