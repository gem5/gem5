# Copyright (c) 2019 The Regents of the University of California
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
import textwrap


class TerminalFormatter:
    def __init__(self, max_width=80):
        # text_width holds the actual width we'll be wrapping to.
        # This takes into account the current terminal size.
        self.__text_width = min(max_width, self.__terminal_size()[0])

    def __terminal_size(self):
        import fcntl, termios, struct

        h, w, hp, wp = struct.unpack(
            "HHHH",
            fcntl.ioctl(
                0, termios.TIOCGWINSZ, struct.pack("HHHH", 0, 0, 0, 0)
            ),
        )
        return w, h

    def __get_paragraphs(self, text, flatten=False):

        """
        This function takes a text and returns a list of constituent
        paragraphs, defining a paragraph as a block of text separated from
        other text by a blank line (or one containing only whitespace). If the
        "flatten" argument is set to true, all line breaks within paragraphs
        will be removed.

        E.g.:

        text = '''Hello, this is
        paragraph number 1.

        This is paragraph number 2.

        And this is
        paragraph number 3.
        '''

        __get_paragraphs(text, False)
        ["Hello, this is\nparagraph number 1", "This is paragraph number 2.",
            "And this is\npagraph number 3."]

        __get_paragraphs(text, True)
        ["Hello, this is paragraph number 1", "This is paragraph number 2.",
            "And this is pagraph number 3."]
        """

        paragraphs = []
        cur_paragraph = []

        for line in text.splitlines():
            stripped = line.strip()
            if not stripped:  # I.e. a blank line.
                paragraphs.append(
                    {False: "\n", True: " "}[flatten].join(cur_paragraph)
                )
                cur_paragraph = []
            else:
                cur_paragraph.append(stripped)

        paragraphs.append(
            {False: "\n", True: " "}[flatten].join(cur_paragraph)
        )

        return paragraphs

    def format_output(self, text, label="", indent=0):
        """
        This function aids in the formatting of outputs. When obtaining
        the list of sim object we desire the output in the following
        format:

        desc: Address to mask loading binaries with, if 0,
              system auto-calculates the mask to be the most
              restrictive, otherwise it obeys a custom mask.

        We must take into account the width of the text, and wrap
        accordingly. We also must display the label.

        Keyword arguments:

        text --- The description text.
        label --- The label of the output (e.g. "desc: ").
        indent --- The white space width before each line.
        """

        if not text.strip():
            return ""

        # The text  may be over multiple lines (as when using triple
        # double quotes). First, we split the text into its constituent
        # paragraphs and remove new line characters from each.
        paragraphs = self.__get_paragraphs(text, True)

        # Wrap and Indent the paragraphs
        wrapper = textwrap.TextWrapper(
            width=max((self.__text_width - indent), 1)
        )
        # The first paragraph is special case due to the inclusion of the label
        formatted_paragraphs = [
            " " * max((indent - len(label)), 0)
            + label
            + wrapper.wrap(paragraphs[0])[0]
        ]
        for paragraph in paragraphs:
            for line in wrapper.wrap(paragraph[1:])[1:]:
                formatted_paragraphs.append(" " * indent + line)
            formatted_paragraphs.append("\n")

        # Remove the last line break
        return "\n".join(formatted_paragraphs[:-1])
