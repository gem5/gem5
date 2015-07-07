#!/bin/sed -f
#
# Copyright (c) 2014-2015 ARM Limited
# All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Andreas Sandberg

# READ BEFORE EDITING:
#
# * Avoid adding or removing newlines (e.g., deleting matched lines). It's
#   much easier to understand the Doxygen logs if they point to the right
#   line in the source files.
#
# * SED can be hard to read, so please document what your replacement rules
#   are supposed to do and why.
#

# Handle TODO/FIXME/BUG comments
/\/\/ \(TODO\|FIXME\|BUG\):/ {
    # Transform the first line of the comment block into a Doxygen C++ comment.
    s/\([^\]\)\/\/ /\1\/\/\/ /;

    : todo_comment_cont
    # Replace any TODO/FIXME/BUG commands with Doxygen equivalents
    s/\(TODO\|FIXME\):/@todo /;
    s/\(BUG\):/@bug /;
    # Get the next line
    n;
    # If this line is only contains whitespace and a comment, it is a
    # conntinuation of the previous line. If so, make it a Doxygen comment.
    s/\([:space:]*\)\/\/\([^\/]\)/\1\/\/\/\2/ ;
    # Try to match another line if the previous s command matched a line.
    t todo_comment_cont;
}
