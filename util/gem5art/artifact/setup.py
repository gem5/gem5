# Copyright (c) 2019, 2021 The Regents of the University of California
# All Rights Reserved.
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
"""A setuptools based setup module."""
from os.path import join
from pathlib import Path

from setuptools import find_namespace_packages
from setuptools import setup


with open(Path(__file__).parent / "README.md", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="gem5art-artifact",
    version="1.4.0",
    description="Artifacts for gem5art",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://www.gem5.org/",
    author="Davis Architecture Research Group (DArchR)",
    author_email="jlowepower@ucdavis.edu",
    license="BSD",
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: BSD License",
        "Topic :: System :: Hardware",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python :: 3",
    ],
    keywords="simulation architecture gem5",
    packages=find_namespace_packages(include=["gem5art.*"]),
    install_requires=["pymongo"],
    python_requires=">=3.6",
    project_urls={
        "Bug Reports": "https://github.com/gem5/issues/",
        "Source": "https://github.com/gem5/gem5/",
        "Documentation": "https://www.gem5.org/documentation/gem5art",
    },
)
