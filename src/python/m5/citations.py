# Copyright (c) 2023 The Regents of The University of California
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

from pathlib import Path
from typing import Type

import m5.options


def add_citation(sim_obj_cls: Type["SimObject"], citation: str):
    """Add a citation to a SimObject class.

    :param sim_obj_cls: The SimObject class to add the citation to.
    :param citation: The citation to add. Should be bibtex compatible
                     entry or entries

    This function will encode the citation into the SimObject class and it
    will be included in the citations in the output directory when the
    SimObject is used. If you have multiple citations, then you should include
    one multiline string with all of the citations.
    """

    sim_obj_cls._citations += citation


def gather_citations(root: "SimObject"):
    """Based on the root SimObject, walk the object hierarchy and gather all
    of the citations together and then print them to citations.bib in the
    output directory.
    """

    citations = {}
    for obj in root.descendants():
        loc = 0
        while loc >= 0:
            key, cite, loc = _get_next_key_entry(obj._citations, loc)
            # If a key repeats, then just overwrite it
            citations[key] = cite

    with open(Path(m5.options.outdir) / "citations.bib", "w") as output:
        output.writelines(citations.values())


def _get_next_key_entry(citations: str, loc: int = 0):
    """Return the key, the citation, and the end of the citation location"""

    start = citations.find("@", loc)
    key_start = citations.find("{", start)
    key_end = citations.find(",", key_start)
    end = citations.find("@", start + 1)
    if end == -1:
        end = len(citations)
        next = -1
    else:
        next = end

    return citations[key_start:key_end], citations[start:end], next


gem5_citations = """@article{Binkert:2011:gem5,
  author       = {Nathan L. Binkert and
                  Bradford M. Beckmann and
                  Gabriel Black and
                  Steven K. Reinhardt and
                  Ali G. Saidi and
                  Arkaprava Basu and
                  Joel Hestness and
                  Derek Hower and
                  Tushar Krishna and
                  Somayeh Sardashti and
                  Rathijit Sen and
                  Korey Sewell and
                  Muhammad Shoaib Bin Altaf and
                  Nilay Vaish and
                  Mark D. Hill and
                  David A. Wood},
  title        = {The gem5 simulator},
  journal      = {{SIGARCH} Comput. Archit. News},
  volume       = {39},
  number       = {2},
  pages        = {1--7},
  year         = {2011},
  url          = {https://doi.org/10.1145/2024716.2024718},
  doi          = {10.1145/2024716.2024718}
}
@article{Lowe-Power:2020:gem5-20,
  author       = {Jason Lowe{-}Power and
                  Abdul Mutaal Ahmad and
                  Ayaz Akram and
                  Mohammad Alian and
                  Rico Amslinger and
                  Matteo Andreozzi and
                  Adri{\\`{a}} Armejach and
                  Nils Asmussen and
                  Srikant Bharadwaj and
                  Gabe Black and
                  Gedare Bloom and
                  Bobby R. Bruce and
                  Daniel Rodrigues Carvalho and
                  Jer{\'{o}}nimo Castrill{\'{o}}n and
                  Lizhong Chen and
                  Nicolas Derumigny and
                  Stephan Diestelhorst and
                  Wendy Elsasser and
                  Marjan Fariborz and
                  Amin Farmahini Farahani and
                  Pouya Fotouhi and
                  Ryan Gambord and
                  Jayneel Gandhi and
                  Dibakar Gope and
                  Thomas Grass and
                  Bagus Hanindhito and
                  Andreas Hansson and
                  Swapnil Haria and
                  Austin Harris and
                  Timothy Hayes and
                  Adrian Herrera and
                  Matthew Horsnell and
                  Syed Ali Raza Jafri and
                  Radhika Jagtap and
                  Hanhwi Jang and
                  Reiley Jeyapaul and
                  Timothy M. Jones and
                  Matthias Jung and
                  Subash Kannoth and
                  Hamidreza Khaleghzadeh and
                  Yuetsu Kodama and
                  Tushar Krishna and
                  Tommaso Marinelli and
                  Christian Menard and
                  Andrea Mondelli and
                  Tiago M{\"{u}}ck and
                  Omar Naji and
                  Krishnendra Nathella and
                  Hoa Nguyen and
                  Nikos Nikoleris and
                  Lena E. Olson and
                  Marc S. Orr and
                  Binh Pham and
                  Pablo Prieto and
                  Trivikram Reddy and
                  Alec Roelke and
                  Mahyar Samani and
                  Andreas Sandberg and
                  Javier Setoain and
                  Boris Shingarov and
                  Matthew D. Sinclair and
                  Tuan Ta and
                  Rahul Thakur and
                  Giacomo Travaglini and
                  Michael Upton and
                  Nilay Vaish and
                  Ilias Vougioukas and
                  Zhengrong Wang and
                  Norbert Wehn and
                  Christian Weis and
                  David A. Wood and
                  Hongil Yoon and
                  {\'{E}}der F. Zulian},
  title        = {The gem5 Simulator: Version 20.0+},
  journal      = {CoRR},
  volume       = {abs/2007.03152},
  year         = {2020},
  url          = {https://arxiv.org/abs/2007.03152},
  eprinttype    = {arXiv},
  eprint       = {2007.03152}
}
"""
