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

from .celery import gem5app
import multiprocessing as mp
import time


@gem5app.task(bind=True, serializer="pickle")
def run_gem5_instance(self, gem5_run, cwd="."):
    """
    Runs a gem5 instance with the script and any parameters to the script.
    Note: this is "bound" which means self is the task that is running this.
    """

    gem5_run.run(self, cwd=cwd)


def run_single_job(run):
    start_time = time.time()
    print(f"Running {' '.join(run.command)} at {time.time()}")
    run.run()
    finish_time = time.time()
    print(
        f"Finished {' '.join(run.command)} at {time.time()}. "
        f"Total time = {finish_time - start_time}"
    )


def run_job_pool(job_list, num_parallel_jobs=mp.cpu_count() // 2):
    """
    Runs gem5 jobs in parallel when Celery is not used.
    Creates as many parallel jobs as core count if no explicit
    job count is provided
    Receives a list of run objects created by the launch script
    """

    pool = mp.Pool(num_parallel_jobs)
    pool.map(run_single_job, job_list)
    pool.close()
    pool.join()
    print(f"All jobs done running!")
