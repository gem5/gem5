# gem5art run package

This package contains Python objects to wrap gem5 runs/experiments.

Please cite the [gem5art paper](https://arch.cs.ucdavis.edu/papers/2021-3-28-gem5art) when using the gem5art packages.
This documentation can be found on the [gem5 website](https://www.gem5.org/documentation/gem5art/)

Each gem5 experiment is wrapped inside a run object.
These run objects contain all of the information required to execute the gem5 experiments and can optionally be executed via the gem5art tasks library (or manually with the `run()` function.).gem5Run interacts with the Artifact class of gem5art to ensure reproducibility of gem5 experiments and also stores the current gem5Run object and the output results in the database for later analysis.

## SE and FS mode runs

Next are two methods (for SE (system-emulation) and FS (full-system) modes of gem5) from gem5Run class which give an idea of the required arguments from a user's perspective to create a gem5Run object:

```python

@classmethod
def createSERun(cls,
                name: str,
                gem5_binary: str,
                run_script: str,
                outdir: str,
                gem5_artifact: Artifact,
                gem5_git_artifact: Artifact,
                run_script_git_artifact: Artifact,
                *params: str,
                timeout: int = 60*15) -> 'gem5Run':
.......


@classmethod
def createFSRun(cls,
                name: str,
                gem5_binary: str,
                run_script: str,
                outdir: str,
                gem5_artifact: Artifact,
                gem5_git_artifact: Artifact,
                run_script_git_artifact: Artifact,
                linux_binary: str,
                disk_image: str,
                linux_binary_artifact: Artifact,
                disk_image_artifact: Artifact,
                *params: str,
                timeout: int = 60*15) -> 'gem5Run':
.......

```

For the user it is important to understand different arguments passed to run objects:

- `name`: name of the run, can act as a tag to search the database to find the required runs (it is expected that user will use a unique name for different experiments)
- `gem5_binary`: path to the actual gem5 binary to be used
- `run_script`: path to the python run script that will be used with gem5 binary
- `outdir`: path to the directory where gem5 results should be written
- `gem5_artifact`: gem5 binary git artifact object
- `gem5_git_artifact`: gem5 source git repo artifact object
- `run_script_git_artifact`: run script artifact object
- `linux_binary` (only full-system): path to the actual linux binary to be used (used by run script as well)
- `disk_image` (only full-system): path to the actual disk image to be used (used by run script as well)
- `linux_binary_artifact` (only full-system): linux binary artifact object
- `disk_image_artifact` (only full-system): disk image artifact object
- `params`: other params to be passed to the run script
- `timeout`: longest time in seconds for which the current gem5 job is allowed to execute

The artifact parameters (`gem5_artifact`, `gem5_git_artifact`, and `run_script_git_artifact`) are used to ensure this is reproducible run.
Apart from the above mentioned parameters, gem5Run class also keeps track of other features of a gem5 run e.g., the start time, the end time, the current status of gem5 run, the kill reason (if the run is finished), etc.

While the user can write their own run script to use with gem5 (with any command line arguments), currently when a `gem5Run` object is created for a full-system experiment using `createFSRun` method, it is assumed that the path to the `linux_binary` and `disk_image` is passed to the run script on the command line (as arguments of the `createFSRun` method).

## Running an experiment

The `gem5Run` object has everything needed to run one gem5 execution.
Normally, this will be performed by using the gem5art *tasks* package.
However, it is also possible to manually execute a gem5 run.

The `run` function executes the gem5 experiment.
It takes two optional parameters: a task associated with the run for bookkeeping and an optional directory to execute the run in.

The `run` function executes the gem5 binary by using `Popen`.
This creates another process to execute gem5.
The `run` function is *blocking* and does not return until the child process has completed.

While the child process is running, every 5 seconds the parent python process will update the status in the `info.json` file.

The `info.json` file is the serialized `gem5run` object which contains all of the run information and the current status.

`gem5Run` objects have 7 possible status states.
These are currently simple strings stored in the `status` property.

- `Created`: The run has been created. This is set in the constructor when either `createSRRun` or `createFSRun` is called.
- `Begin run`: When `run()` is called, after the database is checked, we enter the `Begin run` state.
- `Failed artifact check for ...`: The status is set to this when the artifact check fails
- `Spawning`: Next, just before `Popen` is called, the run enters the `Spawning` state
- `Running`: Once the parent process begins spinning waiting for the child to finish, the run enters the `Running` state.
- `Finished`: When the child finished with exit code `0`, the run enters the `Finished` state.
- `Failed`: When the child finished with a non-zero exit code, the run enters the `Failed` state.

## Run Already in the Database

When starting a run with gem5art, it might complain that the run already exists in the database.
Basically, before launching a gem5 job, gem5art checks if this run matches an existing run in the database.
In order to uniquely identify a run, a single hash is made out of:

- the runscript
- the parameters passed to the runscript
- the artifacts of the run object which, for an SE run, include: gem5 binary artifact, gem5 source git artifact, run script (experiments repo) artifact. For an FS run, the list of artifacts also include linux binary artifact and disk image artifacts in addition to the artifacts of an SE run.

If this hash already exists in the database, gem5art will not launch a new job based on this run object as a run with same parameters would have already been executed.
In case, user still wants to launch this job, the user will have to remove the existing run object from the database.

## Searching the Database to find Runs

### Utility script

gem5art provides the utility `gem5art-getruns` to search the database and retrieve runs.
Based on the parameters, `gem5art-getruns` will dump the results into a file in the json format.

```
usage: gem5art-getruns [-h] [--fs-only] [--limit LIMIT] [--db-uri DB_URI]
                       [-s SEARCH_NAME]
                       filename

Dump all runs from the database into a json file

positional arguments:
  filename              Output file name

optional arguments:
  -h, --help            show this help message and exit
  --fs-only             Only output FS runs
  --limit LIMIT         Limit of the number of runs to return. Default: all
  --db-uri DB_URI       The database to connect to. Default
                        mongodb://localhost:27017
  -s SEARCH_NAME, --search_name SEARCH_NAME
                        Query for the name field
```

### Manually searching the database

Once you start running the experiments with gem5 and want to know the status of those runs, you can look at the gem5Run artifacts in the database.
For this purpose, gem5art provides a method `getRuns`, which you can use as follows:

```python
import gem5art.run
from gem5art.artifact import getDBConnection
db = getDBConnection()
for i in gem5art.run.getRuns(db, fs_only=False, limit=100):
    print(i)
```

The documentation on [getRuns](run.html#gem5art.run.getRuns) is available at the bottom of this page.

## Searching the Database to find Runs with Specific Names

As discussed above, while creating a FS or SE mode Run object, the user has to pass a name field to recognize
a particular set of runs (or experiments).
We expect that the user will take care to use a name string which fully characterizes a set of experiments and can be thought of as a `Nonce`.
For example, if we are running experiments to test linux kernel boot on gem5, we can use a name field `boot_tests_v1` or `boot_tests_[month_year]` (where mont_year correspond to the month and year when the experiments were run).

Later on, the same name can be used to search for relevant gem5 runs in the database.
For this purpose, gem5art provides a method `getRunsByName`, which can be used as follow:

```python
import gem5art.run
from gem5art.artifact import getDBConnection
db = getDBConnection()
for i in gem5art.run.getRunsByName(db, name='boot_tests_v1', fs_only=True, limit=100):
    print(i)
```

The documentation on `getRunsByName` is available [here](run.html#gem5art.run.getRunsByName).

## Runs API Documentation

```eval_rst
Run
---

.. automodule:: gem5art.run
    :members:
    :undoc-members:
```
