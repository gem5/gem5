<img alt="gem5art logo" src="/gem5art.svg" width=150>

# gem5art: Artifact, reproducibility, and testing utilities for gem5

![CI Badge](https://github.com/darchr/gem5art/workflows/CI/badge.svg)
[![Documentation Status](https://readthedocs.org/projects/gem5art/badge/?version=latest)](https://gem5art.readthedocs.io/en/latest/?badge=latest)

See <http://www.gem5.org/documentation/gem5art> for detailed documentation.

## Installing gem5art

To install gem5art, simply use pip.
We suggest creating a virtual environment first.
Note that gem5art requires Python 3, so be sure to use a Python 3 interpreter when creating the virtual environment

```sh
virtualenv -p python3
pip install gem5art-artifact gem5art-run gem5art-tasks
```

It's not required to install all of the gem5art utilities (e.g., you can skip gem5art-tasks if you don't want to use the celery job server).

## Running the tests

Below describes how to run the tests locally before uploading your changes.

### mypy: Python static analysis

[mypy](http://mypy-lang.org/) is a static type checker for Python.
By annotating the code with types and using a static type checker, we can have many of the benefits of a compiled language, with the benefits of Python!
Before contributing any code, please add type annotations and run the type checker.

The type checker must be run for each package separately.

```sh
cd artifact
mypy -p gem5art.artifact
```

```sh
cd run
mypy -p gem5art.run
```

```sh
cd tasks
mypy -p gem5art.tasks
```

You should see something like the following output:

```
Success: no issues found in 3 source files
```

If you see `0 source files`, then it's mostly likely that mypy has been run in the wrong directory.

If there are problems with imports, you may need to add `# type: ignore` after the `import` statement if there are third party packages without type annotations.

### Running the unit tests

We currently only have a small number of unit tests.
Although, we are working on adding more!

To run the unit tests, use the Python `unittest` module.

```sh
python -m unittest
```

You must run this in each package's subdirectory.

The output should be something like the following:
```
...
----------------------------------------------------------------------
Ran 3 tests in 0.141s

OK
```

If you instead see `Ran 0 tests`, then most likely you are in the wrong directory.

## Directory structure

The directory structure is a little strange so we can distribute each Python package separately.
However, they are all part of the gem5art namespace.
See the [Python namespace documentation](https://packaging.python.org/guides/packaging-namespace-packages/) for more details.

## Building for distribution

1. Run the setup.py. This must be done in each subdirectory to get the packages to build correctly.

```sh
python setup.py sdist
```

2. Upload to PyPI

```sh
twine upload dist/*
```

These two steps must be completed for each package (e.g., artifact, run, and tasks).
