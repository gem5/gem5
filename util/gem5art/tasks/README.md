# gem5art tasks package

This package contains two parallel task libraries for running gem5 experiments.
he actual gem5 experiment can be executed with the help of [Python multiprocessing support](https://docs.python.org/3/library/multiprocessing.html), [Celery](http://www.celeryproject.org/) or even without using any job manager (a job can be directly launched by calling `run()` function of gem5Run object).
This package implicitly depends on the gem5art run package.

Please cite the [gem5art paper](https://arch.cs.ucdavis.edu/papers/2021-3-28-gem5art) when using the gem5art packages.
This documentation can be found on the [gem5 website](https://www.gem5.org/documentation/gem5art/)

## Use of Python Multiprocessing

This is a simple way to run gem5 jobs using Python multiprocessing library.
You can use the following function in your job launch script to execute gem5art run objects:

```python
run_job_pool([a list containing all run objects you want to execute], num_parallel_jobs = [Number of parallel jobs you want to run])
```

## Use of Celery

Celery server can run many gem5 tasks asynchronously.
Once a user creates a gem5Run object (discussed previously) while using gem5art, this object needs to be passed to a method `run_gem5_instance()` registered with Celery app, which is responsible for starting a Celery task to run gem5. The other argument needed by the `run_gem5_instance()` is the current working directory.

Celery server can be started with the following command:

```sh
celery -E -A gem5art.tasks.celery worker --autoscale=[number of workers],0
```

This will start a server with events enabled that will accept gem5 tasks as defined in gem5art.
It will autoscale from 0 to desired number of workers.

Celery relies on a message broker `RabbitMQ` for communication between the client and workers.
If not already installed, you need to install `RabbitMQ` on your system (before running celery) using:

```sh
apt-get install rabbitmq-server
```

### Monitoring Celery

Celery does not explicitly show the status of the runs by default.
[flower](https://flower.readthedocs.io/en/latest/), a Python package, is a web-based tool for monitoring and administrating Celery.

To install the flower package,
```sh
pip install flower
```

You can monitor the celery cluster doing the following:

```sh
flower -A gem5art.tasks.celery --port=5555
```
This will start a webserver on port 5555.

### Removing all tasks

```sh
celery -A gem5art.tasks.celery purge
```

### Viewing state of all jobs in celery

```sh
celery -A gem5art.tasks.celery events
```

## Tasks API Documentation

```eval_rst
Task
----

.. automodule:: gem5art.tasks.tasks
    :members:
    :undoc-members:

.. automodule:: gem5art.tasks.celery
    :members:
    :undoc-members:
```
