
# Offline Database Utility

This Python script makes local copies of all the gem5 Resource's external files and data.
This is useful for cases where an internet connection is not permitted.
With this script users can cache a local copy which can be used by gem5.

Two interconnected parts of the gem5 infrastructure are cached using this script.
They are:

1. **The gem5 Resources database**: When the gem5 Standard Library attempts to construct a gem5 Resource it will reference that resource's entry in the gem5 MongoDB database.
It then uses this data to construct the resource.
With this script we download this database as a JSON file for gem5 to use in-place of the database.

2. **gem5 Resource Remote Files**: Resources can (and often do) require files to be downloaded for them to function correctly in gem5.
Once the gem5 Resource is constructed from the database entry the gem5 Resource may download the remote files it requires.
As a simple example, a `DiskImageResource` will contain disk image partitioning information necessary for a gem5 simulation to be setup correctly.
The disk image file is a remote file which is downloaded as this information cannot be stored directly in the database.

The location of these file is stored within the resource's database entry as a URL.

To create a local cache of these remote files we download all the remote files and, when creating the JSON file from the database, update the URLs to remote files to File URIs to their local equivalents.

Once this script is finished running the user will have a directory of all the remote gem5 Resources files and a JSON file representation of the database, with the remote files' URLs updated to local path URIs.

## Running the Script

### Arguments
- `--config-file-path`: Filepath to the gem5 config file.
- `--output-dir`: Output directory path (default: current working directory).


```bash
python3 ./get-resources-from-db.py [--config-file-path CONFIG_FILE_PATH] [--output_dir OUTPUT_DIR]
```

## Functionality

### What is the config file?

The resources config file represents all the sources.

The gem5_default_config adds the gem5 resources MongoDB Atlas database as a datasource.

Documentation on setting up your own data sources can be found here: https://www.gem5.org/documentation/gem5-stdlib/using-local-resources
