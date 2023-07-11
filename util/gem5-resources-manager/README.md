# gem5 Resources Manager

This directory contains the code to convert the JSON file to a MongoDB database. This also contains tools to manage the database as well as the JSON file.

# Table of Contents
- [gem5 Resources Manager](#gem5-resources-manager)
- [Table of Contents](#table-of-contents)
- [Resources Manager](#resources-manager)
  - [Setup](#setup)
  - [Selecting a Database](#selecting-a-database)
    - [MongoDB](#mongodb)
    - [JSON File](#json-file)
  - [Adding a Resource](#adding-a-resource)
  - [Updating a Resource](#updating-a-resource)
  - [Deleting a Resource](#deleting-a-resource)
  - [Adding a New Version](#adding-a-new-version)
  - [Validation](#validation)
- [CLI tool](#cli-tool)
  - [create\_resources\_json](#create_resources_json)
  - [restore\_backup](#restore_backup)
  - [backup\_mongodb](#backup_mongodb)
  - [get\_resource](#get_resource)
- [Changes to Structure of JSON](#changes-to-structure-of-json)
- [Testing](#testing)

# Resources Manager

This is a tool to manage the resources JSON file and the MongoDB database. This tool is used to add, delete, update, view, and search for resources.

## Setup

First, install the requirements:

```bash
pip3 install -r requirements.txt
```

Then run the flask server:

```bash
python3 server.py
```

Then, you can access the server at `http://localhost:5000`.

## Selecting a Database

The Resource Manager currently supports 2 database options: MongoDB and JSON file.

Select the database you want to use by clicking on the button on home page.

### MongoDB

The MongoDB database is hosted on MongoDB Atlas. To use this database, you need to have the MongoDB URI, collection name, and database name.  Once you have the information, enter it into the form and click "login" or "save and login" to login to the database.

Another way to use the MongoDB database is to switch to the Generate URI tab and enter the information there. This would generate a URI that you can use to login to the database.

### JSON File

There are currently 3 ways to use the JSON file:

1. Adding a URL to the JSON file
2. Uploading a JSON file
3. Using an existing JSON file

## Adding a Resource

Once you are logged in, you can use the search bar to search for resources. If the ID doesn't exist, it would be prefilled with the required fields. You can then edit the fields and click "add" to add the resource to the database.

## Updating a Resource

If the ID exists, the form would be prefilled with the existing data. You can then edit the fields and click "update" to update the resource in the database.

## Deleting a Resource

If the ID exists, the form would be prefilled with the existing data. You can then click "delete" to delete the resource from the database.

## Adding a New Version

If the ID exists, the form would be prefilled with the existing data. Change the `resource_version` field to the new version and click "add" to add the new version to the database. You will only be able to add a new version if the `resource_version` field is different from any of the existing versions.

## Validation

The Resource Manager validates the data before adding it to the database. If the data is invalid, it would show an error message and not add the data to the database. The validation is done using the [schema](schema/schema.json) file. The Monaco editor automatically validates the data as you type and displays the errors in the editor.

To view the schema, click on the "Show Schema" button on the left side of the page.

# CLI tool

```bash
usage: gem5_resource_cli.py [-h] [-u URI] [-d DATABASE] [-c COLLECTION] {get_resource,backup_mongodb,restore_backup,create_resources_json} ...

CLI for gem5-resources.

positional arguments:
  {get_resource,backup_mongodb,restore_backup,create_resources_json}
                        The command to run.
    get_resource        Retrieves a resource from the collection based on the given ID. if a resource version is provided, it will retrieve the resource
                        with the given ID and version.
    backup_mongodb      Backs up the MongoDB collection to a JSON file.
    restore_backup      Restores a backup of the MongoDB collection from a JSON file.
    create_resources_json
                        Creates a JSON file of all the resources in the collection.

optional arguments:
  -h, --help            show this help message and exit
  -u URI, --uri URI     The URI of the MongoDB database. (default: None)
  -d DATABASE, --database DATABASE
                        The MongoDB database to use. (default: gem5-vision)
  -c COLLECTION, --collection COLLECTION
                        The MongoDB collection to use. (default: versions_test)
```

By default, the cli uses environment variables to get the URI. You can create a .env file with the `MONGO_URI` variable set to your URI. If you want to use a different URI, you can use the `-u` flag to specify the URI.

## create_resources_json

This command is used to create a new JSON file from the old JSON file. This is used to make the JSON file "parseable" by removing the nested JSON and adding the new fields.

```bash
usage: gem5_resource_cli.py create_resources_json [-h] [-v VERSION] [-o OUTPUT] [-s SOURCE]

optional arguments:
  -h, --help            show this help message and exit
  -v VERSION, --version VERSION
                        The version of the resources to create the JSON file for. (default: dev)
  -o OUTPUT, --output OUTPUT
                        The JSON file to create. (default: resources.json)
  -s SOURCE, --source SOURCE
                        The path to the gem5 source code. (default: )
```

A sample command to run this is:

```bash
python3 gem5_resource_cli.py create_resources_json -o resources_new.json -s ./gem5
```

## restore_backup

This command is used to update the MongoDB database with the new JSON file. This is used to update the database with the new JSON file.

```bash
usage: gem5_resource_cli.py restore_backup [-h] [-f FILE]

optional arguments:
  -h, --help            show this help message and exit

required arguments:
  -f FILE, --file FILE  The JSON file to restore the MongoDB collection from.
```

A sample command to run this is:

```bash
python3 gem5_resource_cli.py restore_backup -f resources.json
```

## backup_mongodb

This command is used to backup the MongoDB database to a JSON file. This is used to create a backup of the database.

```bash
usage: gem5_resource_cli.py backup_mongodb [-h] -f FILE

optional arguments:
  -h, --help            show this help message and exit

required arguments:
  -f FILE, --file FILE  The JSON file to back up the MongoDB collection to.
```

A sample command to run this is:

```bash
python3 gem5_resource_cli.py backup_mongodb -f resources.json
```

## get_resource

This command is used to get a resource from the MongoDB database. This is used to get a resource from the database.

```bash
usage: gem5_resource_cli.py get_resource [-h] -i ID [-v VERSION]

optional arguments:
  -h, --help            show this help message and exit
  -v VERSION, --version VERSION
                        The version of the resource to retrieve.

required arguments:
  -i ID, --id ID        The ID of the resource to retrieve.
```

A sample command to run this is:

```bash
python3 gem5_resource_cli.py get_resource -i x86-ubuntu-18.04-img -v 1.0.0
```
# Changes to Structure of JSON

To view the new schema, see [schema.json](https://resources.gem5.org/gem5-resources-schema.json).

# Testing

To run the tests, run the following command:

```bash
coverage run -m unittest discover -s test -p '*_test.py'
```

To view the coverage report, run the following command:

```bash
coverage report
```
