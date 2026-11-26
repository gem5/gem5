# Overview

This utility contains various scripts that are helpful when maintaining the gem5 resources MongoDB database.
Use Python 3.11 or newer and install the requirements in a fresh virtual
environment. PyMongo provides the `bson` module that these scripts import.
The scripts support MongoDB Server 4.4 or newer.

```
 pip3 install -r requirements.txt
```

## Adding JSON file to MongoDB

We can use the following command to add a JSON file to MongoDB collection:

```
mongoimport --db <dbName> --collection <collectionName> --file <fileName.json> --jsonArray
```

## backup MongoDB

We can mongodump to take a back of a mongoDB databse. We can use the following command:

```
mongodump --uri <uri>
```

## create-new-collection.py

This script grabs all documents from a specified collection in a MongoDB database and creates a new collection with the same documents.

To run this script you use the following command:

```
 python3 ./create-new-collection.py --uri <uri> --db_name <db_name> --collection_name <collection_name> --new_collection_name <new_collection_name>
```

## update-gem5-versions.py

This script grabs all resources categorically from a specified collection in a MongoDB
database and adds a new gem5 version to the gem5_versions field of each
resource.

To run this script you use the following command:

```
python3 ./update-gem5-versions.py --uri <uri> --db <db_name> --collection <collection_name> --version <version> --category <category> --outfile <outfile>
```

## helper.py

 This script  contains helper functions for the scripts in this directory.
