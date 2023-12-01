
# Offline Database Utility

This Python script facilitates downloading all the resources from a database, storing them locally for offline usage. It operates by accessing an API, retrieving resources, and updating references within a JSON file to point to these local copies.

### Running the Script

#### Arguments
- `--config-file-path`: Filepath to the gem5 config file.
- `--output_dir`: Output directory path (default: current working directory).


```bash
python3 ./get_resources_from_db.py [--config-file-path CONFIG_FILE_PATH] [--output_dir OUTPUT_DIR]
```

## Functionality

### What does it do?
- Retrieves resources from specified MongoDB Atlas database using an API.
- Downloads these resources to a local directory.
- Generates a `resources.json` file, updates the url field of resources to reference the local copies.
