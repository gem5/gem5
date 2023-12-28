# Copyright (c) 2023 The Regents of the University of California
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

import base64
import json
import secrets
from pathlib import Path

import jsonschema
import markdown
import requests
from api.json_client import JSONClient
from api.mongo_client import MongoDBClient
from bson import json_util
from cryptography.exceptions import InvalidSignature
from cryptography.fernet import (
    Fernet,
    InvalidToken,
)
from cryptography.hazmat.primitives.kdf.scrypt import Scrypt
from flask import (
    Flask,
    make_response,
    redirect,
    render_template,
    request,
    url_for,
)
from werkzeug.utils import secure_filename

databases = {}

response = requests.get(
    "https://resources.gem5.org/gem5-resources-schema.json"
)
schema = json.loads(response.content)


UPLOAD_FOLDER = Path("database/")
TEMP_UPLOAD_FOLDER = Path("database/.tmp/")
CONFIG_FILE = Path("instance/config.py")
SESSIONS_COOKIE_KEY = "sessions"
ALLOWED_EXTENSIONS = {"json"}
CLIENT_TYPES = ["mongodb", "json"]


app = Flask(__name__, instance_relative_config=True)


if not CONFIG_FILE.exists():
    CONFIG_FILE.parent.mkdir()
    with CONFIG_FILE.open("w+") as f:
        f.write(f"SECRET_KEY = {secrets.token_bytes(32)}")


app.config.from_pyfile(CONFIG_FILE.name)


# Sorts keys in any serialized dict
# Default = True
# Set False to persevere JSON key order
app.json.sort_keys = False


def startup_config_validation():
    """
    Validates the startup configuration.

    Raises:
        ValueError: If the 'SECRET_KEY' is not set or is not of type 'bytes'.
    """
    if not app.secret_key:
        raise ValueError("SECRET_KEY not set")
    if not isinstance(app.secret_key, bytes):
        raise ValueError("SECRET_KEY must be of type 'bytes'")


def startup_dir_file_validation():
    """
    Validates the startup directory and file configuration.

    Creates the required directories if they do not exist.
    """
    for dir in [UPLOAD_FOLDER, TEMP_UPLOAD_FOLDER]:
        if not dir.is_dir():
            dir.mkdir()


with app.app_context():
    startup_config_validation()
    startup_dir_file_validation()


@app.route("/")
def index():
    """
    Renders the index HTML template.

    :return: The rendered index HTML template.
    """
    return render_template("index.html")


@app.route("/login/mongodb")
def login_mongodb():
    """
    Renders the MongoDB login HTML template.

    :return: The rendered MongoDB login HTML template.
    """
    return render_template("login/login_mongodb.html")


@app.route("/login/json")
def login_json():
    """
    Renders the JSON login HTML template.

    :return: The rendered JSON login HTML template.
    """
    return render_template("login/login_json.html")


@app.route("/validateMongoDB", methods=["POST"])
def validate_mongodb():
    """
    Validates the MongoDB connection parameters and redirects to the editor route if successful.

    This route expects a POST request with a JSON payload containing an alias for the session and the listed parameters in order to validate the MongoDB instance.

    This route expects the following JSON payload parameters:
    - uri: The MongoDB connection URI.
    - collection: The name of the collection in the MongoDB database.
    - database: The name of the MongoDB database.
    - alias: The value by which the session will be keyed in `databases`.

    If the 'uri' parameter is empty, a JSON response with an error message and status code 400 (Bad Request) is returned.
    If the connection parameters are valid, the route redirects to the 'editor' route with the appropriate query parameters.

    :return: A redirect response to the 'editor' route or a JSON response with an error message and status code 400.
    """
    global databases
    try:
        databases[request.json["alias"]] = MongoDBClient(
            mongo_uri=request.json["uri"],
            database_name=request.json["database"],
            collection_name=request.json["collection"],
        )
    except Exception as e:
        return {"error": str(e)}, 400
    return redirect(
        url_for("editor", alias=request.json["alias"]),
        302,
    )


@app.route("/validateJSON", methods=["GET"])
def validate_json_get():
    """
    Validates the provided JSON URL and redirects to the editor route if successful.

    This route expects the following query parameters:
    - q: The URL of the JSON file.
    - filename: An optional filename for the uploaded JSON file.

    If the 'q' parameter is empty, a JSON response with an error message and status code 400 (Bad Request) is returned.
    If the JSON URL is valid, the function retrieves the JSON content, saves it to a file, and redirects to the 'editor'
    route with the appropriate query parameters.

    :return: A redirect response to the 'editor' route or a JSON response with an error message and status code 400.
    """
    filename = request.args.get("filename")
    url = request.args.get("q")
    if not url:
        return {"error": "empty"}, 400
    response = requests.get(url)
    if response.status_code != 200:
        return {"error": "invalid status"}, response.status_code
    filename = secure_filename(request.args.get("filename"))
    path = UPLOAD_FOLDER / filename
    if (UPLOAD_FOLDER / filename).is_file():
        temp_path = TEMP_UPLOAD_FOLDER / filename
        with temp_path.open("wb") as f:
            f.write(response.content)
        return {"conflict": "existing file in server"}, 409
    with path.open("wb") as f:
        f.write(response.content)
    global databases
    if filename in databases:
        return {"error": "alias already exists"}, 409
    try:
        databases[filename] = JSONClient(filename)
    except Exception as e:
        return {"error": str(e)}, 400
    return redirect(
        url_for("editor", alias=filename),
        302,
    )


@app.route("/validateJSON", methods=["POST"])
def validate_json_post():
    """
    Validates and processes the uploaded JSON file.

    This route expects a file with the key 'file' in the request files.
    If the file is not present, a JSON response with an error message
    and status code 400 (Bad Request) is returned.
    If the file already exists in the server, a JSON response with a
    conflict error message and status code 409 (Conflict) is returned.
    If the file's filename conflicts with an existing alias, a JSON
    response with an error message and status code 409 (Conflict) is returned.
    If there is an error while processing the JSON file, a JSON response
    with the error message and status code 400 (Bad Request) is returned.
    If the file is successfully processed, a redirect response to the
    'editor' route with the appropriate query parameters is returned.

    :return: A JSON response with an error message and
    status code 400 or 409, or a redirect response to the 'editor' route.
    """
    temp_path = None
    if "file" not in request.files:
        return {"error": "empty"}, 400
    file = request.files["file"]
    filename = secure_filename(file.filename)
    path = UPLOAD_FOLDER / filename
    if path.is_file():
        temp_path = TEMP_UPLOAD_FOLDER / filename
        file.save(temp_path)
        return {"conflict": "existing file in server"}, 409
    file.save(path)
    global databases
    if filename in databases:
        return {"error": "alias already exists"}, 409
    try:
        databases[filename] = JSONClient(filename)
    except Exception as e:
        return {"error": str(e)}, 400
    return redirect(
        url_for("editor", alias=filename),
        302,
    )


@app.route("/existingJSON", methods=["GET"])
def existing_json():
    """
    Handles the request for an existing JSON file.

    This route expects a query parameter 'filename'
    specifying the name of the JSON file.
    If the file is not present in the 'databases',
    it tries to create a 'JSONClient' instance for the file.
    If there is an error while creating the 'JSONClient'
    instance, a JSON response with the error message
    and status code 400 (Bad Request) is returned.
    If the file is present in the 'databases', a redirect
    response to the 'editor' route with the appropriate
    query parameters is returned.

    :return: A JSON response with an error message
    and status code 400, or a redirect response to the 'editor' route.
    """
    filename = request.args.get("filename")
    global databases
    if filename not in databases:
        try:
            databases[filename] = JSONClient(filename)
        except Exception as e:
            return {"error": str(e)}, 400
    return redirect(
        url_for("editor", alias=filename),
        302,
    )


@app.route("/existingFiles", methods=["GET"])
def get_existing_files():
    """
    Retrieves the list of existing files in the upload folder.

    This route returns a JSON response containing the names of the existing files in the upload folder configured in the
    Flask application.

    :return: A JSON response with the list of existing files.
    """
    files = [f.name for f in UPLOAD_FOLDER.iterdir() if f.is_file()]
    return json.dumps(files)


@app.route("/resolveConflict", methods=["GET"])
def resolve_conflict():
    """
    Resolves file conflict with JSON files.

    This route expects the following query parameters:
    - filename: The name of the file that is conflicting or an updated name for it to resolve the name conflict
    - resolution: A resolution option, defined as follows:
        - clearInput: Deletes the conflicting file and does not proceed with login
        - openExisting: Opens the existing file in `UPLOAD_FOLDER`
        - overwrite: Overwrites the existing file with the conflicting file
        - newFilename: Renames conflicting file, moving it to `UPLOAD_FOLDER`

    If the resolution parameter is not from the list given, an error is returned.

    The conflicting file in `TEMP_UPLOAD_FOLDER` is deleted.

    :return: A JSON response containing an error, or a success response, or a redirect to the editor.
    """
    filename = secure_filename(request.args.get("filename"))
    resolution = request.args.get("resolution")
    resolution_options = [
        "clearInput",
        "openExisting",
        "overwrite",
        "newFilename",
    ]
    temp_path = TEMP_UPLOAD_FOLDER / filename
    if not resolution:
        return {"error": "empty"}, 400
    if resolution not in resolution_options:
        return {"error": "invalid resolution"}, 400
    if resolution == resolution_options[0]:
        temp_path.unlink()
        return {"success": "input cleared"}, 204
    if resolution in resolution_options[-2:]:
        next(TEMP_UPLOAD_FOLDER.glob("*")).replace(UPLOAD_FOLDER / filename)
    if temp_path.is_file():
        temp_path.unlink()
    global databases
    if filename in databases:
        return {"error": "alias already exists"}, 409
    try:
        databases[filename] = JSONClient(filename)
    except Exception as e:
        return {"error": str(e)}, 400
    return redirect(
        url_for("editor", alias=filename),
        302,
    )


@app.route("/editor")
def editor():
    """
    Renders the editor page based on the specified database type.

    This route expects a GET request with specific query parameters:
    - "alias": An optional alias for the MongoDB database.

    The function checks if the query parameters are present. If not, it returns a 404 error.

    The function determines the database type based on the instance of the client object stored in the databases['alias']. If the type is not in the
    "CLIENT_TYPES" configuration, it returns a 404 error.

    :return: The rendered editor template based on the specified database type.
    """
    global databases
    if not request.args:
        return render_template("404.html"), 404
    alias = request.args.get("alias")
    if alias not in databases:
        return render_template("404.html"), 404

    client_type = ""
    if isinstance(databases[alias], JSONClient):
        client_type = CLIENT_TYPES[1]
    elif isinstance(databases[alias], MongoDBClient):
        client_type = CLIENT_TYPES[0]
    else:
        return render_template("404.html"), 404

    response = make_response(
        render_template("editor.html", client_type=client_type, alias=alias)
    )

    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"

    return response


@app.route("/help")
def help():
    """
    Renders the help page.

    This route reads the contents of the "help.md" file located in the "static" folder and renders it as HTML using the
    Markdown syntax. The rendered HTML is then passed to the "help.html" template for displaying the help page.

    :return: The rendered help page HTML.
    """
    with Path("static/help.md").open("r") as f:
        return render_template(
            "help.html", rendered_html=markdown.markdown(f.read())
        )


@app.route("/find", methods=["POST"])
def find():
    """
    Finds a resource based on the provided search criteria.

    This route expects a POST request with a JSON payload containing the alias of the session which is to be searched for the
    resource and the search criteria.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is returned.

    The Client API is used to find the resource by calling `find_resource()` on the session where the operation is
    accomplished by the concrete client class.

    The result of the `find_resource` operation is returned as a JSON response.

    :return: A JSON response containing the result of the `find_resource` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    return database.find_resource(request.json)


@app.route("/update", methods=["POST"])
def update():
    """
    Updates a resource with provided changes.

    This route expects a POST request with a JSON payload containing the alias of the session which contains the resource
    that is to be updated and the data for updating the resource.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is returned.

    The Client API is used to update the resource by calling `update_resource()` on the session where the operation is
    accomplished by the concrete client class.

    The `_add_to_stack` function of the session is called to insert the operation, update, and necessary data onto the revision
    operations stack.

    The result of the `update_resource` operation is returned as a JSON response. It contains the original and the modified resources.

    :return: A JSON response containing the result of the `update_resource` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    original_resource = request.json["original_resource"]
    modified_resource = request.json["resource"]
    status = database.update_resource(
        {
            "original_resource": original_resource,
            "resource": modified_resource,
        }
    )
    database._add_to_stack(
        {
            "operation": "update",
            "resource": {
                "original_resource": modified_resource,
                "resource": original_resource,
            },
        }
    )
    return status


@app.route("/versions", methods=["POST"])
def getVersions():
    """
    Retrieves the versions of a resource based on the provided search criteria.

    This route expects a POST request with a JSON payload containing the alias of the session which contains the resource
    whose versions are to be retrieved and the search criteria.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is returned.

    The Client API is used to get the versions of a resource by calling `get_versions()` on the session where the operation is
    accomplished by the concrete client class.

    The result of the `get_versions` operation is returned as a JSON response.

    :return: A JSON response containing the result of the `get_versions` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    return database.get_versions(request.json)


@app.route("/categories", methods=["GET"])
def getCategories():
    """
    Retrieves the categories of the resources.

    This route returns a JSON response containing the categories of the resources. The categories are obtained from the
    "enum" property of the "category" field in the schema.

    :return: A JSON response with the categories of the resources.
    """
    return json.dumps(schema["properties"]["category"]["enum"])


@app.route("/schema", methods=["GET"])
def getSchema():
    """
    Retrieves the schema definition of the resources.

    This route returns a JSON response containing the schema definition of the resources. The schema is obtained from the
    `schema` variable.

    :return: A JSON response with the schema definition of the resources.
    """
    return json_util.dumps(schema)


@app.route("/keys", methods=["POST"])
def getFields():
    """
    Retrieves the required fields for a specific category based on the provided data.

    This route expects a POST request with a JSON payload containing the data for retrieving the required fields.
    The function constructs an empty object `empty_object` with the "category" and "id" values from the request payload.

    The function then uses the JSONSchema validator to validate the `empty_object` against the `schema`. It iterates
    through the validation errors and handles two types of errors:

    1. "is a required property" error: If a required property is missing in the `empty_object`, the function retrieves
       the default value for that property from the schema and sets it in the `empty_object`.

    2. "is not valid under any of the given schemas" error: If a property is not valid under the current schema, the
       function evolves the validator to use the schema corresponding to the requested category. It then iterates
       through the validation errors again and handles any missing required properties as described in the previous
       step.

    Finally, the `empty_object` with the required fields populated (including default values if applicable) is returned
    as a JSON response.

    :return: A JSON response containing the `empty_object` with the required fields for the specified category.
    """
    empty_object = {
        "category": request.json["category"],
        "id": request.json["id"],
    }
    validator = jsonschema.Draft7Validator(schema)
    errors = list(validator.iter_errors(empty_object))
    for error in errors:
        if "is a required property" in error.message:
            required = error.message.split("'")[1]
            empty_object[required] = error.schema["properties"][required][
                "default"
            ]
        if "is not valid under any of the given schemas" in error.message:
            validator = validator.evolve(
                schema=error.schema["definitions"][request.json["category"]]
            )
            for e in validator.iter_errors(empty_object):
                if "is a required property" in e.message:
                    required = e.message.split("'")[1]
                    if "default" in e.schema["properties"][required]:
                        empty_object[required] = e.schema["properties"][
                            required
                        ]["default"]
                    else:
                        empty_object[required] = ""
    return json.dumps(empty_object)


@app.route("/delete", methods=["POST"])
def delete():
    """
    Deletes a resource.

    This route expects a POST request with a JSON payload containing the alias of the session from which a resource is to be
    deleted and the data for deleting the resource.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is returned.

    The Client API is used to delete the resource by calling `delete_resource()` on the session where the operation is
    accomplished by the concrete client class.

    The `_add_to_stack` function of the session is called to insert the operation, delete, and necessary data onto the revision
    operations stack.

    The result of the `delete` operation is returned as a JSON response.

    :return: A JSON response containing the result of the `delete` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    resource = request.json["resource"]
    status = database.delete_resource(resource)
    database._add_to_stack({"operation": "delete", "resource": resource})
    return status


@app.route("/insert", methods=["POST"])
def insert():
    """
    Inserts a new resource.

    This route expects a POST request with a JSON payload containing the alias of the session to which the data
    is to be inserted and the data for inserting the resource.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is returned.

    The Client API is used to insert the new resource by calling `insert_resource()` on the session where the operation is
    accomplished by the concrete client class.

    The `_add_to_stack` function of the session is called to insert the operation, insert, and necessary data onto the revision
    operations stack.

    The result of the `insert` operation is returned as a JSON response.

    :return: A JSON response containing the result of the `insert` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    resource = request.json["resource"]
    status = database.insert_resource(resource)
    database._add_to_stack({"operation": "insert", "resource": resource})
    return status


@app.route("/undo", methods=["POST"])
def undo():
    """
    Undoes last operation performed on the session.

    This route expects a POST request with a JSON payload containing the alias of the session whose last operation
    is to be undone.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is returned.

    The Client API is used to undo the last operation performed on the session by calling `undo_operation()` on the
    session where the operation is accomplished by the concrete client class.

    The result of the `undo_operation` operation is returned as a JSON response.

    :return: A JSON response containing the result of the `undo_operation` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    return database.undo_operation()


@app.route("/redo", methods=["POST"])
def redo():
    """
    Redoes last operation performed on the session.

    This route expects a POST request with a JSON payload containing the alias of the session whose last operation
    is to be redone.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is returned.

    The Client API is used to redo the last operation performed on the session by calling `redo_operation()` on the
    session where the operation is accomplished by the concrete client class.

    The result of the `redo_operation` operation is returned as a JSON response.

    :return: A JSON response containing the result of the `redo_operation` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    return database.redo_operation()


@app.route("/getRevisionStatus", methods=["POST"])
def get_revision_status():
    """
    Gets the status of revision operations.

    This route expects a POST request with a JSON payload containing the alias of the session whose revision operations
    statuses is being requested.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is
    returned.

    The Client API is used to get the status of the revision operations by calling `get_revision_status()` on the
    session where the operation is accomplished by the concrete client class.

    The result of the `get_revision_status` is returned as a JSON response.

    :return: A JSON response contain the result of the `get_revision_status` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    return database.get_revision_status()


def fernet_instance_generation(password):
    """
    Generates Fernet instance for use in Saving and Loading Session.

    Utilizes Scrypt Key Derivation Function with `SECRET_KEY` as salt value and recommended
    values for `length`, `n`, `r`, and `p` parameters. Derives key using `password`. Derived
    key is then used to initialize Fernet instance.

    :param password: User provided password
    :return: Fernet instance
    """
    return Fernet(
        base64.urlsafe_b64encode(
            Scrypt(salt=app.secret_key, length=32, n=2**16, r=8, p=1).derive(
                password.encode()
            )
        )
    )


@app.route("/saveSession", methods=["POST"])
def save_session():
    """
    Generates ciphertext of session that is to be saved.

    This route expects a POST request with a JSON payload containing the alias of the session that is to be
    saved and a password to be used in encrypting the session data.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is
    returned.

    The `save_session()` method is called to get the necessary session data from the corresponding `Client`
    as a dictionary.

    A Fernet instance, using the user provided password, is instantiated. The session data is encrypted using this
    instance. If an Exception is raised, an error response is returned.

    The result of the save_session operation is returned as a JSON response. The ciphertext is returned or an error
    message if an error occurred.

    :return: A JSON response containing the result of the save_session operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    session = databases[alias].save_session()
    try:
        fernet_instance = fernet_instance_generation(request.json["password"])
        ciphertext = fernet_instance.encrypt(json.dumps(session).encode())
    except (TypeError, ValueError):
        return {"error": "Failed to Encrypt Session!"}, 400
    return {"ciphertext": ciphertext.decode()}, 200


@app.route("/loadSession", methods=["POST"])
def load_session():
    """
    Loads session from data specified in user request.

    This route expects a POST request with a JSON payload containing the encrypted ciphertext containing the session
    data, the alias of the session that is to be restored, and the password associated with it.

    A Fernet instance, using the user provided password, is instantiated. The session data is decrypted using this
    instance. If an Exception is raised, an error response is returned.

    The `Client` type is retrieved from the session data and a redirect to the appropriate login with the stored
    parameters from the session data is applied.

    The result of the load_session operation is returned either as a JSON response containing the error message
    or a redirect.

    :return: A JSON response containing the error of the load_session operation or a redirect.
    """
    alias = request.json["alias"]
    session = request.json["session"]
    try:
        fernet_instance = fernet_instance_generation(request.json["password"])
        session_data = json.loads(fernet_instance.decrypt(session))
    except (InvalidSignature, InvalidToken):
        return {"error": "Incorrect Password! Please Try Again!"}, 400
    client_type = session_data["client"]
    if client_type == CLIENT_TYPES[0]:
        try:
            databases[alias] = MongoDBClient(
                mongo_uri=session_data["uri"],
                database_name=session_data["database"],
                collection_name=session_data["collection"],
            )
        except Exception as e:
            return {"error": str(e)}, 400

        return redirect(
            url_for("editor", type=CLIENT_TYPES[0], alias=alias),
            302,
        )
    elif client_type == CLIENT_TYPES[1]:
        return redirect(
            url_for("existing_json", filename=session_data["filename"]),
            302,
        )
    else:
        return {"error": "Invalid Client Type!"}, 409


@app.errorhandler(404)
def handle404(error):
    """
    Error handler for 404 (Not Found) errors.

    This function is called when a 404 error occurs. It renders the "404.html" template and returns it as a response with
    a status code of 404.

    :param error: The error object representing the 404 error.
    :return: A response containing the rendered "404.html" template with a status code of 404.
    """
    return render_template("404.html"), 404


@app.route("/checkExists", methods=["POST"])
def checkExists():
    """
    Checks if a resource exists based on the provided data.

    This route expects a POST request with a JSON payload containing the alias of the session in which it is to be
    determined whether a given resource exists and the necessary data for checking the existence of the resource.

    The alias is used in retrieving the session from `databases`. If the session is not found, an error is
    returned.

    The Client API is used to check the existence of the resource by calling `check_resource_exists()` on the
    session where the operation is accomplished by the concrete client class.

    The result of the `check_resource_exists` is returned as a JSON response.

    :return: A JSON response contain the result of the `check_resource_exists` operation.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    database = databases[alias]
    return database.check_resource_exists(request.json)


@app.route("/logout", methods=["POST"])
def logout():
    """
    Logs the user out of the application.

    Deletes the alias from the `databases` dictionary.

    :param alias: The alias of the database to logout from.

    :return: A redirect to the index page.
    """
    alias = request.json["alias"]
    if alias not in databases:
        return {"error": "database not found"}, 400
    databases.pop(alias)
    return (redirect(url_for("index")), 302)


if __name__ == "__main__":
    app.run(debug=True)
