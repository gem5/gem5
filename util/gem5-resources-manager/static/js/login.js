function handleMongoDBLogin(event) {
  event.preventDefault();
  const activeTab = document.getElementById("mongodb-login-tabs").querySelector(".nav-link.active").getAttribute("id");

  activeTab === "enter-uri-tab" ? handleEnteredURI() : handleGenerateURI();

  return;
}

function handleEnteredURI() {
  const uri = document.getElementById('uri').value;
  const collection = document.getElementById('collection').value;
  const database = document.getElementById('database').value;
  const alias = document.getElementById('alias').value;
  const emptyInputs = [{ type: "Alias", value: alias }, { type: "Collection", value: collection }, { type: "Database", value: database }, { type: "URI", value: uri }];
  let error = false;

  for (let i = 0; i < emptyInputs.length; i++) {
    if (emptyInputs[i].value === "") {
      appendAlert("Error", `${emptyInputs[i].type}`, `Cannot Proceed Without ${emptyInputs[i].type} Value!`, 'danger');
      error = true;
    }
  }

  if (error) {
    return;
  }

  handleMongoURLFetch(uri, collection, database, alias);
}

function handleGenerateURI() {
  const connection = document.getElementById('connection').checked;
  const username = document.getElementById('username').value;
  const password = document.getElementById('password').value;
  const collection = document.getElementById('collectionGenerate').value;
  const database = document.getElementById('databaseGenerate').value;
  const host = document.getElementById('host').value;
  const alias = document.getElementById('aliasGenerate').value;
  const options = document.getElementById('options').value.split(",");
  let generatedURI = "";
  const emptyInputs = [{ type: "Alias", value: alias }, { type: "Host", value: host }, { type: "Collection", value: collection }, { type: "Database", value: database }];
  let error = false;

  for (let i = 0; i < emptyInputs.length; i++) {
    if (emptyInputs[i].value === "") {
      appendAlert("Error", `${emptyInputs[i].type}`, `Cannot Proceed Without ${emptyInputs[i].type} Value!`, 'danger');
      error = true;
    }
  }

  if (error) {
    return;
  }

  generatedURI = connection ? "mongodb+srv://" : "mongodb://";
  if (username && password) {
    generatedURI += `${encodeURIComponent(username)}:${encodeURIComponent(password)}@`;
  }

  generatedURI += host;

  if (options.length) {
    generatedURI += `/?${options.join("&")}`;
  }

  handleMongoURLFetch(generatedURI, collection, database, alias);
}

function handleMongoURLFetch(uri, collection, database, alias) {
  toggleInteractables(true);

  fetch("/validateMongoDB",
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        uri: uri,
        collection: collection,
        database: database,
        alias: alias
      })
    })
    .then((res) => {
      toggleInteractables(false);

      if (!res.ok) {
        res.json()
          .then(error => {
            appendAlert('Error!', 'mongodbValidationError', `${error.error}`, 'danger');
          });
        return;
      }

      res.redirected ? window.location = res.url : appendAlert('Error!', 'invalidRes', 'Invalid Server Response!', 'danger');
    })
}

function handleJSONLogin(event) {
  event.preventDefault();
  const activeTab = document.getElementById("json-login-tabs").querySelector(".nav-link.active").getAttribute("id");
  if (activeTab === "remote-tab") {
    handleRemoteJSON();
  } else if (activeTab === "existing-tab") {
    const filename = document.getElementById("existing-dropdown").value;
    if (filename !== "No Existing Files") {
      toggleInteractables(true);

      fetch(`/existingJSON?filename=${filename}`,
        {
          method: 'GET',
          headers: {
            'Content-Type': 'application/json'
          }
        })
        .then((res) => {
          toggleInteractables(false);

          if (res.status !== 200) {
            appendAlert('Error!', 'invalidURL', 'Invalid JSON File URL!', 'danger');
          }
          if (res.redirected) {
            window.location = res.url;
          }
        })
    }
  } else {
    handleUploadJSON();
  }
  return;
}

function handleRemoteJSON() {
  const url = document.getElementById("jsonRemoteURL").value;
  const filename = document.getElementById("remoteFilename").value;
  const emptyInputs = [{ type: "URL", value: url }, { type: "Filename", value: filename }];
  let error = false;

  for (let i = 0; i < emptyInputs.length; i++) {
    if (emptyInputs[i].value === "") {
      appendAlert("Error", `${emptyInputs[i].type}`, `Cannot Proceed Without ${emptyInputs[i].type} Value!`, 'danger');
      error = true;
    }
  }

  if (error) {
    return;
  }

  const params = new URLSearchParams();
  params.append('filename', filename + ".json");
  params.append('q', url);

  const flask_url = `/validateJSON?${params.toString()}`;

  toggleInteractables(true);

  fetch(flask_url, {
    method: 'GET',
  })
    .then((res) => {
      toggleInteractables(false);

      if (res.status === 400) {
        appendAlert('Error!', 'invalidURL', 'Invalid JSON File URL!', 'danger');
      }

      if (res.status === 409) {
        const myModal = new bootstrap.Modal(document.getElementById('conflictResolutionModal'), { focus: true, keyboard: false });
        document.getElementById("header-filename").textContent = `"${filename}"`;
        myModal.show();
      }

      if (res.redirected) {
        window.location = res.url;
      }
    })
}

var filename;

function handleUploadJSON() {
  const jsonFile = document.getElementById("jsonFile");
  const file = jsonFile.files[0];

  if (jsonFile.value === "") {
    appendAlert('Error!', 'emptyUpload', 'Cannot Proceed Without Uploading a File!', 'danger');
    return;
  }

  filename = file.name;

  const form = new FormData();
  form.append("file", file);

  toggleInteractables(true);

  fetch("/validateJSON", {
    method: 'POST',
    body: form
  })
    .then((res) => {
      toggleInteractables(false);

      if (res.status === 400) {
        appendAlert('Error!', 'invalidUpload', 'Invalid JSON File Upload!', 'danger');
      }

      if (res.status === 409) {
        const myModal = new bootstrap.Modal(document.getElementById('conflictResolutionModal'), { focus: true, keyboard: false });
        document.getElementById("header-filename").textContent = `"${filename}"`;
        myModal.show();
      }

      if (res.redirected) {
        window.location = res.url;
      }
    })
}

function saveConflictResolution() {
  const conflictResolutionModal = bootstrap.Modal.getInstance(document.getElementById("conflictResolutionModal"));
  const selectedValue = document.querySelector('input[name="conflictRadio"]:checked').id;
  const activeTab = document.getElementById("json-login-tabs").querySelector(".nav-link.active").getAttribute("id");

  if (selectedValue === null) {
    appendAlert('Error!', 'nullRadio', 'Fatal! Null Radio!', 'danger');
    return;
  }

  if (selectedValue === "clearInput") {
    if (activeTab === "upload-tab") {
      document.getElementById("jsonFile").value = '';
    }

    if (activeTab === "remote-tab") {
      document.getElementById('remoteFilename').value = '';
      document.getElementById('jsonRemoteURL').value = '';
    }

    conflictResolutionModal.hide();
    handleConflictResolution("clearInput", filename.split(".")[0]);
    return;
  }

  if (selectedValue === "openExisting") {
    conflictResolutionModal.hide();
    handleConflictResolution("openExisting", filename.split(".")[0]);
    return;
  }

  if (selectedValue === "overwrite") {
    conflictResolutionModal.hide();
    handleConflictResolution("overwrite", filename.split(".")[0]);
    return;
  }

  if (selectedValue === "newFilename") {
    const updatedFilename = document.getElementById("updatedFilename").value;
    if (updatedFilename === "") {
      appendAlert('Error!', 'emptyFilename', 'Must Enter A New Name!', 'danger');
      return;
    }

    if (`${updatedFilename}.json` === filename) {
      appendAlert('Error!', 'sameFilenames', 'Cannot Have Same Name as Current!', 'danger');
      return;
    }

    conflictResolutionModal.hide();
    handleConflictResolution("newFilename", updatedFilename);
    return;
  }
}

function handleConflictResolution(resolution, filename) {
  const params = new URLSearchParams();
  params.append('resolution', resolution);
  params.append('filename', filename !== "" ? filename + ".json" : "");

  const flask_url = `/resolveConflict?${params.toString()}`;
  toggleInteractables(true);

  fetch(flask_url, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json'
    }
  })
    .then((res) => {
      toggleInteractables(false);

      if (res.status === 204) {
        console.log("Input Cleared, Cached File Deleted, Resources Unset");
        return;
      }

      if (res.status !== 200) {
        appendAlert('Error!', 'didNotRedirect', 'Server Did Not Redirect!', 'danger');
        return;
      }

      if (res.redirected) {
        window.location = res.url;
      }
    })
}

window.onload = () => {
  if (window.location.pathname === "/login/json") {
    fetch('/existingFiles', {
      method: 'GET',
    })
      .then((res) => res.json())
      .then((data) => {
        let select = document.getElementById("existing-dropdown");
        if (data.length === 0) {
          data = ["No Existing Files"];
        }
        data.forEach((files) => {
          let option = document.createElement("option");
          option.value = files;
          option.innerHTML = files;
          select.appendChild(option);
        });
      });
  }
}
