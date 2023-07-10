const diffEditorContainer = document.getElementById("diff-editor");
var diffEditor;
var originalModel;
var modifiedModel;

const schemaEditorContainer = document.getElementById("schema-editor");
var schemaEditor;
var schemaModel;

const schemaButton = document.getElementById("schema-toggle");
const editingActionsButtons = Array.from(
  document.querySelectorAll("#editing-actions button")
);
var editingActionsState;

const tooltipTriggerList = document.querySelectorAll('[data-bs-toggle="tooltip"]');
tooltipTriggerList.forEach(tooltip => {
  tooltip.setAttribute("data-bs-trigger", "hover");
});
const tooltipList = [...tooltipTriggerList].map(tooltipTriggerEl => new bootstrap.Tooltip(tooltipTriggerEl));

require.config({
  paths: {
    vs: "https://cdnjs.cloudflare.com/ajax/libs/monaco-editor/0.26.1/min/vs",
  },
});
require(["vs/editor/editor.main"], () => {
  originalModel = monaco.editor.createModel(`{\n}`, "json");
  modifiedModel = monaco.editor.createModel(`{\n}`, "json");
  diffEditor = monaco.editor.createDiffEditor(diffEditorContainer, {
    theme: "vs-dark",
    language: "json",
    automaticLayout: true,
  });
  diffEditor.setModel({
    original: originalModel,
    modified: modifiedModel,
  });
  fetch("/schema")
    .then((res) => res.json())
    .then((data) => {
      monaco.languages.json.jsonDefaults.setDiagnosticsOptions({
        trailingCommas: "error",
        comments: "error",
        validate: true,
        schemas: [
          {
            uri: "http://json-schema.org/draft-07/schema",
            fileMatch: ["*"],
            schema: data,
          },
        ],
      });

      schemaEditor = monaco.editor.create(schemaEditorContainer, {
        theme: "vs-dark",
        language: "json",
        automaticLayout: true,
        readOnly: true,
      });

      schemaModel = monaco.editor.createModel(`{\n}`, "json");
      schemaEditor.setModel(schemaModel);
      schemaModel.setValue(JSON.stringify(data, null, 4));

      schemaEditorContainer.style.display = "none";
    });
});

let clientType = document.getElementById('client-type');
clientType.textContent = clientType.textContent === "mongodb" ? "MongoDB" : clientType.textContent.toUpperCase();

const revisionButtons = [document.getElementById("undo-operation"), document.getElementById("redo-operation")];
revisionButtons.forEach(btn => {
  btn.disabled = true;
});

const editorGroupIds = [];
document.querySelectorAll(".editorButtonGroup button, .revisionButtonGroup button")
  .forEach(btn => {
    editorGroupIds.push(btn.id);
  });

function checkErrors() {
  let errors = monaco.editor.getModelMarkers({ resource: modifiedModel.uri });
  if (errors.length > 0) {
    console.log(errors);
    let str = "";
    errors.forEach((error) => {
      str += error.message + "\n";
    });
    appendAlert('Error!', 'schemaError', { str }, 'danger');
    return true;
  }
  return false;
}
let didChange = false;

function update(e) {
  e.preventDefault();
  if (checkErrors()) {
    return;
  }
  let json = JSON.parse(modifiedModel.getValue());
  let original_json = JSON.parse(originalModel.getValue());

  console.log(json);
  fetch("/update", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      resource: json,
      original_resource: original_json,
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => res.json())
    .then(async (data) => {
      console.log(data);
      await addVersions();
      //Select last option
      document.getElementById("version-dropdown").value =
        json["resource_version"];
      console.log(document.getElementById("version-dropdown").value);
      find(e);
    });
}

function addNewResource(e) {
  e.preventDefault();
  if (checkErrors()) {
    return;
  }
  let json = JSON.parse(modifiedModel.getValue());
  console.log(json);
  fetch("/insert", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      resource: json,
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => res.json())
    .then(async (data) => {
      console.log(data);
      await addVersions();
      //Select last option
      document.getElementById("version-dropdown").value =
        json["resource_version"];
      console.log(document.getElementById("version-dropdown").value);
      find(e);
    });
}

function addVersion(e) {
  e.preventDefault();
  console.log("add version");
  if (checkErrors()) {
    return;
  }
  let json = JSON.parse(modifiedModel.getValue());
  console.log(json["resource_version"]);
  fetch("/checkExists", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      id: json["id"],
      resource_version: json["resource_version"],
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => res.json())
    .then((data) => {
      console.log(data["exists"]);
      if (data["exists"] == true) {
        appendAlert("Error!", "existingResourceVersion", "Resource version already exists!", "danger");
        return;
      } else {
        fetch("/insert", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            resource: json,
            alias: document.getElementById("alias").innerText,
          }),
        })
          .then((res) => res.json())
          .then(async (data) => {
            console.log("added version");
            console.log(data);
            await addVersions();
            //Select last option
            document.getElementById("version-dropdown").value =
              json["resource_version"];
            console.log(document.getElementById("version-dropdown").value);
            find(e);
          });
      }
    });
}

function deleteRes(e) {
  e.preventDefault();
  console.log("delete");
  let id = document.getElementById("id").value;
  let resource_version = JSON.parse(originalModel.getValue())[
    "resource_version"
  ];
  let json = JSON.parse(originalModel.getValue());
  console.log(resource_version);
  fetch("/delete", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      resource: json,
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => res.json())
    .then(async (data) => {
      console.log(data);
      await addVersions();
      //Select first option
      document.getElementById("version-dropdown").value =
        document.getElementById("version-dropdown").options[0].value;
      console.log(document.getElementById("version-dropdown").value);
      find(e);
    });
}

document.getElementById("id").onchange = function () {
  console.log("id changed");
  didChange = true;
};

async function addVersions() {
  let select = document.getElementById("version-dropdown");
  select.innerHTML = "Latest";
  await fetch("/versions", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      id: document.getElementById("id").value,
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => res.json())
    .then((data) => {
      let select = document.getElementById("version-dropdown");
      if (data.length == 0) {
        data = [{ resource_version: "Latest" }];
      }
      data.forEach((version) => {
        let option = document.createElement("option");
        option.value = version["resource_version"];
        option.innerText = version["resource_version"];
        select.appendChild(option);
      });
    });
}

function find(e) {
  e.preventDefault();
  if (didChange) {
    addVersions();
    didChange = false;
  }

  closeSchema();

  toggleInteractables(true, editorGroupIds, () => {
    diffEditor.updateOptions({ readOnly: true });
    updateRevisionBtnsDisabledAttr();
  });

  fetch("/find", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      id: document.getElementById("id").value,
      resource_version: document.getElementById("version-dropdown").value,
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => res.json())
    .then((data) => {
      console.log(data);
      toggleInteractables(false, editorGroupIds, () => {
        diffEditor.updateOptions({ readOnly: false });
        updateRevisionBtnsDisabledAttr();
      });

      if (data["exists"] == false) {
        fetch("/keys", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            category: document.getElementById("category").value,
            id: document.getElementById("id").value,
          }),
        })
          .then((res) => res.json())
          .then((data) => {
            console.log(data)
            data["id"] = document.getElementById("id").value;
            data["category"] = document.getElementById("category").value;
            originalModel.setValue(JSON.stringify(data, null, 4));
            modifiedModel.setValue(JSON.stringify(data, null, 4));

            document.getElementById("add_new_resource").disabled = false;
            document.getElementById("add_version").disabled = true;
            document.getElementById("delete").disabled = true;
            document.getElementById("update").disabled = true;
          });
      } else {
        console.log(data);
        originalModel.setValue(JSON.stringify(data, null, 4));
        modifiedModel.setValue(JSON.stringify(data, null, 4));

        document.getElementById("version-dropdown").value =
          data.resource_version;
        document.getElementById("category").value = data.category;

        document.getElementById("add_new_resource").disabled = true;
        document.getElementById("add_version").disabled = false;
        document.getElementById("delete").disabled = false;
        document.getElementById("update").disabled = false;
      }
    });
}

window.onload = () => {
  let ver_dropdown = document.getElementById("version-dropdown");
  let option = document.createElement("option");
  option.value = "Latest";
  option.innerHTML = "Latest";
  ver_dropdown.appendChild(option);
  fetch("/categories")
    .then((res) => res.json())
    .then((data) => {
      console.log(data);
      let select = document.getElementById("category");
      data.forEach((category) => {
        let option = document.createElement("option");
        option.value = category;
        option.innerHTML = category;
        select.appendChild(option);
      });
      fetch("/keys", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          category: document.getElementById("category").value,
          id: "",
        }),
      })
        .then((res) => res.json())
        .then((data) => {
          data["id"] = "";
          data["category"] = document.getElementById("category").value;
          originalModel.setValue(JSON.stringify(data, null, 4));
          modifiedModel.setValue(JSON.stringify(data, null, 4));
          document.getElementById("add_new_resource").disabled = false;
        });
    });

  checkExistingSavedSession();
};

const myModal = new bootstrap.Modal("#ConfirmModal", {
  keyboard: false,
});

let confirmButton = document.getElementById("confirm");

function showModal(event, callback) {
  event.preventDefault();
  myModal.show();
  confirmButton.onclick = () => {
    callback(event);
    myModal.hide();
  };
}

let editorTitle = document.getElementById("editor-title");

function showSchema() {
  if (diffEditorContainer.style.display !== "none") {
    diffEditorContainer.style.display = "none";
    schemaEditorContainer.classList.add("editor-sizing");
    schemaEditor.setPosition({ column: 1, lineNumber: 1 });
    schemaEditor.revealPosition({ column: 1, lineNumber: 1 });
    schemaEditorContainer.style.display = "block";

    editingActionsState = editingActionsButtons.map(
      (button) => button.disabled
    );

    editingActionsButtons.forEach((btn) => {
      btn.disabled = true;
    });

    editorTitle.children[0].style.display = "none";
    editorTitle.children[1].textContent = "Schema (Read Only)";

    schemaButton.textContent = "Close Schema";
    schemaButton.onclick = closeSchema;
  }
}

function closeSchema() {
  if (schemaEditorContainer.style.display !== "none") {
    schemaEditorContainer.style.display = "none";
    diffEditorContainer.style.display = "block";

    editingActionsButtons.forEach((btn, i) => {
      btn.disabled = editingActionsState[i];
    });

    editorTitle.children[0].style.display = "unset";
    editorTitle.children[1].textContent = "Edited";

    schemaButton.textContent = "Show Schema";
    schemaButton.onclick = showSchema;
  }
}

const saveSessionBtn = document.getElementById("saveSession");
saveSessionBtn.disabled = true;

let password = document.getElementById("session-password");
password.addEventListener("input", () => {
  saveSessionBtn.disabled = password.value === "";
});

function showSaveSessionModal() {
  const saveSessionModal = new bootstrap.Modal(document.getElementById('saveSessionModal'), {
    focus: true, keyboard: false
  });
  saveSessionModal.show();
}

function saveSession() {
  alias = document.getElementById("alias").innerText;

  bootstrap.Modal.getInstance(document.getElementById("saveSessionModal")).hide();

  let preserveDisabled = [];
  document.querySelectorAll(".editorButtonGroup button, .revisionButtonGroup button")
    .forEach(btn => {
      btn.disabled === true ? preserveDisabled.push(btn.id) : null;
    });

  toggleInteractables(true);

  fetch("/saveSession", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      alias: alias,
      password: document.getElementById("session-password").value
    }),
  })
    .then((res) => {
      document.getElementById("saveSessionForm").reset();

      toggleInteractables(false, preserveDisabled);

      res.json()
        .then((data) => {
          if (res.status === 400) {
            appendAlert('Error!', 'saveSessionError', `${data["error"]}`, 'danger');
            return;
          }

          let sessions = JSON.parse(localStorage.getItem("sessions")) || {};
          sessions[alias] = data["ciphertext"];
          localStorage.setItem("sessions", JSON.stringify(sessions));

          document.getElementById("showSaveSessionModal").innerText = "Session Saved";
          checkExistingSavedSession();
        })
    })
}

function executeRevision(event, operation) {
  if (!["undo", "redo"].includes(operation)) {
    appendAlert("Error!", "invalidRevOp", "Fatal! Invalid Revision Operation!", "danger");
    return;
  }

  toggleInteractables(true, editorGroupIds, () => {
    diffEditor.updateOptions({ readOnly: true });
    updateRevisionBtnsDisabledAttr();
  });
  fetch(`/${operation}`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then(() => {
      toggleInteractables(false, editorGroupIds, () => {
        diffEditor.updateOptions({ readOnly: false });
        updateRevisionBtnsDisabledAttr();
      });
      find(event);
    })
}

function updateRevisionBtnsDisabledAttr() {
  fetch("/getRevisionStatus", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => res.json())
    .then((data) => {
      revisionButtons[0].disabled = data.undo;
      revisionButtons[1].disabled = data.redo;
    })
}

function logout() {
  toggleInteractables(true);

  fetch("/logout", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      alias: document.getElementById("alias").innerText,
    }),
  })
    .then((res) => {
      toggleInteractables(false);

      if (res.status !== 302) {
        res.json()
          .then((data) => {
            appendAlert('Error!', 'logoutError', `${data["error"]}`, 'danger');
            return;
          })
      }

      window.location = res.url;
    })
}

function checkExistingSavedSession() {
  document.getElementById("existing-session-warning").style.display =
    document.getElementById("alias").innerText in JSON.parse(localStorage.getItem("sessions") || "{}")
      ? "flex"
      : "none";
}

document.getElementById("close-save-session-modal").addEventListener("click", () => {
  document.getElementById("saveSessionModal").querySelector("form").reset();
  saveSessionBtn.disabled = password.value === "";
});
