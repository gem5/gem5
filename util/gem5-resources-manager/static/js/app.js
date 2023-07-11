const loadingContainer = document.getElementById("loading-container");
const alertPlaceholder = document.getElementById('liveAlertPlaceholder');
const interactiveElems = document.querySelectorAll('button, input, select');

const appendAlert = (errorHeader, id, message, type) => {
  const alertDiv = document.createElement('div');
  alertDiv.classList.add("alert", `alert-${type}`, "alert-dismissible", "fade", "show", "d-flex", "flex-column", "shadow-sm");
  alertDiv.setAttribute("role", "alert");
  alertDiv.setAttribute("id", id);
  alertDiv.style.maxWidth = "320px";

  alertDiv.innerHTML = [
    `  <div class="d-flex align-items-center main-text-semi">`,
    `    <svg xmlns="http://www.w3.org/2000/svg" fill="currentColor" height="1.5rem" class="bi bi-exclamation-octagon-fill me-3" viewBox="0 0 16 16">`,
    `      <path d="M11.46.146A.5.5 0 0 0 11.107 0H4.893a.5.5 0 0 0-.353.146L.146 4.54A.5.5 0 0 0 0 4.893v6.214a.5.5 0 0 0 .146.353l4.394 4.394a.5.5 0 0 0
              .353.146h6.214a.5.5 0 0 0 .353-.146l4.394-4.394a.5.5 0 0 0 .146-.353V4.893a.5.5 0 0 0-.146-.353L11.46.146zM8 4c.535 0 .954.462.9.995l-.35
              3.507a.552.552 0 0 1-1.1 0L7.1 4.995A.905.905 0 0 1 8 4zm.002 6a1 1 0 1 1 0 2 1 1 0 0 1 0-2z"/>`,
    `    </svg>`,
    `    <span class="main-text-regular">${errorHeader}</span>`,
    `      <button type="button" class="btn-close" data-bs-dismiss="alert" aria-label="Close"></button>`,
    `    </div>`,
    `  <hr />`,
    `  <div>${message}</div>`,
  ].join('');

  window.scrollTo(0, 0);

  alertPlaceholder.append(alertDiv);

  setTimeout(function () {
    bootstrap.Alert.getOrCreateInstance(document.getElementById(`${id}`)).close();
  }, 5000);
}

function toggleInteractables(isBlocking, excludedOnNotBlockingIds = [], otherBlockingUpdates = () => {}) {
  if (isBlocking) {
    loadingContainer.classList.add("d-flex");
    interactiveElems.forEach(elems => {
      elems.disabled = true;
    });
    window.scrollTo(0, 0);
    otherBlockingUpdates();
    return;
  }

  setTimeout(() => {
    loadingContainer.classList.remove("d-flex");
    interactiveElems.forEach(elems => {
      !excludedOnNotBlockingIds.includes(elems.id) ? elems.disabled = false : null;
    });
    otherBlockingUpdates();
  }, 250);
}

function showResetSavedSessionsModal() {
  let sessions = localStorage.getItem("sessions");
  if (sessions === null) {
    appendAlert('Error!', 'noSavedSessions', `No Saved Sessions Exist!`, 'danger');
    return;
  }
  sessions = JSON.parse(sessions);

  const resetSavedSessionsModal = new bootstrap.Modal(document.getElementById('resetSavedSessionsModal'), {
    focus: true, keyboard: false
  });


  let select = document.getElementById("delete-session-dropdown");
  select.innerHTML = "";
  Object.keys(sessions).forEach((alias) => {
    let option = document.createElement("option");
    option.value = alias;
    option.innerHTML = alias;
    select.appendChild(option);
  });

  document.getElementById("selected-session").innerText = `"${document.getElementById("delete-session-dropdown").value}"`;

  resetSavedSessionsModal.show();
}

function resetSavedSessions() {
  bootstrap.Modal.getInstance(document.getElementById("resetSavedSessionsModal")).hide();

  const sessions = JSON.parse(localStorage.getItem("sessions"));
  if (sessions === null) {
    appendAlert('Error!', 'noSavedSessions', `No Saved Sessions Exist!`, 'danger');
    return;
  }

  const activeTab = document.getElementById("reset-tabs").querySelector(".nav-link.active").getAttribute("id");
  if (activeTab === "delete-one-tab") {
    const deleteOneConfirmation = document.getElementById("delete-one-confirmation").value;
    if (deleteOneConfirmation !== document.getElementById("delete-session-dropdown").value) {
      document.getElementById("resetSavedSessionsModal").querySelectorAll("form").forEach(form => {
        form.reset();
      })
      appendAlert('Error!', 'noSavedSessions', `Invalid Confirmation Entry!`, 'danger');
      return;
    }

    delete sessions[document.getElementById("delete-session-dropdown").value];
    Object.keys(sessions).length === 0
      ? localStorage.removeItem("sessions")
      : localStorage.setItem("sessions", JSON.stringify(sessions));

    } else {
    const deleteAllConfirmation = document.getElementById("delete-all-confirmation").value;
    if (deleteAllConfirmation !== "Delete All") {
      document.getElementById("resetSavedSessionsModal").querySelectorAll("form").forEach(form => {
        form.reset();
      })
      appendAlert('Error!', 'noSavedSessions', `Invalid Confirmation Entry!`, 'danger');
      return;
    }

    localStorage.removeItem("sessions");
  }

  appendAlert('Success!', 'resetCookies', `Saved Session Reset Successful!`, 'success');
  setTimeout(() => {
    location.reload();
  }, 750);
}

document.getElementById("close-reset-modal").addEventListener("click", () => {
  document.getElementById("resetSavedSessionsModal").querySelectorAll("form").forEach(form => {
    form.reset();
  })
});

document.getElementById("delete-session-dropdown").addEventListener("change", () => {
  document.getElementById("selected-session").innerText =
    `"${document.getElementById("delete-session-dropdown").value}"`;
});
