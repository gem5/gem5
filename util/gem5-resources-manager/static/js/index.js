window.onload = () => {
  let select = document.getElementById("sessions-dropdown");
  const sessions = JSON.parse(localStorage.getItem("sessions"));

  if (sessions === null) {
    document.getElementById("showSavedSessionModal").disabled = true;
    return;
  }

  Object.keys(sessions).forEach((alias) => {
    let option = document.createElement("option");
    option.value = alias;
    option.innerHTML = alias;
    select.appendChild(option);
  });
}

const loadSessionBtn = document.getElementById("loadSession");
loadSessionBtn.disabled = true;

let password = document.getElementById("session-password");
password.addEventListener("input", () => {
  loadSessionBtn.disabled = password.value === "";
});

document.getElementById("close-load-session-modal").addEventListener("click", () => {
  document.getElementById("savedSessionModal").querySelector("form").reset();
})

function showSavedSessionModal() {
  const savedSessionModal = new bootstrap.Modal(document.getElementById('savedSessionModal'), { focus: true, keyboard: false });
  savedSessionModal.show();
}

function loadSession() {
  bootstrap.Modal.getInstance(document.getElementById("savedSessionModal")).hide();

  const alias = document.getElementById("sessions-dropdown").value;
  const session = JSON.parse(localStorage.getItem("sessions"))[alias];

  if (session === null) {
    appendAlert("Error!", "sessionNotFound", "Saved Session Not Found!", "danger");
    return;
  }

  toggleInteractables(true);

  fetch("/loadSession", {
    method: "POST",
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      password: document.getElementById("session-password").value,
      alias: alias,
      session: session
    })
  })
    .then((res) => {
      toggleInteractables(false);

      if (res.status !== 200) {
        res.json()
          .then((error) => {
            document.getElementById("savedSessionModal").querySelector("form").reset();
            appendAlert("Error!", "invalidStatus", `${error["error"]}`, "danger");
            return;
          })
      }

      if (res.redirected) {
        window.location = res.url;
      }
    })
}
