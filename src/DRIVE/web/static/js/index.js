const socket = io();
let stage = 0;

const button = document.getElementById("mainButton");

function closeStopModal(response) {
  document.getElementById("stop-modal").style.display = "none";

  if (!response) {
    return
  }

  socket.emit("stop_drive", { reason: response });

  button.textContent = "Start Drive";
  stage = 0;
}

function handleButtonClick() {
  if (stage === 0) {
    socket.emit("start_geofencing");
    button.textContent = "End Geofencing";
    stage++;
  } else if (stage === 1) {
    socket.emit("start_drive");
    button.textContent = "Stop";
    stage++;
  } else if (stage === 2) {
    document.getElementById("stop-modal").style.display = "flex";
  }
}

socket.on("state_transition", (state) => {
  console.log("state transition to: " + state);

  switch (state) {
    case "ready_state":
      break;
    default:
      break;
  }
});

function saveDataset(event) {
  event.preventDefault();
  const datasetName = document.getElementById("dataset-name-input").value;
  socket.emit("save_dataset", { name: datasetName });
  console.log("Dataset saved with name:", datasetName);
}

function updateDatasets() {
  socket.emit("update_datasets");
}

function loadGeofence(event) {
  event.preventDefault();
  const datasetName = document.getElementById("dataset-load-select").value;
  socket.emit("load_geofence", { name: datasetName });
  console.log("Geofence loaded from name:", datasetName);
}

socket.on("datasets", (data) => {
  let select = document.getElementById("dataset-load-select");
  let v = select.value;
  s = "";
  data.forEach(d => {
    s += "<option value=\"" + d + "\">" + d + "</option>";
  });
  select.innerHTML = s;
  select.value = v;
});

socket.on("confirm_geofence", (state) => {
  console.log("confirm geofence called but how??");
});


//
// Graphs
//

const inputSpaceImg = document.getElementById("input_space");
const robotVizImg = document.getElementById("robot_viz");

function updateImageInversion() {
  const isLight = window.matchMedia('(prefers-color-scheme: light)').matches;
  [inputSpaceImg, robotVizImg].forEach(img => {
    if (isLight) {
      img.classList.add("invert-colors");
    } else {
      img.classList.remove("invert-colors");
    }
  });
}

updateImageInversion();
window.matchMedia('(prefers-color-scheme: light)').addEventListener("change", updateImageInversion);

socket.on("input_space_update", (data) => {
  inputSpaceImg.src = `data:image/png;base64,${data.image_data}`;
});

socket.on("robot_vizualisation_update", (data) => {
  robotVizImg.src = `data:image/png;base64,${data.image_data}`;
});


//
// Forms
//

document.getElementById("roboticist-select").addEventListener("change", handleSelectChange);
document.getElementById("robot-select").addEventListener("change", handleSelectChange);
document.getElementById("terrain-select").addEventListener("change", handleSelectChange);

function handleSelectChange(event) {
  const selectElement = event.target;
  const selectedValue = selectElement.value;

  if (selectedValue === "new") {
    openModal(selectElement.id);
    selectElement.selectedIndex = 0;
  }
}

function openModal(selectId) {
  sessionStorage.setItem("currentSelect", selectId);

  document.querySelectorAll(".modal-form").forEach(form => form.style.display = "none");
  document.getElementById(selectId + "-form").style.display = "block";
  document.getElementById("modal").style.display = "flex";
}

function closeModal() {
  document.getElementById("modal").style.display = "none";
}

document.querySelectorAll(".modal-form").forEach(form => {
  form.addEventListener("submit", function (event) {
    event.preventDefault();
    const currentSelect = sessionStorage.getItem("currentSelect");

    let newOptionValue;
    if (currentSelect === "roboticist-select") {
      const name = document.getElementById("newRoboticistName").value.trim();
      const lastName = document.getElementById("newRoboticistLastName").value.trim();
      const email = document.getElementById("newRoboticistEmail").value.trim();
      const experience = document.getElementById("newRoboticistExperience").value.trim();
      const org = document.getElementById("newRoboticistOrg").value.trim();

      if (!name || !lastName || !email || !experience || !org) return;
      newOptionValue = `${name} ${lastName} (${org})`;
    } else {
      newOptionValue = this.querySelector(".newOptionValue").value.trim();
    }

    if (newOptionValue) {
      const selectElement = document.getElementById(currentSelect);
      const newOption = document.createElement("option");
      newOption.value = newOptionValue.toLowerCase().replace(/\s+/g, "_");
      newOption.textContent = newOptionValue;
      const lastOption = selectElement.options[selectElement.options.length - 1];
      selectElement.insertBefore(newOption, lastOption);
      selectElement.value = newOption.value;
      closeModal();
    }
  });
});


//
// Commands
//

const skipCommandButton = document.getElementById("skip-command-button")

function setCurrentStep(step) {
  document.getElementById('current-step').textContent = step;
}

function setTotalSteps(total) {
  document.getElementById('total-steps').textContent = total;
}

function skipCommandButtonClick() {
  socket.emit("skip_command");
}

socket.on("skippable_state_start", (data) => {
  skipCommandButton.disabled = false;
});

socket.on("skippable_state_end", (data) => {
  skipCommandButton.disabled = true;
});