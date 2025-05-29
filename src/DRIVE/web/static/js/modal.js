function updateTractionFields() {
    const tractionType = document.getElementById("robotTraction").value;
    const tractionDetails = document.getElementById("traction-details");
    tractionDetails.innerHTML = "";

    if (tractionType === "wheeled") {
        tractionDetails.innerHTML = `
        <label for="tireModel">Tire model:</label>
        <input type="text" id="tireModel" required />
  
        <label for="treadDepth">Tread depth:</label>
        <input type="text" id="treadDepth" required />
  
        <label for="tirePressure">Tire pressure (starting from front-right, clockwise):</label>
        <input type="text" id="tirePressure" required />
      `;
    } else if (tractionType === "tracked") {
        tractionDetails.innerHTML = `
        <label for="trackModel">Model of tracks:</label>
        <input type="text" id="trackModel" required />
      `;
    } else if (tractionType === "legged") {
        tractionDetails.innerHTML = `
        <label for="numLegs">Number of legs:</label>
        <input type="number" id="numLegs" min="1" required />
      `;
    } else if (tractionType === "other") {
        tractionDetails.innerHTML = `
        <label for="otherTraction">Specify traction mechanism:</label>
        <input type="text" id="otherTraction" required />
      `;
    }
}

function updateParticleSizeField() {
    const terrainHardness = document.getElementById("terrainHardness").value;
    const container = document.getElementById("particle-size-container");
    container.innerHTML = "";

    if (terrainHardness === "deformable") {
        container.innerHTML = `
        <label for="particleSize">If deformable, specify the most present particle size:</label>
        <select id="particleSize">
          <option value="clay">Smaller than 0.05 mm (clay)</option>
          <option value="sand">0.05 mm to 2.0 mm (sand)</option>
          <option value="small_gravel">2.0 mm to 20 mm (small gravel)</option>
          <option value="big_gravel">20 mm to 63 mm (big gravel)</option>
          <option value="cobbles">Over 63 mm (Cobbles)</option>
        </select>
      `;
    }
}

document.getElementById("robot-select-form").addEventListener("submit", function (event) {
    event.preventDefault();

    const name = document.getElementById("robotName").value.trim();
    const manufacturer = document.getElementById("robotManufacturer").value.trim();
    const modification = document.getElementById("robotModification").value;
    const weight = document.getElementById("robotWeight").value.trim();
    const asymmetry = document.getElementById("robotAsymmetry").value;
    const traction = document.getElementById("robotTraction").value;
    let tractionDetails = "";

    if (traction === "wheeled") {
        const tireModel = document.getElementById("tireModel").value.trim();
        const treadDepth = document.getElementById("treadDepth").value.trim();
        const tirePressure = document.getElementById("tirePressure").value.trim();
        tractionDetails = `Wheeled: ${tireModel}, ${treadDepth}, ${tirePressure}`;
    } else if (traction === "tracked") {
        tractionDetails = `Tracked: ${document.getElementById("trackModel").value.trim()}`;
    } else if (traction === "legged") {
        tractionDetails = `Legged: ${document.getElementById("numLegs").value} legs`;
    } else if (traction === "other") {
        tractionDetails = `Other: ${document.getElementById("otherTraction").value.trim()}`;
    }

    const suspension = document.getElementById("robotSuspension").value;
    const baseline = document.getElementById("robotBaseline").value.trim();
    const wheelRadius = document.getElementById("robotWheelRadius").value.trim();
    const sensors = document.getElementById("robotSensors").value.trim();
    const algorithm = document.getElementById("robotAlgorithm").value.trim();
    const speedTest = document.getElementById("robotSpeedTest").value;

    if (!manufacturer || !weight || !baseline || !wheelRadius || !sensors || !algorithm) {
        alert("Please fill out all required fields.");
        return;
    }

    const newRobotName = `${name} (${manufacturer})`;

    const selectElement = document.getElementById("robot-select");
    const newOption = document.createElement("option");
    newOption.value = newRobotName.toLowerCase().replace(/\s+/g, "_");
    newOption.textContent = newRobotName;

    const lastOption = selectElement.options[selectElement.options.length - 1];
    selectElement.insertBefore(newOption, lastOption);
    selectElement.value = newOption.value;

    closeModal();
});

document.getElementById("terrain-select-form").addEventListener("submit", function (event) {
    event.preventDefault();

    const terrainType = document.getElementById("terrainType").value.trim();
    const terrainUniform = document.getElementById("terrainUniform").value;
    const terrainHardness = document.getElementById("terrainHardness").value;
    const terrainInclination = document.getElementById("terrainInclination").value.trim();
    const terrainFlatness = document.getElementById("terrainFlatness").value;
    const terrainWetness = document.getElementById("terrainWetness").value;
    const terrainContamination = document.getElementById("terrainContamination").value.trim();

    let particleSize = "";
    if (terrainHardness === "deformable") {
        particleSize = document.getElementById("particleSize").value;
    }

    const weatherCondition = document.getElementById("weatherCondition").value.trim();
    const weatherDayNight = document.getElementById("weatherDayNight").value;
    const weatherTemperature = document.getElementById("weatherTemperature").value.trim();
    const weatherGroundFrozen = document.getElementById("weatherGroundFrozen").value;
    const weatherFreezeLastNight = document.getElementById("weatherFreezeLastNight").value;

    if (!terrainType || !terrainInclination || !weatherCondition || !weatherTemperature) {
        alert("Please fill out all required fields.");
        return;
    }

    const newTerrainName = `${terrainType} - ${weatherCondition} (${weatherTemperature}°C)`;

    const selectElement = document.getElementById("terrain-select");
    const newOption = document.createElement("option");
    newOption.value = newTerrainName.toLowerCase().replace(/\s+/g, "_");
    newOption.textContent = newTerrainName;

    const lastOption = selectElement.options[selectElement.options.length - 1];
    selectElement.insertBefore(newOption, lastOption);
    selectElement.value = newOption.value;

    closeModal();
});

function setupImagePreview(inputId, previewId) {
    document.getElementById(inputId).addEventListener("change", function (event) {
        const file = event.target.files[0];
        if (file) {
            const reader = new FileReader();
            reader.onload = function (e) {
                const preview = document.getElementById(previewId);
                preview.src = e.target.result;
                preview.style.display = "block";
            };
            reader.readAsDataURL(file);
        }
    });
}

setupImagePreview("robot-image", "robot-preview");
setupImagePreview("robot-env-image", "robot-env-preview");
setupImagePreview("env-image", "env-preview");
setupImagePreview("ground-image", "ground-preview");

window.addEventListener("DOMContentLoaded", () => {
    updateTractionFields();
    updateParticleSizeField();

    document.getElementById("robotTraction").addEventListener("change", updateTractionFields);
    document.getElementById("terrainHardness").addEventListener("change", updateParticleSizeField);
});