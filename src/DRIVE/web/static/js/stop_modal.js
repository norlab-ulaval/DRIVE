const reasonSelect = document.getElementById("reason-select");
const submitButton = document.getElementById("stop-reason-submit");
const otherReasonInput = document.getElementById("other-reason-input");
const reasonForm = document.getElementById("reason-form");

reasonSelect.addEventListener("change", () => {
    const value = reasonSelect.value;
    submitButton.disabled = value === "";

    const isOther = value === "other";
    otherReasonInput.required = isOther;
    otherReasonInput.style.display = isOther ? "flex" : "none";
});

reasonForm.addEventListener("submit", () => {
    const reason = reasonSelect.value === "other" ? otherReasonInput.value : reasonSelect.value;
    closeStopModal(reason);
});