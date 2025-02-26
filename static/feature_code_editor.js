import configOptions from "./main.js";

let codeMirrorOptions = {
  lineNumbers: true,
  mode: "python",
  theme: "monokai"
}

// Uses the inlined CodeMirror script
const CodeMirror = window["CodeMirror"]

// It is better to separate the two code mirror instances in case some customization is needed

// Custom algorithm code
var customAlgInput = /** @type {HTMLElement} */ document.getElementById("custom-alg-input-area");

var codeMirrorAlgInit = CodeMirror.fromTextArea(customAlgInput, codeMirrorOptions);

codeMirrorAlgInit.setSize("100%", "100%");

// Terminal code
var customTerminalInput = /** @type {HTMLElement} */ document.getElementById("custom-terminal-code-input-area");

var codeMirrorTerminalInit = CodeMirror.fromTextArea(customTerminalInput, codeMirrorOptions);

codeMirrorTerminalInit.setSize("100%", "100%");

// Update the configOptions variable whenever the user switches tabs

// Attach a listener to all the tab buttons
let tabButtons = document.getElementsByClassName("tab-button");

for (let i = 0; i < tabButtons.length; i++) {
    tabButtons[i].addEventListener('click', syncConfigOptions);
}

function syncConfigOptions() {
  // Update the two aspects of the custom code (alg & terminal) when the user switches tabs
  configOptions.custom_alg = codeMirrorAlgInit.getValue();

  configOptions.custom_term_code = codeMirrorTerminalInit.getValue();
}