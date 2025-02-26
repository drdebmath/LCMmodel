import {resizeCanvas} from './main.js'

// Uses the inlined panzoom library
const Panzoom = window["Panzoom"]

// Elements
let canvas_zoom_in_button = document.getElementById('canvas-zoom-in-button')
let canvas_zoom_out_button = document.getElementById('canvas-zoom-out-button')

const canvas = document.getElementById('canvas')
const panzoom = Panzoom(canvas, {
  maxScale: 5
})

// Resize the canvas whenever the user zooms in or out (or pinches)
// If this is not done, then the internal canvas will not resize to the absolute scope of the zoom
canvas_zoom_in_button.addEventListener('click', (e) => {
    panzoom.zoomIn(e);
    // resizeCanvas();
  }
)

canvas_zoom_out_button.addEventListener('click', (e) => {
  panzoom.zoomOut(e);
    // resizeCanvas();
  }
)

canvas.addEventListener('wheel', (e) => {
    panzoom.zoomWithWheel(e);
    // resizeCanvas();
  }
)