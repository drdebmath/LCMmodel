// Uses the inlined panzoom library
const Panzoom = window["Panzoom"]

// Elements
let canvas_zoom_in_button = document.getElementById('canvas-zoom-in-button')
let canvas_zoom_out_button = document.getElementById('canvas-zoom-out-button')

const canvas = document.getElementById('canvas')
const panzoom = Panzoom(canvas, {
  maxScale: 5
})
panzoom.pan(10, 10)
panzoom.zoom(2, { animate: true })

canvas_zoom_in_button.addEventListener('click', panzoom.zoomIn)
canvas_zoom_out_button.addEventListener('click', panzoom.zoomOut)
canvas.parentElement.addEventListener('wheel', panzoom.zoomWithWheel)