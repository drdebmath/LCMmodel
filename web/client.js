/* Page side of the worker protocol (see worker.js). Owns the frame loop:
 * while pulling, it asks for one frame per animation frame with at most one
 * request in flight, and hands the previous frame's arrays back to the
 * worker to refill, so playback allocates nothing per frame. */

const ARRAYS = ["xy", "flags", "light", "fault", "task", "target", "circle"];

export class SimClient {
  constructor(url) {
    this.worker = new Worker(url, { type: "module" });
    this.handlers = {};
    this.pending = false;
    this.pulling = false;
    this.current = null; // frame being displayed; never sent back
    this.recycle = null; // previous frame's arrays, returned on the next request
    this.csvWaiters = [];
    this.inspectSeq = 0;
    this.run = 0;        // id of the current run; frames from earlier runs are dropped
    this.worker.onmessage = ({ data }) => this.receive(data);
    this.worker.onerror = (e) => this.emit("error", e.message || "The simulation worker failed to load.");
    this.post({ type: "hello" });
  }

  on(name, fn) { this.handlers[name] = fn; return this; }
  emit(name, value) { this.handlers[name]?.(value); }
  post(message, transfer) { this.worker.postMessage(message, transfer ?? []); }

  receive(data) {
    switch (data.type) {
      case "ready": this.emit("ready", data.algorithms); break;
      case "frame": {
        // A frame of an earlier run (in flight when a new run started) must not
        // overwrite the new one, or mark it ended.
        if (data.run !== this.run) break;
        this.pending = false;
        if (this.current) this.recycle = Object.fromEntries(ARRAYS.map((k) => [k, this.current[k]]));
        this.current = data;
        this.emit("frame", data);
        break;
      }
      case "robot":
        if (data.seq === this.inspectSeq) this.emit("robot", data);
        break;
      case "csv": this.csvWaiters.shift()?.(data); break;
      case "error": this.pending = false; this.emit("error", data.message); break;
    }
  }

  start(config) {
    this.pending = false;
    this.post({ type: "start", config, run: ++this.run });
  }
  play() { this.post({ type: "play" }); this.startPulling(); }
  pause() { this.stopPulling(); this.post({ type: "pause" }); this.requestFrame(); }
  speed(value) { this.post({ type: "speed", value }); }
  step(unit, amount = 1) { this.stopPulling(); this.post({ type: "step", unit, amount }); }
  inspect(id) { this.post({ type: "inspect", id, seq: ++this.inspectSeq }); }
  stop() { this.stopPulling(); this.post({ type: "stop" }); }
  csv() { return new Promise((resolve) => { this.csvWaiters.push(resolve); this.post({ type: "csv" }); }); }

  requestFrame() {
    if (this.pending) return;
    this.pending = true;
    const buffers = this.recycle;
    this.recycle = null;
    if (buffers) this.post({ type: "frame", buffers }, ARRAYS.map((k) => buffers[k].buffer));
    else this.post({ type: "frame" });
  }

  startPulling() {
    if (this.pulling) return;
    this.pulling = true;
    const loop = () => {
      if (!this.pulling) return;
      this.requestFrame();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }
  stopPulling() { this.pulling = false; }
}
