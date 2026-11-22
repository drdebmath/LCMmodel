/**
 * `advance` result codes, shared with `web/worker.js`.
 * @enum {0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9}
 */
export const Stop = Object.freeze({
    Budget: 0, "0": "Budget",
    UntilTime: 1, "1": "UntilTime",
    Ended: 2, "2": "Ended",
    MaxEvents: 3, "3": "MaxEvents",
    MaxTime: 4, "4": "MaxTime",
    Stalled: 5, "5": "Stalled",
    MaxTurns: 6, "6": "MaxTurns",
    /**
     * Only `advance_to_move` returns it: a robot has just started a move.
     */
    Moved: 7, "7": "Moved",
    MaxRounds: 8, "8": "MaxRounds",
    /**
     * An `await` schedule waits for the next turn or round (adversary mode).
     */
    AwaitingSchedule: 9, "9": "AwaitingSchedule",
});

/**
 * A ring run, stepped by the page one activation at a time.
 */
export class WasmRing {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        WasmRingFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_wasmring_free(ptr, 0);
    }
    /**
     * `config_json` is a `RingConfig`.
     *
     * # Errors
     * Malformed JSON or an invalid ring.
     * @param {string} config_json
     */
    constructor(config_json) {
        const ptr0 = passStringToWasm0(config_json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.wasmring_new(ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        this.__wbg_ptr = ret[0];
        WasmRingFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * The run now, as JSON: positions, status, counts, the task every robot
     * would take, the next robot and what it would do.
     * @returns {string}
     */
    state() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmring_state(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * One activation as JSON (`null` once the run is over).
     * @returns {string}
     */
    step() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmring_step(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
}
if (Symbol.dispose) WasmRing.prototype[Symbol.dispose] = WasmRing.prototype.free;

export class WasmSimulation {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        WasmSimulationFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_wasmsimulation_free(ptr, 0);
    }
    /**
     * Handles up to `max_events` events, stopping early before the first
     * event after `until_time`, at a configured limit, or at the end.
     * @param {number} max_events
     * @param {number} until_time
     * @returns {Stop}
     */
    advance(max_events, until_time) {
        const ret = wasm.wasmsimulation_advance(this.__wbg_ptr, max_events, until_time);
        return ret;
    }
    /**
     * Handles events one by one, as `advance` does, but stops as soon as a robot starts a move
     * (`Stop::Moved`; `moved_robot` says which), even if that is at `until_time` itself. The page
     * uses it to skip turns in which nobody moves, and to catch the start of every walk,
     * including one that begins at the instant the last one ends.
     * @param {number} max_events
     * @param {number} until_time
     * @returns {Stop}
     */
    advance_to_move(max_events, until_time) {
        const ret = wasm.wasmsimulation_advance_to_move(this.__wbg_ptr, max_events, until_time);
        return ret;
    }
    /**
     * When robot `i` arrives, if it is on a move right now; NaN if it is not.
     * @param {number} i
     * @returns {number}
     */
    arrival_time(i) {
        const ret = wasm.wasmsimulation_arrival_time(this.__wbg_ptr, i);
        return ret;
    }
    /**
     * A schedule ending in `await` has run out: the run waits for the
     * adversary's next turn (`push_turn`) or round (`push_round`).
     * @returns {boolean}
     */
    awaiting_schedule() {
        const ret = wasm.wasmsimulation_awaiting_schedule(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Captures the state at the current time; read it with the getters below.
     */
    capture_frame() {
        wasm.wasmsimulation_capture_frame(this.__wbg_ptr);
    }
    /**
     * `[cx0, cy0, r0, …]`, NaN where a robot has no circle.
     * @returns {Float32Array}
     */
    circle() {
        const ret = wasm.wasmsimulation_circle(this.__wbg_ptr);
        return ret;
    }
    /**
     * `frames: "compass"`: each robot's compass deviation from the global
     * north, in radians (empty otherwise).
     * @returns {Float64Array}
     */
    compass() {
        const ret = wasm.wasmsimulation_compass(this.__wbg_ptr);
        return ret;
    }
    /**
     * k-Async: Looks put off so far because they would have broken the bound.
     * @returns {number}
     */
    deferred_looks() {
        const ret = wasm.wasmsimulation_deferred_looks(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {boolean}
     */
    ended() {
        const ret = wasm.wasmsimulation_ended(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Sequential and SSYNC/FSYNC: epochs in which every live robot has been active.
     * @returns {number}
     */
    epochs_completed() {
        const ret = wasm.wasmsimulation_epochs_completed(this.__wbg_ptr);
        return ret;
    }
    /**
     * Events handled so far. `f64` because JS numbers are exact up to 2⁵³.
     * @returns {number}
     */
    event_count() {
        const ret = wasm.wasmsimulation_event_count(this.__wbg_ptr);
        return ret;
    }
    /**
     * What robot `i` would see and which rule it would apply if it looked
     * now, in its algorithm's words (`undefined` if it would not compute or
     * the algorithm gives no explanation). Changes nothing about the run.
     * @param {number} i
     * @returns {string | undefined}
     */
    explain(i) {
        const ret = wasm.wasmsimulation_explain(this.__wbg_ptr, i);
        let v1;
        if (ret[0] !== 0) {
            v1 = getStringFromWasm0(ret[0], ret[1]);
            wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        }
        return v1;
    }
    /**
     * Every robot at the current time as CSV, in full precision.
     * @returns {string}
     */
    export_csv() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmsimulation_export_csv(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Per robot: 0 none, 1 crash, 2 byzantine, 3 omission, 4 delay.
     * @returns {Uint8Array}
     */
    fault() {
        const ret = wasm.wasmsimulation_fault(this.__wbg_ptr);
        return ret;
    }
    /**
     * Captures the current state straight into caller-owned arrays, so the
     * page can hand the same buffers back every frame instead of allocating
     * new ones. Each array must have the length the getters below return.
     * @param {Float32Array} xy
     * @param {Uint8Array} flags
     * @param {Uint8Array} light
     * @param {Uint8Array} fault
     * @param {Uint8Array} task
     * @param {Float32Array} target
     * @param {Float32Array} circle
     */
    fill_frame(xy, flags, light, fault, task, target, circle) {
        var ptr0 = passArrayF32ToWasm0(xy, wasm.__wbindgen_malloc);
        var len0 = WASM_VECTOR_LEN;
        var ptr1 = passArray8ToWasm0(flags, wasm.__wbindgen_malloc);
        var len1 = WASM_VECTOR_LEN;
        var ptr2 = passArray8ToWasm0(light, wasm.__wbindgen_malloc);
        var len2 = WASM_VECTOR_LEN;
        var ptr3 = passArray8ToWasm0(fault, wasm.__wbindgen_malloc);
        var len3 = WASM_VECTOR_LEN;
        var ptr4 = passArray8ToWasm0(task, wasm.__wbindgen_malloc);
        var len4 = WASM_VECTOR_LEN;
        var ptr5 = passArrayF32ToWasm0(target, wasm.__wbindgen_malloc);
        var len5 = WASM_VECTOR_LEN;
        var ptr6 = passArrayF32ToWasm0(circle, wasm.__wbindgen_malloc);
        var len6 = WASM_VECTOR_LEN;
        wasm.wasmsimulation_fill_frame(this.__wbg_ptr, ptr0, len0, xy, ptr1, len1, flags, ptr2, len2, light, ptr3, len3, fault, ptr4, len4, task, ptr5, len5, target, ptr6, len6, circle);
    }
    /**
     * Per robot: bits 0–1 state, bit 2 frozen, bit 3 terminated, bit 4 has target.
     * @returns {Uint8Array}
     */
    flags() {
        const ret = wasm.wasmsimulation_flags(this.__wbg_ptr);
        return ret;
    }
    /**
     * World coordinates the frame's positions, targets and circle centres are
     * measured from (a robot's world position is origin + offset, added in f64).
     * @returns {number}
     */
    frame_origin_x() {
        const ret = wasm.wasmsimulation_frame_origin_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    frame_origin_y() {
        const ret = wasm.wasmsimulation_frame_origin_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    frame_time() {
        const ret = wasm.wasmsimulation_frame_time(this.__wbg_ptr);
        return ret;
    }
    /**
     * SSYNC or FSYNC: robots act in rounds.
     * @returns {boolean}
     */
    in_rounds() {
        const ret = wasm.wasmsimulation_in_rounds(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Per robot: 0 none, 1 blue, 2 red, 3 green.
     * @returns {Uint8Array}
     */
    light() {
        const ret = wasm.wasmsimulation_light(this.__wbg_ptr);
        return ret;
    }
    /**
     * The robot `advance_to_move` last saw start a move, or -1.
     * @returns {number}
     */
    moved_robot() {
        const ret = wasm.wasmsimulation_moved_robot(this.__wbg_ptr);
        return ret;
    }
    /**
     * Robots sharing each robot's point; empty unless multiplicity detection is on.
     * @returns {Uint16Array}
     */
    multiplicity() {
        const ret = wasm.wasmsimulation_multiplicity(this.__wbg_ptr);
        return ret;
    }
    /**
     * `config_json` is a `SimConfig` (docs/core-schema.md §2).
     *
     * # Errors
     * Malformed JSON, an invalid field or an unknown algorithm.
     * @param {string} config_json
     */
    constructor(config_json) {
        const ptr0 = passStringToWasm0(config_json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.wasmsimulation_new(ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        this.__wbg_ptr = ret[0];
        WasmSimulationFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Where robot `i` would go if it looked now, `[x, y]`, without changing
     * the run; empty if it would not compute (crashed, terminated, …).
     * @param {number} i
     * @returns {Float64Array}
     */
    preview_target(i) {
        const ret = wasm.wasmsimulation_preview_target(this.__wbg_ptr, i);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Adds a round in which `robots` look and move together.
     *
     * # Errors
     * Not an SSYNC run with `await` rounds, an empty round, or no such robot.
     * @param {Uint32Array} robots
     */
    push_round(robots) {
        const ptr0 = passArray32ToWasm0(robots, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.wasmsimulation_push_round(this.__wbg_ptr, ptr0, len0);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
    /**
     * Adds a turn for `robot`, stopped after `stop` of the way (NaN: let it
     * arrive, or stop it as the stop policy says).
     *
     * # Errors
     * Not a sequential run with an `await` schedule, no such robot, or a
     * stopping point the movement settings do not allow.
     * @param {number} robot
     * @param {number} stop
     */
    push_turn(robot, stop) {
        const ret = wasm.wasmsimulation_push_turn(this.__wbg_ptr, robot, stop);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
    /**
     * @returns {number}
     */
    robot_count() {
        const ret = wasm.wasmsimulation_robot_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Everything known about robot `i` at the current time, as JSON; `null`
     * for an index out of range.
     * @param {number} i
     * @returns {string}
     */
    robot_info(i) {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmsimulation_robot_info(this.__wbg_ptr, i);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * SSYNC/FSYNC: rounds begun so far.
     * @returns {number}
     */
    round_count() {
        const ret = wasm.wasmsimulation_round_count(this.__wbg_ptr);
        return ret;
    }
    /**
     * SSYNC/FSYNC: the robots active in the round under way.
     * @returns {Uint32Array}
     */
    round_members() {
        const ret = wasm.wasmsimulation_round_members(this.__wbg_ptr);
        return ret;
    }
    /**
     * How many robots see every other robot right now (mutual visibility).
     * @returns {number}
     */
    seeing_everyone() {
        const ret = wasm.wasmsimulation_seeing_everyone(this.__wbg_ptr);
        return ret;
    }
    /**
     * `true` under the sequential scheduler.
     * @returns {boolean}
     */
    sequential() {
        const ret = wasm.wasmsimulation_sequential(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Draw the robots as they are at time `t`, a time between the last event handled and the
     * next one (a walk has no events inside it, so this is what makes it glide). NaN, or any
     * time before the run's clock, goes back to drawing the run's own time.
     * @param {number} t
     */
    set_view_time(t) {
        wasm.wasmsimulation_set_view_time(this.__wbg_ptr, t);
    }
    /**
     * Handles exactly one event and reports it as
     * `[time, robot (-1 = none), kind, outcome]`: kind 0 crash, 1 look,
     * 2 wait, 3 visualize, -1 none; outcome is `StepOutcome::code`.
     * @returns {Float64Array}
     */
    step_one() {
        const ret = wasm.wasmsimulation_step_one(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * How far the farthest robot that has not crashed is from the centre of those robots
     * right now: a measure of how spread out the swarm is, for the page's convergence plot.
     * @returns {number}
     */
    swarm_size() {
        const ret = wasm.wasmsimulation_swarm_size(this.__wbg_ptr);
        return ret;
    }
    /**
     * `[tx0, ty0, …]`, NaN where a robot has no target.
     * @returns {Float32Array}
     */
    target() {
        const ret = wasm.wasmsimulation_target(this.__wbg_ptr);
        return ret;
    }
    /**
     * Per robot: 0 none, 1 red, 2 blue.
     * @returns {Uint8Array}
     */
    task() {
        const ret = wasm.wasmsimulation_task(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    terminated_count() {
        const ret = wasm.wasmsimulation_terminated_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Visualize ticks among the events handled: they belong to no robot.
     * @returns {number}
     */
    tick_count() {
        const ret = wasm.wasmsimulation_tick_count(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    time() {
        const ret = wasm.wasmsimulation_time(this.__wbg_ptr);
        return ret;
    }
    /**
     * Sequential scheduler: turns taken so far (one per Look).
     * @returns {number}
     */
    turn_count() {
        const ret = wasm.wasmsimulation_turn_count(this.__wbg_ptr);
        return ret;
    }
    /**
     * `[x0, y0, x1, y1, …]`, offsets from the frame origin.
     * @returns {Float32Array}
     */
    xy() {
        const ret = wasm.wasmsimulation_xy(this.__wbg_ptr);
        return ret;
    }
}
if (Symbol.dispose) WasmSimulation.prototype[Symbol.dispose] = WasmSimulation.prototype.free;

/**
 * A SqGathering run played one turn at a time (the step-by-step page).
 */
export class WasmSqTrace {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        WasmSqTraceFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_wasmsqtrace_free(ptr, 0);
    }
    /**
     * # Errors
     * Malformed JSON, or a configuration the core or SqGathering refuses.
     * @param {string} config_json
     */
    constructor(config_json) {
        const ptr0 = passStringToWasm0(config_json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.wasmsqtrace_new(ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        this.__wbg_ptr = ret[0];
        WasmSqTraceFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {string}
     */
    report() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmsqtrace_report(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * The start positions `[x0, y0, x1, y1, …]` (stacked starts applied).
     * @returns {Float64Array}
     */
    start_positions() {
        const ret = wasm.wasmsqtrace_start_positions(this.__wbg_ptr);
        return ret;
    }
    /**
     * `"running"`, `"gathered"`, `"incomplete"` or `"stalled"`.
     * @returns {string}
     */
    status() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmsqtrace_status(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Plays the next turn and returns it as JSON, or `null` once the run
     * has stopped.
     * @returns {string}
     */
    step() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.wasmsqtrace_step(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
}
if (Symbol.dispose) WasmSqTrace.prototype[Symbol.dispose] = WasmSqTrace.prototype.free;

/**
 * The algorithms the core knows, as JSON `[{"key": …, "name": …,
 * "sequential_only": …, "multiplicities": …}, …]`: `sequential_only` when
 * the algorithm refuses the async scheduler, `multiplicities` when it works
 * with points that hold several robots (the page rings them).
 * @returns {string}
 */
export function algorithms() {
    let deferred1_0;
    let deferred1_1;
    try {
        const ret = wasm.algorithms();
        deferred1_0 = ret[0];
        deferred1_1 = ret[1];
        return getStringFromWasm0(ret[0], ret[1]);
    } finally {
        wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
    }
}

/**
 * Where `config_json`'s settings leave the model its algorithm was designed
 * or proved for, as a JSON list of sentences (empty inside the model).
 *
 * # Errors
 * Malformed JSON.
 * @param {string} config_json
 * @returns {string}
 */
export function notes(config_json) {
    let deferred3_0;
    let deferred3_1;
    try {
        const ptr0 = passStringToWasm0(config_json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.notes(ptr0, len0);
        var ptr2 = ret[0];
        var len2 = ret[1];
        if (ret[3]) {
            ptr2 = 0; len2 = 0;
            throw takeFromExternrefTable0(ret[2]);
        }
        deferred3_0 = ptr2;
        deferred3_1 = len2;
        return getStringFromWasm0(ptr2, len2);
    } finally {
        wasm.__wbindgen_free(deferred3_0, deferred3_1, 1);
    }
}

/**
 * Runs a SqGathering `SimConfig` to the end: `{"report": …, "trace": [turn,
 * …]}` (`trace` empty unless `with_trace`). Same output as
 * `lcm seq --format json --trace FILE`.
 *
 * # Errors
 * Malformed JSON, or a configuration the core or SqGathering refuses.
 * @param {string} config_json
 * @param {boolean} with_trace
 * @returns {string}
 */
export function sq_gathering_trace(config_json, with_trace) {
    let deferred3_0;
    let deferred3_1;
    try {
        const ptr0 = passStringToWasm0(config_json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.sq_gathering_trace(ptr0, len0, with_trace);
        var ptr2 = ret[0];
        var len2 = ret[1];
        if (ret[3]) {
            ptr2 = 0; len2 = 0;
            throw takeFromExternrefTable0(ret[2]);
        }
        deferred3_0 = ptr2;
        deferred3_1 = len2;
        return getStringFromWasm0(ptr2, len2);
    } finally {
        wasm.__wbindgen_free(deferred3_0, deferred3_1, 1);
    }
}
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg_Error_30c8987f7c2ed4e2: function(arg0, arg1) {
            const ret = Error(getStringFromWasm0(arg0, arg1));
            return ret;
        },
        __wbg___wbindgen_copy_to_typed_array_88899a52af046901: function(arg0, arg1, arg2) {
            new Uint8Array(arg2.buffer, arg2.byteOffset, arg2.byteLength).set(getArrayU8FromWasm0(arg0, arg1));
        },
        __wbg___wbindgen_throw_41e9ee4f547fc59a: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbg_new_from_slice_23f60f47cde8d664: function(arg0, arg1) {
            const ret = new Uint16Array(getArrayU16FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_new_from_slice_37a3c3a194266cb8: function(arg0, arg1) {
            const ret = new Uint32Array(getArrayU32FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_new_from_slice_9a868026ffa4208a: function(arg0, arg1) {
            const ret = new Uint8Array(getArrayU8FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_new_from_slice_ca6ad97db1f4779a: function(arg0, arg1) {
            const ret = new Float32Array(getArrayF32FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_new_from_slice_f545fd22ddc142b8: function(arg0, arg1) {
            const ret = new Float64Array(getArrayF64FromWasm0(arg0, arg1));
            return ret;
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./lcm_wasm_bg.js": import0,
    };
}

const WasmRingFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_wasmring_free(ptr, 1));
const WasmSimulationFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_wasmsimulation_free(ptr, 1));
const WasmSqTraceFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_wasmsqtrace_free(ptr, 1));

function getArrayF32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

function getArrayU16FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint16ArrayMemory0().subarray(ptr / 2, ptr / 2 + len);
}

function getArrayU32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayU8FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint8ArrayMemory0().subarray(ptr / 1, ptr / 1 + len);
}

let cachedFloat32ArrayMemory0 = null;
function getFloat32ArrayMemory0() {
    if (cachedFloat32ArrayMemory0 === null || cachedFloat32ArrayMemory0.byteLength === 0) {
        cachedFloat32ArrayMemory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32ArrayMemory0;
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint16ArrayMemory0 = null;
function getUint16ArrayMemory0() {
    if (cachedUint16ArrayMemory0 === null || cachedUint16ArrayMemory0.byteLength === 0) {
        cachedUint16ArrayMemory0 = new Uint16Array(wasm.memory.buffer);
    }
    return cachedUint16ArrayMemory0;
}

let cachedUint32ArrayMemory0 = null;
function getUint32ArrayMemory0() {
    if (cachedUint32ArrayMemory0 === null || cachedUint32ArrayMemory0.byteLength === 0) {
        cachedUint32ArrayMemory0 = new Uint32Array(wasm.memory.buffer);
    }
    return cachedUint32ArrayMemory0;
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function passArray32ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 4, 4) >>> 0;
    getUint32ArrayMemory0().set(arg, ptr / 4);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArray8ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 1, 1) >>> 0;
    getUint8ArrayMemory0().set(arg, ptr / 1);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArrayF32ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 4, 4) >>> 0;
    getFloat32ArrayMemory0().set(arg, ptr / 4);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

function takeFromExternrefTable0(idx) {
    const value = wasm.__wbindgen_externrefs.get(idx);
    wasm.__externref_table_dealloc(idx);
    return value;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    };
}

let WASM_VECTOR_LEN = 0;

let wasmModule, wasmInstance, wasm;
function __wbg_finalize_init(instance, module) {
    wasmInstance = instance;
    wasm = instance.exports;
    wasmModule = module;
    cachedFloat32ArrayMemory0 = null;
    cachedFloat64ArrayMemory0 = null;
    cachedUint16ArrayMemory0 = null;
    cachedUint32ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (!module.ok) {
            throw new Error(`failed to fetch Wasm: ${module.status} ${module.statusText} fetching '${module.url}'`);
        }

        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('lcm_wasm_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };
