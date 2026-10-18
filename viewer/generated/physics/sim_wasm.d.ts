/* tslint:disable */
/* eslint-disable */

export class Dryden {
    free(): void;
    [Symbol.dispose](): void;
    constructor(seed: number);
    step(dt: number, speed: number, lengths: Float64Array, sigma: Float64Array): Float64Array;
}

export class Kernel {
    free(): void;
    [Symbol.dispose](): void;
    derivative(state: Float64Array, command: Float64Array, wind: Float64Array): Float64Array;
    inverted_launch(trim: Float64Array, time_s: number, hand_height_m: number): Float64Array;
    constructor(parameters: Float64Array, mass: number);
    replay(state: Float64Array, commands: Float64Array, wind: Float64Array, voltage: number, rpm: number, dt: number, substeps: number, causal: boolean): Float64Array;
    surface_step(state: Float64Array, command: Float64Array, wind: Float64Array, voltage: number, rpm: number, dt: number, env: Float64Array, settings: Float64Array): Float64Array;
}

export class Simulator {
    free(): void;
    [Symbol.dispose](): void;
    ground_start(): void;
    constructor(config_json: string);
    release(): void;
    step(command: Float64Array, wind: Float64Array, dt_s: number): Float64Array;
    readonly rpm: number;
    readonly state: Float64Array;
    readonly time_s: number;
}

export function config_schema(): string;

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_dryden_free: (a: number, b: number) => void;
    readonly __wbg_kernel_free: (a: number, b: number) => void;
    readonly __wbg_simulator_free: (a: number, b: number) => void;
    readonly config_schema: () => [number, number];
    readonly dryden_new: (a: number) => [number, number, number];
    readonly dryden_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => [number, number, number, number];
    readonly kernel_derivative: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => [number, number, number, number];
    readonly kernel_inverted_launch: (a: number, b: number, c: number, d: number, e: number) => [number, number, number, number];
    readonly kernel_new: (a: number, b: number, c: number) => [number, number, number];
    readonly kernel_replay: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number) => [number, number, number, number];
    readonly kernel_surface_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number) => [number, number, number, number];
    readonly simulator_ground_start: (a: number) => void;
    readonly simulator_new: (a: number, b: number) => [number, number, number];
    readonly simulator_release: (a: number) => void;
    readonly simulator_rpm: (a: number) => number;
    readonly simulator_state: (a: number) => [number, number];
    readonly simulator_step: (a: number, b: number, c: number, d: number, e: number, f: number) => [number, number, number, number];
    readonly simulator_time_s: (a: number) => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __externref_table_dealloc: (a: number) => void;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
