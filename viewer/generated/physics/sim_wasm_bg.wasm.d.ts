/* tslint:disable */
/* eslint-disable */
export const memory: WebAssembly.Memory;
export const __wbg_dryden_free: (a: number, b: number) => void;
export const __wbg_kernel_free: (a: number, b: number) => void;
export const __wbg_simulator_free: (a: number, b: number) => void;
export const config_schema: () => [number, number];
export const dryden_new: (a: number) => [number, number, number];
export const dryden_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => [number, number, number, number];
export const kernel_derivative: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => [number, number, number, number];
export const kernel_inverted_launch: (a: number, b: number, c: number, d: number, e: number) => [number, number, number, number];
export const kernel_new: (a: number, b: number, c: number) => [number, number, number];
export const kernel_replay: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number) => [number, number, number, number];
export const kernel_surface_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number) => [number, number, number, number];
export const simulator_ground_start: (a: number) => void;
export const simulator_new: (a: number, b: number) => [number, number, number];
export const simulator_release: (a: number) => void;
export const simulator_rpm: (a: number) => number;
export const simulator_state: (a: number) => [number, number];
export const simulator_step: (a: number, b: number, c: number, d: number, e: number, f: number) => [number, number, number, number];
export const simulator_time_s: (a: number) => number;
export const __wbindgen_externrefs: WebAssembly.Table;
export const __wbindgen_free: (a: number, b: number, c: number) => void;
export const __externref_table_dealloc: (a: number) => void;
export const __wbindgen_malloc: (a: number, b: number) => number;
export const __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
export const __wbindgen_start: () => void;
