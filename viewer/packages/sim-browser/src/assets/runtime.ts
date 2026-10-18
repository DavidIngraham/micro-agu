/** Versioned runtime files, checked against the build's recorded SHA-256 hashes. */
export class RuntimeAssets {
  private constructor(
    private base: string,
    private hashes: Record<string, string>,
  ) {}
  static async load(base: string) {
    const response = await fetch(new URL("build.json", base), {
      cache: "no-cache",
    });
    if (!response.ok) throw Error("Simulator build manifest is unavailable");
    const data = await response.json();
    for (const name of [
      "arduplane.js",
      "arduplane.wasm",
      "sim_wasm.js",
      "sim_wasm_bg.wasm",
      "highdof.json",
    ])
      if (!/^[a-f0-9]{64}$/.test(data.sha256?.[name] ?? ""))
        throw Error(`Build manifest lacks ${name}`);
    if (data.physics_source?.language !== "Rust")
      throw Error("Unsupported physics runtime");
    return new RuntimeAssets(base, data.sha256);
  }
  has(name: string) {
    return Object.hasOwn(this.hashes, name);
  }
  url(name: string) {
    const url = new URL(name, this.base),
      hash = this.hashes[name];
    if (!hash) throw Error(`Undeclared runtime asset ${name}`);
    url.searchParams.set("v", hash);
    return url.href;
  }
  async bytes(name: string) {
    const response = await fetch(this.url(name));
    if (!response.ok) throw Error(`Cannot load ${name}`);
    const bytes = await response.arrayBuffer();
    const hash = Array.from(
      new Uint8Array(await crypto.subtle.digest("SHA-256", bytes)),
      (b) => b.toString(16).padStart(2, "0"),
    ).join("");
    if (hash !== this.hashes[name])
      throw Error(
        `Simulator asset mismatch: ${name}. Reload after publishing completes.`,
      );
    return bytes;
  }
}
