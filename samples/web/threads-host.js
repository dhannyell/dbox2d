// threadsHost prepares the main instance of a wasm-threads binary: shared
// memory, the wake import, and __threadsSpawn, which starts one Worker.
function threadsHost(go, module) {
  const memory = new WebAssembly.Memory({ shared: true, initial: 2048, maximum: 16384 });
  go.importObject.env = { ...go.importObject.env, memory };
  let view = new Int32Array(memory.buffer);
  const words = () => {
    if (view.buffer.byteLength !== memory.buffer.byteLength) view = new Int32Array(memory.buffer);
    return view;
  };
  go.importObject.threads = {
    // The browser does not let the main thread block.
    wait: () => { throw new Error("threads.wait on the main thread"); },
    notify: (addr) => { Atomics.notify(words(), addr >> 2); },
  };
  globalThis.__threadsSpawn = (index, stackTop, tlsBase) => {
    new Worker("thread.js").postMessage({ memory, module, index, stackTop, tlsBase });
  };
}
