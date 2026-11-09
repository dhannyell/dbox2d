// One Worker: an instance over the shared memory. A thread must not call the
// JS host, so every host import throws, except time and wait and notify.
onmessage = (event) => {
  const { memory, module, index, stackTop, tlsBase } = event.data;
  const tlsBlock = 64; // internal/wasmthreads tlsBlock in the dbox2d module
  let view = new Int32Array(memory.buffer);
  const words = () => {
    if (view.buffer.byteLength !== memory.buffer.byteLength) view = new Int32Array(memory.buffer);
    return view;
  };
  const imports = {};
  for (const item of WebAssembly.Module.imports(module)) {
    imports[item.module] = imports[item.module] || {};
    imports[item.module][item.name] = item.kind === "memory" ? memory : () => {
      throw new Error("host import called on a thread: " + item.name);
    };
  }
  imports.threads.wait = (addr, value) => { Atomics.wait(words(), addr >> 2, value); };
  imports.threads.notify = (addr) => { Atomics.notify(words(), addr >> 2); };
  imports.gojs["runtime.ticks"] = () => BigInt(Math.floor((performance.timeOrigin + performance.now()) * 1e6));
  const instance = new WebAssembly.Instance(module, imports);
  instance.exports.__stack_pointer.value = stackTop;
  if (instance.exports.__wasm_init_tls) {
    if (instance.exports.__tls_size.value > tlsBlock) throw new Error("TLS block too small");
    instance.exports.__wasm_init_tls(tlsBase);
  }
  instance.exports.threadsMain(index);
};
