(function (window) {
  const listeners = {};

  function on(event, handler) {
    if (!listeners[event]) listeners[event] = [];
    listeners[event].push(handler);
    return () => off(event, handler);
  }

  function off(event, handler) {
    if (!listeners[event]) return;
    listeners[event] = listeners[event].filter((fn) => fn !== handler);
  }

  function emit(event, payload) {
    if (!listeners[event]) return;
    listeners[event].forEach((handler) => handler(payload));
  }

  window.EventBus = { on, off, emit };
})(window);
