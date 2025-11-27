export class TelemetryBus {
  constructor() {
    this.listeners = new Map();
  }

  on(event, handler) {
    const handlers = this.listeners.get(event) ?? [];
    handlers.push(handler);
    this.listeners.set(event, handlers);
    return () => this.off(event, handler);
  }

  off(event, handler) {
    const handlers = this.listeners.get(event);
    if (!handlers) return;
    this.listeners.set(
      event,
      handlers.filter((fn) => fn !== handler)
    );
  }

  emit(event, payload) {
    (this.listeners.get(event) ?? []).forEach((fn) => fn(payload));
  }
}
