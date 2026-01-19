// System Dashboard - Telemetry & Metrics HUD
function initSystemDashboard() {
  const dashboard = document.getElementById("sys-dashboard");
  if (!dashboard) {
    console.warn("Dashboard not found");
    return;
  }

  // Telemetry state
  const telemetry = {
    node: "ACTIVE",
    latency: 24,
    tps: 120,
    gpu: 45,
    vops: 65
  };

  // Animation loop
  const updateInterval = setInterval(() => {
    // Simulate realistic jitter
    telemetry.latency = 20 + Math.random() * 10;
    telemetry.tps = 100 + Math.random() * 70;
    telemetry.gpu = Math.max(20, Math.min(95, telemetry.gpu + (Math.random() - 0.5) * 15));
    telemetry.vops = 40 + Math.random() * 40;

    // Random node state changes (rare)
    if (Math.random() > 0.95) {
      telemetry.node = telemetry.node === "ACTIVE" ? "IDLE" : "ACTIVE";
    }

    // Update DOM
    const nodeEl = document.getElementById("hud-node");
    const latEl = document.getElementById("hud-lat");
    const tpsEl = document.getElementById("hud-tps");
    const gpuEl = document.getElementById("hud-gpu");
    const vopsEl = document.getElementById("hud-vops");

    if (nodeEl) nodeEl.textContent = telemetry.node;
    if (latEl) latEl.textContent = telemetry.latency.toFixed(1);
    if (tpsEl) tpsEl.textContent = Math.floor(telemetry.tps);
    if (gpuEl) gpuEl.textContent = Math.floor(telemetry.gpu);
    if (vopsEl) vopsEl.textContent = Math.floor(telemetry.vops);

    // Update colors based on state
    if (nodeEl) {
      nodeEl.style.color = telemetry.node === "ACTIVE" ? "#34d399" : "#f97316";
    }
  }, 500);

  // Cleanup on page unload
  window.addEventListener("beforeunload", () => clearInterval(updateInterval));

  return {
    getTelemetry: () => telemetry,
    stop: () => clearInterval(updateInterval)
  };
}

// Initialize when DOM is ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initSystemDashboard);
} else {
  initSystemDashboard();
}
