function animateSystemStatus() {
  const tps = document.getElementById("sys-tps");
  const lat = document.getElementById("sys-lat");
  const hudTPS = document.getElementById("hud-tps");
  const hudLat = document.getElementById("hud-lat");
  const hudGPU = document.getElementById("hud-gpu");
  const hudVops = document.getElementById("hud-vops");
  const hudNode = document.getElementById("hud-node");
  const heroNode = document.getElementById("sys-state");

  const setMetric = (el, value) => {
    if (!el) return;
    el.textContent = value;
    el.classList.remove("metric-flash");
    void el.offsetWidth; // restart animation
    el.classList.add("metric-flash");
  };

  const cycleNode = () => {
    const states = ["ACTIVE", "STANDBY", "ACTIVE"];
    const state = states[Math.floor(Math.random() * states.length)];
    setMetric(hudNode, state);
    setMetric(heroNode, state);
    [hudNode, heroNode].forEach((el) => {
      if (!el) return;
      el.classList.toggle("text-green-400", state === "ACTIVE");
      el.classList.toggle("text-yellow-300", state === "STANDBY");
    });
  };

  setInterval(() => {
    const nextTPS = Math.floor(80 + Math.random() * 120);
    const nextLat = (18 + Math.random() * 12).toFixed(1);
    const nextGPU = (35 + Math.random() * 45).toFixed(0);
    const nextVops = Math.floor(900 + Math.random() * 600);

    setMetric(tps, nextTPS);
    setMetric(lat, nextLat);
    setMetric(hudTPS, nextTPS);
    setMetric(hudLat, nextLat);
    setMetric(hudGPU, nextGPU);
    setMetric(hudVops, nextVops);
    cycleNode();
  }, 1000);
}

animateSystemStatus();
