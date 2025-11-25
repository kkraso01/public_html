function animateSystemStatus() {
  const tps = document.getElementById("sys-tps");
  const lat = document.getElementById("sys-lat");

  setInterval(() => {
    tps.textContent = Math.floor(80 + Math.random() * 50);
    lat.textContent = (20 + Math.random() * 10).toFixed(1);
  }, 600);
}

animateSystemStatus();
