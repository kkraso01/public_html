// LLM Dream Reconstructor - Token hallucination engine
const canvas = document.getElementById("dream-core");
const ctx = canvas.getContext("2d");

// Setup canvas with device pixel ratio scaling
const dpr = window.devicePixelRatio || 1;
canvas.width = window.innerWidth * dpr;
canvas.height = window.innerHeight * dpr;
ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

let tokens = [];
let entropy = 0.98;
let scrollDepth = 0;
let projects = [];
let currentProjectIndex = 0;

// Project content from CV
const hallucinations = [
  {
    title: "IDENTITY FRAGMENT — KONSTANTIN KRASOVITSKIY",
    section: "contact",
    content: "Software Engineer | Researcher | AI Systems Architect\nB.Sc. Computer Science (2025) | M.Sc. Artificial Intelligence (current)\nUniversity of Cyprus\nEmail: kkraso01@ucy.ac.cy | Tel: +357-96083290"
  },
  {
    title: "NEURAL MILESTONE — LLM-MS",
    section: "publication",
    content: "Multi-Model LLM Search Engine — published at LLM + Vector Data Workshop (Hong Kong, May 2025)\nA distributed system orchestrating multiple LLM backends with intelligent routing, vector database integration, and latency optimization.\nCo-authored with Stelios Christou & Demetrios Zeinalipour-Yazti"
  },
  {
    title: "SKILL MATRIX RECONSTRUCTION",
    section: "skills",
    content: "Backend: Python (Flask/FastAPI), JavaScript, Java, C#\nFrontend: HTML5, CSS3, Canvas APIs, Vanilla JS\nInfrastructure: Linux, Docker, Docker Swarm, Bash\nQA: Playwright, Selenium, K6 Performance Testing\nAI/Data: SQL, ChromaDB, Open-source LLMs, Vector Databases"
  },
  {
    title: "EXPERIENCE LAYER — KIOS RESEARCH CENTER",
    section: "experience",
    content: "Software Engineer (Nov 2025 – Present)\nDesign & develop backend/frontend for EU and industry research projects. Deploy microservices, implement REST APIs, integrate Docker-based systems. Maintain code quality through testing and version control."
  },
  {
    title: "EXPERIENCE ECHO — QA AUTOMATION",
    section: "experience",
    content: "QA Automation Engineer — Techlink LTD (Sep 2023 – Oct 2025)\nDesigned test scenarios, automated UI tests with Playwright/C# and Selenium/Java. Ran K6 performance tests on Docker Swarm. Built utilities for test data acceleration and bug verification."
  },
  {
    title: "RESEARCH FRAGMENT — DMSL LAB",
    section: "experience",
    content: "Special Research Scientist (Mar 2025 – Present)\nBuilt Flask backend integrating ChromaDB and open-source LLMs. Deployed and optimized local LLM inference. Provisioned VMs and supported researcher workflows."
  },
  {
    title: "ACHIEVEMENT COLLAPSE — ROBOTICS",
    section: "awards",
    content: "Robotex International 2018: 2nd Place (Lego Sumo) — Estonia\nRobotex Cyprus: 1st Place (Line Following), 3rd Place (Sumo)\nWorld Robotics Olympiad 2019: 2nd Place — Cyprus\nTeam Lead developing control software for EV3, NXT, Arduino platforms"
  },
  {
    title: "TEMPORAL EDGE — CURRENT PROJECTS",
    section: "current",
    content: "Manuscript: 'LLME: Distributed Large Language Model Service on the Edge'\nThesis: 'Multi-Model LLM Search Engines'\nRole: Scientific Committee & Judge at Robotex Cyprus (Mar 2022 – Present)"
  }
];

function randomToken() {
  const chars = "abcdefghijklmnopqrstuvwxyz0123456789[](){}";
  return chars[Math.floor(Math.random() * chars.length)];
}

function createTokenStream() {
  const x = Math.random() * window.innerWidth;
  const y = -20;
  const vx = (Math.random() - 0.5) * 2;
  const vy = 1 + Math.random() * 3;
  const char = randomToken();
  const life = 1;
  
  return { x, y, vx, vy, char, life, hue: Math.random() * 360 };
}

function drawToken(token) {
  const alpha = Math.max(0, token.life * 0.8);
  ctx.fillStyle = `hsla(${token.hue}, 100%, 60%, ${alpha})`;
  ctx.font = "bold 14px monospace";
  ctx.fillText(token.char, token.x, token.y);
}

function updateEntropy() {
  // Entropy decays toward stable state (2x faster)
  entropy = Math.max(0.2, entropy * 0.998 - 0.00002);
  return entropy;
}

function simulateTokenFlow() {
  // Create new tokens with variable intensity based on entropy (2x faster)
  const tokensPerFrame = Math.ceil(entropy * 16);
  for (let i = 0; i < tokensPerFrame; i++) {
    tokens.push(createTokenStream());
  }
  
  // Update existing tokens
  tokens = tokens
    .map(t => ({
      ...t,
      x: t.x + t.vx,
      y: t.y + t.vy,
      life: t.life - 0.02
    }))
    .filter(t => t.life > 0 && t.y < window.innerHeight + 50);
  
  // Draw tokens
  tokens.forEach(drawToken);
}

function drawPixelGlitch() {
  // Glitch effect based on entropy - higher entropy = more glitches
  if (entropy > 0.7 && Math.random() < entropy * 0.1) {
    const x = Math.random() * window.innerWidth;
    const y = Math.random() * window.innerHeight;
    const w = 20 + Math.random() * 60;
    const h = 5 + Math.random() * 20;
    
    ctx.fillStyle = `rgba(0, 255, 255, ${entropy * 0.3})`;
    ctx.fillRect(x, y, w, h);
  }
}

function drawHallucination(text) {
  // Render text as floating particles that slowly stabilize
  const x = window.innerWidth / 2;
  const y = window.innerHeight / 2;
  
  ctx.save();
  ctx.globalAlpha = Math.min(1, (1 - entropy) * 2);
  ctx.fillStyle = "#00ffff";
  ctx.font = "18px monospace";
  ctx.textAlign = "center";
  
  // Wobbly text effect based on entropy
  const wobble = entropy * 10;
  ctx.translate(x + (Math.random() - 0.5) * wobble, y + (Math.random() - 0.5) * wobble);
  
  const lines = text.split("\n");
  lines.forEach((line, idx) => {
    ctx.fillText(line, 0, idx * 25);
  });
  
  ctx.restore();
}

function updateHUD() {
  document.getElementById("entropy").textContent = entropy.toFixed(2);
  document.getElementById("depth").textContent = Math.floor(scrollDepth * 100);
}

function handleScroll(e) {
  // Scroll updates depth in dream space (2x faster)
  scrollDepth += e.deltaY * 0.0002;
  scrollDepth = Math.max(0, Math.min(1, scrollDepth));
  currentProjectIndex = Math.floor(scrollDepth * hallucinations.length);
}

function resize() {
  canvas.width = window.innerWidth * dpr;
  canvas.height = window.innerHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function draw() {
  // Fade canvas with slight trail
  ctx.fillStyle = "rgba(0, 0, 0, 0.15)";
  ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);
  
  // Update entropy
  updateEntropy();
  
  // Draw glitches
  drawPixelGlitch();
  
  // Simulate token flow
  simulateTokenFlow();
  
  // Draw project hallucination based on scroll depth
  const normalizedIndex = scrollDepth * hallucinations.length;
  const projectIndex = Math.floor(normalizedIndex);
  const project = hallucinations[Math.min(projectIndex, hallucinations.length - 1)];
  
  if (project) {
    // Progressive text reveal based on entropy collapse
    const revealRatio = 1 - entropy;
    const contentLength = Math.floor(revealRatio * project.content.length);
    const displayText = `${project.title}\n\n${project.content.substring(0, contentLength)}`;
    drawHallucination(displayText);
    
    // Draw section indicator
    drawSectionIndicator(project.section, projectIndex + 1, hallucinations.length);
  }
  
  updateHUD();
  requestAnimationFrame(draw);
}

function drawSectionIndicator(section, current, total) {
  ctx.save();
  ctx.fillStyle = "rgba(0, 255, 255, 0.4)";
  ctx.font = "12px monospace";
  ctx.textAlign = "left";
  ctx.fillText(`[${current}/${total}] ${section.toUpperCase()}`, 20, window.innerHeight - 20);
  ctx.restore();
}

// Event listeners
window.addEventListener("resize", resize);
window.addEventListener("wheel", handleScroll, { passive: true });

// Start animation
draw();
