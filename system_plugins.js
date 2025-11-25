// System Plugins Architecture - Extensible plugin system
window.SystemPlugins = {
  registry: {},
  nodes: [],
  animations: [],
  modes: {
    normal: { particleCount: 100, nodeCount: 5 },
    performance: { particleCount: 50, nodeCount: 3 },
    heavy: { particleCount: 200, nodeCount: 8 }
  },
  currentMode: "normal",

  /**
   * Register a custom node
   * @param {string} name - Node identifier
   * @param {Object} config - Node configuration
   */
  registerNode: function(name, config) {
    this.nodes.push({
      name,
      ...config,
      created: Date.now(),
      active: true
    });
    this.registry[name] = config;
    console.log(`[SystemPlugins] Registered node: ${name}`);
  },

  /**
   * Register a custom animation
   * @param {string} name - Animation identifier
   * @param {Function} animationFn - Animation function(t, nodes, particles)
   */
  registerAnimation: function(name, animationFn) {
    this.animations.push({
      name,
      fn: animationFn,
      registered: Date.now()
    });
    console.log(`[SystemPlugins] Registered animation: ${name}`);
  },

  /**
   * Activate all registered nodes
   */
  activateAll: function() {
    this.nodes.forEach(node => {
      node.active = true;
    });
    console.log(`[SystemPlugins] Activated ${this.nodes.length} nodes`);
  },

  /**
   * Deactivate all registered nodes
   */
  deactivateAll: function() {
    this.nodes.forEach(node => {
      node.active = false;
    });
    console.log(`[SystemPlugins] Deactivated ${this.nodes.length} nodes`);
  },

  /**
   * Set system mode (normal, performance, heavy)
   * @param {string} mode - Mode name
   */
  setMode: function(mode) {
    if (this.modes[mode]) {
      this.currentMode = mode;
      console.log(`[SystemPlugins] Mode set to: ${mode}`);
      return true;
    }
    console.warn(`[SystemPlugins] Unknown mode: ${mode}`);
    return false;
  },

  /**
   * Get current mode configuration
   */
  getMode: function() {
    return this.modes[this.currentMode];
  },

  /**
   * Get all registered nodes
   */
  getNodes: function() {
    return this.nodes;
  },

  /**
   * Get all registered animations
   */
  getAnimations: function() {
    return this.animations;
  },

  /**
   * Simulate load scenario
   * @param {number} intensity - 0-100 representing system load
   */
  simulateLoad: function(intensity) {
    const normalized = Math.max(0, Math.min(100, intensity));
    if (normalized > 70) {
      this.setMode("heavy");
    } else if (normalized > 40) {
      this.setMode("normal");
    } else {
      this.setMode("performance");
    }
    console.log(`[SystemPlugins] Load simulation: ${normalized}%`);
  },

  /**
   * Export current plugin state
   */
  export: function() {
    return {
      nodes: this.nodes,
      animations: this.animations,
      mode: this.currentMode,
      timestamp: Date.now()
    };
  },

  /**
   * Log system info
   */
  info: function() {
    console.log("=== SystemPlugins Info ===");
    console.log(`Registered nodes: ${this.nodes.length}`);
    console.log(`Registered animations: ${this.animations.length}`);
    console.log(`Current mode: ${this.currentMode}`);
    console.log(`Mode config:`, this.getMode());
    console.log("===========================");
  }
};

// Example plugin registration (can be extended by users)
SystemPlugins.registerNode("llm-gpt4", {
  type: "llm",
  model: "GPT-4",
  capacity: 1000,
  latency: 50
});

SystemPlugins.registerNode("vector-db", {
  type: "database",
  engine: "ChromaDB",
  capacity: 10000,
  queryTime: 5
});

SystemPlugins.registerNode("embedder", {
  type: "embedder",
  model: "text-embedding-3-large",
  batchSize: 32
});

// Export for external use
console.log("[SystemPlugins] System initialized. Use window.SystemPlugins to interact.");
