/**
 * Viewport Observer Utility
 * Wraps canvas animation loops with IntersectionObserver to pause rendering
 * when the canvas is not visible, improving performance.
 */

window.ViewportObserver = (function () {
  const observers = new Map();

  /**
   * Observe a canvas element and control an animation loop
   * @param {HTMLCanvasElement} canvas - The canvas to observe
   * @param {Object} control - Control object with isRunning and optionally onVisibilityChange
   * @param {number} threshold - IntersectionObserver threshold (default: 0.1)
   */
  function observe(canvas, control, threshold = 0.1) {
    if (!canvas) return;

    if (observers.has(canvas)) {
      observers.get(canvas).disconnect();
    }

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          const wasRunning = control.isRunning;
          control.isRunning = entry.isIntersecting;

          // Trigger optional callback
          if (control.onVisibilityChange) {
            control.onVisibilityChange(entry.isIntersecting);
          }

          // Debug log (optional, comment out in production)
        //   console.log(`[ViewportObserver] ${canvas.id || 'canvas'}: ${control.isRunning ? 'ACTIVE' : 'PAUSED'}`);
        });
      },
      { threshold }
    );

    observer.observe(canvas);
    observers.set(canvas, observer);
  }

  /**
   * Stop observing a canvas and resume animation
   * @param {HTMLCanvasElement} canvas - The canvas to unobserve
   */
  function unobserve(canvas) {
    if (observers.has(canvas)) {
      observers.get(canvas).disconnect();
      observers.delete(canvas);
    }
  }

  /**
   * Wrap a requestAnimationFrame loop to respect the control.isRunning flag
   * @param {Function} drawFn - The draw/render function
   * @param {Object} control - Control object with isRunning property
   * @returns {Function} A wrapped RAF function that can be called repeatedly
   */
  function createAnimationLoop(drawFn, control) {
    return function loop() {
      if (control.isRunning) {
        drawFn();
      }
      requestAnimationFrame(loop);
    };
  }

  return {
    observe,
    unobserve,
    createAnimationLoop
  };
})();
