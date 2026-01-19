/**
 * Recruiter Mode Toggle
 * Optional UI enhancement to surface high-signal content for recruiters
 * Preserves the full research lab experience when OFF
 */

(function () {
  const STORAGE_KEY = 'portfolioRecruiterMode';
  const TOGGLE_CLASS = 'recruiter-mode';

  // Initialize from localStorage or default to OFF
  let isRecruiterMode = localStorage.getItem(STORAGE_KEY) === 'true';

  function applyRecruiterMode() {
    document.body.classList.toggle(TOGGLE_CLASS, isRecruiterMode);
    
    // Update toggle button state
    const toggleBtn = document.getElementById('recruiter-mode-toggle');
    if (toggleBtn) {
      toggleBtn.textContent = isRecruiterMode ? 'Recruiter Mode: ON' : 'Recruiter Mode: OFF';
      toggleBtn.setAttribute('aria-pressed', isRecruiterMode);
    }

    // Save preference
    localStorage.setItem(STORAGE_KEY, isRecruiterMode);
  }

  function toggleRecruiterMode() {
    isRecruiterMode = !isRecruiterMode;
    applyRecruiterMode();
  }

  // Apply on load
  document.addEventListener('DOMContentLoaded', () => {
    applyRecruiterMode();

    // Attach toggle handler
    const toggleBtn = document.getElementById('recruiter-mode-toggle');
    if (toggleBtn) {
      toggleBtn.addEventListener('click', toggleRecruiterMode);
    }
  });

  // Expose to window for debugging
  window.RecruiterMode = {
    toggle: toggleRecruiterMode,
    isActive: () => isRecruiterMode,
  };
})();
